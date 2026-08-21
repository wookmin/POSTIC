"""MediaPipe 랜드마크에서 시상면 굽힘각을 뽑는다.

카메라도 모터도 모르는 순수 함수만 둔다. 랜드마크처럼 생긴 객체(.x .y .z
.visibility)만 주면 되므로 하드웨어 없이 테스트할 수 있다.

MediaPipe world landmark 규약:
  원점은 골반 중점, x 는 오른쪽, y 는 아래쪽, z 는 깊이이며 작을수록 카메라에
  가깝다. 따라서 앞으로 숙이면 어깨의 z 가 골반보다 작아진다.
"""

import math
from collections import deque
from dataclasses import dataclass
from statistics import median

NOSE = 0
LEFT_EAR = 7
RIGHT_EAR = 8
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_HIP = 23
RIGHT_HIP = 24

# 책상에 앉으면 골반은 거의 항상 프레임 밖이거나 가려진다. world 좌표에서
# 골반은 원점으로 고정되어 정보를 주지 않으므로 가시성 판정에서 뺀다.
REQUIRED = (LEFT_SHOULDER, RIGHT_SHOULDER)


@dataclass(frozen=True)
class PostureAngles:
    """한 시점의 자세. 각도는 도 단위, 양수가 앞으로 숙인 방향이다."""

    timestamp: float
    torso_pitch_deg: float
    neck_pitch_deg: float
    confidence: float

    @property
    def valid(self):
        return self.confidence > 0.0


def midpoint(a, b):
    return ((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)


def sagittal_angle(origin, tip):
    """origin 에서 tip 으로 향하는 벡터의 시상면 기울기(도).

    수직으로 서 있으면 0, 앞으로(카메라 쪽으로) 기울면 양수.
    """
    dy = tip[1] - origin[1]
    dz = tip[2] - origin[2]
    return math.degrees(math.atan2(-dz, -dy))


def min_visibility(landmarks, indices):
    values = []
    for index in indices:
        values.append(getattr(landmarks[index], "visibility", 1.0))
    return min(values) if values else 0.0


def head_reference(world, landmarks, threshold):
    """머리 방향을 대표할 점. 귀가 보이면 귀 중점, 아니면 코."""
    ear_visibility = min_visibility(landmarks, (LEFT_EAR, RIGHT_EAR))
    if ear_visibility >= threshold:
        return midpoint(world[LEFT_EAR], world[RIGHT_EAR])
    nose = world[NOSE]
    return (nose.x, nose.y, nose.z)


def extract_angles(world, landmarks, timestamp, min_visibility_threshold=0.5,
                   invert_torso=False, invert_neck=False):
    """랜드마크 한 쌍에서 PostureAngles 를 만든다. 사람이 안 보이면 None.

    world 는 3D 월드 좌표, landmarks 는 visibility 를 가진 이미지 좌표다.
    """
    if not world or not landmarks:
        return None

    confidence = min_visibility(landmarks, REQUIRED)
    if confidence < min_visibility_threshold:
        return None

    shoulder = midpoint(world[LEFT_SHOULDER], world[RIGHT_SHOULDER])
    hip = midpoint(world[LEFT_HIP], world[RIGHT_HIP])
    torso = sagittal_angle(hip, shoulder)

    head = head_reference(world, landmarks, min_visibility_threshold)
    # 목은 상체에 실려 있으므로 상체 기울기를 뺀 상대각을 쓴다.
    # 로봇도 직렬 체인이라 목 관절은 그 아래 관절 위에서 움직인다.
    neck = sagittal_angle(shoulder, head) - torso

    if invert_torso:
        torso = -torso
    if invert_neck:
        neck = -neck

    return PostureAngles(
        timestamp=timestamp,
        torso_pitch_deg=torso,
        neck_pitch_deg=neck,
        confidence=confidence,
    )


def clamp_angles(angles, max_torso_deg, max_neck_deg):
    """입력 각을 허용 범위로 자른다. 값이 튀어도 로봇이 급격히 움직이지 않게."""
    return PostureAngles(
        timestamp=angles.timestamp,
        torso_pitch_deg=max(-max_torso_deg, min(max_torso_deg,
                                                angles.torso_pitch_deg)),
        neck_pitch_deg=max(-max_neck_deg, min(max_neck_deg,
                                              angles.neck_pitch_deg)),
        confidence=angles.confidence,
    )


def smooth(previous, current, alpha, neck_alpha=None):
    """지수평활. alpha 가 1 이면 필터 없음.

    neck_alpha 를 따로 주면 목에 더 강한(낮은) 필터를 걸 수 있다.
    정면 카메라에서 목 추정은 노이즈가 심하므로 별도 감쇠가 필요하다.
    """
    if previous is None or alpha >= 1.0:
        return current
    na = neck_alpha if neck_alpha is not None else alpha
    return PostureAngles(
        timestamp=current.timestamp,
        torso_pitch_deg=(alpha * current.torso_pitch_deg
                         + (1 - alpha) * previous.torso_pitch_deg),
        neck_pitch_deg=(na * current.neck_pitch_deg
                        + (1 - na) * previous.neck_pitch_deg),
        confidence=current.confidence,
    )


@dataclass(frozen=True)
class PostureReference:
    """바른 자세를 기준으로 잡은 값. 계통 편향을 빼는 데 쓴다.

    골반을 추정으로 채우기 때문에 상체각에는 사람과 자리마다 다른 일정한
    치우침이 생긴다. 실측에서 가만히 앉아 있어도 평균이 20도로 나왔다.
    편향은 이렇게 빼고, 남는 잡음은 필터가 담당한다.
    """

    torso_pitch_deg: float
    neck_pitch_deg: float


def apply_reference(angles, reference):
    if reference is None:
        return angles
    return PostureAngles(
        timestamp=angles.timestamp,
        torso_pitch_deg=angles.torso_pitch_deg - reference.torso_pitch_deg,
        neck_pitch_deg=angles.neck_pitch_deg - reference.neck_pitch_deg,
        confidence=angles.confidence,
    )


class MedianFilter:
    """각 축에 중앙값 필터를 건다.

    단안 깊이는 가끔 크게 튄다. 평균은 그 한 프레임에 끌려가지만 중앙값은
    버틴다. 창이 커질수록 조용해지고 그만큼 늦어지는데, 에코가 이미 지연을
    두고 있으므로 여기서 생기는 지연은 그 안에 묻힌다.
    """

    def __init__(self, window):
        if window < 1:
            raise ValueError("window 는 1 이상이어야 합니다")
        self.window = window
        self._torso = deque(maxlen=window)
        self._neck = deque(maxlen=window)

    def reset(self):
        self._torso.clear()
        self._neck.clear()

    def apply(self, angles):
        self._torso.append(angles.torso_pitch_deg)
        self._neck.append(angles.neck_pitch_deg)
        return PostureAngles(
            timestamp=angles.timestamp,
            torso_pitch_deg=median(self._torso),
            neck_pitch_deg=median(self._neck),
            confidence=angles.confidence,
        )
