"""이미지 좌표(2D) 기반 자세 추정.

정면 카메라에서 Z축(깊이)은 단안 RGB로 추정하기 때문에 노이즈가 크다.
대신 이미지의 Y좌표 비율만으로 상체와 목의 기울기를 판정한다.

원리:
  - 바로 앉으면: 코 → 어깨 → 골반이 수직으로 정렬 (Y 간격이 균등)
  - 숙이면: 코가 어깨에 가까워지고, 어깨가 골반에 가까워짐
  - 이 비율 변화를 '각도와 유사한 값'으로 환산한다

장점:
  - 카메라 위치/각도에 덜 민감
  - Z축 노이즈 영향 제로
  - 정면 카메라에서도 안정적
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


def _mid_y(landmarks, idx_a, idx_b):
    """두 랜드마크의 이미지 Y좌표 중점."""
    return (landmarks[idx_a].y + landmarks[idx_b].y) / 2.0


def _mid_x(landmarks, idx_a, idx_b):
    return (landmarks[idx_a].x + landmarks[idx_b].x) / 2.0


def min_visibility(landmarks, indices):
    values = []
    for index in indices:
        values.append(getattr(landmarks[index], "visibility", 1.0))
    return min(values) if values else 0.0


def extract_angles(world, landmarks, timestamp, min_visibility_threshold=0.5,
                   invert_torso=False, invert_neck=False):
    """이미지 좌표에서 자세 각도를 추정한다.

    world 인자는 호환성을 위해 받지만 사용하지 않는다 (2D 전용).
    landmarks 는 이미지 좌표(.x .y 가 0~1 범위)와 visibility 를 가진다.

    반환값의 torso_pitch_deg / neck_pitch_deg 는 실제 '도'가 아니라
    비율을 도 단위로 환산한 유사값이다. 방향과 크기 감각은 동일하다.
    """
    if not landmarks:
        return None

    confidence = min_visibility(landmarks, REQUIRED)
    if confidence < min_visibility_threshold:
        return None

    # --- 2D 좌표 추출 ---
    shoulder_y = _mid_y(landmarks, LEFT_SHOULDER, RIGHT_SHOULDER)
    shoulder_x = _mid_x(landmarks, LEFT_SHOULDER, RIGHT_SHOULDER)
    hip_y = _mid_y(landmarks, LEFT_HIP, RIGHT_HIP)
    hip_x = _mid_x(landmarks, LEFT_HIP, RIGHT_HIP)

    # 머리: 귀가 보이면 귀 중점, 아니면 코
    ear_vis = min_visibility(landmarks, (LEFT_EAR, RIGHT_EAR))
    if ear_vis >= min_visibility_threshold:
        head_y = _mid_y(landmarks, LEFT_EAR, RIGHT_EAR)
        head_x = _mid_x(landmarks, LEFT_EAR, RIGHT_EAR)
    else:
        head_y = landmarks[NOSE].y
        head_x = landmarks[NOSE].x

    # --- 상체 기울기 (torso pitch) ---
    # 바로 앉으면 어깨-골반 벡터가 수직(dx≈0). 숙이면 어깨가 앞(카메라 쪽)으로
    # 이동하지만 정면이라 X축 변화로 나타남 + Y축으로 어깨가 골반에 가까워짐.
    #
    # 핵심 지표: 어깨-골반의 X 편차를 Y 거리로 나눈 비율.
    # 정면 카메라에서 숙이면 어깨 Y가 골반 Y에 접근하므로 Y거리가 줄어든다.
    # 하지만 더 robust한 방법: 어깨-골반 벡터의 기울기 각도.
    torso_dy = hip_y - shoulder_y  # 양수 (골반이 아래)
    torso_dx = hip_x - shoulder_x  # 보통 0에 가까움

    if torso_dy <= 0.01:
        # 어깨가 골반 아래에 있으면 비정상 (완전히 엎드린 상태)
        torso_pitch_deg = 45.0
    else:
        # 바로 앉으면 어깨-골반이 순수 수직 → angle≈0
        # 숙이면 어깨가 앞으로 → 이미지에서 어깨 Y가 올라감 → torso_dy 줄어듦
        # 이걸 기준 대비 줄어든 비율로 환산.
        # 하지만 정면에서는 X 편차가 더 신뢰도 높음.
        # 두 가지를 결합: atan2(dx, dy)
        torso_angle_rad = math.atan2(abs(torso_dx), torso_dy)
        torso_pitch_deg = math.degrees(torso_angle_rad)

    # --- 목 기울기 (neck pitch) = 머리-어깨 상대각 ---
    # 고개를 숙이면 머리 Y가 어깨 Y에 가까워짐 (또는 X로 치우침)
    neck_dy = shoulder_y - head_y   # 양수 (머리가 위)
    neck_dx = head_x - shoulder_x   # 머리가 어깨 대비 좌우 치우침

    if neck_dy <= 0.01:
        neck_pitch_deg = 35.0
    else:
        # 바로 있으면 머리가 어깨 위에 수직 → angle≈0
        # 고개를 숙이면 머리 Y가 어깨에 접근 + X로 쏠림
        neck_angle_rad = math.atan2(abs(neck_dx), neck_dy)
        neck_pitch_deg = math.degrees(neck_angle_rad)

    # 부호 결정: X 양수(오른쪽) 방향으로 치우치면 양수
    # 하지만 우리 로봇은 앞뒤만 있으므로, 부호는 항상 양수(숙인 정도)로 사용
    # invert 옵션으로 방향 뒤집기 가능
    if invert_torso:
        torso_pitch_deg = -torso_pitch_deg
    if invert_neck:
        neck_pitch_deg = -neck_pitch_deg

    return PostureAngles(
        timestamp=timestamp,
        torso_pitch_deg=torso_pitch_deg,
        neck_pitch_deg=neck_pitch_deg,
        confidence=confidence,
    )


def clamp_angles(angles, max_torso_deg, max_neck_deg):
    """입력 각을 허용 범위로 자른다."""
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
    """바른 자세를 기준으로 잡은 값.

    2D 비율 기반에서도 사람과 카메라 위치마다 '바른 자세'의 수치가 다르다.
    캘리브레이션으로 그 값을 재서 빼면 0 = 바른 자세가 된다.
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
    """각 축에 중앙값 필터를 건다."""

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
