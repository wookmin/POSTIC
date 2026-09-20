"""이미지 좌표(2D) 기반 자세 추정.

정면 카메라에서 Z축(깊이)은 단안 RGB로 추정하기 때문에 노이즈가 크다.
대신 신체 크기로 정규화한 Y축 압축량과 좌우 기울기를 분리한다.

원리:
  - 바로 앉으면: 코 → 어깨 → 골반이 수직으로 정렬 (Y 간격이 균등)
  - 앞으로 숙이면: 머리-어깨와 어깨-골반의 세로 간격이 줄어든다
  - 좌우로 기울이면: 어깨선·골반선이 기울거나 몸통 중심이 옆으로 이동한다
  - 두 값을 분리해 좌우 기울기를 구부정함으로 잘못 판정하지 않는다

장점:
  - 사람과 카메라 사이 거리 변화에 덜 민감
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
    lateral_tilt_deg: float = 0.0
    torso_compression: float = 0.0
    neck_compression: float = 0.0

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
    hips_visible = (min_visibility(landmarks, (LEFT_HIP, RIGHT_HIP))
                    >= min_visibility_threshold)

    # 머리: 귀가 보이면 귀 중점, 아니면 코
    ear_vis = min_visibility(landmarks, (LEFT_EAR, RIGHT_EAR))
    if ear_vis >= min_visibility_threshold:
        head_y = _mid_y(landmarks, LEFT_EAR, RIGHT_EAR)
    else:
        head_y = landmarks[NOSE].y

    # --- 전방 굽힘 proxy ---
    # 단안 정면 카메라는 실제 깊이 방향 각도를 볼 수 없다. 어깨 너비를
    # 신체 크기로 사용해 세로 간격을 정규화한다. 이 값은 실제 각도가 아니라
    # 노트북 카메라용 자세 변화 지표다.
    shoulder_width = abs(
        landmarks[RIGHT_SHOULDER].x - landmarks[LEFT_SHOULDER].x)
    scale = max(shoulder_width, 0.02)
    neck_gap = max(0.0, shoulder_y - head_y)

    # 사용자별 calibration 없이 쓰기 위한 보수적인 기준이다.
    if hips_visible:
        hip_y = _mid_y(landmarks, LEFT_HIP, RIGHT_HIP)
        hip_x = _mid_x(landmarks, LEFT_HIP, RIGHT_HIP)
        hip_width = abs(landmarks[RIGHT_HIP].x - landmarks[LEFT_HIP].x)
        scale = max(shoulder_width, hip_width, 0.02)
        torso_gap = max(0.0, hip_y - shoulder_y)
        torso_ratio = torso_gap / scale
        torso_compression = max(0.0, min(1.0,
                                         (1.35 - torso_ratio) / 0.55))
    else:
        # 노트북 카메라가 어깨 위주로 잡혀도 사람과 목 자세는 측정한다.
        # 골반이 보이지 않는 구간의 척추 굽힘값은 추측하지 않고 0으로 둔다.
        hip_x = None
        torso_compression = 0.0
    neck_ratio = neck_gap / scale
    neck_compression = max(0.0, min(1.0, (0.95 - neck_ratio) / 0.45))
    torso_pitch_deg = torso_compression * 30.0
    neck_pitch_deg = neck_compression * 25.0

    # --- 좌우 기울기 ---
    # 어깨선·골반선의 기울기와 두 중심의 수평 이격을 함께 본다. 앞으로
    # 숙여도 양쪽 선이 수평이고 중심이 유지되면 lateral 값은 작게 남는다.
    shoulder_dx = abs(landmarks[RIGHT_SHOULDER].x
                     - landmarks[LEFT_SHOULDER].x)
    shoulder_dy = abs(landmarks[RIGHT_SHOULDER].y
                     - landmarks[LEFT_SHOULDER].y)
    # 좌우 반전된 영상에서도 dx의 부호 때문에 180도가 나오지 않도록
    # 선분의 방향이 아니라 기울기의 크기만 계산한다.
    shoulder_line = math.degrees(math.atan2(shoulder_dy,
                                            max(shoulder_dx, 0.02)))
    lateral_values = [abs(shoulder_line)]
    if hips_visible:
        hip_dx = abs(landmarks[RIGHT_HIP].x - landmarks[LEFT_HIP].x)
        hip_dy = abs(landmarks[RIGHT_HIP].y - landmarks[LEFT_HIP].y)
        hip_line = math.degrees(math.atan2(hip_dy, max(hip_dx, 0.02)))
        center_offset = abs(shoulder_x - hip_x) / scale
        # torso_gap을 분모로 쓰면 앞으로 숙일수록 같은 작은 중심 오차가
        # lateral 값으로 과장된다. 몸 크기(scale)를 기준으로 계산한다.
        center_tilt = math.degrees(math.atan2(center_offset, 1.0))
        lateral_values.extend((abs(hip_line), center_tilt))
    lateral_tilt_deg = max(lateral_values)

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
        lateral_tilt_deg=lateral_tilt_deg,
        torso_compression=torso_compression,
        neck_compression=neck_compression,
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
        lateral_tilt_deg=angles.lateral_tilt_deg,
        torso_compression=angles.torso_compression,
        neck_compression=angles.neck_compression,
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
        lateral_tilt_deg=(alpha * current.lateral_tilt_deg
                          + (1 - alpha) * previous.lateral_tilt_deg),
        torso_compression=(alpha * current.torso_compression
                           + (1 - alpha) * previous.torso_compression),
        neck_compression=(na * current.neck_compression
                          + (1 - na) * previous.neck_compression),
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
        lateral_tilt_deg=angles.lateral_tilt_deg,
        torso_compression=angles.torso_compression,
        neck_compression=angles.neck_compression,
    )


class MedianFilter:
    """각 축에 중앙값 필터를 건다."""

    def __init__(self, window):
        if window < 1:
            raise ValueError("window 는 1 이상이어야 합니다")
        self.window = window
        self._torso = deque(maxlen=window)
        self._neck = deque(maxlen=window)
        self._lateral = deque(maxlen=window)
        self._torso_compression = deque(maxlen=window)
        self._neck_compression = deque(maxlen=window)

    def reset(self):
        self._torso.clear()
        self._neck.clear()
        self._lateral.clear()
        self._torso_compression.clear()
        self._neck_compression.clear()

    def apply(self, angles):
        self._torso.append(angles.torso_pitch_deg)
        self._neck.append(angles.neck_pitch_deg)
        self._lateral.append(angles.lateral_tilt_deg)
        self._torso_compression.append(angles.torso_compression)
        self._neck_compression.append(angles.neck_compression)
        return PostureAngles(
            timestamp=angles.timestamp,
            torso_pitch_deg=median(self._torso),
            neck_pitch_deg=median(self._neck),
            confidence=angles.confidence,
            lateral_tilt_deg=median(self._lateral),
            torso_compression=median(self._torso_compression),
            neck_compression=median(self._neck_compression),
        )
