"""규칙 기반 자세 분류기.

Gemini 호출 전에 빠르게 자세 상태를 판정한다.
매 프레임 호출해도 부하가 없는 순수 함수다.
"""

from dataclasses import dataclass
from typing import Literal

from src.perception.posture_features import PostureAngles

PostureLabel = Literal["good", "slouch", "forward_head", "slouch_and_forward"]


@dataclass(frozen=True)
class PostureState:
    """한 시점의 자세 판정 결과."""
    label: PostureLabel
    torso_severity: float  # 0.0~1.0, 임계값 대비 비율
    neck_severity: float


# 임계값 (도). 이 이상이면 나쁜 자세로 판정.
TORSO_THRESHOLD_DEG = 12.0
NECK_THRESHOLD_DEG = 10.0


def classify(angles: PostureAngles) -> PostureState:
    """PostureAngles 에서 자세 상태를 판정한다.

    양수가 앞으로 숙인 방향이므로, 임계값보다 크면 나쁜 자세.
    """
    if not angles.valid:
        return PostureState(label="good", torso_severity=0.0, neck_severity=0.0)

    torso = angles.torso_pitch_deg
    neck = angles.neck_pitch_deg

    torso_bad = torso > TORSO_THRESHOLD_DEG
    neck_bad = neck > NECK_THRESHOLD_DEG

    torso_severity = max(0.0, min(1.0, torso / 45.0)) if torso > 0 else 0.0
    neck_severity = max(0.0, min(1.0, neck / 35.0)) if neck > 0 else 0.0

    if torso_bad and neck_bad:
        label = "slouch_and_forward"
    elif torso_bad:
        label = "slouch"
    elif neck_bad:
        label = "forward_head"
    else:
        label = "good"

    return PostureState(
        label=label,
        torso_severity=torso_severity,
        neck_severity=neck_severity,
    )
