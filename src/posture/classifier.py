"""규칙 기반 자세 분류기.

Gemini 호출 전에 빠르게 자세 상태를 판정한다.
매 프레임 호출해도 부하가 없는 순수 함수다.
"""

from dataclasses import dataclass
from typing import Literal

from src.perception.posture_features import PostureAngles

PostureLabel = Literal[
    "good", "slouch", "forward_head", "slouch_and_forward",
    "lateral_tilt", "unknown",
]


@dataclass(frozen=True)
class PostureState:
    """한 시점의 자세 판정 결과."""
    label: PostureLabel
    torso_severity: float  # 0.0~1.0, 임계값 대비 비율
    neck_severity: float

    @property
    def is_bad(self):
        """관측 가능한 나쁜 자세인지 반환한다."""
        return self.label in {
            "slouch", "forward_head", "slouch_and_forward", "lateral_tilt",
        }

    @property
    def is_triggerable_bad(self):
        """현재 로봇의 pitch 고정 포즈로 표현할 수 있는 나쁜 자세인지 반환한다."""
        return self.label in {"slouch", "forward_head", "slouch_and_forward"}


# 임계값 (도). 이 이상이면 나쁜 자세로 판정.
TORSO_THRESHOLD_DEG = 12.0
NECK_THRESHOLD_DEG = 10.0
LATERAL_THRESHOLD_DEG = 12.0


def classify(angles: PostureAngles) -> PostureState:
    """PostureAngles 에서 자세 상태를 판정한다.

    양수가 앞으로 숙인 방향이므로, 임계값보다 크면 나쁜 자세.
    """
    if not angles.valid:
        # 사람을 놓친 프레임을 정상 자세로 취급하면 나쁜 자세 타이머가
        # 잘못 초기화되거나, 반대로 관측되지 않은 시간이 지속시간에
        # 포함될 수 있다. 정상/나쁨과 별개의 관측 불가 상태로 전파한다.
        return PostureState(label="unknown", torso_severity=0.0,
                            neck_severity=0.0)

    torso = angles.torso_pitch_deg
    neck = angles.neck_pitch_deg

    torso_bad = torso > TORSO_THRESHOLD_DEG
    neck_bad = neck > NECK_THRESHOLD_DEG
    lateral_bad = angles.lateral_tilt_deg > LATERAL_THRESHOLD_DEG

    torso_severity = max(0.0, min(1.0, torso / 45.0)) if torso > 0 else 0.0
    neck_severity = max(0.0, min(1.0, neck / 35.0)) if neck > 0 else 0.0

    # 현재 로봇은 pitch만 표현할 수 있으므로 좌우 기울기는 별도 상태로
    # 남긴다. 이를 slouch로 바꾸면 정면 웹캠에서 오작동할 때 로봇이 움직인다.
    if lateral_bad:
        label = "lateral_tilt"
    elif torso_bad and neck_bad:
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
