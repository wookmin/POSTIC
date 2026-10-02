"""자세 이벤트를 고정된 모션 명령으로 변환한다.

관측값을 모터 각도로 직접 복사하지 않고, 자세 라벨에 대응하는 고정 포즈만
호출한다. 변환 결과도 SafetyGate를 통과해야 한다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from src.perception.posture_features import PostureAngles


@dataclass(frozen=True)
class BehaviorAction:
    behavior: str
    pose: Optional[PostureAngles]
    duration_sec: float
    speech: str
    hold_until_good: bool = False


class BehaviorExecutor:
    """자세 반응 이벤트를 안전한 high-level 명령으로 변환한다."""

    CONDITIONS = frozenset({"posture_trigger"})

    def __init__(self, config: dict, safety_gate, condition=None):
        self.safety_gate = safety_gate
        experiment = config.get("experiment") or {}
        self.condition = condition or experiment.get("condition",
                                                     "posture_trigger")
        if self.condition not in self.CONDITIONS:
            raise ValueError(f"지원하지 않는 개입 조건: {self.condition}")
        intervention = config.get("intervention") or {}
        self.duration_sec = float(
            intervention.get("action_duration_sec",
                            experiment.get("action_duration_sec", 1.0)))
        self.fixed_poses = {
            # 몸통 각도는 4개 관절에 나뉘므로 작은 값은 실제로 거의
            # 보이지 않는다. 기본값도 시연용 과장 포즈로 둔다.
            "bad_posture": {"torso_pitch_deg": 60.0, "neck_pitch_deg": 35.0},
            "slouch": {"torso_pitch_deg": 14.0, "neck_pitch_deg": 4.0},
            "forward_head": {"torso_pitch_deg": 4.0, "neck_pitch_deg": 14.0},
            "slouch_and_forward": {
                "torso_pitch_deg": 14.0, "neck_pitch_deg": 14.0,
            },
        }
        for label, values in (intervention.get("poses") or {}).items():
            if label in self.fixed_poses:
                self.fixed_poses[label] = {
                    "torso_pitch_deg": float(
                        values.get("torso_pitch_deg",
                                  self.fixed_poses[label]["torso_pitch_deg"])),
                    "neck_pitch_deg": float(
                        values.get("neck_pitch_deg",
                                  self.fixed_poses[label]["neck_pitch_deg"])),
                }
        self._generic_pose_configured = (
            "bad_posture" in (intervention.get("poses") or {}))

    def build(self, event):
        """CorrectionEvent를 실행 가능한 BehaviorAction으로 만든다."""
        decision = event.decision
        if decision.action == "ignore":
            return None

        posture_label = getattr(event, "posture_label", None)
        if posture_label is None:
            return None
        use_generic = (self._generic_pose_configured
                        or posture_label == "lateral_tilt")
        values = (self.fixed_poses["bad_posture"] if use_generic
                  else self.fixed_poses.get(posture_label))
        if values is None:
            return None
        pose = PostureAngles(
            timestamp=time.monotonic(),
            torso_pitch_deg=values["torso_pitch_deg"],
            neck_pitch_deg=values["neck_pitch_deg"],
            confidence=1.0,
        )
        return BehaviorAction(
            behavior="bad_posture" if use_generic else posture_label,
            pose=self.safety_gate.clamp_pose(pose),
            duration_sec=self.safety_gate.clamp_duration(self.duration_sec),
            hold_until_good=True,
            speech="",
        )
