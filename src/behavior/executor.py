"""자세 이벤트를 고정된 모션 명령으로 변환한다.

Gemini가 직접 모터 각도나 속도를 생성하지 않도록, 실행 가능한 행동은
코드/설정에 미리 선언한다. 자세 반응 모드에서는 관측 각도를 복사하지 않고
자세 라벨에 대응하는 고정 포즈만 호출한다. 변환 결과도 SafetyGate를
통과해야 한다.
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
    """실험 조건에 맞는 행동을 안전한 high-level 명령으로 변환한다."""

    CONDITIONS = frozenset({
        "voice", "mirror", "posture_trigger",
    })

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
            "bad_posture": {"torso_pitch_deg": 40.0, "neck_pitch_deg": 30.0},
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

        if self.condition == "voice":
            # 음성 조건에서는 모터가 추가 자극을 만들지 않도록 자세를
            # 덮어쓰지 않는다. speech만 실험 자극으로 사용한다.
            return BehaviorAction(
                behavior="voice",
                pose=None,
                duration_sec=self.safety_gate.clamp_duration(self.duration_sec),
                speech=decision.speech,
            )

        if self.condition == "posture_trigger":
            posture_label = getattr(event, "posture_label", None)
            if posture_label is None:
                # 오래된 테스트/이벤트와의 호환. 새 이벤트는 반드시
                # posture_label을 전달한다.
                posture_label = {
                    "mimic_slouch": "slouch",
                    "mimic_forward_head": "forward_head",
                }.get(decision.behavior)
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
                # 현재 프로토타입은 음성 장치 없이 모션만 검증한다.
                speech="",
            )

        if self.condition == "mirror":
            observed = event.observed_angles
            if observed is None:
                # 실시간 이벤트는 항상 관측 자세를 포함해야 한다. 누락된
                # 이벤트는 고정 포즈를 추측하지 않고 모션 없이 종료한다.
                return BehaviorAction(
                    behavior="mirror",
                    pose=None,
                    duration_sec=self.safety_gate.clamp_duration(self.duration_sec),
                    speech="",
                )
            pose = PostureAngles(
                timestamp=time.monotonic(),
                torso_pitch_deg=observed.torso_pitch_deg,
                neck_pitch_deg=observed.neck_pitch_deg,
                confidence=observed.confidence,
            )
            return BehaviorAction(
                behavior="mirror",
                pose=self.safety_gate.clamp_pose(pose),
                duration_sec=self.safety_gate.clamp_duration(self.duration_sec),
                speech="",
            )

        raise ValueError(f"지원하지 않는 개입 조건: {self.condition}")
