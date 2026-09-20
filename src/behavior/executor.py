"""LLM 행동 이름을 고정된 모션 명령으로 변환한다.

Gemini가 직접 모터 각도나 속도를 생성하지 않도록, 실행 가능한 행동은
코드/설정에 미리 선언한다. 변환 결과도 SafetyGate를 통과해야 한다.
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


class BehaviorExecutor:
    """실험 조건에 맞는 행동을 안전한 high-level 명령으로 변환한다."""

    CONDITIONS = frozenset({
        "voice", "mirror",
    })

    def __init__(self, config: dict, safety_gate, condition=None):
        self.safety_gate = safety_gate
        experiment = config.get("experiment") or {}
        self.condition = condition or experiment.get("condition", "mirror")
        if self.condition not in self.CONDITIONS:
            raise ValueError(f"지원하지 않는 개입 조건: {self.condition}")
        self.duration_sec = float(experiment.get("action_duration_sec", 1.0))

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
