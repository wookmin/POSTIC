"""행동 관리자 — 자세 정책과 행동 실행을 통합한다.

별도 스레드에서 주기적으로 자세를 평가하고, 정책에 따라 고정 행동 이벤트를
큐에 넣는다. 모델이나 외부 API는 사용하지 않아 같은 입력에서 같은 행동을
재현할 수 있다.

제어 루프(50Hz)와 완전히 독립적이다. 기본 자세 반응 모드에서는 제어 루프가
중립을 유지하고, BehaviorManager가 나쁜 자세 지속을 확인했을 때만 개입한다.
"""

import threading
import time
from dataclasses import dataclass
from queue import SimpleQueue
from typing import Optional

from src.perception.posture_features import PostureAngles
from src.posture.classifier import PostureState, classify
from src.posture.policy import PolicyConfig, PolicyState, should_trigger, urgency_level


@dataclass
class BehaviorDecision:
    """로컬 정책이 만든 결정. 모터 각도는 포함하지 않는다."""

    action: str
    behavior: str
    speech: str = ""


@dataclass
class CorrectionEvent:
    """교정 이벤트. 메인 루프가 읽어 모션 큐에 전달한다."""
    timestamp: float
    decision: BehaviorDecision
    posture_label: str
    urgency: int
    observed_angles: Optional[PostureAngles] = None


class BehaviorManager(threading.Thread):
    """판단 스레드. daemon=True 라 메인이 끝나면 같이 죽는다.

    사용법:
        manager = BehaviorManager(config)
        manager.start()

        # 메인 루프에서 매 프레임:
        manager.update_posture(angles)

        # 교정 이벤트 소비:
        event = manager.poll_event()
        if event:
            print(event.decision.speech)
    """

    def __init__(self, config: dict, condition=None):
        super().__init__(name="behavior", daemon=True)
        experiment = config.get("experiment") or {}
        self.condition = condition or experiment.get("condition",
                                                    "posture_trigger")
        if self.condition != "posture_trigger":
            raise ValueError("현재 지원하는 개입 조건은 posture_trigger 뿐입니다")
        correction = config.get("correction", {})
        self._config = PolicyConfig(
            sustain_seconds=correction.get("sustain_seconds", 3.0),
            max_corrections_per_run=correction.get(
                "max_corrections_per_run", None),
            cooldown_seconds=correction.get("cooldown_seconds", 30.0),
            escalation_seconds=correction.get("escalation_seconds", 60.0),
        )
        self._check_interval = correction.get("check_interval_sec", 1.0)
        self._policy_state = PolicyState()
        self._events = SimpleQueue()
        # threading.Thread.join() 내부에서 사용하는 _stop() 메서드를
        # 가리지 않도록 별도 이름을 사용한다.
        self._stop_event = threading.Event()

        # 최신 자세를 메인 스레드가 넣고, 판단 스레드가 읽는다.
        self._lock = threading.Lock()
        self._policy_lock = threading.Lock()
        self._latest_angles: Optional[PostureAngles] = None
        self._latest_posture: Optional[PostureState] = None

    def update_posture(self, angles: PostureAngles):
        """메인 루프가 매 프레임 호출. 최신 자세를 갱신한다."""
        posture = classify(angles)
        with self._lock:
            self._latest_angles = angles
            self._latest_posture = posture

    def poll_event(self) -> Optional[CorrectionEvent]:
        """비블로킹. 교정 이벤트가 있으면 꺼내고 없으면 None."""
        if self._events.empty():
            return None
        return self._events.get_nowait()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=self._check_interval)
            if self._stop_event.is_set():
                break
            self._evaluate()

    def _evaluate(self):
        """현재 자세를 정책에 대조하고, 필요하면 개입 이벤트를 만든다."""
        with self._lock:
            angles = self._latest_angles
            posture = self._latest_posture

        if angles is None or posture is None:
            return

        now = time.monotonic()

        # 이벤트 소비 측에서 정책을 재무장할 수 있으므로 트리거와 강도
        # 계산을 같은 잠금으로 묶어 상태가 반쯤 갱신되지 않게 한다.
        with self._policy_lock:
            if not should_trigger(self._policy_state, posture, now,
                                  self._config):
                return
            urgency = urgency_level(self._policy_state, self._config)
        decision = self._fixed_decision(posture.label)

        if decision.action == "ignore":
            self.rearm_after_skipped_event()
            return

        event = CorrectionEvent(
            timestamp=now,
            decision=decision,
            posture_label=posture.label,
            urgency=urgency,
            observed_angles=angles,
        )
        self._events.put(event)

    def rearm_after_skipped_event(self):
        """실제로 실행되지 않은 이벤트를 다시 감지할 수 있게 한다."""
        with self._lock:
            posture = self._latest_posture

        with self._policy_lock:
            state = self._policy_state
            state.armed = True
            state.last_correction_at = -9999.0
            state.correction_count = max(0, state.correction_count - 1)
            if posture is not None and posture.is_triggerable_bad:
                # 지금 자세가 여전히 나쁘더라도 처음부터 지속시간을 다시
                # 세어, 큐 지연 때문에 즉시 재개입하지 않게 한다.
                state.bad_since = time.monotonic()
                state.last_label = posture.label
            else:
                state.bad_since = None
                state.last_label = (posture.label
                                    if posture is not None else "unknown")

    @staticmethod
    def _fixed_decision(posture_label: str) -> BehaviorDecision:
        """자세 라벨을 고정 개입 이벤트로 바꾼다.

        모터 동작은 BehaviorExecutor가 설정된 포즈로 변환한다. 이 단계에서
        모델이 각도나 새로운 행동을 만들어낼 수 없게 해 실험 반복성을 지킨다.
        """
        behaviors = {
            "lateral_tilt": "mimic_bad_posture",
            "slouch": "mimic_slouch",
            "forward_head": "mimic_forward_head",
            "slouch_and_forward": "mimic_slouch",
        }
        return BehaviorDecision(
            action="gentle_remind",
            behavior=behaviors.get(posture_label, "neutral"),
            speech="",
        )
