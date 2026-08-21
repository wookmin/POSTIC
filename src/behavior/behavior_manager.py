"""행동 관리자 — 판단 스레드와 행동 실행을 통합한다.

별도 스레드에서 주기적으로 자세를 평가하고, 정책에 따라 Gemini 를 호출해
교정 행동을 결정한다. 결정된 행동은 메인 루프가 소비할 수 있도록 큐에 넣는다.

제어 루프(50Hz)와 완전히 독립적이다. 제어 루프는 에코(따라하기)를 계속하고,
BehaviorManager 가 교정이 필요하다고 판단하면 그때만 개입한다.
"""

import threading
import time
from dataclasses import dataclass
from queue import SimpleQueue
from typing import Optional

from src.gemini.client import GeminiDecision, decide_posture
from src.perception.posture_features import PostureAngles
from src.posture.classifier import PostureState, classify
from src.posture.policy import PolicyConfig, PolicyState, should_trigger, urgency_level


@dataclass
class CorrectionEvent:
    """교정 이벤트. 메인 루프가 읽어서 TTS 출력 등에 쓴다."""
    timestamp: float
    decision: GeminiDecision
    posture_label: str
    urgency: int


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

    def __init__(self, config: dict):
        super().__init__(name="behavior", daemon=True)
        correction = config.get("correction", {})
        self._config = PolicyConfig(
            sustain_seconds=correction.get("sustain_seconds", 5.0),
            cooldown_seconds=correction.get("cooldown_seconds", 30.0),
            escalation_seconds=correction.get("escalation_seconds", 60.0),
        )
        self._check_interval = correction.get("check_interval_sec", 1.0)
        self._policy_state = PolicyState()
        self._events = SimpleQueue()
        self._stop = threading.Event()

        # 최신 자세를 메인 스레드가 넣고, 판단 스레드가 읽는다.
        self._lock = threading.Lock()
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
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            self._stop.wait(timeout=self._check_interval)
            if self._stop.is_set():
                break
            self._evaluate()

    def _evaluate(self):
        """현재 자세를 정책에 대조하고, 필요하면 Gemini 를 호출한다."""
        with self._lock:
            angles = self._latest_angles
            posture = self._latest_posture

        if angles is None or posture is None:
            return

        now = time.monotonic()

        if not should_trigger(self._policy_state, posture, now, self._config):
            return

        # 트리거됨 → Gemini 호출
        urgency = urgency_level(self._policy_state, self._config)
        features = {
            "torso_pitch_deg": angles.torso_pitch_deg,
            "neck_pitch_deg": angles.neck_pitch_deg,
            "duration_sec": self._config.sustain_seconds,
            "since_last_correction": now - self._policy_state.last_correction_at
                                     if self._policy_state.last_correction_at > 0
                                     else 999,
        }

        try:
            decision = decide_posture(posture.label, features, urgency)
        except Exception as exc:
            # API 실패 — 조용히 넘긴다. 다음 주기에 다시 시도.
            print(f"[BehaviorManager] Gemini 호출 실패: {exc}")
            return

        if decision.action == "ignore":
            return

        event = CorrectionEvent(
            timestamp=now,
            decision=decision,
            posture_label=posture.label,
            urgency=urgency,
        )
        self._events.put(event)
