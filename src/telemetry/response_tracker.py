"""개입 후 사용자 반응을 계산하는 상태 추적기."""

from __future__ import annotations

import time


class ResponseTracker:
    """교정 개입의 반응시간·무시율·자세 유지시간을 기록한다."""

    def __init__(self, event_logger, response_timeout_sec=15.0):
        if response_timeout_sec <= 0:
            raise ValueError("response_timeout_sec 은 0 보다 커야 합니다")
        self.logger = event_logger
        self.response_timeout_sec = response_timeout_sec
        self._active = None

    def intervention(self, now, posture_label, behavior):
        """새 개입을 시작한다. 미해결 이전 개입은 무시된 것으로 닫는다."""
        if self._active is not None:
            self._finish_ignored(now, "superseded")
        self._active = {
            "started_at": now,
            "posture_label": posture_label,
            "behavior": behavior,
            "responded_at": None,
            "good_since": None,
        }
        self.logger.record(
            "intervention",
            timestamp=now,
            posture=posture_label,
            behavior=behavior,
        )

    def update(self, now, posture_label):
        """현재 분류 결과로 반응과 유지시간을 갱신한다."""
        active = self._active
        if active is None:
            return

        if active["responded_at"] is None:
            if posture_label == "good":
                active["responded_at"] = now
                active["good_since"] = now
                self.logger.record(
                    "posture_corrected",
                    timestamp=now,
                    response_time_sec=round(now - active["started_at"], 4),
                )
            elif now - active["started_at"] >= self.response_timeout_sec:
                self._finish_ignored(now, "timeout")
            return

        if posture_label != "good":
            self.logger.record(
                "posture_maintenance",
                timestamp=now,
                good_duration_sec=round(now - active["good_since"], 4),
                observation_complete=True,
            )
            self._active = None

    def close(self, now=None):
        """세션 종료 시 열린 관찰을 불완전한 상태로 기록한다."""
        now = time.monotonic() if now is None else now
        if self._active is None:
            return
        if self._active["responded_at"] is None:
            self._finish_ignored(now, "session_finished")
        else:
            self.logger.record(
                "posture_maintenance",
                timestamp=now,
                good_duration_sec=round(now - self._active["good_since"], 4),
                observation_complete=False,
            )
            self._active = None

    def _finish_ignored(self, now, reason):
        active = self._active
        self.logger.record(
            "intervention_ignored",
            timestamp=now,
            posture=active["posture_label"],
            behavior=active["behavior"],
            reason=reason,
            observation_complete=reason != "session_finished",
        )
        self._active = None
