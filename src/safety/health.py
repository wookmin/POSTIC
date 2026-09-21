"""런타임 의존성의 stale 상태를 감시하는 얇은 watchdog."""

from __future__ import annotations

import time


class SafeStopRequested(RuntimeError):
    """추적을 즉시 중지하고 정리 루틴으로 진입해야 하는 상태."""


class HealthMonitor:
    """카메라 프레임과 제어 루프 heartbeat를 감시한다.

    사람 미검출은 정상적인 상황이므로 이 클래스가 pose 부재를 장애로
    판단하지 않는다. 사람 이탈은 기존 IdlePolicy가 처리한다.
    """

    def __init__(self, config: dict):
        safety = config.get("safety") or {}
        health = safety.get("health") or {}
        self.camera_timeout_sec = float(health.get("camera_timeout_sec", 2.0))
        self.control_timeout_sec = float(health.get("control_timeout_sec", 1.0))
        if self.camera_timeout_sec <= 0 or self.control_timeout_sec <= 0:
            raise ValueError("health timeout은 0 보다 커야 합니다")
        self.started_at = None
        self.last_frame_at = None

    def start(self, now=None):
        now = time.monotonic() if now is None else now
        self.started_at = now
        self.last_frame_at = None

    def record_frame(self, now=None):
        self.last_frame_at = time.monotonic() if now is None else now

    def check(self, now, control_heartbeat_at):
        """문제가 있으면 사람이 읽을 수 있는 safe-stop 사유를 반환한다."""
        if self.started_at is None:
            self.start(now)

        if self.last_frame_at is None:
            if now - self.started_at > self.camera_timeout_sec:
                return "camera_timeout: 프레임이 갱신되지 않았습니다"
        elif now - self.last_frame_at > self.camera_timeout_sec:
            return "camera_timeout: 마지막 프레임이 너무 오래되었습니다"

        if (control_heartbeat_at is not None
                and now - control_heartbeat_at > self.control_timeout_sec):
            return "control_timeout: 제어 루프 heartbeat가 끊겼습니다"
        return None
