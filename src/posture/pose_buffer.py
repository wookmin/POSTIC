"""인식 스레드와 제어 스레드 사이의 최신 자세 전달 상자."""

import threading


class PoseBuffer:
    """최신 자세 하나를 보관한다. 스레드 안전하다."""

    def __init__(self):
        self._latest = None
        self._lock = threading.Lock()

    def push(self, sample):
        """샘플을 넣는다. 과거로 되돌아가는 타임스탬프는 무시한다."""
        with self._lock:
            if (self._latest is not None
                    and sample.timestamp <= self._latest.timestamp):
                return False
            self._latest = sample
            return True

    def latest(self):
        """가장 최근 관측값을 반환한다."""
        with self._lock:
            return self._latest
