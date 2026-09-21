"""타임스탬프가 붙은 자세 샘플의 링 버퍼.

에코의 핵심이다. 인식 스레드가 push 하고, 제어 스레드가 "지금 - 지연" 시점의
값을 sample 로 꺼낸다. 두 스레드의 주기가 달라도 되고, 인식이 느려져도 재생
템포는 벽시계 기준으로 일정하게 유지된다.
"""

import bisect
import threading
from collections import deque

from src.perception.posture_features import PostureAngles


class PoseBuffer:
    """시간 순서로 쌓이는 자세 버퍼. 스레드 안전하다."""

    def __init__(self, span_seconds):
        if span_seconds <= 0:
            raise ValueError("span_seconds 는 0 보다 커야 합니다")
        self._span = span_seconds
        self._samples = deque()
        self._lock = threading.Lock()

    def __len__(self):
        with self._lock:
            return len(self._samples)

    def push(self, sample):
        """샘플을 넣는다. 과거로 되돌아가는 타임스탬프는 무시한다."""
        with self._lock:
            if self._samples and sample.timestamp <= self._samples[-1].timestamp:
                return False
            self._samples.append(sample)
            cutoff = sample.timestamp - self._span
            while len(self._samples) > 1 and self._samples[0].timestamp < cutoff:
                self._samples.popleft()
            return True

    def span(self):
        """버퍼가 담고 있는 (가장 오래된, 가장 최근) 타임스탬프."""
        with self._lock:
            if not self._samples:
                return None
            return self._samples[0].timestamp, self._samples[-1].timestamp

    def latest(self):
        """가장 최근 관측값을 반환한다.

        미러링처럼 지연된 샘플이 필요한 모드와 달리, 자세 반응 모드는
        카메라가 현재 사람을 보고 있는지만 확인하면 되므로 최신값을 쓴다.
        """
        with self._lock:
            return self._samples[-1] if self._samples else None

    def sample(self, when):
        """when 시점의 자세를 선형 보간해 돌려준다. 범위 밖이면 None.

        버퍼가 아직 채워지지 않았거나(워밍업) 요청 시각이 미래면 None 이다.
        호출자는 None 을 "아직 재생할 것이 없음"으로 다뤄야 한다.
        """
        with self._lock:
            if not self._samples:
                return None
            if when < self._samples[0].timestamp:
                return None
            if when > self._samples[-1].timestamp:
                return None

            # deque 는 bisect 를 직접 지원하지 않으므로 타임스탬프 리스트를 만든다.
            # 버퍼 크기는 span * push_rate (보통 6초 * 30fps = 180) 로 작아서
            # 리스트 생성 비용은 무시할 수 있다.
            times = [s.timestamp for s in self._samples]
            index = bisect.bisect_left(times, when)
            if times[index] == when:
                return self._samples[index]

            before = self._samples[index - 1]
            after = self._samples[index]

        gap = after.timestamp - before.timestamp
        ratio = 0.0 if gap <= 0 else (when - before.timestamp) / gap
        return PostureAngles(
            timestamp=when,
            torso_pitch_deg=_lerp(before.torso_pitch_deg,
                                  after.torso_pitch_deg, ratio),
            neck_pitch_deg=_lerp(before.neck_pitch_deg,
                                 after.neck_pitch_deg, ratio),
            confidence=min(before.confidence, after.confidence),
            lateral_tilt_deg=_lerp(before.lateral_tilt_deg,
                                   after.lateral_tilt_deg, ratio),
            torso_compression=_lerp(before.torso_compression,
                                    after.torso_compression, ratio),
            neck_compression=_lerp(before.neck_compression,
                                   after.neck_compression, ratio),
        )


def _lerp(start, end, ratio):
    return start + (end - start) * ratio
