"""제어 지령이 하드웨어에 나가기 전 마지막 관문.

관절 한계는 JointMapper 가 이미 자르지만, 여기서는 시간축을 본다.
한 주기에 얼마나 크게 움직여도 되는지, 사람이 사라졌을 때 무엇을 할지.
"""

from dataclasses import dataclass

TICKS_PER_DEG = 4096 / 360.0

STATE_TRACKING = "tracking"
STATE_RETURNING = "returning"
STATE_IDLE = "idle"
STATE_SAFE_STOP = "safe_stop"


@dataclass
class SlewLimiter:
    """제어 주기당 관절 변화량을 제한한다.

    지연 버퍼가 비었다가 다시 차거나 인식이 튀면 목표가 순간적으로 크게
    바뀔 수 있다. 그대로 내보내면 관절이 튀어 아래 단에 충격이 간다.
    """

    max_step_ticks: float

    def apply(self, current, targets):
        if self.max_step_ticks <= 0:
            raise ValueError("max_step_ticks 는 0 보다 커야 합니다")
        limited = {}
        for name, target in targets.items():
            if name not in current:
                limited[name] = target
                continue
            delta = target - current[name]
            if delta > self.max_step_ticks:
                delta = self.max_step_ticks
            elif delta < -self.max_step_ticks:
                delta = -self.max_step_ticks
            limited[name] = int(round(current[name] + delta))
        return limited


def max_step_ticks(max_step_deg):
    return max_step_deg * TICKS_PER_DEG


class IdlePolicy:
    """사람이 사라졌을 때의 상태 기계.

    tracking -> (사람 없음) -> returning -> (중립 도달 후 대기) -> idle
    idle 에서 토크를 해제한다. 굽은 자세로 계속 버티면 아래 단이 과부하로
    고장나기 때문이다. 실제로 base_pitch 가 그렇게 한 번 죽었다.
    """

    def __init__(self, return_seconds, release_seconds, tolerance_ticks=15,
                 grace_seconds=0.5):
        self.return_seconds = return_seconds
        self.release_seconds = release_seconds
        self.tolerance_ticks = tolerance_ticks
        # 인식은 몇 프레임씩 끊긴다. 그때마다 중립으로 출발했다 돌아오면
        # 로봇이 덜컥거린다. 이 시간만큼 연속으로 안 보여야 복귀를 시작한다.
        self.grace_seconds = grace_seconds
        self.state = STATE_TRACKING
        self._left_at = None
        self._missing_since = None
        self._settled_at = None

    def update(self, now, person_visible, current, neutral):
        """상태를 갱신하고 (state, 토크 유지 여부) 를 돌려준다."""
        if person_visible:
            self.state = STATE_TRACKING
            self._left_at = None
            self._missing_since = None
            self._settled_at = None
            return STATE_TRACKING, True

        if self.state == STATE_TRACKING:
            if self._missing_since is None:
                self._missing_since = now
            if now - self._missing_since < self.grace_seconds:
                # 아직 유예 중이다. 마지막 자세를 유지한다.
                return STATE_TRACKING, True
            self.state = STATE_RETURNING
            self._left_at = now
            self._settled_at = None

        if self.state == STATE_RETURNING:
            at_neutral = all(
                abs(current.get(name, value) - value) <= self.tolerance_ticks
                for name, value in neutral.items())
            timed_out = (self._left_at is not None
                         and now - self._left_at >= self.return_seconds)
            if at_neutral or timed_out:
                if self._settled_at is None:
                    self._settled_at = now
                if now - self._settled_at >= self.release_seconds:
                    self.state = STATE_IDLE
                    return STATE_IDLE, False
            return STATE_RETURNING, True

        return STATE_IDLE, False
