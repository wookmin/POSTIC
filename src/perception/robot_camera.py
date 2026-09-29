"""로봇에 탑재되어 로봇과 함께 움직이는 카메라의 판정 게이트.

카메라가 몸통·목 위에 달리면 로봇이 숙일 때 화면 전체가 바뀐다. 이 시점
변화는 사용자의 자세 변화와 구분할 수 없으므로, 로봇이 정해진 관측 자세에
멈춘 뒤 사용자를 다시 안정적으로 찾았을 때만 판정을 재개한다.

    로봇 움직임/안정화 중 ─→ 판정 중지 (blind)
    관측 자세 안정화 완료 ─→ 연속으로 유효한 관측이 reacquire_sec 이어지면 재개
"""

import math


class ReacquireGate:
    """관측 재개 여부를 결정하는 상태 기계. IO 가 없어 단독으로 테스트한다."""

    def __init__(self, reacquire_sec):
        reacquire_sec = float(reacquire_sec)
        if not math.isfinite(reacquire_sec) or reacquire_sec < 0:
            raise ValueError("reacquire_sec 는 0 이상이어야 합니다")
        self.reacquire_sec = reacquire_sec
        # 시작 직후에도 카메라가 안정되기 전 프레임은 쓰지 않는다.
        self.blind = True
        self._valid_since = None

    def update(self, now, robot_ready, person_valid):
        """(이번 프레임을 판정에 써도 되는지, 방금 재확보했는지) 를 돌려준다."""
        if not robot_ready:
            self.blind = True
            self._valid_since = None
            return False, False

        if not self.blind:
            # 재확보 이후 사람을 놓치는 것은 일반적인 관측 불가(unknown)다.
            return True, False

        if not person_valid:
            # 재확보 중에 놓치면 처음부터 다시 센다.
            self._valid_since = None
            return False, False

        if self._valid_since is None:
            self._valid_since = now
        if now - self._valid_since < self.reacquire_sec:
            return False, False

        self.blind = False
        self._valid_since = None
        return True, True
