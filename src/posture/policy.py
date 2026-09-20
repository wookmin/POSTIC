"""교정 정책 — 언제 Gemini 를 호출할지 결정한다.

나쁜 자세가 일정 시간 지속돼야 교정을 트리거하고,
교정 후에는 쿨다운 동안 재호출하지 않는다.
짧게 자세가 나빠졌다 바로 돌아오면 무시한다.
"""

from dataclasses import dataclass

from src.posture.classifier import PostureState


@dataclass
class PolicyConfig:
    """정책 파라미터. posture.yaml 의 correction 섹션에서 읽는다."""
    sustain_seconds: float = 5.0      # 나쁜 자세가 이만큼 지속돼야 트리거
    cooldown_seconds: float = 30.0    # 교정 후 재호출 금지 시간
    escalation_seconds: float = 60.0  # 같은 문제 반복 시 강도 높이는 기준


@dataclass
class PolicyState:
    """정책의 내부 상태. BehaviorManager 가 소유한다."""
    bad_since: float | None = None        # 나쁜 자세 시작 시각
    last_correction_at: float = -9999.0   # 마지막 교정 시각 (초기: 과거로 설정해 첫 트리거 허용)
    correction_count: int = 0             # 연속 교정 횟수 (좋아지면 초기화)
    last_label: str = "good"
    armed: bool = True                    # 정상 회복 전에는 재트리거하지 않음


def should_trigger(state: PolicyState, posture: PostureState,
                   now: float, config: PolicyConfig) -> bool:
    """Gemini 교정을 트리거해야 하는지 판단한다.

    True 를 반환하면 호출자가 고정 개입 이벤트를 만들거나, 호환 모드에서
    Gemini를 호출한다.
    상태를 직접 갱신하므로 매 프레임 한 번만 호출해야 한다.
    """
    # unknown은 나쁜 자세가 아니다. 관측 불가 구간을 지속시간에 포함하지
    # 않기 위해 타이머를 끊고, 다시 보인 시점부터 새로 측정한다.
    if posture.label == "unknown":
        state.bad_since = None
        state.last_label = "unknown"
        return False

    # 좋은 자세로 돌아왔을 때만 재무장한다. lateral_tilt은 인식은 하되
    # 현재 pitch-only 로봇이 표현할 고정 포즈가 없으므로 트리거하지 않으며,
    # 이미 실행된 이벤트를 lateral_tilt만으로 다시 무장시키지 않는다.
    if posture.label == "good":
        state.bad_since = None
        # 마지막 교정 후 20초 이상 좋은 자세면 연속 카운트 리셋
        if state.last_correction_at > 0 and now - state.last_correction_at > 20.0:
            state.correction_count = 0
        state.last_label = "good"
        state.armed = True
        return False

    if posture.label == "lateral_tilt":
        state.bad_since = None
        state.last_label = "lateral_tilt"
        return False

    is_bad = posture.is_triggerable_bad

    # 현재 분류기에 없는 비트리거 상태는 관측 불가로 취급한다.
    if not is_bad:
        state.bad_since = None
        state.last_label = posture.label
        return False

    # 한 번 트리거된 뒤 같은 나쁜 자세가 계속되면, 정상 자세로 돌아오기
    # 전까지는 같은 이벤트를 다시 만들지 않는다.
    if not state.armed:
        return False

    # 나쁜 자세 시작 기록
    if state.bad_since is None:
        state.bad_since = now
        state.last_label = posture.label
        return False

    # 쿨다운 중이면 트리거하지 않음
    if now - state.last_correction_at < config.cooldown_seconds:
        return False

    # 지속 시간 체크
    duration = now - state.bad_since
    if duration < config.sustain_seconds:
        return False

    # 트리거!
    state.last_correction_at = now
    state.correction_count += 1
    state.bad_since = None  # 리셋해서 다음 판단은 새로 시작
    state.last_label = posture.label
    state.armed = False
    return True


def urgency_level(state: PolicyState, config: PolicyConfig) -> int:
    """0~3 강도. 반복될수록 높아진다."""
    if state.correction_count <= 1:
        return 1  # gentle
    if state.correction_count <= 3:
        return 2  # moderate
    return 3      # strong
