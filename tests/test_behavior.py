"""자세 분류기와 교정 정책 테스트."""

import pytest

from src.perception.posture_features import PostureAngles
from src.posture.classifier import classify
from src.posture.policy import PolicyConfig, PolicyState, should_trigger, urgency_level


class TestClassifier:
    def test_good_posture(self):
        angles = PostureAngles(1.0, 5.0, 3.0, 1.0)
        state = classify(angles)
        assert state.label == "good"

    def test_slouch(self):
        angles = PostureAngles(1.0, 20.0, 3.0, 1.0)
        state = classify(angles)
        assert state.label == "slouch"

    def test_forward_head(self):
        angles = PostureAngles(1.0, 5.0, 15.0, 1.0)
        state = classify(angles)
        assert state.label == "forward_head"

    def test_both_bad(self):
        angles = PostureAngles(1.0, 20.0, 15.0, 1.0)
        state = classify(angles)
        assert state.label == "slouch_and_forward"

    def test_invalid_angles_are_good(self):
        angles = PostureAngles(1.0, 30.0, 20.0, 0.0)  # confidence=0 → invalid
        state = classify(angles)
        assert state.label == "good"

    def test_severity_scales(self):
        mild = classify(PostureAngles(1.0, 15.0, 0.0, 1.0))
        severe = classify(PostureAngles(1.0, 40.0, 0.0, 1.0))
        assert severe.torso_severity > mild.torso_severity


class TestPolicy:
    def setup_method(self):
        self.config = PolicyConfig(
            sustain_seconds=3.0,
            cooldown_seconds=10.0,
            escalation_seconds=60.0,
        )

    def test_good_posture_never_triggers(self):
        state = PolicyState()
        good = classify(PostureAngles(1.0, 5.0, 3.0, 1.0))
        for t in range(100):
            assert should_trigger(state, good, float(t), self.config) is False

    def test_bad_posture_triggers_after_sustain(self):
        state = PolicyState()
        bad = classify(PostureAngles(1.0, 25.0, 3.0, 1.0))
        # t=0~2: 아직 sustain 미충족
        for t in range(3):
            assert should_trigger(state, bad, float(t), self.config) is False
        # t=3: sustain 달성 → 트리거
        assert should_trigger(state, bad, 3.0, self.config) is True

    def test_cooldown_prevents_retrigger(self):
        state = PolicyState()
        bad = classify(PostureAngles(1.0, 25.0, 3.0, 1.0))
        # 첫 트리거
        for t in range(4):
            should_trigger(state, bad, float(t), self.config)
        # 쿨다운 중 (10초 이내)
        for t in range(5, 13):
            assert should_trigger(state, bad, float(t), self.config) is False

    def test_good_posture_resets_timer(self):
        state = PolicyState()
        bad = classify(PostureAngles(1.0, 25.0, 3.0, 1.0))
        good = classify(PostureAngles(1.0, 5.0, 3.0, 1.0))
        should_trigger(state, bad, 0.0, self.config)
        should_trigger(state, bad, 1.0, self.config)
        # 자세 바로잡음
        should_trigger(state, good, 2.0, self.config)
        # 다시 나빠짐 — 타이머 리셋이므로 3초 더 필요
        should_trigger(state, bad, 3.0, self.config)
        should_trigger(state, bad, 4.0, self.config)
        should_trigger(state, bad, 5.0, self.config)
        assert should_trigger(state, bad, 6.0, self.config) is True

    def test_urgency_escalates(self):
        state = PolicyState()
        state.correction_count = 1
        assert urgency_level(state, self.config) == 1
        state.correction_count = 2
        assert urgency_level(state, self.config) == 2
        state.correction_count = 4
        assert urgency_level(state, self.config) == 3
