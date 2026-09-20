"""자세 분류기와 교정 정책 테스트."""

import pytest

from src.perception.posture_features import PostureAngles
from src.posture.classifier import classify
from src.posture.policy import PolicyConfig, PolicyState, should_trigger, urgency_level
from src.behavior.behavior_manager import BehaviorManager
from src.behavior.executor import BehaviorExecutor


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

    def test_lateral_tilt_does_not_become_slouch_trigger(self):
        angles = PostureAngles(1.0, 5.0, 3.0, 1.0,
                               lateral_tilt_deg=20.0)
        state = classify(angles)
        assert state.label == "lateral_tilt"
        assert state.is_bad is True
        assert state.is_triggerable_bad is True

    def test_invalid_angles_are_unknown(self):
        angles = PostureAngles(1.0, 30.0, 20.0, 0.0)  # confidence=0 → invalid
        state = classify(angles)
        assert state.label == "unknown"
        assert state.is_bad is False

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

        # 쿨다운이 지나도 같은 나쁜 자세 구간에서는 다시 호출하지 않는다.
        assert should_trigger(state, bad, 20.0, self.config) is False
        assert state.correction_count == 1

    def test_bad_posture_rearms_only_after_good_posture(self):
        state = PolicyState()
        bad = classify(PostureAngles(1.0, 25.0, 3.0, 1.0))
        good = classify(PostureAngles(1.0, 5.0, 3.0, 1.0))
        for t in range(4):
            should_trigger(state, bad, float(t), self.config)
        assert state.armed is False
        should_trigger(state, good, 10.0, self.config)
        assert state.armed is True
        should_trigger(state, bad, 11.0, self.config)
        should_trigger(state, bad, 12.0, self.config)
        should_trigger(state, bad, 13.0, self.config)
        assert should_trigger(state, bad, 14.0, self.config) is True

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

    def test_unknown_does_not_count_as_bad_or_good(self):
        state = PolicyState()
        bad = classify(PostureAngles(1.0, 25.0, 3.0, 1.0))
        unknown = classify(PostureAngles(1.0, 25.0, 3.0, 0.0))
        should_trigger(state, bad, 0.0, self.config)
        should_trigger(state, bad, 1.0, self.config)
        assert should_trigger(state, unknown, 10.0, self.config) is False
        assert state.bad_since is None
        assert should_trigger(state, bad, 11.0, self.config) is False

    def test_lateral_tilt_triggers_generic_correction(self):
        state = PolicyState()
        lateral = classify(PostureAngles(1.0, 5.0, 3.0, 1.0,
                                         lateral_tilt_deg=20.0))

        for timestamp in range(3):
            assert should_trigger(state, lateral, float(timestamp),
                                  self.config) is False
        assert should_trigger(state, lateral, 3.0, self.config) is True

    def test_pitch_problem_takes_priority_over_lateral_label(self):
        state = classify(PostureAngles(1.0, 25.0, 15.0, 1.0,
                                       lateral_tilt_deg=20.0))

        assert state.label == "slouch_and_forward"

    def test_skipped_event_can_be_rearmed(self):
        manager = BehaviorManager({
            "experiment": {"condition": "posture_trigger"},
        })
        manager._policy_state.armed = False
        manager._policy_state.last_correction_at = 10.0
        manager._policy_state.correction_count = 1
        manager.update_posture(PostureAngles(1.0, 25.0, 3.0, 1.0))

        manager.rearm_after_skipped_event()

        assert manager._policy_state.armed is True
        assert manager._policy_state.last_correction_at == -9999.0
        assert manager._policy_state.correction_count == 0
        assert manager._policy_state.bad_since is not None

    def test_urgency_escalates(self):
        state = PolicyState()
        state.correction_count = 1
        assert urgency_level(state, self.config) == 1
        state.correction_count = 2
        assert urgency_level(state, self.config) == 2
        state.correction_count = 4
        assert urgency_level(state, self.config) == 3


class FakeGate:
    def clamp_pose(self, pose):
        return pose

    def clamp_duration(self, duration):
        return min(1.5, duration)


class TestBehaviorExecutor:
    def event(self, action="gentle_remind", behavior="mimic_slouch",
              angles=None, posture_label="slouch"):
        decision = type("Decision", (), {
            "action": action,
            "behavior": behavior,
            "speech": "등을 펴보세요.",
        })()
        return type("Event", (), {
            "decision": decision,
            "observed_angles": angles,
            "posture_label": posture_label,
        })()

    def test_mirror_without_observation_does_not_guess_pose(self):
        executor = BehaviorExecutor(
            {"experiment": {"condition": "mirror"}}, FakeGate())
        action = executor.build(self.event())
        assert action.behavior == "mirror"
        assert action.pose is None

    def test_ignore_does_not_create_action(self):
        executor = BehaviorExecutor({}, FakeGate())
        assert executor.build(self.event(action="ignore")) is None

    def test_mirror_uses_observed_posture(self):
        observed = PostureAngles(1.0, 21.0, 13.0, 1.0)
        executor = BehaviorExecutor({"experiment": {"condition": "mirror"}},
                                    FakeGate())
        action = executor.build(self.event(angles=observed))
        assert action.behavior == "mirror"
        assert action.pose.torso_pitch_deg == 21.0
        assert action.pose.neck_pitch_deg == 13.0
        assert action.speech == ""

    def test_posture_trigger_uses_fixed_pose_not_observed_posture(self):
        observed = PostureAngles(1.0, 21.0, 13.0, 1.0)
        executor = BehaviorExecutor({
            "experiment": {"condition": "posture_trigger"},
            "intervention": {
                "action_duration_sec": 1.0,
                "poses": {
                    "slouch": {
                        "torso_pitch_deg": 8.0,
                        "neck_pitch_deg": 2.0,
                    },
                },
            },
        }, FakeGate())
        action = executor.build(self.event(angles=observed,
                                           posture_label="slouch"))
        assert action.behavior == "slouch"
        assert action.pose.torso_pitch_deg == 8.0
        assert action.pose.neck_pitch_deg == 2.0
        assert action.pose.torso_pitch_deg != observed.torso_pitch_deg
        assert action.speech == ""

    def test_posture_trigger_uses_one_generic_pose_for_lateral_bad_posture(self):
        executor = BehaviorExecutor({
            "experiment": {"condition": "posture_trigger"},
            "intervention": {
                "poses": {
                    "bad_posture": {
                        "torso_pitch_deg": 14.0,
                        "neck_pitch_deg": 14.0,
                    },
                },
            },
        }, FakeGate())

        action = executor.build(self.event(posture_label="lateral_tilt"))

        assert action.behavior == "bad_posture"
        assert action.pose.torso_pitch_deg == 14.0
        assert action.pose.neck_pitch_deg == 14.0

    def test_posture_trigger_ignores_unknown_label(self):
        executor = BehaviorExecutor(
            {"experiment": {"condition": "posture_trigger"}}, FakeGate())
        assert executor.build(self.event(posture_label="unknown")) is None

    def test_voice_condition_has_no_motion_override(self):
        executor = BehaviorExecutor({"experiment": {"condition": "voice"}},
                                    FakeGate())
        action = executor.build(self.event())
        assert action.behavior == "voice"
        assert action.pose is None
        assert action.speech == "등을 펴보세요."

    def test_unsupported_condition_is_rejected(self):
        try:
            BehaviorExecutor({"experiment": {"condition": "display"}},
                             FakeGate())
        except ValueError as exc:
            assert "지원하지 않는 개입 조건" in str(exc)
        else:
            raise AssertionError("지원하지 않는 조건이 허용됨")
