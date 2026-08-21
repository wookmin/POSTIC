"""관절 매핑과 안전 계층 테스트. 하드웨어 없이 돈다."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.perception.posture_features import PostureAngles  # noqa: E402
from src.robot.joint_mapper import (  # noqa: E402
    TICKS_PER_DEG, JointMapper, MappingError,
)
from src.safety.supervisor import (  # noqa: E402
    STATE_IDLE, STATE_RETURNING, STATE_TRACKING, IdlePolicy, SlewLimiter,
    max_step_ticks,
)


def joints():
    return {
        "base_pitch": {"id": 1, "role": "load", "direction": 1,
                       "zero_position": 2048, "min_position": 1877,
                       "max_position": 2219},
        "waist_pitch": {"id": 4, "role": "load", "direction": 1,
                        "zero_position": 2048, "min_position": 1820,
                        "max_position": 2276},
        "spine_lower_pitch": {"id": 5, "role": "spine", "direction": 1,
                              "zero_position": 2048, "min_position": 1820,
                              "max_position": 2276},
        "spine_upper_pitch": {"id": 8, "role": "spine", "direction": 1,
                              "zero_position": 2048, "min_position": 1820,
                              "max_position": 2276},
        "neck_pitch": {"id": 9, "role": "neck", "direction": 1,
                       "zero_position": 3018, "min_position": 2620,
                       "max_position": 3416},
    }


DISTRIBUTION = {
    "base_pitch": 0.15,
    "waist_pitch": 0.25,
    "spine_lower_pitch": 0.30,
    "spine_upper_pitch": 0.30,
}


class TestJointMapperValidation:
    def test_rejects_empty_joints(self):
        with pytest.raises(MappingError):
            JointMapper({}, DISTRIBUTION)

    def test_rejects_weights_not_summing_to_one(self):
        broken = dict(DISTRIBUTION, base_pitch=0.5)
        with pytest.raises(MappingError, match="분배 비율 합"):
            JointMapper(joints(), broken)

    def test_rejects_unknown_joint_in_distribution(self):
        broken = dict(DISTRIBUTION)
        broken.pop("base_pitch")
        broken["nope"] = 0.15
        with pytest.raises(MappingError, match="분배 비율에 없는 관절"):
            JointMapper(joints(), broken)

    def test_rejects_missing_torso_joint(self):
        broken = dict(DISTRIBUTION)
        broken.pop("base_pitch")
        broken["waist_pitch"] = 0.40
        with pytest.raises(MappingError, match="빠진 관절"):
            JointMapper(joints(), broken)

    def test_requires_exactly_one_neck(self):
        two = joints()
        two["waist_pitch"]["role"] = "neck"
        with pytest.raises(MappingError, match="목 관절"):
            JointMapper(two, DISTRIBUTION)

    def test_rejects_missing_limits(self):
        broken = joints()
        broken["base_pitch"]["min_position"] = None
        with pytest.raises(MappingError, match="min_position"):
            JointMapper(broken, DISTRIBUTION)

    def test_rejects_zero_outside_range(self):
        broken = joints()
        broken["base_pitch"]["zero_position"] = 3000
        with pytest.raises(MappingError, match="zero_position"):
            JointMapper(broken, DISTRIBUTION)


class TestJointMapping:
    def setup_method(self):
        self.mapper = JointMapper(joints(), DISTRIBUTION)

    def test_neutral_pose_maps_to_zero_positions(self):
        angles = PostureAngles(0.0, 0.0, 0.0, 1.0)
        targets = self.mapper.to_targets(angles)
        assert targets["base_pitch"] == 2048
        assert targets["neck_pitch"] == 3018

    def test_torso_angle_is_split_by_weight(self):
        angles = PostureAngles(0.0, 20.0, 0.0, 1.0)
        degrees = self.mapper.degrees_for(angles)
        assert degrees["base_pitch"] == pytest.approx(3.0)
        assert degrees["waist_pitch"] == pytest.approx(5.0)
        assert degrees["spine_upper_pitch"] == pytest.approx(6.0)
        assert sum(degrees[n] for n in DISTRIBUTION) == pytest.approx(20.0)

    def test_base_receives_the_smallest_share(self):
        angles = PostureAngles(0.0, 40.0, 0.0, 1.0)
        degrees = self.mapper.degrees_for(angles)
        assert degrees["base_pitch"] == min(degrees[n] for n in DISTRIBUTION)

    def test_neck_angle_goes_to_neck_joint_only(self):
        angles = PostureAngles(0.0, 0.0, 10.0, 1.0)
        degrees = self.mapper.degrees_for(angles)
        assert degrees["neck_pitch"] == pytest.approx(10.0)
        assert all(degrees[n] == 0.0 for n in DISTRIBUTION)

    def test_ticks_follow_degrees(self):
        angles = PostureAngles(0.0, 20.0, 0.0, 1.0)
        targets = self.mapper.to_targets(angles)
        expected = 2048 + 3.0 * TICKS_PER_DEG
        assert targets["base_pitch"] == pytest.approx(round(expected), abs=1)

    def test_direction_inverts_sign(self):
        flipped = joints()
        flipped["base_pitch"]["direction"] = -1
        mapper = JointMapper(flipped, DISTRIBUTION)
        angles = PostureAngles(0.0, 20.0, 0.0, 1.0)
        assert mapper.to_targets(angles)["base_pitch"] < 2048

    def test_extreme_angle_is_clamped_to_operational_limits(self):
        angles = PostureAngles(0.0, 200.0, 200.0, 1.0)
        targets = self.mapper.to_targets(angles)
        for name, spec in joints().items():
            assert spec["min_position"] <= targets[name] <= spec["max_position"]

    def test_negative_extreme_is_clamped_too(self):
        angles = PostureAngles(0.0, -200.0, -200.0, 1.0)
        targets = self.mapper.to_targets(angles)
        for name, spec in joints().items():
            assert spec["min_position"] <= targets[name] <= spec["max_position"]


class TestSlewLimiter:
    def test_small_moves_pass_through(self):
        limiter = SlewLimiter(max_step_ticks=20)
        assert limiter.apply({"a": 100}, {"a": 110}) == {"a": 110}

    def test_large_move_is_capped(self):
        limiter = SlewLimiter(max_step_ticks=20)
        assert limiter.apply({"a": 100}, {"a": 500}) == {"a": 120}

    def test_negative_move_is_capped(self):
        limiter = SlewLimiter(max_step_ticks=20)
        assert limiter.apply({"a": 100}, {"a": -500}) == {"a": 80}

    def test_unknown_joint_passes_through(self):
        limiter = SlewLimiter(max_step_ticks=20)
        assert limiter.apply({}, {"a": 500}) == {"a": 500}

    def test_rejects_non_positive_step(self):
        with pytest.raises(ValueError):
            SlewLimiter(max_step_ticks=0).apply({"a": 1}, {"a": 2})

    def test_degree_conversion(self):
        assert max_step_ticks(1.5) == pytest.approx(1.5 * TICKS_PER_DEG)


class TestIdlePolicy:
    """복귀와 토크 해제 순서만 본다. 유예는 TestIdleGracePeriod 가 따로 다룬다."""

    def setup_method(self):
        self.neutral = {"a": 100, "b": 200}
        self.policy = IdlePolicy(return_seconds=2.0, release_seconds=1.0,
                                 grace_seconds=0.0)

    def test_visible_person_keeps_tracking(self):
        state, torque = self.policy.update(0.0, True, {"a": 500}, self.neutral)
        assert state == STATE_TRACKING and torque is True

    def test_person_leaves_starts_returning(self):
        self.policy.update(0.0, True, {"a": 500, "b": 500}, self.neutral)
        state, torque = self.policy.update(1.0, False, {"a": 500, "b": 500},
                                           self.neutral)
        assert state == STATE_RETURNING and torque is True

    def test_reaching_neutral_then_waiting_releases_torque(self):
        self.policy.update(0.0, True, self.neutral, self.neutral)
        self.policy.update(1.0, False, self.neutral, self.neutral)
        state, torque = self.policy.update(1.5, False, self.neutral, self.neutral)
        assert state == STATE_RETURNING and torque is True
        state, torque = self.policy.update(2.2, False, self.neutral, self.neutral)
        assert state == STATE_IDLE and torque is False

    def test_return_times_out_even_if_neutral_never_reached(self):
        stuck = {"a": 900, "b": 900}
        self.policy.update(0.0, True, stuck, self.neutral)
        self.policy.update(1.0, False, stuck, self.neutral)
        state, _ = self.policy.update(2.0, False, stuck, self.neutral)
        assert state == STATE_RETURNING
        self.policy.update(3.0, False, stuck, self.neutral)
        state, torque = self.policy.update(4.1, False, stuck, self.neutral)
        assert state == STATE_IDLE and torque is False

    def test_person_returning_resumes_tracking_from_idle(self):
        self.policy.update(0.0, False, self.neutral, self.neutral)
        self.policy.update(5.0, False, self.neutral, self.neutral)
        state, torque = self.policy.update(6.0, True, self.neutral, self.neutral)
        assert state == STATE_TRACKING and torque is True


class TestIdleGracePeriod:
    def setup_method(self):
        self.neutral = {"a": 100}
        self.policy = IdlePolicy(return_seconds=2.0, release_seconds=1.0,
                                 grace_seconds=0.5)

    def test_brief_dropout_keeps_tracking(self):
        # 인식이 몇 프레임 끊긴다고 중립으로 출발하면 로봇이 덜컥거린다.
        self.policy.update(0.0, True, {"a": 500}, self.neutral)
        state, torque = self.policy.update(0.2, False, {"a": 500}, self.neutral)
        assert state == STATE_TRACKING and torque is True

    def test_sustained_absence_starts_returning(self):
        self.policy.update(0.0, True, {"a": 500}, self.neutral)
        self.policy.update(0.2, False, {"a": 500}, self.neutral)
        state, _ = self.policy.update(0.8, False, {"a": 500}, self.neutral)
        assert state == STATE_RETURNING

    def test_reappearing_during_grace_resets_it(self):
        self.policy.update(0.0, True, {"a": 500}, self.neutral)
        self.policy.update(0.3, False, {"a": 500}, self.neutral)
        self.policy.update(0.4, True, {"a": 500}, self.neutral)
        # 유예가 초기화되었으므로 0.6초 뒤에도 아직 tracking 이어야 한다.
        state, _ = self.policy.update(0.6, False, {"a": 500}, self.neutral)
        assert state == STATE_TRACKING
