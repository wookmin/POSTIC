"""새 6축 pitch/yaw/roll 매핑 테스트."""

from src.perception.posture_features import PostureAngles
from src.robot.joint_mapper import JointMapper, TICKS_PER_DEG


def joints():
    return {
        "waist_lower_pitch": {
            "id": 3, "group": "torso", "input_axis": "torso_pitch",
            "direction": 1, "zero_position": 3042,
            "min_position": 0, "max_position": 4095,
            "relative_min_deg": -12, "relative_max_deg": 12,
            "wrap_position": True,
        },
        "waist_upper_pitch": {
            "id": 5, "group": "torso", "input_axis": "torso_pitch",
            "direction": -1, "zero_position": 59,
            "min_position": 0, "max_position": 4095,
            "relative_min_deg": -12, "relative_max_deg": 12,
            "wrap_position": True,
        },
        "neck_roll": {
            "id": 6, "group": "neck", "role": "neck",
            "input_axis": "neck_roll", "direction": 1,
            "zero_position": 4087, "min_position": 0, "max_position": 4095,
            "relative_min_deg": -12, "relative_max_deg": 12,
            "wrap_position": True,
        },
    }


DISTRIBUTION = {
    "torso_pitch": {
        "waist_lower_pitch": 0.5,
        "waist_upper_pitch": 0.5,
    },
    "neck_roll": {"neck_roll": 1.0},
}


def test_multi_axis_zero_pose_keeps_measured_positions():
    mapper = JointMapper(joints(), DISTRIBUTION)
    pose = PostureAngles(0.0, 0.0, 0.0, 1.0)
    assert mapper.to_targets(pose) == {
        "waist_lower_pitch": 3042,
        "waist_upper_pitch": 59,
        "neck_roll": 4087,
    }


def test_multi_axis_distribution_and_direction_are_applied():
    mapper = JointMapper(joints(), DISTRIBUTION)
    pose = PostureAngles(0.0, 12.0, 0.0, 1.0, neck_roll_deg=6.0)
    targets = mapper.to_targets(pose)
    assert targets["waist_lower_pitch"] == round(3042 + 6 * TICKS_PER_DEG)
    assert targets["waist_upper_pitch"] == round(59 - 6 * TICKS_PER_DEG) % 4096
    assert targets["neck_roll"] == round(4087 + 6 * TICKS_PER_DEG) % 4096


def test_wrap_position_uses_shortest_delta():
    mapper = JointMapper(joints(), DISTRIBUTION)
    assert mapper.position_delta("neck_roll", 4090, 5) == 11
    assert mapper.position_delta("neck_roll", 5, 4090) == -11
