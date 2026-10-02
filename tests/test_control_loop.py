"""실험 조건이 실제 모터 출력까지 전달되는지 테스트한다."""

from types import SimpleNamespace

from src.main import ControlLoop
from src.perception.posture_features import PostureAngles


class FakeMapper:
    joints = {"joint": {"min_position": 0, "max_position": 100}}

    def neutral_targets(self):
        return {"joint": 50}

    def to_targets(self, angles):
        return {"joint": 60}


class GimbalMapper:
    joints = {
        "spine": {
            "min_position": 0,
            "max_position": 1000,
            "direction": 1,
        },
        "neck": {
            "min_position": 0,
            "max_position": 1000,
            "direction": 1,
        },
    }
    neck_joint = "neck"

    def neutral_targets(self):
        return {"spine": 100, "neck": 500}

    def to_targets(self, angles):
        return {"spine": 200, "neck": 600}


class FakeGate:
    def limit_targets(self, current, targets):
        return dict(targets)


class WriterRecorder:
    def __init__(self):
        self.calls = []

    def prepare(self):
        self.calls.append("prepare")

    def read_positions(self):
        self.calls.append("read_positions")
        return {"joint": 50}

    def set_torque(self, enabled):
        self.calls.append(("torque", enabled))

    def write_targets(self, targets):
        self.calls.append(("write", dict(targets)))


class OneSampleBuffer:
    def __init__(self):
        self.loop = None

    def latest(self):
        self.loop.stop_event.set()
        return PostureAngles(1.0, 20.0, 10.0, 1.0)


class NeutralOnlyBuffer:
    def __init__(self):
        self.loop = None

    def latest(self):
        self.loop.stop_event.set()
        return PostureAngles(1.0, 20.0, 10.0, 1.0)


def test_posture_trigger_keeps_robot_neutral_without_event():
    buffer = NeutralOnlyBuffer()
    loop = ControlLoop(
        buffer=buffer,
        mapper=FakeMapper(),
        writer=None,
        config={
            "echo": {"control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 2.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.5,
            },
        },
        safety_gate=FakeGate(),
        condition="posture_trigger",
    )
    buffer.loop = loop

    loop._loop()

    assert loop.motion_enabled is True
    assert loop.last_targets == {"joint": 50}


def test_posture_trigger_holds_rest_pose_without_event():
    buffer = NeutralOnlyBuffer()
    writer = WriterRecorder()
    loop = ControlLoop(
        buffer=buffer,
        mapper=FakeMapper(),
        writer=writer,
        config={
            "echo": {"control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 2.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.5,
            },
        },
        safety_gate=FakeGate(),
        condition="posture_trigger",
    )
    buffer.loop = loop

    loop._loop()

    assert ("torque", True) in writer.calls
    assert any(call[0] == "write" for call in writer.calls
               if isinstance(call, tuple))


def test_camera_gimbal_cancels_body_rotation_at_neck():
    loop = ControlLoop(
        buffer=None,
        mapper=GimbalMapper(),
        writer=None,
        config={
            "echo": {"control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 2.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.5,
            },
            "camera_gimbal": {"enabled": True, "joint": "neck"},
        },
        safety_gate=FakeGate(),
    )

    targets = loop._targets_for_pose(PostureAngles(1.0, 20.0, 10.0, 1.0))

    assert targets["spine"] == 200
    assert targets["neck"] == 400


def test_posture_trigger_stays_neutral_after_behavior_release():
    loop = ControlLoop(
        buffer=None,
        mapper=FakeMapper(),
        writer=None,
        config={
            "echo": {"control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 2.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.5,
            },
        },
        safety_gate=FakeGate(),
        condition="posture_trigger",
    )
    action = SimpleNamespace(
        pose=PostureAngles(1.0, 8.0, 2.0, 1.0),
        duration_sec=0.0,
    )
    loop.submit_behavior(action)
    started = loop._behavior_started_at
    loop.commanded = {"joint": 60}

    desired, torque = loop._posture_trigger_output(
        started + 1.0, True, "tracking")
    assert desired == {"joint": 50}
    assert torque is True

    loop.commanded = {"joint": 50}
    desired, torque = loop._posture_trigger_output(
        started + 4.0, True, "tracking")
    assert desired == {"joint": 50}
    assert torque is False

    desired, torque = loop._posture_trigger_output(
        started + 4.1, True, "tracking")
    assert desired == {"joint": 50}
    assert torque is False


def test_posture_trigger_holds_pose_until_good_posture():
    loop = ControlLoop(
        buffer=None,
        mapper=FakeMapper(),
        writer=None,
        config={
            "echo": {"control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 2.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.5,
            },
        },
        safety_gate=FakeGate(),
        condition="posture_trigger",
    )
    action = SimpleNamespace(
        pose=PostureAngles(1.0, 30.0, 25.0, 1.0),
        duration_sec=0.0,
        hold_until_good=True,
    )
    loop.submit_behavior(action)
    started = loop._behavior_started_at

    # duration_sec가 지나도 나쁜 자세가 계속되면 포즈를 유지한다.
    desired, torque = loop._posture_trigger_output(
        started + 10.0, True, "tracking", "slouch")
    assert desired == {"joint": 60}
    assert torque is True

    # 정상 자세가 관측된 순간부터 중립 복귀를 시작한다.
    loop.commanded = {"joint": 60}
    desired, torque = loop._posture_trigger_output(
        started + 11.0, True, "tracking", "good")
    assert desired == {"joint": 50}
    assert torque is True
