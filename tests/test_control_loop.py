"""실험 조건이 실제 모터 출력까지 전달되는지 테스트한다."""

from src.main import ControlLoop
from src.perception.posture_features import PostureAngles


class FakeMapper:
    joints = {"joint": {"min_position": 0, "max_position": 100}}

    def neutral_targets(self):
        return {"joint": 50}

    def to_targets(self, angles):
        return {"joint": 60}


class FakeGate:
    def limit_targets(self, current, targets):
        return dict(targets)


class WriterSpy:
    def __getattr__(self, name):
        def fail(*args, **kwargs):
            raise AssertionError(f"voice 조건에서 writer.{name} 호출됨")
        return fail


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

    def sample(self, when):
        self.loop.stop_event.set()
        return PostureAngles(when, 20.0, 10.0, 1.0)

    def latest(self):
        self.loop.stop_event.set()
        return PostureAngles(1.0, 20.0, 10.0, 1.0)


def test_voice_condition_never_touches_motor_writer():
    buffer = OneSampleBuffer()
    loop = ControlLoop(
        buffer=buffer,
        mapper=FakeMapper(),
        writer=WriterSpy(),
        config={
            "echo": {"delay_sec": 0.8, "control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 2.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.5,
            },
        },
        safety_gate=FakeGate(),
        condition="voice",
    )
    buffer.loop = loop

    loop._loop()

    assert loop.motion_enabled is False


class NeutralOnlyBuffer:
    def __init__(self):
        self.loop = None

    def sample(self, when):
        self.loop.stop_event.set()
        return PostureAngles(when, 20.0, 10.0, 1.0)

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
            "echo": {"delay_sec": 0.8, "control_hz": 1000},
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
            "echo": {"delay_sec": 0.8, "control_hz": 1000},
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


def test_legacy_mirror_writes_neutral_when_person_returns():
    class MirrorBuffer:
        def __init__(self):
            self.loop = None
            self.calls = 0

        def sample(self, when):
            self.calls += 1
            if self.calls == 1:
                return PostureAngles(when, 20.0, 10.0, 1.0)
            self.loop.stop_event.set()
            return None

        def latest(self):
            return None

    buffer = MirrorBuffer()
    writer = WriterRecorder()
    loop = ControlLoop(
        buffer=buffer,
        mapper=FakeMapper(),
        writer=writer,
        config={
            "echo": {"delay_sec": 0.0, "control_hz": 1000},
            "motion": {
                "return_to_neutral_sec": 0.0,
                "idle_release_sec": 3.0,
                "person_lost_grace_sec": 0.0,
            },
        },
        safety_gate=FakeGate(),
        condition="mirror",
    )
    buffer.loop = loop

    loop._loop()

    writes = [call for call in writer.calls
              if isinstance(call, tuple) and call[0] == "write"]
    assert writes[0][1] == {"joint": 60}
    assert writes[1][1] == {"joint": 50}
