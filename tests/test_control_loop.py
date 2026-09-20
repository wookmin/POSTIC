"""실험 조건이 실제 모터 출력까지 전달되는지 테스트한다."""

from src.main import ControlLoop
from src.perception.posture_features import PostureAngles


class FakeMapper:
    joints = {"joint": {"min_position": 0, "max_position": 100}}

    def neutral_targets(self):
        return {"joint": 50}

    def to_targets(self, angles):
        raise AssertionError("voice 조건은 자세를 모터 목표로 변환하면 안 됨")


class FakeGate:
    def limit_targets(self, current, targets):
        return dict(targets)


class WriterSpy:
    def __getattr__(self, name):
        def fail(*args, **kwargs):
            raise AssertionError(f"voice 조건에서 writer.{name} 호출됨")
        return fail


class OneSampleBuffer:
    def __init__(self):
        self.loop = None

    def sample(self, when):
        self.loop.stop_event.set()
        return PostureAngles(when, 20.0, 10.0, 1.0)


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
