"""로봇 탑재 카메라 모드: 로봇 움직임 중 판정 중지와 재확보를 테스트한다."""

from types import SimpleNamespace

import pytest

from src.behavior.behavior_manager import BehaviorManager
from src.camera.camera_stream import CameraError, CameraStream
from src.main import (
    PHASE_INTERVENING, PHASE_OBSERVING, PHASE_RETURNING, PHASE_SETTLING,
    ControlLoop,
)
from src.perception.posture_features import PostureAngles
from src.perception.robot_camera import ReacquireGate


class FakeMapper:
    joints = {"joint": {"min_position": 0, "max_position": 100}}

    def neutral_targets(self):
        return {"joint": 50}

    def to_targets(self, angles):
        return {"joint": 60}


class FakeGate:
    def limit_targets(self, current, targets):
        return dict(targets)


def make_loop(camera_mount="robot", **robot_camera):
    config = {
        "echo": {"delay_sec": 0.0, "control_hz": 1000},
        "motion": {
            "return_to_neutral_sec": 2.0,
            "idle_release_sec": 3.0,
            "person_lost_grace_sec": 0.5,
        },
        "perception": {"camera_mount": camera_mount},
        "robot_camera": {
            "settle_sec": 1.0,
            "intervention_hold_sec": 4.0,
            **robot_camera,
        },
    }
    loop = ControlLoop(buffer=None, mapper=FakeMapper(), writer=None,
                       config=config, safety_gate=FakeGate(),
                       condition="posture_trigger")
    # 시작 자세(관측 자세)가 중립과 다른 경우를 가정한다.
    loop._rest_targets = {"joint": 40}
    loop.commanded = {"joint": 40}
    return loop


def hold_action():
    return SimpleNamespace(
        pose=PostureAngles(1.0, 60.0, 35.0, 1.0),
        duration_sec=1.5,
        hold_until_good=True,
    )


def observe(loop, now=0.0):
    """관측 자세에서 안정화 시간을 채워 OBSERVING 으로 만든다."""
    loop._update_perception_phase(now)
    loop._update_perception_phase(now + 1.0)
    assert loop.perception_phase == PHASE_OBSERVING


# --- ReacquireGate ---

def test_gate_is_blind_until_person_seen_long_enough():
    gate = ReacquireGate(0.5)
    assert gate.update(0.0, True, True) == (False, False)
    assert gate.update(0.4, True, True) == (False, False)
    assert gate.update(0.5, True, True) == (True, True)
    assert gate.update(0.6, True, True) == (True, False)


def test_gate_restarts_count_when_person_lost_during_reacquire():
    gate = ReacquireGate(0.5)
    gate.update(0.0, True, True)
    gate.update(0.3, True, False)
    assert gate.update(0.4, True, True) == (False, False)
    assert gate.update(0.8, True, True) == (False, False)
    assert gate.update(0.9, True, True) == (True, True)


def test_gate_closes_when_robot_moves_and_reacquires_after():
    gate = ReacquireGate(0.5)
    gate.update(0.0, True, True)
    gate.update(0.5, True, True)
    assert gate.update(1.0, False, True) == (False, False)
    assert gate.update(2.0, True, True) == (False, False)
    assert gate.update(2.5, True, True) == (True, True)


def test_gate_treats_person_loss_after_reacquire_as_plain_unknown():
    gate = ReacquireGate(0.0)
    assert gate.update(0.0, True, True) == (True, True)
    # 재확보 뒤 사람을 놓친 프레임도 사용 가능(unknown 으로 판정)하다.
    assert gate.update(0.1, True, False) == (True, False)


def test_gate_rejects_negative_reacquire_time():
    with pytest.raises(ValueError):
        ReacquireGate(-1.0)


# --- ControlLoop: 로봇 카메라 모드 ---

def test_fixed_camera_is_always_ready():
    loop = make_loop(camera_mount="fixed")
    assert loop.robot_camera is False
    assert loop.perception_ready is True


def test_robot_camera_waits_for_settle_before_observing():
    loop = make_loop()
    assert loop.perception_ready is False
    loop._update_perception_phase(0.0)
    assert loop.perception_phase == PHASE_SETTLING
    loop._update_perception_phase(0.9)
    assert loop.perception_phase == PHASE_SETTLING
    loop._update_perception_phase(1.0)
    assert loop.perception_ready is True


def test_robot_camera_rejects_behavior_while_not_observing():
    loop = make_loop()
    assert loop.submit_behavior(hold_action()) is False
    assert loop._behavior_action is None


def test_robot_camera_blinds_perception_as_soon_as_behavior_submitted():
    loop = make_loop()
    observe(loop)
    assert loop.submit_behavior(hold_action()) is True
    assert loop.perception_phase == PHASE_INTERVENING
    assert loop.perception_ready is False


def test_robot_camera_holds_pose_for_fixed_time_regardless_of_view():
    loop = make_loop()
    observe(loop)
    loop.submit_behavior(hold_action())
    started = loop._behavior_started_at

    # 카메라가 사용자를 놓쳐도(책상을 보고 있어도) 포즈를 취소하지 않는다.
    desired, torque = loop._robot_camera_output(started + 3.9)
    assert desired == {"joint": 60}
    assert torque is True

    # 유지 시간이 끝나면 중립이 아니라 관측 자세로 돌아간다.
    desired, torque = loop._robot_camera_output(started + 4.0)
    assert desired == {"joint": 40}
    assert torque is True


def test_robot_camera_phase_follows_return_then_settle():
    loop = make_loop()
    observe(loop)
    loop.submit_behavior(hold_action())
    started = loop._behavior_started_at

    loop.commanded = {"joint": 60}
    loop._update_perception_phase(started + 1.0)
    assert loop.perception_phase == PHASE_INTERVENING

    loop._robot_camera_output(started + 4.0)      # 유지 시간 종료
    loop.commanded = {"joint": 58}                # 복귀 중 (허용 오차 약 17틱 밖)
    loop._update_perception_phase(started + 4.1)
    assert loop.perception_phase == PHASE_RETURNING

    loop.commanded = {"joint": 40}                # 관측 자세 도달
    loop._update_perception_phase(started + 5.0)
    assert loop.perception_phase == PHASE_SETTLING
    loop._update_perception_phase(started + 6.0)
    assert loop.perception_phase == PHASE_OBSERVING


def test_robot_camera_keeps_torque_at_observe_pose():
    loop = make_loop()
    observe(loop)
    # 토크를 풀면 컬럼이 처지면서 카메라 시점이 바뀐다.
    desired, torque = loop._robot_camera_output(10.0)
    assert desired == {"joint": 40}
    assert torque is True


def test_robot_camera_rejects_invalid_hold_time():
    with pytest.raises(ValueError):
        make_loop(intervention_hold_sec=0.0)
    with pytest.raises(ValueError):
        make_loop(intervention_hold_sec=120.0)


def test_fixed_camera_still_holds_until_good():
    loop = make_loop(camera_mount="fixed")
    loop.submit_behavior(hold_action())
    started = loop._behavior_started_at
    desired, _ = loop._posture_trigger_output(
        started + 10.0, True, "tracking", "slouch")
    assert desired == {"joint": 60}


# --- BehaviorManager ---

def test_rearm_after_intervention_restarts_sustain_timer():
    manager = BehaviorManager({"correction": {}}, condition="posture_trigger")
    state = manager._policy_state
    state.armed = False
    state.bad_since = 5.0
    state.correction_count = 2

    manager.rearm_after_intervention()

    assert state.armed is True
    assert state.bad_since is None
    # 반복 횟수는 강도 계산을 위해 유지한다.
    assert state.correction_count == 2


# --- CameraStream ---

def test_camera_stream_rejects_unsupported_rotation():
    with pytest.raises(CameraError):
        CameraStream(0, rotate=45)
