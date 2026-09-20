"""관절별 토크 상태와 부분 통신 실패 테스트."""

import pytest

from src.robot import joint_writer as module


class FakeSyncWrite:
    def __init__(self, *args):
        pass


class FakePacket:
    def __init__(self, failing_ids=()):
        self.failing_ids = set(failing_ids)
        self.writes = []

    def write1ByteTxRx(self, port, motor_id, addr, value):
        self.writes.append((motor_id, addr, value))
        if motor_id in self.failing_ids:
            return 1, 0
        return module.COMM_SUCCESS, 0

    def getTxRxResult(self, code):
        return f"comm-{code}"

    def getRxPacketError(self, code):
        return f"error-{code}"

    def read4ByteTxRx(self, port, motor_id, addr):
        if motor_id in self.failing_ids:
            return 0, 1, 0
        return 50, module.COMM_SUCCESS, 0


def make_writer(monkeypatch, failing_ids=()):
    monkeypatch.setattr(module, "GroupSyncWrite", FakeSyncWrite)
    packet = FakePacket(failing_ids)
    writer = module.JointWriter(packet, object(), {"a": 1, "b": 2})
    return writer, packet


def test_partial_torque_enable_keeps_failed_joint_unknown(monkeypatch):
    writer, packet = make_writer(monkeypatch, failing_ids={2})

    with pytest.raises(module.WriterError):
        writer.set_torque(True)

    assert writer.torque_states == {"a": True, "b": None}
    assert writer.torque_on is False
    assert [item[0] for item in packet.writes] == [1, 2]


def test_torque_off_retries_unknown_and_does_not_skip_known_joint(monkeypatch):
    writer, packet = make_writer(monkeypatch)
    writer.set_torque(True)
    packet.writes.clear()

    writer.set_torque(False)

    assert writer.torque_states == {"a": False, "b": False}
    assert [item[0] for item in packet.writes] == [1, 2]


def test_read_positions_rejects_partial_result(monkeypatch):
    writer, _ = make_writer(monkeypatch, failing_ids={2})

    with pytest.raises(module.WriterError, match="현재 위치를 읽지 못한 관절"):
        writer.read_positions()
