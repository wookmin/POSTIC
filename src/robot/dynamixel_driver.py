"""Dynamixel 버스 접속과 설정 로딩.

포트/baudrate/ID 를 스크립트마다 하드코딩하지 않고 config/robot.yaml 한 곳에서
읽는다. 예전 스크립트들이 57600 과 ID [1,2] 를 각자 박아두고 있어서 버스 설정이
바뀔 때마다 조용히 어긋났는데, 그걸 막기 위한 모듈이다.
"""

from contextlib import contextmanager
from pathlib import Path

import yaml

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError as exc:
    raise ImportError(
        "dynamixel_sdk 를 찾을 수 없습니다. "
        "source /opt/ros/humble/setup.bash 후 다시 실행하세요."
    ) from exc

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOT_CONFIG = PROJECT_ROOT / "config" / "robot.yaml"
JOINTS_CONFIG = PROJECT_ROOT / "config" / "joints.yaml"


class BusError(RuntimeError):
    pass


def load_robot_config(path=ROBOT_CONFIG):
    if not Path(path).exists():
        raise BusError(f"설정 파일이 없습니다: {path}")
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    bus = config.get("bus") or {}
    for key in ("port", "baudrate", "protocol"):
        if key not in bus:
            raise BusError(f"{path}: bus.{key} 가 없습니다")
    return config


def load_joints(path=JOINTS_CONFIG):
    """관절 이름 -> 설정 딕셔너리. 아직 매핑이 없으면 빈 dict."""
    if not Path(path).exists():
        return {}
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return config.get("joints") or {}


def configured_ids(config=None):
    config = config or load_robot_config()
    return list((config.get("motors") or {}).get("ids") or [])


@contextmanager
def open_bus(config=None, port=None, baudrate=None):
    """포트를 열고 (packet, port) 를 넘긴다. 빠져나올 때 항상 닫는다."""
    config = config or load_robot_config()
    bus = config["bus"]
    port_name = port or bus["port"]
    baud = baudrate or bus["baudrate"]

    handler = PortHandler(port_name)
    packet = PacketHandler(float(bus["protocol"]))

    if not handler.openPort():
        raise BusError(f"{port_name} 포트를 열 수 없습니다. "
                       "U2D2 연결과 dialout 권한을 확인하세요.")
    try:
        if not handler.setBaudRate(baud):
            raise BusError(f"baudrate {baud} 설정 실패")
        yield packet, handler
    finally:
        handler.closePort()


def ping_all(packet, port):
    """buscast ping. {id: [model_number, firmware]} 를 돌려준다."""
    found, _ = packet.broadcastPing(port)
    return found


def check(comm, err, packet, what):
    if comm != COMM_SUCCESS:
        raise BusError(f"{what}: 통신 실패 - {packet.getTxRxResult(comm)}")
    if err:
        raise BusError(f"{what}: 모터 오류 - {packet.getRxPacketError(err)}")
