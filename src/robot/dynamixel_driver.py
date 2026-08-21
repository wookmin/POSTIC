"""Dynamixel 버스 접속과 설정 로딩.

포트/baudrate/ID 를 스크립트마다 하드코딩하지 않고 config/robot.yaml 한 곳에서
읽는다. 예전 스크립트들이 57600 과 ID [1,2] 를 각자 박아두고 있어서 버스 설정이
바뀔 때마다 조용히 어긋났는데, 그걸 막기 위한 모듈이다.
"""

import signal
from contextlib import contextmanager
from pathlib import Path

import yaml

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError as exc:
    raise ImportError(
        "dynamixel_sdk 를 찾을 수 없습니다. "
        "~/dynamixel-venv/bin/python 으로 실행하거나 "
        "pip install -r requirements.txt 를 먼저 실행하세요."
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
    """버스에 물려 있는 전체 ID. 스캔 결과와 대조할 때 쓴다."""
    config = config or load_robot_config()
    return list((config.get("motors") or {}).get("ids") or [])


def active_ids(config=None):
    """실제로 구조에 연결되어 움직이는 축만. 없으면 전체로 떨어진다.

    유닛당 축 하나는 구조에 물려 있지 않다. 그 축에 토크를 거는 것은
    발열만 만들고 얻는 게 없으므로, 구동 도구는 이 목록을 기본값으로 쓴다.
    """
    config = config or load_robot_config()
    motors = config.get("motors") or {}
    return list(motors.get("active_ids") or motors.get("ids") or [])


@contextmanager
def open_bus(config=None, port=None, baudrate=None):
    """포트를 열고 (packet, port) 를 넘긴다. 빠져나올 때 항상 닫는다."""
    config = config or load_robot_config()
    bus = config["bus"]
    port_name = port or bus["port"]
    baud = baudrate or bus["baudrate"]

    handler = PortHandler(port_name)
    packet = PacketHandler(float(bus["protocol"]))

    # 장치 파일이 없으면 openPort() 는 False 를 돌려주는 대신 pyserial 예외를
    # 던진다. U2D2 를 뽑았을 때 원시 트레이스백이 노출되지 않게 감싼다.
    try:
        opened = handler.openPort()
    except Exception as exc:
        raise BusError(
            f"{port_name} 을 열 수 없습니다: {exc}\n"
            "U2D2 가 연결되어 있는지, dialout 권한이 있는지 확인하세요."
        ) from exc
    if not opened:
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


def install_signal_guards():
    """SIGTERM / SIGHUP 을 예외로 바꿔 finally 정리 코드가 반드시 돌게 한다.

    파이썬 기본 핸들러는 이 신호에서 즉시 종료하므로 토크를 켜둔 채 프로세스가
    사라진다. SSH 세션이 끊기면(SIGHUP) 모터가 계속 인가 상태로 남아 발열한다.
    """
    def handler(signum, frame):
        raise KeyboardInterrupt(f"{signal.Signals(signum).name} 수신")

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, handler)


ADDR_HARDWARE_ERROR = 70
ERROR_ALERT_BIT = 0x80

HARDWARE_ERROR_BITS = {
    0x01: "입력 전압 오류",
    0x04: "과열 오류",
    0x08: "엔코더 오류",
    0x10: "전기 충격 또는 전원 부족",
    0x20: "과부하 오류",
}


def read_hardware_error(packet, port, motor_id):
    """Hardware Error Status 를 읽는다.

    하드웨어 에러가 걸린 모터는 모든 응답 패킷에 alert 비트(0x80)를 세운다.
    일반적인 오류 검사를 그대로 적용하면 에러 상태를 읽는 것 자체가 실패해서
    원인을 알 수 없게 되므로, 여기서는 패킷 오류 필드를 무시한다.
    """
    value, comm, _ = packet.read1ByteTxRx(port, motor_id, ADDR_HARDWARE_ERROR)
    if comm != COMM_SUCCESS:
        raise BusError(f"ID {motor_id} 하드웨어 에러 확인: "
                       f"{packet.getTxRxResult(comm)}")
    return value


def describe_hardware_error(value):
    causes = [text for bit, text in HARDWARE_ERROR_BITS.items() if value & bit]
    return ", ".join(causes) if causes else "없음"
