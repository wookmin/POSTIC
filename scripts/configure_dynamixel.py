#!/usr/bin/env python3
"""Dynamixel EEPROM 설정 도구.

기본은 dry-run 이다. 실제로 쓰려면 --apply 를 붙여야 한다.
EEPROM 쓰기는 Torque Enable 이 0 일 때만 허용되므로, 토크가 걸린 모터가
있으면 아무것도 쓰지 않고 중단한다.

사용법:

    # 전체 모터 Return Delay Time 을 0 으로
    ~/dynamixel-venv/bin/python scripts/configure_dynamixel.py --return-delay 0 --apply

    # ID 106 -> 4, 111 -> 5
    ~/dynamixel-venv/bin/python scripts/configure_dynamixel.py --set-id 106:4 --set-id 111:5 --apply
"""

import argparse
import sys
from pathlib import Path

VENV_HINT = "~/dynamixel-venv/bin/python"

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    from src.robot.dynamixel_driver import load_joints, load_robot_config
except ImportError as exc:
    sys.exit(f"{exc}\n"
             f"  {VENV_HINT} 으로 실행하세요.\n"
             "  패키지가 없으면: pip install -r requirements.txt")

_BUS = load_robot_config()["bus"]

DEFAULT_PORT = _BUS["port"]
DEFAULT_BAUD = _BUS["baudrate"]

ADDR_ID = 7
ADDR_HOMING_OFFSET = 20
ADDR_PRESENT_POSITION = 132
ADDR_MAX_POSITION = 48
ADDR_MIN_POSITION = 52
CENTER_TICKS = 2048
ADDR_RETURN_DELAY = 9
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64

OPERATING_MODES = {
    0: "전류 제어",
    1: "속도 제어",
    3: "위치 제어",
    4: "확장 위치(멀티턴)",
    5: "전류기반 위치",
    16: "PWM",
}

# 필드 이름 -> (주소, 표시명). ID 변경은 검증 방식이 달라 따로 처리한다.
SIMPLE_FIELDS = {
    "min_position_limit": (ADDR_MIN_POSITION, 4, "Min Position Limit"),
    "max_position_limit": (ADDR_MAX_POSITION, 4, "Max Position Limit"),
    "return_delay": (ADDR_RETURN_DELAY, 1, "Return Delay"),
    "operating_mode": (ADDR_OPERATING_MODE, 1, "동작 모드"),
    "homing_offset": (ADDR_HOMING_OFFSET, 4, "Homing Offset"),
}

# Homing Offset 은 부호 있는 4바이트다. XL430 계열의 허용 범위.
HOMING_OFFSET_MIN = -1044479
HOMING_OFFSET_MAX = 1044479


def check(comm, err, packet, what):
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"{what}: 통신 실패 - {packet.getTxRxResult(comm)}")
    if err:
        raise RuntimeError(f"{what}: 모터 오류 - {packet.getRxPacketError(err)}")


def read_field(packet, port, motor_id, addr, size, what):
    reader = {1: packet.read1ByteTxRx, 4: packet.read4ByteTxRx}[size]
    value, comm, err = reader(port, motor_id, addr)
    check(comm, err, packet, what)
    if size == 4 and value >= (1 << 31):
        value -= 1 << 32
    return value


def write_field(packet, port, motor_id, addr, size, value, what):
    writer = {1: packet.write1ByteTxRx, 4: packet.write4ByteTxRx}[size]
    mask = 0xFF if size == 1 else 0xFFFFFFFF
    comm, err = writer(port, motor_id, addr, value & mask)
    check(comm, err, packet, what)


def read1(packet, port, motor_id, addr, what):
    value, comm, err = packet.read1ByteTxRx(port, motor_id, addr)
    check(comm, err, packet, what)
    return value


def write1(packet, port, motor_id, addr, value, what):
    comm, err = packet.write1ByteTxRx(port, motor_id, addr, value)
    check(comm, err, packet, what)


def parse_id_pair(text):
    try:
        old, new = text.split(":")
        return int(old), int(new)
    except ValueError:
        raise argparse.ArgumentTypeError(f"--set-id 형식은 OLD:NEW 입니다: {text!r}")


def main():
    parser = argparse.ArgumentParser(description="Dynamixel EEPROM 설정 (기본 dry-run)")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--return-delay", type=int, metavar="N",
                        help="Return Delay Time (0~254, 단위 2us)")
    parser.add_argument("--limits-from-config", action="store_true",
                        help="config/joints.yaml 의 mechanical_min/max 를 "
                             "EEPROM 위치 한계에 굽는다. 코드 버그가 나도 "
                             "하드웨어가 막는 최후 방어선이 된다")
    parser.add_argument("--home-here", nargs="?", type=int, const=CENTER_TICKS,
                        metavar="CENTER",
                        help="현재 물리 위치가 CENTER(기본 2048) 로 읽히도록 "
                             "Homing Offset 을 계산해 설정한다. 가동 구간이 "
                             "0/4095 경계를 넘을 때 중앙을 다시 잡는 용도")
    parser.add_argument("--homing-offset", type=int, metavar="N",
                        help="Homing Offset. 보고 위치를 이만큼 이동시킨다. "
                             "가동 구간이 0/4095 경계를 넘을 때 쓴다")
    parser.add_argument("--operating-mode", type=int, choices=sorted(OPERATING_MODES),
                        metavar="MODE",
                        help="동작 모드. 3=위치 제어, 1=속도 제어")
    parser.add_argument("--set-id", type=parse_id_pair, action="append", default=[],
                        metavar="OLD:NEW", help="ID 변경. 반복 지정 가능")
    parser.add_argument("--ids", type=int, nargs="+",
                        help="--return-delay / --operating-mode 대상 ID. 생략하면 스캔된 전체")
    parser.add_argument("--apply", action="store_true",
                        help="실제로 EEPROM 에 쓴다. 없으면 계획만 출력")
    args = parser.parse_args()

    if args.home_here is not None and args.homing_offset is not None:
        parser.error("--home-here 와 --homing-offset 은 함께 쓸 수 없습니다")
    if args.home_here is not None and not 0 <= args.home_here <= 4095:
        parser.error("--home-here 의 CENTER 는 0~4095 범위입니다")

    if (args.return_delay is None and args.operating_mode is None
            and args.homing_offset is None and args.home_here is None
            and not args.limits_from_config and not args.set_id):
        parser.error("--return-delay, --operating-mode, --homing-offset, "
                     "--set-id 중 하나는 지정해야 합니다")
    if (args.homing_offset is not None
            and not HOMING_OFFSET_MIN <= args.homing_offset <= HOMING_OFFSET_MAX):
        parser.error(f"--homing-offset 은 {HOMING_OFFSET_MIN}~{HOMING_OFFSET_MAX} 범위입니다")
    if args.return_delay is not None and not 0 <= args.return_delay <= 254:
        parser.error("--return-delay 는 0~254 범위입니다")

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)

    try:
        opened = port.openPort()
    except Exception as exc:
        sys.exit(f"{args.port} 을 열 수 없습니다: {exc}\n"
                 "  U2D2 가 연결되어 있는지, dialout 권한이 있는지 확인하세요.")
    if not opened:
        sys.exit(f"{args.port} 포트를 열 수 없습니다. U2D2 연결을 확인하세요.")
    if not port.setBaudRate(args.baud):
        port.closePort()
        sys.exit(f"baudrate {args.baud} 설정 실패")

    try:
        present, _ = packet.broadcastPing(port)
        present = sorted(present)
        if not present:
            sys.exit(f"{args.baud} bps 에서 응답한 모터가 없습니다.")
        print(f"버스 상 ID: {present}\n")

        # --- 사전 검증 -------------------------------------------------
        torque_on = [i for i in present
                     if read1(packet, port, i, ADDR_TORQUE_ENABLE, f"ID {i} 토크 확인")]
        if torque_on:
            sys.exit(f"ID {torque_on} 에 토크가 걸려 있습니다. "
                     "EEPROM 쓰기 전에 토크를 해제하세요.")

        new_ids = [new for _, new in args.set_id]
        if len(set(new_ids)) != len(new_ids):
            sys.exit("--set-id 의 새 ID 가 서로 중복됩니다.")

        # 지정한 순서대로 하나씩 적용했을 때 중간에 ID 충돌이 없는지 시뮬레이션한다.
        # 예: 2->1 을 먼저 하면 3->2 는 안전하지만, 순서가 반대면 충돌한다.
        simulated = set(present)
        for old, new in args.set_id:
            if old not in simulated:
                sys.exit(f"ID {old} 가 이 순서에서는 버스에 존재하지 않습니다.")
            if not 0 <= new <= 252:
                sys.exit(f"새 ID {new} 는 0~252 범위여야 합니다.")
            if new in simulated and new != old:
                sys.exit(f"ID {old} -> {new}: 그 시점에 ID {new} 가 사용 중입니다. "
                         "--set-id 순서를 바꾸세요.")
            simulated.discard(old)
            simulated.add(new)

        # --- 계획 -----------------------------------------------------
        plan = []

        if args.limits_from_config:
            joints = load_joints()
            if not joints:
                sys.exit("config/joints.yaml 에 joints 가 비어 있습니다.")
            wanted_ids = set(args.ids) if args.ids else None
            for name, spec in joints.items():
                motor_id = spec.get("id")
                if wanted_ids is not None and motor_id not in wanted_ids:
                    continue
                if motor_id not in present:
                    sys.exit(f"{name}: ID {motor_id} 가 버스에 없습니다.")
                low = spec.get("mechanical_min")
                high = spec.get("mechanical_max")
                if low is None or high is None:
                    sys.exit(f"{name}: mechanical_min/max 가 없습니다.")
                if not 0 <= low < high <= 4095:
                    sys.exit(f"{name}: 기계적 한계 {low}~{high} 가 "
                             "0~4095 범위를 벗어납니다.")
                for field, addr, value in (
                    ("min_position_limit", ADDR_MIN_POSITION, low),
                    ("max_position_limit", ADDR_MAX_POSITION, high),
                ):
                    current = read_field(packet, port, motor_id, addr, 4,
                                         f"ID {motor_id} {field} 확인")
                    if current != value:
                        plan.append((field, motor_id, current, value))

        # --home-here 는 모터마다 값이 다르므로 개별 계산해 계획에 넣는다.
        #
        # 위치 제어 모드에서 Homing Offset 은 결과 위치가 0~4095 안에 떨어질
        # 때만 반영된다. 범위를 벗어나는 값은 조용히 무시되므로, 저장된
        # 오프셋이 실제로 적용 중이라고 가정하고 역산하면 값이 발산한다.
        # 그래서 적용 단계에서는 오프셋을 0 으로 되돌려 원위치를 읽은 뒤
        # 다시 계산한다. 여기서 잡는 값은 그 결과의 추정치다.
        if args.home_here is not None:
            for motor_id in (args.ids or present):
                if motor_id not in present:
                    sys.exit(f"ID {motor_id} 가 버스에 없습니다.")
                current_offset = read_field(packet, port, motor_id,
                                            ADDR_HOMING_OFFSET, 4,
                                            f"ID {motor_id} Homing Offset 확인")
                reported = read_field(packet, port, motor_id,
                                      ADDR_PRESENT_POSITION, 4,
                                      f"ID {motor_id} 현재 위치 확인")
                estimate = current_offset + (args.home_here - reported)
                plan.append(("home_here", motor_id, current_offset, estimate))

        requested = {
            "return_delay": args.return_delay,
            "operating_mode": args.operating_mode,
            "homing_offset": args.homing_offset,
        }
        for field, wanted in requested.items():
            if wanted is None:
                continue
            addr, size, _ = SIMPLE_FIELDS[field]
            targets = args.ids or present
            for motor_id in targets:
                if motor_id not in present:
                    sys.exit(f"ID {motor_id} 가 버스에 없습니다.")
                current = read_field(packet, port, motor_id, addr, size,
                                     f"ID {motor_id} {field} 확인")
                if current != wanted:
                    plan.append((field, motor_id, current, wanted))
        for old, new in args.set_id:
            plan.append(("id", old, old, new))

        if not plan:
            print("변경할 것이 없습니다. 이미 원하는 값입니다.")
            return 0

        for kind, motor_id, before, after in plan:
            if kind in SIMPLE_FIELDS:
                label = SIMPLE_FIELDS[kind][2]
            elif kind == "home_here":
                label = "Homing Offset (중앙 재설정, 추정)"
            else:
                label = "ID"
            if kind == "operating_mode":
                before = f"{before}({OPERATING_MODES.get(before, '?')})"
                after = f"{after}({OPERATING_MODES.get(after, '?')})"
            print(f"  ID {motor_id:>3}  {label}: {before} -> {after}")
        print()

        if not args.apply:
            print("dry-run 입니다. 실제로 쓰려면 --apply 를 붙이세요.")
            return 0

        # --- 적용 -----------------------------------------------------
        for kind, motor_id, before, after in plan:
            if kind == "home_here":
                # 저장된 오프셋이 실제로 적용 중인지 알 수 없다. 위치 제어
                # 모드에서는 결과가 0~4095 를 벗어나는 오프셋이 조용히 무시되기
                # 때문이다. 그래서 0 으로 되돌려 원위치를 확정한 뒤 재계산한다.
                write_field(packet, port, motor_id, ADDR_HOMING_OFFSET, 4, 0,
                            f"ID {motor_id} Homing Offset 초기화")
                raw = read_field(packet, port, motor_id, ADDR_PRESENT_POSITION,
                                 4, f"ID {motor_id} 원위치 확인")
                wanted = args.home_here - raw
                if not HOMING_OFFSET_MIN <= wanted <= HOMING_OFFSET_MAX:
                    sys.exit(f"ID {motor_id}: 필요한 오프셋 {wanted} 가 "
                             "허용 범위를 벗어납니다.")
                write_field(packet, port, motor_id, ADDR_HOMING_OFFSET, 4,
                            wanted, f"ID {motor_id} Homing Offset 쓰기")
                actual = read_field(packet, port, motor_id,
                                    ADDR_PRESENT_POSITION, 4,
                                    f"ID {motor_id} 적용 후 위치 확인")
                status = ("OK" if actual == args.home_here
                          else f"불일치 (보고 위치 {actual})")
                print(f"  ID {motor_id:>3}  원위치 {raw} -> 오프셋 {wanted}, "
                      f"보고 위치 {actual}  [{status}]")
            elif kind in SIMPLE_FIELDS:
                addr, size, label = SIMPLE_FIELDS[kind]
                write_field(packet, port, motor_id, addr, size, after,
                            f"ID {motor_id} {label} 쓰기")
                actual = read_field(packet, port, motor_id, addr, size,
                                    f"ID {motor_id} {label} 검증")
                status = "OK" if actual == after else f"불일치({actual})"
                print(f"  ID {motor_id:>3}  {label} -> {after}  [{status}]")
            else:
                write1(packet, port, motor_id, ADDR_ID, after,
                       f"ID {motor_id} -> {after} 쓰기")
                actual = read1(packet, port, after, ADDR_ID,
                               f"새 ID {after} 검증")
                status = "OK" if actual == after else f"불일치({actual})"
                print(f"  ID {motor_id:>3}  ID -> {after}  [{status}]")

        print()
        after_ping, _ = packet.broadcastPing(port)
        print(f"적용 후 버스 상 ID: {sorted(after_ping)}")
        return 0
    finally:
        port.closePort()


if __name__ == "__main__":
    sys.exit(main())
