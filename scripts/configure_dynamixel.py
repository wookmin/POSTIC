#!/usr/bin/env python3
"""Dynamixel EEPROM 설정 도구.

기본은 dry-run 이다. 실제로 쓰려면 --apply 를 붙여야 한다.
EEPROM 쓰기는 Torque Enable 이 0 일 때만 허용되므로, 토크가 걸린 모터가
있으면 아무것도 쓰지 않고 중단한다.

사용법:
    source /opt/ros/humble/setup.bash

    # 전체 모터 Return Delay Time 을 0 으로
    python3 scripts/configure_dynamixel.py --return-delay 0 --apply

    # ID 106 -> 4, 111 -> 5
    python3 scripts/configure_dynamixel.py --set-id 106:4 --set-id 111:5 --apply
"""

import argparse
import sys

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.exit(
        "dynamixel_sdk 를 찾을 수 없습니다.\n"
        "  source /opt/ros/humble/setup.bash  후 다시 실행하세요."
    )

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000

ADDR_ID = 7
ADDR_RETURN_DELAY = 9
ADDR_TORQUE_ENABLE = 64


def check(comm, err, packet, what):
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"{what}: 통신 실패 - {packet.getTxRxResult(comm)}")
    if err:
        raise RuntimeError(f"{what}: 모터 오류 - {packet.getRxPacketError(err)}")


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
    parser.add_argument("--set-id", type=parse_id_pair, action="append", default=[],
                        metavar="OLD:NEW", help="ID 변경. 반복 지정 가능")
    parser.add_argument("--ids", type=int, nargs="+",
                        help="--return-delay 대상 ID. 생략하면 스캔된 전체")
    parser.add_argument("--apply", action="store_true",
                        help="실제로 EEPROM 에 쓴다. 없으면 계획만 출력")
    args = parser.parse_args()

    if args.return_delay is None and not args.set_id:
        parser.error("--return-delay 또는 --set-id 중 하나는 지정해야 합니다")
    if args.return_delay is not None and not 0 <= args.return_delay <= 254:
        parser.error("--return-delay 는 0~254 범위입니다")

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)

    if not port.openPort():
        sys.exit(f"{args.port} 포트를 열 수 없습니다.")
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
        if args.return_delay is not None:
            targets = args.ids or present
            for motor_id in targets:
                if motor_id not in present:
                    sys.exit(f"ID {motor_id} 가 버스에 없습니다.")
                current = read1(packet, port, motor_id, ADDR_RETURN_DELAY,
                                f"ID {motor_id} Return Delay 확인")
                if current != args.return_delay:
                    plan.append(("return_delay", motor_id, current, args.return_delay))
        for old, new in args.set_id:
            plan.append(("id", old, old, new))

        if not plan:
            print("변경할 것이 없습니다. 이미 원하는 값입니다.")
            return 0

        for kind, motor_id, before, after in plan:
            label = "Return Delay" if kind == "return_delay" else "ID"
            print(f"  ID {motor_id:>3}  {label}: {before} -> {after}")
        print()

        if not args.apply:
            print("dry-run 입니다. 실제로 쓰려면 --apply 를 붙이세요.")
            return 0

        # --- 적용 -----------------------------------------------------
        for kind, motor_id, before, after in plan:
            if kind == "return_delay":
                write1(packet, port, motor_id, ADDR_RETURN_DELAY, after,
                       f"ID {motor_id} Return Delay 쓰기")
                actual = read1(packet, port, motor_id, ADDR_RETURN_DELAY,
                               f"ID {motor_id} Return Delay 검증")
                status = "OK" if actual == after else f"불일치({actual})"
                print(f"  ID {motor_id:>3}  Return Delay -> {after}  [{status}]")
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
