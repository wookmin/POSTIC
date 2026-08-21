#!/usr/bin/env python3
"""토크·워치독을 내리고 하드웨어 에러 래치를 푸는 복구 도구.

이 스크립트는 로봇이 이미 이상한 상태일 때 쓰는 것이므로, 다른 스크립트와 달리
패킷 오류에 관대하다. 하드웨어 에러가 걸린 모터는 모든 응답에 alert 비트(0x80)를
세우는데, 그걸 오류로 취급하면 정작 복구가 불가능해진다.

사용법:
    ~/dynamixel-venv/bin/python scripts/torque_off.py
    ~/dynamixel-venv/bin/python scripts/torque_off.py --clear-errors
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from src.robot.dynamixel_driver import (
        BusError, describe_hardware_error, open_bus, ping_all,
    )
except ImportError as exc:
    sys.exit(f"{exc}\n  ~/dynamixel-venv/bin/python 으로 실행하세요.")

ADDR_HARDWARE_ERROR = 70
ADDR_TORQUE_ENABLE = 64
ADDR_BUS_WATCHDOG = 98

COMM_SUCCESS = 0


def read1(packet, port, motor_id, addr):
    """alert 비트를 무시하고 읽는다. 통신 자체가 실패하면 None."""
    value, comm, _ = packet.read1ByteTxRx(port, motor_id, addr)
    return None if comm != COMM_SUCCESS else value


def write1(packet, port, motor_id, addr, value):
    """alert 비트를 무시하고 쓴다. 통신이 성공하면 True."""
    comm, _ = packet.write1ByteTxRx(port, motor_id, addr, value)
    return comm == COMM_SUCCESS


def main():
    parser = argparse.ArgumentParser(description="토크 해제 및 에러 복구")
    parser.add_argument("--ids", type=int, nargs="+",
                        help="대상 ID. 생략하면 응답하는 전체")
    parser.add_argument("--clear-errors", action="store_true",
                        help="하드웨어 에러가 걸린 모터를 재부팅해 래치를 푼다. "
                             "과부하라면 먼저 손으로 자세를 되돌려 부하를 덜 것")
    args = parser.parse_args()

    try:
        with open_bus() as (packet, port):
            present = sorted(ping_all(packet, port))
            if not present:
                sys.exit("응답하는 모터가 없습니다. 전원과 배선을 확인하세요.")

            targets = args.ids or present
            missing = [i for i in targets if i not in present]
            if missing:
                sys.exit(f"ID {missing} 가 버스에 없습니다. 응답: {present}")

            watchdogs, released, failed = [], [], []

            for motor_id in targets:
                # 워치독이 남으면 다음 실행에서 레지스터 쓰기가 거부된다.
                if read1(packet, port, motor_id, ADDR_BUS_WATCHDOG):
                    if write1(packet, port, motor_id, ADDR_BUS_WATCHDOG, 0):
                        watchdogs.append(motor_id)
                    else:
                        failed.append(f"ID {motor_id} 워치독 해제")

                if read1(packet, port, motor_id, ADDR_TORQUE_ENABLE):
                    if write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 0):
                        released.append(motor_id)
                    else:
                        failed.append(f"ID {motor_id} 토크 해제")

            print(f"워치독 해제: {watchdogs}" if watchdogs else "워치독: 전부 정상")
            print(f"토크 해제: {released}" if released else "토크: 전부 해제 상태")
            for item in failed:
                print(f"실패: {item}")

            faulted = {}
            for motor_id in targets:
                value = read1(packet, port, motor_id, ADDR_HARDWARE_ERROR)
                if value:
                    faulted[motor_id] = value

            if not faulted:
                print("하드웨어 에러: 없음")
                return 1 if failed else 0

            for motor_id, value in faulted.items():
                print(f"ID {motor_id} 하드웨어 에러 0x{value:02X} "
                      f"({describe_hardware_error(value)})")

            if not args.clear_errors:
                print("\n에러는 래치되어 재부팅 전까지 풀리지 않습니다. "
                      "--clear-errors 를 붙이세요.")
                print("과부하라면 재부팅 전에 손으로 자세를 중립에 가깝게 "
                      "되돌려 부하를 덜어주세요.")
                return 1

            for motor_id in faulted:
                comm, _ = packet.reboot(port, motor_id)
                if comm != COMM_SUCCESS:
                    print(f"ID {motor_id} 재부팅 실패: {packet.getTxRxResult(comm)}")
                else:
                    print(f"ID {motor_id} 재부팅 요청")
            time.sleep(1.5)

            still = {}
            for motor_id in faulted:
                value = read1(packet, port, motor_id, ADDR_HARDWARE_ERROR)
                if value is None:
                    still[motor_id] = "응답 없음"
                elif value:
                    still[motor_id] = f"0x{value:02X}"
            if still:
                print(f"\n아직 해제되지 않음: {still}")
                print("전원(SMPS)을 껐다 켜야 할 수 있습니다.")
                return 1

            print(f"\n에러 해제 완료: {sorted(faulted)}")
            return 1 if failed else 0
    except BusError as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    sys.exit(main())
