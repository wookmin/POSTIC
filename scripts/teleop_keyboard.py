#!/usr/bin/env python3
"""키보드로 관절을 하나씩 미세 조정하는 텔레옵.

이전 ~/dxl_keyboard_3_4.py 를 대체한다. 그 스크립트는 57600 bps 와 ID [3,4] 를
박아두고 w/s, d/a 키를 각 ID 에 직접 대응시켜서 모터가 2개를 넘으면 쓸 수 없었다.
이제 버스 설정은 config/robot.yaml 에서 읽고, 숫자키로 대상 ID 를 골라 10개까지
다룬다. 위치 한계(Min/Max Position Limit)를 넘는 지령은 클램프한다.

사용법:
    ~/dynamixel-venv/bin/python scripts/teleop_keyboard.py --ids 1 2 3
"""

import argparse
import sys
import termios
import tty
from pathlib import Path

VENV_HINT = "~/dynamixel-venv/bin/python"

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from src.robot.dynamixel_driver import (
        BusError, check, configured_ids, load_robot_config, open_bus, ping_all,
    )
except ImportError as exc:
    sys.exit(f"{exc}\n"
             f"  {VENV_HINT} 으로 실행하세요.\n"
             "  패키지가 없으면: pip install -r requirements.txt")

ADDR_MAX_POSITION = 48
ADDR_MIN_POSITION = 52
ADDR_TORQUE_ENABLE = 64
ADDR_HW_ERROR = 70
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

EDGE_MARGIN = 20
SAFE_VELOCITY = 40
SAFE_ACCEL = 20
SELECT_KEYS = "1234567890"


def read1(packet, port, motor_id, addr, what):
    value, comm, err = packet.read1ByteTxRx(port, motor_id, addr)
    check(comm, err, packet, what)
    return value


def read4(packet, port, motor_id, addr, what):
    value, comm, err = packet.read4ByteTxRx(port, motor_id, addr)
    check(comm, err, packet, what)
    return value - (1 << 32) if value >= (1 << 31) else value


def write1(packet, port, motor_id, addr, value, what):
    comm, err = packet.write1ByteTxRx(port, motor_id, addr, value)
    check(comm, err, packet, what)


def write4(packet, port, motor_id, addr, value, what):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value & 0xFFFFFFFF)
    check(comm, err, packet, what)


def getch():
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def print_help(ids, step):
    print("\r키보드 텔레옵")
    print("\r-------------")
    for index, motor_id in enumerate(ids):
        print(f"\r  {SELECT_KEYS[index]} : ID {motor_id} 선택")
    print(f"\r  w / s : 선택된 ID  +{step} / -{step} ticks")
    print("\r  [ / ] : step 절반 / 두 배")
    print("\r  p     : 현재 위치 출력")
    print("\r  q     : 종료 (토크 해제)")
    print("\r")


def main():
    parser = argparse.ArgumentParser(description="키보드 관절 텔레옵")
    parser.add_argument("--ids", type=int, nargs="+",
                        help="조작할 ID. 생략하면 config/robot.yaml 의 motors.ids")
    parser.add_argument("--step", type=int, default=20,
                        help="키 1회당 이동 ticks. 기본 20 (약 1.8도)")
    args = parser.parse_args()

    config = load_robot_config()
    ids = args.ids or configured_ids(config)
    if not ids:
        sys.exit("조작할 ID 가 없습니다. --ids 로 지정하세요.")
    if len(ids) > len(SELECT_KEYS):
        sys.exit(f"ID 는 최대 {len(SELECT_KEYS)} 개까지 지원합니다.")

    step = args.step
    try:
        with open_bus(config) as (packet, port):
            present = ping_all(packet, port)
            missing = [i for i in ids if i not in present]
            if missing:
                sys.exit(f"ID {missing} 가 버스에 없습니다. 응답: {sorted(present)}")

            limits = {}
            positions = {}
            saved_profile = {}
            engaged = []

            for motor_id in ids:
                hw = read1(packet, port, motor_id, ADDR_HW_ERROR,
                           f"ID {motor_id} 하드웨어 에러 확인")
                if hw:
                    sys.exit(f"ID {motor_id} 하드웨어 에러 0x{hw:02X}. 중단합니다.")

                low = read4(packet, port, motor_id, ADDR_MIN_POSITION, "Min Position")
                high = read4(packet, port, motor_id, ADDR_MAX_POSITION, "Max Position")
                limits[motor_id] = (low + EDGE_MARGIN, high - EDGE_MARGIN)
                positions[motor_id] = read4(packet, port, motor_id,
                                            ADDR_PRESENT_POSITION, "현재 위치")
                saved_profile[motor_id] = (
                    read4(packet, port, motor_id, ADDR_PROFILE_VELOCITY, "Velocity"),
                    read4(packet, port, motor_id, ADDR_PROFILE_ACCEL, "Accel"),
                )

            try:
                for motor_id in ids:
                    write4(packet, port, motor_id, ADDR_PROFILE_VELOCITY,
                           SAFE_VELOCITY, "Profile Velocity")
                    write4(packet, port, motor_id, ADDR_PROFILE_ACCEL,
                           SAFE_ACCEL, "Profile Acceleration")
                    write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 1, "토크 인가")
                    engaged.append(motor_id)

                active = ids[0]
                print_help(ids, step)
                print(f"\r선택: ID {active}   시작 위치: {positions}\r")

                while True:
                    key = getch()

                    if key in ("q", "\x03"):
                        break

                    if key in SELECT_KEYS[:len(ids)]:
                        active = ids[SELECT_KEYS.index(key)]
                        print(f"\r선택: ID {active} "
                              f"(현재 {positions[active]} ticks)\r")
                        continue

                    if key == "[":
                        step = max(1, step // 2)
                        print(f"\rstep = {step}\r")
                        continue
                    if key == "]":
                        step = min(500, step * 2)
                        print(f"\rstep = {step}\r")
                        continue

                    if key == "p":
                        current = read4(packet, port, active,
                                        ADDR_PRESENT_POSITION, "현재 위치")
                        print(f"\rID {active} 현재 {current} ticks "
                              f"({current * 360 / 4096:.1f}도)\r")
                        continue

                    if key not in ("w", "s"):
                        continue

                    low, high = limits[active]
                    target = positions[active] + (step if key == "w" else -step)
                    clamped = max(low, min(high, target))
                    if clamped != target:
                        print(f"\rID {active} 위치 한계 {low}~{high} 에서 클램프\r")
                    positions[active] = clamped
                    write4(packet, port, active, ADDR_GOAL_POSITION, clamped,
                           f"ID {active} 목표 위치")
                    print(f"\rID {active} -> {clamped} ticks "
                          f"({clamped * 360 / 4096:.1f}도)\r")
            finally:
                print("\r토크 해제 중\r")
                for motor_id in engaged:
                    try:
                        write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 0,
                               "토크 해제")
                        velocity, accel = saved_profile[motor_id]
                        write4(packet, port, motor_id, ADDR_PROFILE_VELOCITY,
                               velocity, "Profile Velocity 복원")
                        write4(packet, port, motor_id, ADDR_PROFILE_ACCEL,
                               accel, "Profile Acceleration 복원")
                    except Exception as exc:
                        print(f"\rID {motor_id} 정리 중 오류: {exc}\r", file=sys.stderr)
                print("\r완료\r")
        return 0
    except BusError as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    sys.exit(main())
