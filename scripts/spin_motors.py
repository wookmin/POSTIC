#!/usr/bin/env python3
"""지정한 ID 를 속도 제어 모드로 짧게 돌려보는 통전 확인 도구.

이전 ~/motor_test.py 를 대체한다. 그 스크립트는 포트/baudrate/ID 를 직접 박아두고
있었는데(57600, ID [1,2]), 버스가 1 Mbps 로 바뀌고 ID 가 1~10 으로 재배열된 뒤로는
맞지 않는다. 이제 버스 설정은 config/robot.yaml 에서 읽고, 움직일 ID 는 반드시
명령행에서 직접 지정해야 한다.

사용법:
    ~/dynamixel-venv/bin/python scripts/spin_motors.py --ids 1 2          # 계획만 출력
    ~/dynamixel-venv/bin/python scripts/spin_motors.py --ids 1 2 --go     # 실제로 돌린다
"""

import argparse
import sys
import time
from pathlib import Path

VENV_HINT = "~/dynamixel-venv/bin/python"

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from src.robot.dynamixel_driver import (
        BusError, check, load_robot_config, open_bus, ping_all,
    )
except ImportError as exc:
    sys.exit(f"{exc}\n"
             f"  {VENV_HINT} 으로 실행하세요.\n"
             "  패키지가 없으면: pip install -r requirements.txt")

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_GOAL_VELOCITY = 104
ADDR_HW_ERROR = 70

MODE_VELOCITY = 1
MODE_POSITION = 3

MAX_SPEED = 50      # 약 11 rpm. 뼈대 확인용이라 낮게 제한한다.
MAX_DURATION = 3.0


def write1(packet, port, motor_id, addr, value, what):
    comm, err = packet.write1ByteTxRx(port, motor_id, addr, value)
    check(comm, err, packet, what)


def write4(packet, port, motor_id, addr, value, what):
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value & 0xFFFFFFFF)
    check(comm, err, packet, what)


def read1(packet, port, motor_id, addr, what):
    value, comm, err = packet.read1ByteTxRx(port, motor_id, addr)
    check(comm, err, packet, what)
    return value


def stop(packet, port, motor_id):
    write4(packet, port, motor_id, ADDR_GOAL_VELOCITY, 0, f"ID {motor_id} 속도 0")
    write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 0, f"ID {motor_id} 토크 해제")


def main():
    parser = argparse.ArgumentParser(description="ID 를 속도 제어로 짧게 돌려본다")
    parser.add_argument("--ids", type=int, nargs="+", required=True,
                        help="돌릴 ID. 기본값 없음 — 반드시 직접 지정한다")
    parser.add_argument("--speed", type=int, default=15,
                        help=f"Goal Velocity. 최대 {MAX_SPEED}")
    parser.add_argument("--duration", type=float, default=1.0,
                        help=f"회전 시간(초). 최대 {MAX_DURATION}")
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--yes", action="store_true",
                        help="ID 마다 Enter 확인을 건너뛴다")
    parser.add_argument("--go", action="store_true",
                        help="실제로 돌린다. 없으면 계획만 출력")
    args = parser.parse_args()

    if not 0 < args.speed <= MAX_SPEED:
        parser.error(f"--speed 는 1~{MAX_SPEED} 범위입니다")
    if not 0 < args.duration <= MAX_DURATION:
        parser.error(f"--duration 은 0 초과 {MAX_DURATION} 이하입니다")

    speed = -args.speed if args.reverse else args.speed
    config = load_robot_config()

    try:
        with open_bus(config) as (packet, port):
            present = ping_all(packet, port)
            missing = [i for i in args.ids if i not in present]
            if missing:
                sys.exit(f"ID {missing} 가 버스에 없습니다. 응답: {sorted(present)}")

            for motor_id in args.ids:
                hw = read1(packet, port, motor_id, ADDR_HW_ERROR,
                           f"ID {motor_id} 하드웨어 에러 확인")
                if hw:
                    sys.exit(f"ID {motor_id} 하드웨어 에러 0x{hw:02X}. 중단합니다.")

            print(f"대상 ID   : {args.ids}")
            print(f"속도      : {speed} ({'역방향' if args.reverse else '정방향'})")
            print(f"회전 시간 : {args.duration}초")
            print(f"포트      : {config['bus']['port']} @ "
                  f"{config['bus']['baudrate']} bps\n")

            if not args.go:
                print("계획만 출력했습니다. 실제로 돌리려면 --go 를 붙이세요.")
                return 0

            touched = []
            try:
                for motor_id in args.ids:
                    write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 0,
                           f"ID {motor_id} 토크 해제")
                    write1(packet, port, motor_id, ADDR_OPERATING_MODE, MODE_VELOCITY,
                           f"ID {motor_id} 속도 제어 모드")
                    write4(packet, port, motor_id, ADDR_PROFILE_ACCEL, 20,
                           f"ID {motor_id} 가속도")
                    touched.append(motor_id)

                for motor_id in args.ids:
                    if not args.yes:
                        input(f"ID {motor_id} 를 {args.duration}초간 회전합니다. Enter: ")
                    write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 1,
                           f"ID {motor_id} 토크 인가")
                    write4(packet, port, motor_id, ADDR_GOAL_VELOCITY, speed,
                           f"ID {motor_id} 속도 지령")
                    time.sleep(args.duration)
                    stop(packet, port, motor_id)
                    print(f"ID {motor_id} 정지")
            finally:
                for motor_id in touched:
                    try:
                        stop(packet, port, motor_id)
                        write1(packet, port, motor_id, ADDR_OPERATING_MODE,
                               MODE_POSITION, f"ID {motor_id} 위치 제어 복귀")
                    except Exception as exc:
                        print(f"ID {motor_id} 정리 중 오류: {exc}", file=sys.stderr)
                print("전체 정지, 위치 제어 모드 복귀 완료")
        return 0
    except BusError as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    sys.exit(main())
