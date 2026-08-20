#!/usr/bin/env python3
"""ID 하나를 소각도로 흔들어 어느 관절인지 눈으로 확인하는 도구.

한 번에 ID 하나만 움직인다. 움직임이 끝나면 원래 위치로 되돌리고 토크를 해제한다.
실제로 움직이려면 --go 를 반드시 붙여야 한다.

사용법:
    source /opt/ros/humble/setup.bash
    python3 scripts/identify_joints.py --id 1            # 계획만 출력
    python3 scripts/identify_joints.py --id 1 --go       # 실제로 흔든다
"""

import argparse
import sys
import time

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.exit(
        "dynamixel_sdk 를 찾을 수 없습니다.\n"
        "  source /opt/ros/humble/setup.bash  후 다시 실행하세요."
    )

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1000000

ADDR_MIN_POSITION = 52
ADDR_MAX_POSITION = 48
ADDR_TORQUE_ENABLE = 64
ADDR_HW_ERROR = 70
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TICKS_PER_REV = 4096
EDGE_MARGIN = 20          # 기계적 한계에서 남겨둘 여유 ticks
SAFE_VELOCITY = 30        # Profile Velocity, 약 7 rpm
SAFE_ACCEL = 10           # Profile Acceleration


def check(comm, err, packet, what):
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"{what}: 통신 실패 - {packet.getTxRxResult(comm)}")
    if err:
        raise RuntimeError(f"{what}: 모터 오류 - {packet.getRxPacketError(err)}")


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
    comm, err = packet.write4ByteTxRx(port, motor_id, addr, value)
    check(comm, err, packet, what)


def degrees_to_ticks(degrees):
    return int(round(degrees * TICKS_PER_REV / 360.0))


def ticks_to_degrees(ticks):
    return ticks * 360.0 / TICKS_PER_REV


def pick_target(position, delta, low, high):
    """여유가 있는 방향으로 목표 위치를 잡는다. 양쪽 다 좁으면 None."""
    if position + delta <= high:
        return position + delta
    if position - delta >= low:
        return position - delta
    return None


def wait_until_reached(packet, port, motor_id, target, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = read4(packet, port, motor_id, ADDR_PRESENT_POSITION, "현재 위치")
        if abs(current - target) <= 10:
            return current
        time.sleep(0.05)
    return read4(packet, port, motor_id, ADDR_PRESENT_POSITION, "현재 위치")


def main():
    parser = argparse.ArgumentParser(description="ID 하나를 소각도로 흔들어 관절 식별")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--degrees", type=float, default=10.0,
                        help="흔들 각도. 기본 10도")
    parser.add_argument("--repeat", type=int, default=2, help="왕복 횟수")
    parser.add_argument("--go", action="store_true",
                        help="실제로 움직인다. 없으면 계획만 출력")
    args = parser.parse_args()

    if not 0 < args.degrees <= 30:
        parser.error("--degrees 는 0 초과 30 이하로 제한합니다")

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)

    if not port.openPort():
        sys.exit(f"{args.port} 포트를 열 수 없습니다.")
    if not port.setBaudRate(args.baud):
        port.closePort()
        sys.exit(f"baudrate {args.baud} 설정 실패")

    motor_id = args.id
    torque_was_enabled = False
    saved_velocity = saved_accel = None
    origin = None

    try:
        present, _ = packet.broadcastPing(port)
        if motor_id not in present:
            sys.exit(f"ID {motor_id} 가 버스에 없습니다. 응답: {sorted(present)}")

        hw = read1(packet, port, motor_id, ADDR_HW_ERROR, "하드웨어 에러 확인")
        if hw:
            sys.exit(f"ID {motor_id} 하드웨어 에러 0x{hw:02X}. 움직이지 않습니다.")

        origin = read4(packet, port, motor_id, ADDR_PRESENT_POSITION, "원위치 읽기")
        low = read4(packet, port, motor_id, ADDR_MIN_POSITION, "Min Position")
        high = read4(packet, port, motor_id, ADDR_MAX_POSITION, "Max Position")
        low += EDGE_MARGIN
        high -= EDGE_MARGIN

        delta = degrees_to_ticks(args.degrees)
        target = pick_target(origin, delta, low, high)
        if target is None:
            sys.exit(f"ID {motor_id}: 현재 위치 {origin} 에서 양방향 모두 여유가 "
                     f"부족합니다 (허용 {low}~{high}). --degrees 를 줄이세요.")

        direction = "+" if target > origin else "-"
        print(f"ID {motor_id}")
        print(f"  원위치   : {origin} ticks ({ticks_to_degrees(origin):.1f}°)")
        print(f"  목표      : {target} ticks ({ticks_to_degrees(target):.1f}°)"
              f"  [{direction}{args.degrees}°]")
        print(f"  허용 범위 : {low} ~ {high} ticks")
        print(f"  왕복      : {args.repeat}회, 저속(Profile Velocity {SAFE_VELOCITY})")
        print()

        if not args.go:
            print("계획만 출력했습니다. 실제로 움직이려면 --go 를 붙이세요.")
            return 0

        saved_velocity = read4(packet, port, motor_id, ADDR_PROFILE_VELOCITY,
                               "Profile Velocity 저장")
        saved_accel = read4(packet, port, motor_id, ADDR_PROFILE_ACCEL,
                            "Profile Acceleration 저장")
        write4(packet, port, motor_id, ADDR_PROFILE_VELOCITY, SAFE_VELOCITY,
               "Profile Velocity 설정")
        write4(packet, port, motor_id, ADDR_PROFILE_ACCEL, SAFE_ACCEL,
               "Profile Acceleration 설정")

        write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 1, "토크 인가")
        torque_was_enabled = True

        for round_index in range(1, args.repeat + 1):
            write4(packet, port, motor_id, ADDR_GOAL_POSITION, target, "목표 위치")
            reached = wait_until_reached(packet, port, motor_id, target, timeout=4.0)
            print(f"  {round_index}회 이동 -> {reached} ticks "
                  f"({ticks_to_degrees(reached):.1f}°)", flush=True)

            write4(packet, port, motor_id, ADDR_GOAL_POSITION, origin, "원위치 복귀")
            reached = wait_until_reached(packet, port, motor_id, origin, timeout=4.0)
            print(f"  {round_index}회 복귀 -> {reached} ticks "
                  f"({ticks_to_degrees(reached):.1f}°)", flush=True)

        hw = read1(packet, port, motor_id, ADDR_HW_ERROR, "하드웨어 에러 재확인")
        print(f"\n  종료 후 Hardware Error: 0x{hw:02X}")
        return 0
    finally:
        try:
            if torque_was_enabled:
                write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 0, "토크 해제")
                print("  토크 해제 완료")
            if saved_velocity is not None:
                write4(packet, port, motor_id, ADDR_PROFILE_VELOCITY, saved_velocity,
                       "Profile Velocity 복원")
            if saved_accel is not None:
                write4(packet, port, motor_id, ADDR_PROFILE_ACCEL, saved_accel,
                       "Profile Acceleration 복원")
        except Exception as exc:
            print(f"  정리 중 오류: {exc}", file=sys.stderr)
        finally:
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
