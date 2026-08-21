#!/usr/bin/env python3
"""키보드로 관절을 하나씩 움직여보는 텔레옵. 위치 제어와 속도 제어를 모두 다룬다.

이 로봇은 유닛마다 축 하나는 위치 제어, 하나는 속도 제어로 설정돼 있다.
같은 w/s 키가 축의 모드에 따라 다르게 동작한다.

  위치 제어 축: w/s 가 목표 위치를 ±step ticks 옮긴다. 위치 한계와
                시작 위치 기준 이동 상한(--max-travel-deg) 안에서 클램프된다.
  속도 제어 축: w/s 가 목표 속도를 ±speed-step 올리고 내린다. 위치 한계가
                적용되지 않으므로 Bus Watchdog 을 걸어 통신이 끊기면
                1초 안에 스스로 멈추게 한다.

사용법:
    ~/dynamixel-venv/bin/python scripts/teleop_keyboard.py
    ~/dynamixel-venv/bin/python scripts/teleop_keyboard.py --ids 1 4 5
"""

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from src.robot.dynamixel_driver import (
        BusError, active_ids, check, describe_hardware_error,
        install_signal_guards, load_joints, read_hardware_error,
        load_robot_config, open_bus, ping_all,
    )
except ImportError as exc:
    sys.exit(f"{exc}\n  ~/dynamixel-venv/bin/python 으로 실행하세요.")

ADDR_MAX_POSITION = 48
ADDR_MIN_POSITION = 52
ADDR_TORQUE_ENABLE = 64
ADDR_HW_ERROR = 70
ADDR_BUS_WATCHDOG = 98
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11

MODE_VELOCITY = 1
MODE_POSITION = 3

TICKS_PER_REV = 4096
EDGE_MARGIN = 20
SAFE_PROFILE_VELOCITY = 40
SAFE_PROFILE_ACCEL = 20
MAX_VELOCITY = 40              # 약 9 rpm. 뼈대 확인용이라 낮게 묶는다.
WATCHDOG_UNITS = 50            # x20ms = 1초
KEEPALIVE_SEC = 0.25           # 워치독이 물리기 전에 다시 써준다
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


def degrees_to_ticks(degrees):
    return int(round(degrees * TICKS_PER_REV / 360.0))


def ticks_to_degrees(ticks):
    return ticks * 360.0 / TICKS_PER_REV


def getch_nonblocking(timeout):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def print_help(ids, modes, step, speed_step):
    print("\r키보드 텔레옵")
    print("\r-------------")
    for index, motor_id in enumerate(ids):
        kind = "속도" if modes[motor_id] == MODE_VELOCITY else "위치"
        print(f"\r  {SELECT_KEYS[index]} : ID {motor_id} 선택  [{kind} 제어]")
    print(f"\r  w / s    : 위치축 ±{step} ticks  |  속도축 ±{speed_step}")
    print("\r  스페이스 : 위치축 시작 위치 복귀  |  속도축 정지")
    print("\r  x        : 속도축 전부 정지")
    print("\r  [ / ]    : step 절반 / 두 배")
    print("\r  p        : 현재 위치 출력")
    print("\r  , / .    : 현재 위치를 이 관절의 min / max 로 기록")
    print("\r  q        : 종료 (전부 정지 후 토크 해제)")
    print("\r")


def main():
    parser = argparse.ArgumentParser(description="키보드 관절 텔레옵")
    parser.add_argument("--ids", type=int, nargs="+",
                        help="조작할 ID. 생략하면 config/robot.yaml 의 "
                             "motors.active_ids (구조에 연결된 축만)")
    parser.add_argument("--step", type=int, default=20,
                        help="위치축 키 1회당 ticks. 기본 20 (약 1.8도)")
    parser.add_argument("--speed-step", type=int, default=5,
                        help=f"속도축 키 1회당 증분. 최대 속도 {MAX_VELOCITY}")
    parser.add_argument("--max-travel-deg", type=float, default=25.0,
                        help="위치축의 시작 위치 기준 최대 이동 각도. 기본 25도")
    parser.add_argument("--no-travel-limit", action="store_true",
                        help="시작 위치 기준 이동 상한을 없앤다. 기계적 한계를 "
                             "직접 찾을 때 쓴다. EEPROM 위치 한계만 남는다")
    args = parser.parse_args()

    # raw 모드 키 입력이 필요하다. 터미널이 아니면 토크를 걸기 전에 멈춘다.
    if not sys.stdin.isatty():
        sys.exit("터미널에서 실행해야 합니다.\n"
                 "  원격이면: ssh -t ubuntu 'cd ~/posture_robot_proto && "
                 "~/dynamixel-venv/bin/python scripts/teleop_keyboard.py'")

    install_signal_guards()

    config = load_robot_config()
    ids = args.ids or active_ids(config)
    if not ids:
        sys.exit("조작할 ID 가 없습니다. --ids 로 지정하세요.")
    if len(ids) > len(SELECT_KEYS):
        sys.exit(f"ID 는 최대 {len(SELECT_KEYS)} 개까지 지원합니다.")

    step = args.step
    saved_terminal = None

    try:
        with open_bus(config) as (packet, port):
            present = ping_all(packet, port)
            missing = [i for i in ids if i not in present]
            if missing:
                sys.exit(f"ID {missing} 가 버스에 없습니다. 응답: {sorted(present)}")

            modes, limits, positions, origins = {}, {}, {}, {}
            marks = {}

            # 운용 한계는 config/joints.yaml 이 단일 소스다.
            operational = {}
            for name, spec in load_joints().items():
                low, high = spec.get("min_position"), spec.get("max_position")
                if spec.get("id") in ids and low is not None and high is not None:
                    operational[spec["id"]] = (low, high, name)

            # 접힌 자세에서 토크를 켜면 아래 단이 즉시 과부하로 죽는다.
            # 실제로 base_pitch 가 이 방식으로 한 번 고장났다.
            folded = []
            for motor_id, (low, high, name) in operational.items():
                position = read4(packet, port, motor_id,
                                 ADDR_PRESENT_POSITION, f"ID {motor_id} 현재 위치")
                if not low <= position <= high:
                    excess = position - high if position > high else position - low
                    folded.append(
                        f"  {name} (ID {motor_id}) 현재 {position}, "
                        f"운용 범위 {low}~{high} 에서 {excess:+d} ticks "
                        f"({ticks_to_degrees(excess):+.1f}도) 벗어남")
            if folded:
                sys.exit("현재 자세가 운용 범위를 벗어나 있습니다.\n"
                         + "\n".join(folded)
                         + "\n\n토크가 꺼진 상태이니 손으로 컬럼을 세워 "
                           "각 관절을 범위 안으로 되돌린 뒤 다시 실행하세요.\n"
                           "이대로 토크를 걸면 아래 단이 과부하로 고장납니다.")
            speeds = {}
            saved_profile = {}
            engaged = []

            for motor_id in ids:
                hw = read_hardware_error(packet, port, motor_id)
                if hw:
                    sys.exit(
                        f"ID {motor_id} 하드웨어 에러 0x{hw:02X} "
                        f"({describe_hardware_error(hw)}). 중단합니다.\n"
                        "  에러는 래치되어 재부팅 전까지 풀리지 않습니다:\n"
                        "    ~/dynamixel-venv/bin/python scripts/torque_off.py "
                        "--clear-errors\n"
                        "  과부하라면 재부팅 전에 자세를 손으로 중립에 가깝게 "
                        "되돌려 부하를 덜어주세요."
                    )

                mode = read1(packet, port, motor_id, ADDR_OPERATING_MODE, "동작 모드")
                if mode not in (MODE_POSITION, MODE_VELOCITY):
                    sys.exit(f"ID {motor_id} 는 지원하지 않는 모드입니다 ({mode}).")
                modes[motor_id] = mode

                saved_profile[motor_id] = (
                    read4(packet, port, motor_id, ADDR_PROFILE_VELOCITY, "Velocity"),
                    read4(packet, port, motor_id, ADDR_PROFILE_ACCEL, "Accel"),
                )

                if mode == MODE_POSITION:
                    low = read4(packet, port, motor_id, ADDR_MIN_POSITION, "Min Position")
                    high = read4(packet, port, motor_id, ADDR_MAX_POSITION, "Max Position")
                    start = read4(packet, port, motor_id,
                                  ADDR_PRESENT_POSITION, "현재 위치")
                    positions[motor_id] = start
                    origins[motor_id] = start
                    bound_low, bound_high = low + EDGE_MARGIN, high - EDGE_MARGIN
                    if motor_id in operational:
                        op_low, op_high, _ = operational[motor_id]
                        bound_low = max(bound_low, op_low)
                        bound_high = min(bound_high, op_high)
                    if args.no_travel_limit:
                        limits[motor_id] = (bound_low, bound_high)
                    else:
                        travel = degrees_to_ticks(args.max_travel_deg)
                        limits[motor_id] = (
                            max(bound_low, start - travel),
                            min(bound_high, start + travel),
                        )
                else:
                    speeds[motor_id] = 0

            try:
                # 이전 실행이 비정상 종료되면 워치독이 발동된 채 남는다. 그 상태에서는
                # Profile 레지스터 쓰기가 out-of-range 로 거부되므로 가장 먼저 푼다.
                for motor_id in ids:
                    watchdog = read1(packet, port, motor_id, ADDR_BUS_WATCHDOG,
                                     f"ID {motor_id} 워치독 확인")
                    if watchdog:
                        write1(packet, port, motor_id, ADDR_BUS_WATCHDOG, 0,
                               f"ID {motor_id} 워치독 해제")
                        if watchdog == 255:
                            print(f"ID {motor_id} 워치독이 발동 상태였습니다. 해제했습니다.")

                for motor_id in ids:
                    write4(packet, port, motor_id, ADDR_PROFILE_ACCEL,
                           SAFE_PROFILE_ACCEL, f"ID {motor_id} Profile Acceleration")
                    if modes[motor_id] == MODE_POSITION:
                        write4(packet, port, motor_id, ADDR_PROFILE_VELOCITY,
                               SAFE_PROFILE_VELOCITY, f"ID {motor_id} Profile Velocity")
                    else:
                        # 속도축은 위치 한계가 없다. 통신이 끊기면 스스로 멈추게 한다.
                        write1(packet, port, motor_id, ADDR_BUS_WATCHDOG,
                               WATCHDOG_UNITS, f"ID {motor_id} 워치독 설정")
                    write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 1,
                           f"ID {motor_id} 토크 인가")
                    engaged.append(motor_id)

                saved_terminal = termios.tcgetattr(sys.stdin.fileno())
                tty.setraw(sys.stdin.fileno())

                active = ids[0]
                print_help(ids, modes, step, args.speed_step)
                if args.no_travel_limit:
                    print("\r이동 상한 해제. config/joints.yaml 의 운용 한계와 "
                          "EEPROM 한계는 그대로 적용됩니다\r")
                print(f"\r선택: ID {active} [{'속도' if modes[active] == MODE_VELOCITY else '위치'}]\r")

                last_keepalive = time.monotonic()
                while True:
                    key = getch_nonblocking(0.05)

                    now = time.monotonic()
                    if now - last_keepalive >= KEEPALIVE_SEC:
                        # 속도가 0 인 축도 반드시 써줘야 한다. 안 그러면 그 축의
                        # 워치독이 발동해 이후 모든 쓰기가 out-of-range 로 거부된다.
                        for motor_id, speed in speeds.items():
                            write4(packet, port, motor_id, ADDR_GOAL_VELOCITY,
                                   speed, f"ID {motor_id} keepalive")
                        last_keepalive = now

                    if key is None:
                        continue
                    if key == "":
                        # stdin 이 닫혔다. 빈 문자열은 어떤 문자열에도 in 이 참이라
                        # 여기서 걸러내지 않으면 아래 키 판정이 오작동한다.
                        break
                    if key in ("q", "\x03"):
                        break

                    if key in SELECT_KEYS[:len(ids)]:
                        active = ids[SELECT_KEYS.index(key)]
                        kind = "속도" if modes[active] == MODE_VELOCITY else "위치"
                        print(f"\r선택: ID {active} [{kind} 제어]\r")
                        continue

                    if key == "x":
                        for motor_id in speeds:
                            speeds[motor_id] = 0
                            write4(packet, port, motor_id, ADDR_GOAL_VELOCITY, 0,
                                   f"ID {motor_id} 정지")
                        print("\r속도축 전부 정지\r")
                        continue

                    if key == "[":
                        step = max(1, step // 2)
                        print(f"\rstep = {step}\r")
                        continue
                    if key == "]":
                        step = min(500, step * 2)
                        print(f"\rstep = {step}\r")
                        continue

                    if key in (",", "."):
                        current = read4(packet, port, active,
                                        ADDR_PRESENT_POSITION, "현재 위치")
                        bound = "min" if key == "," else "max"
                        marks.setdefault(active, {})[bound] = current
                        print(f"\rID {active} {bound} = {current} ticks "
                              f"({ticks_to_degrees(current):.1f}도) 기록\r")
                        continue

                    if key == "p":
                        current = read4(packet, port, active,
                                        ADDR_PRESENT_POSITION, "현재 위치")
                        print(f"\rID {active} 현재 {current} ticks "
                              f"({ticks_to_degrees(current):.1f}도)\r")
                        continue

                    if modes[active] == MODE_VELOCITY:
                        if key == " ":
                            speeds[active] = 0
                        elif key in ("w", "s"):
                            delta = args.speed_step if key == "w" else -args.speed_step
                            speeds[active] = max(-MAX_VELOCITY,
                                                 min(MAX_VELOCITY,
                                                     speeds[active] + delta))
                        else:
                            continue
                        write4(packet, port, active, ADDR_GOAL_VELOCITY,
                               speeds[active], f"ID {active} 속도")
                        print(f"\rID {active} 속도 -> {speeds[active]}\r")
                        continue

                    if key == " ":
                        target = origins[active]
                    elif key in ("w", "s"):
                        target = positions[active] + (step if key == "w" else -step)
                    else:
                        continue

                    low, high = limits[active]
                    clamped = max(low, min(high, target))
                    if clamped != target:
                        print(f"\rID {active} 한계 도달 ({low}~{high}). "
                              f"--max-travel-deg 를 늘리세요\r")
                    positions[active] = clamped
                    write4(packet, port, active, ADDR_GOAL_POSITION, clamped,
                           f"ID {active} 목표 위치")
                    print(f"\rID {active} -> {clamped} ticks "
                          f"({ticks_to_degrees(clamped):.1f}도)\r")
            finally:
                if saved_terminal is not None:
                    termios.tcsetattr(sys.stdin.fileno(),
                                      termios.TCSADRAIN, saved_terminal)
                print("\n정리 중")
                for motor_id in engaged:
                    try:
                        if modes[motor_id] == MODE_VELOCITY:
                            write4(packet, port, motor_id, ADDR_GOAL_VELOCITY, 0,
                                   f"ID {motor_id} 정지")
                            write1(packet, port, motor_id, ADDR_BUS_WATCHDOG, 0,
                                   f"ID {motor_id} 워치독 해제")
                        write1(packet, port, motor_id, ADDR_TORQUE_ENABLE, 0,
                               f"ID {motor_id} 토크 해제")
                        velocity, accel = saved_profile[motor_id]
                        write4(packet, port, motor_id, ADDR_PROFILE_VELOCITY,
                               velocity, f"ID {motor_id} Profile Velocity 복원")
                        write4(packet, port, motor_id, ADDR_PROFILE_ACCEL,
                               accel, f"ID {motor_id} Profile Acceleration 복원")
                    except Exception as exc:
                        print(f"ID {motor_id} 정리 중 오류: {exc}", file=sys.stderr)
                print("전부 정지, 토크 해제 완료")

                if marks:
                    print("\n기록된 가동범위 — config/joints.yaml 에 옮기세요:")
                    for motor_id in sorted(marks):
                        bounds = marks[motor_id]
                        low_mark = bounds.get("min")
                        high_mark = bounds.get("max")
                        print(f"  # ID {motor_id}")
                        print(f"    min_position: {low_mark if low_mark is not None else 'null  # 미기록'}")
                        print(f"    max_position: {high_mark if high_mark is not None else 'null  # 미기록'}")
                        if low_mark is not None and high_mark is not None:
                            span = abs(high_mark - low_mark)
                            middle = (low_mark + high_mark) // 2
                            print(f"    zero_position: {middle}")
                            print(f"    # 범위 {span} ticks ({ticks_to_degrees(span):.1f}도)")
        return 0
    except KeyboardInterrupt as exc:
        print(f"\n중단됨: {exc}")
        return 0
    except BusError as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    sys.exit(main())
