#!/usr/bin/env python3
"""토크를 끈 채 손으로 움직이며 각 관절의 가동범위를 기록한다 (백드라이브 티칭).

모터로 기계적 스톱까지 밀면 그 순간 부하가 최대로 걸려 과부하 에러가 난다.
토크를 끄고 손으로 움직이면 모터가 힘을 쓰지 않으므로 그 위험이 없고, 5축을
한 번에 훑을 수 있다.

관절을 양쪽 끝까지 천천히 움직이면 최소·최대가 자동으로 갱신된다.
끝나면 Ctrl-C 를 누른다. 접촉 지점에서 --backoff-deg 만큼 안쪽으로 물린
값이 config/joints.yaml 형식으로 출력된다.

주의: 토크가 꺼져 있어 컬럼이 중력으로 처진다. 손으로 받치면서 하세요.

사용법:
    ~/dynamixel-venv/bin/python scripts/teach_limits.py
    ~/dynamixel-venv/bin/python scripts/teach_limits.py --write
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import yaml
    from src.robot.dynamixel_driver import (
        BusError, JOINTS_CONFIG, active_ids, describe_hardware_error,
        install_signal_guards, load_joints, open_bus, ping_all,
        read_hardware_error,
    )
except ImportError as exc:
    sys.exit(f"{exc}\n  ~/dynamixel-venv/bin/python 으로 실행하세요.")

ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132

TICKS_PER_REV = 4096
SAMPLE_HZ = 20
COMM_SUCCESS = 0


def ticks_to_degrees(ticks):
    return ticks * 360.0 / TICKS_PER_REV


def degrees_to_ticks(degrees):
    return int(round(degrees * TICKS_PER_REV / 360.0))


def read_position(packet, port, motor_id):
    value, comm, _ = packet.read4ByteTxRx(port, motor_id, ADDR_PRESENT_POSITION)
    if comm != COMM_SUCCESS:
        return None
    return value - (1 << 32) if value >= (1 << 31) else value


def joint_names_by_id(joints):
    return {spec["id"]: name for name, spec in joints.items()}


def main():
    parser = argparse.ArgumentParser(description="백드라이브 가동범위 티칭")
    parser.add_argument("--ids", type=int, nargs="+",
                        help="대상 ID. 생략하면 config/robot.yaml 의 active_ids")
    parser.add_argument("--backoff-deg", type=float, default=3.0,
                        help="접촉 지점에서 물러날 여유 각도. 기본 3도")
    parser.add_argument("--write", action="store_true",
                        help="결과를 config/joints.yaml 에 바로 반영한다")
    args = parser.parse_args()

    if args.backoff_deg < 0:
        parser.error("--backoff-deg 는 0 이상이어야 합니다")

    install_signal_guards()
    joints = load_joints()
    names = joint_names_by_id(joints)

    try:
        with open_bus() as (packet, port):
            present = ping_all(packet, port)
            ids = args.ids or active_ids()
            missing = [i for i in ids if i not in present]
            if missing:
                sys.exit(f"ID {missing} 가 버스에 없습니다. 응답: {sorted(present)}")

            faulted = {}
            for motor_id in ids:
                value = read_hardware_error(packet, port, motor_id)
                if value:
                    faulted[motor_id] = value
            if faulted:
                for motor_id, value in faulted.items():
                    print(f"ID {motor_id} 하드웨어 에러 0x{value:02X} "
                          f"({describe_hardware_error(value)})")
                sys.exit("먼저 scripts/torque_off.py --clear-errors 로 해제하세요.")

            # 손으로 움직여야 하므로 토크를 확실히 내린다.
            for motor_id in ids:
                comm, _ = packet.write1ByteTxRx(port, motor_id,
                                                ADDR_TORQUE_ENABLE, 0)
                if comm != COMM_SUCCESS:
                    sys.exit(f"ID {motor_id} 토크 해제 실패")

            tracked = {}
            for motor_id in ids:
                start = read_position(packet, port, motor_id)
                if start is None:
                    sys.exit(f"ID {motor_id} 위치를 읽지 못했습니다.")
                tracked[motor_id] = {"min": start, "max": start, "now": start}

            print("토크를 해제했습니다. 컬럼이 처지니 손으로 받치세요.")
            print("각 관절을 양쪽 끝까지 천천히 움직이세요. 끝나면 Ctrl-C.\n")
            for motor_id in ids:
                label = names.get(motor_id, f"ID {motor_id}")
                print(f"  {label} (ID {motor_id})")
            print()

            interval = 1.0 / SAMPLE_HZ
            printed = False
            try:
                while True:
                    for motor_id in ids:
                        position = read_position(packet, port, motor_id)
                        if position is None:
                            continue
                        entry = tracked[motor_id]
                        entry["now"] = position
                        entry["min"] = min(entry["min"], position)
                        entry["max"] = max(entry["max"], position)

                    if printed:
                        sys.stdout.write(f"\033[{len(ids)}A")
                    for motor_id in ids:
                        entry = tracked[motor_id]
                        span = entry["max"] - entry["min"]
                        label = names.get(motor_id, f"ID {motor_id}")
                        sys.stdout.write(
                            "\033[2K"
                            f"  {label:<20} 현재 {entry['now']:>5}  "
                            f"min {entry['min']:>5}  max {entry['max']:>5}  "
                            f"범위 {ticks_to_degrees(span):>6.1f}도\n")
                    sys.stdout.flush()
                    printed = True
                    time.sleep(interval)
            except KeyboardInterrupt:
                print("\n기록을 마칩니다.\n")

            backoff = degrees_to_ticks(args.backoff_deg)
            results = {}
            for motor_id in ids:
                entry = tracked[motor_id]
                low = entry["min"] + backoff
                high = entry["max"] - backoff
                if low >= high:
                    print(f"ID {motor_id}: 기록된 범위가 백오프({args.backoff_deg}도)"
                          "보다 좁습니다. 더 움직여보세요.")
                    continue
                results[motor_id] = {
                    "min_position": low,
                    "max_position": high,
                    "zero_position": (low + high) // 2,
                }

            if not results:
                print("기록된 관절이 없습니다.")
                return 1

            print(f"백오프 {args.backoff_deg}도 적용 결과:\n")
            for motor_id, spec in results.items():
                label = names.get(motor_id, f"ID {motor_id}")
                span = spec["max_position"] - spec["min_position"]
                print(f"  {label} (ID {motor_id})")
                print(f"    min_position: {spec['min_position']}")
                print(f"    max_position: {spec['max_position']}")
                print(f"    zero_position: {spec['zero_position']}")
                print(f"    # 가동 {ticks_to_degrees(span):.1f}도")

            if not args.write:
                print("\nconfig/joints.yaml 에 반영하려면 --write 를 붙이세요.")
                return 0

            document = yaml.safe_load(JOINTS_CONFIG.read_text(encoding="utf-8"))
            updated = []
            for name, spec in (document.get("joints") or {}).items():
                if spec.get("id") in results:
                    spec.update(results[spec["id"]])
                    updated.append(name)
            JOINTS_CONFIG.write_text(
                yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
                encoding="utf-8")
            print(f"\nconfig/joints.yaml 갱신: {updated}")
            print("주석이 사라졌을 수 있으니 git diff 로 확인하세요.")
            return 0
    except BusError as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    sys.exit(main())
