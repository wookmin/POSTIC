#!/usr/bin/env python3
"""Dynamixel 버스 ID 스캔 (읽기 전용).

토크 인가나 목표위치 쓰기는 하지 않는다. broadcast ping으로 응답하는 ID를
찾고, 각 모터의 EEPROM/RAM 상태를 읽어 표로 출력한다.

사용법:
    source /opt/ros/humble/setup.bash
    python3 scripts/scan_dynamixel.py
    python3 scripts/scan_dynamixel.py --port /dev/ttyUSB0 --baud 57600
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
BAUD_CANDIDATES = (57600, 1000000, 115200, 2000000, 3000000, 9600)

# X 시리즈 프로토콜 2.0 컨트롤 테이블 주소
ADDR = {
    "firmware": (6, 1),
    "baud_code": (8, 1),
    "return_delay": (9, 1),
    "operating_mode": (11, 1),
    "min_position": (52, 4),
    "max_position": (48, 4),
    "torque_enable": (64, 1),
    "hw_error": (70, 1),
    "present_position": (132, 4),
    "voltage": (144, 2),
    "temperature": (146, 1),
}

MODEL_NAMES = {
    1000: "XH430-W350", 1010: "XH430-W210", 1020: "XM430-W350",
    1030: "XM430-W210", 1040: "XH430-V350", 1050: "XH430-V210",
    1060: "XL430-W250", 1070: "XC430-W150", 1080: "XC430-W240",
    1090: "2XL430-W250", 1120: "XM540-W270", 1130: "XM540-W150",
    1160: "2XC430-W250", 1190: "XL330-M077", 1200: "XL330-M288",
    1210: "XC330-T181", 1220: "XC330-T288", 1230: "XC330-M181",
    1240: "XC330-M288", 12: "AX-12A", 18: "AX-18A", 30: "MX-28",
}

BAUD_CODES = {
    0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000,
    5: 3000000, 6: 4000000, 7: 4500000,
}

OPERATING_MODES = {
    0: "전류 제어", 1: "속도 제어", 3: "위치 제어",
    4: "확장 위치(멀티턴)", 5: "전류기반 위치", 16: "PWM",
}

HW_ERROR_BITS = {
    0x01: "입력 전압 오류",
    0x04: "과열 오류",
    0x08: "엔코더 오류",
    0x10: "전기 충격/전원 부족",
    0x20: "과부하 오류",
}


def read_field(packet, port, motor_id, name):
    addr, size = ADDR[name]
    reader = {
        1: packet.read1ByteTxRx,
        2: packet.read2ByteTxRx,
        4: packet.read4ByteTxRx,
    }[size]
    value, comm, err = reader(port, motor_id, addr)
    if comm != COMM_SUCCESS or err:
        return None
    return value


def to_signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value


def ticks_to_degrees(ticks):
    return to_signed32(ticks) * 360.0 / 4096.0


def scan_baud(packet, port, baud):
    if not port.setBaudRate(baud):
        return None
    found, _ = packet.broadcastPing(port)
    return found


def describe(packet, port, motor_id, model_number):
    fields = {name: read_field(packet, port, motor_id, name) for name in ADDR}
    model_name = MODEL_NAMES.get(model_number, f"미확인(model={model_number})")

    hw = fields["hw_error"]
    if hw is None:
        errors = "읽기 실패"
    else:
        causes = [text for bit, text in HW_ERROR_BITS.items() if hw & bit]
        errors = f"0x{hw:02X} ({', '.join(causes)})" if causes else f"0x{hw:02X} (없음)"

    print(f"  ID {motor_id:>3}  {model_name}")
    print(f"        펌웨어        : {fields['firmware']}")
    print(f"        Baud(EEPROM)  : {BAUD_CODES.get(fields['baud_code'], '?')}"
          f" (code={fields['baud_code']})")
    print(f"        Return Delay  : {fields['return_delay']} (x2 us)")
    print(f"        동작 모드     : "
          f"{OPERATING_MODES.get(fields['operating_mode'], fields['operating_mode'])}")
    print(f"        Torque Enable : {fields['torque_enable']}"
          f"{'  ← 토크 인가 상태' if fields['torque_enable'] else ''}")
    print(f"        Hardware Error: {errors}")

    if fields["present_position"] is not None:
        pos = to_signed32(fields["present_position"])
        print(f"        현재 위치     : {pos} ticks "
              f"({ticks_to_degrees(fields['present_position']):.1f}°)")
    if fields["min_position"] is not None and fields["max_position"] is not None:
        print(f"        위치 한계     : {to_signed32(fields['min_position'])} ~ "
              f"{to_signed32(fields['max_position'])} ticks")
    if fields["voltage"] is not None:
        print(f"        전압          : {fields['voltage'] / 10:.1f} V")
    if fields["temperature"] is not None:
        print(f"        온도          : {fields['temperature']} °C")
    print()


def main():
    parser = argparse.ArgumentParser(description="Dynamixel ID 스캔 (읽기 전용)")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, action="append",
                        help="검사할 baudrate. 반복 지정 가능. 생략하면 전체 스윕")
    parser.add_argument("--proto", type=float, default=2.0, choices=[1.0, 2.0])
    args = parser.parse_args()

    bauds = args.baud or list(BAUD_CANDIDATES)

    port = PortHandler(args.port)
    packet = PacketHandler(args.proto)

    if not port.openPort():
        sys.exit(f"{args.port} 포트를 열 수 없습니다. U2D2 연결과 dialout 권한을 확인하세요.")

    total = 0
    try:
        for baud in bauds:
            print(f"[{baud} bps] 스캔 중...", flush=True)
            found = scan_baud(packet, port, baud)
            if found is None:
                print(f"  baudrate {baud} 설정 실패\n")
                continue
            if not found:
                print("  응답 없음\n")
                continue

            print(f"  {len(found)}개 응답\n")
            for motor_id in sorted(found):
                model_number = found[motor_id][0]
                describe(packet, port, motor_id, model_number)
                total += 1
    finally:
        port.closePort()

    if total == 0:
        print("응답한 모터가 없습니다. 전원(SMPS), 데이지체인 배선, U2D2 연결을 확인하세요.")
        return 1
    print(f"총 {total}개 모터 확인.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
