#!/usr/bin/env python3
"""neck_pitch(ID 9) 텔레메트리를 계속 로깅한다. main.py 와 동시에 돌려서
'한동안 잘 돌다가 갑자기' 뜨는 과부하 에러 직전 상황을 잡는 용도.

읽기만 한다. 토크나 목표位置는 건드리지 않는다.

사용법:
    (터미널 A) python -m src.main --no-preview --move --condition posture_trigger
    (터미널 B) python scripts/monitor_neck.py
"""

import csv
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from dynamixel_sdk import PortHandler, PacketHandler
    from src.robot.dynamixel_driver import load_robot_config, load_joints
except ImportError as exc:
    sys.exit(f"{exc}\n  pip install -r requirements.txt 로 패키지를 설치하세요.")

ADDR_HW_ERROR = 70
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_VOLTAGE = 144
ADDR_TEMPERATURE = 146

HW_ERROR_BITS = {
    0x01: "입력 전압 오류",
    0x04: "과열 오류",
    0x08: "엔코더 오류",
    0x10: "전기 충격/전원 부족",
    0x20: "과부하 오류",
}


def describe(value):
    causes = [text for bit, text in HW_ERROR_BITS.items() if value & bit]
    return ", ".join(causes) if causes else "없음"


def main():
    cfg = load_robot_config()
    bus = cfg["bus"]
    joints = load_joints()
    motor_id = joints["neck_pitch"]["id"]

    port = PortHandler(bus["port"])
    packet = PacketHandler(2.0)
    if not port.openPort():
        sys.exit(f"{bus['port']} 을 열 수 없습니다.")
    if not port.setBaudRate(bus["baudrate"]):
        sys.exit("baudrate 설정 실패")

    log_path = Path(__file__).resolve().parents[1] / "data" / "neck_monitor.csv"
    log_path.parent.mkdir(exist_ok=True)
    print(f"ID {motor_id} 모니터링 시작. Ctrl+C 로 종료. 로그: {log_path}")

    with log_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "position", "current_mA", "voltage_V",
                         "temp_C", "hw_error_hex", "hw_error_desc"])
        f.flush()

        last_hw = 0
        try:
            while True:
                pos, c1, _ = packet.read4ByteTxRx(port, motor_id, ADDR_PRESENT_POSITION)
                cur, c2, _ = packet.read2ByteTxRx(port, motor_id, ADDR_PRESENT_CURRENT)
                cur = cur - 65536 if cur >= 32768 else cur
                volt, c3, _ = packet.read2ByteTxRx(port, motor_id, ADDR_VOLTAGE)
                temp, c4, _ = packet.read1ByteTxRx(port, motor_id, ADDR_TEMPERATURE)
                hw, c5, _ = packet.read1ByteTxRx(port, motor_id, ADDR_HW_ERROR)

                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                desc = describe(hw)
                writer.writerow([ts, pos, cur, volt / 10.0 if c3 == 0 else "",
                                 temp, f"0x{hw:02X}", desc])
                f.flush()

                marker = ""
                if hw and not last_hw:
                    marker = "  <<< 에러 발생!"
                last_hw = hw

                print(f"{ts}  pos={pos:5d}  current={cur:5d}mA  "
                     f"volt={volt/10.0:.1f}V  temp={temp}C  "
                     f"hw=0x{hw:02X} ({desc}){marker}", flush=True)

                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n중지.")
        finally:
            port.closePort()


if __name__ == "__main__":
    main()
