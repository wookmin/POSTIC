import json
import subprocess
import time

from vosk import Model, KaldiRecognizer, SetLogLevel
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

SetLogLevel(-1)

PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL = 2.0
MODEL_PATH = "/home/wookmin/voice_models/vosk-model-small-ko-0.22"

MOTOR_IDS = [1, 2]
SPEED = 50          # 약 3.4 rpm
ACCELERATION = 50
WATCHDOG = 50       # 1초 동안 통신이 없으면 자동 정지

port = PortHandler(PORT)
packet = PacketHandler(PROTOCOL)
current_speed = {1: 0, 2: 0}

def write1(motor_id, address, value):
    result, error = packet.write1ByteTxRx(
        port, motor_id, address, value
    )
    if result != COMM_SUCCESS:
        raise RuntimeError(packet.getTxRxResult(result))
    if error:
        raise RuntimeError(packet.getRxPacketError(error))

def write4(motor_id, address, value):
    result, error = packet.write4ByteTxRx(
        port, motor_id, address, value & 0xFFFFFFFF
    )
    if result != COMM_SUCCESS:
        raise RuntimeError(packet.getTxRxResult(result))
    if error:
        raise RuntimeError(packet.getRxPacketError(error))

def stop_motor(motor_id):
    write4(motor_id, 104, 0)
    write1(motor_id, 64, 0)
    current_speed[motor_id] = 0
    print(f"ID {motor_id} 정지")

def stop_all():
    for motor_id in MOTOR_IDS:
        try:
            stop_motor(motor_id)
        except Exception as e:
            print(f"ID {motor_id} 정지 오류:", e)

def set_speed(motor_id, speed):
    write1(motor_id, 64, 1)
    write4(motor_id, 104, speed)
    current_speed[motor_id] = speed
    print(f"ID {motor_id} 속도 설정: {speed}")

def handle_command(text):
    text = text.replace(" ", "")
    print("인식:", text)

    if "종료" in text:
        stop_all()
        return False

    if any(word in text for word in ["정지", "멈춰", "멈춰라", "그만"]):
        stop_all()
        return True

    if any(word in text for word in ["전체", "모두", "전부"]) and "시작" in text:
        set_speed(1, SPEED)
        set_speed(2, SPEED)
        return True

    is_motor_1 = any(word in text for word in ["일번", "첫번째", "1번"])
    is_motor_2 = any(word in text for word in ["이번", "두번째", "2번"])

    reverse = "반대" in text or "역방향" in text
    start = "시작" in text

    if is_motor_1 and (start or reverse):
        set_speed(1, -SPEED if reverse else SPEED)
        return True

    if is_motor_2 and (start or reverse):
        set_speed(2, -SPEED if reverse else SPEED)
        return True

    return True

if not port.openPort():
    raise SystemExit("U2D2 포트를 열 수 없습니다.")

if not port.setBaudRate(BAUDRATE):
    raise SystemExit("Baudrate 설정에 실패했습니다.")

record = None

try:
    # 토크 OFF 상태에서 속도 제어 모드 설정
    for motor_id in MOTOR_IDS:
        write1(motor_id, 64, 0)
        write1(motor_id, 98, 0)
        write1(motor_id, 11, 1)
        write4(motor_id, 108, ACCELERATION)
        write1(motor_id, 98, WATCHDOG)

    model = Model(MODEL_PATH)

    grammar = json.dumps([
        "전체 시작", "모두 시작", "전부 시작",
        "일번 시작", "일 번 시작", "첫 번째 시작", "1번 시작",
        "이번 시작", "이 번 시작", "두 번째 시작", "2번 시작",
        "일번 반대", "일 번 반대", "첫 번째 반대", "1번 반대",
        "이번 반대", "이 번 반대", "두 번째 반대", "2번 반대",
        "정지", "멈춰", "멈춰라", "그만", "종료",
        "[unk]"
    ], ensure_ascii=False)

    recognizer = KaldiRecognizer(model, 16000, grammar)

    record = subprocess.Popen(
        [
            "arecord", "-q",
            "-D", "plughw:1,0",
            "-f", "S16_LE",
            "-r", "16000",
            "-c", "1",
            "-t", "raw", "-"
        ],
        stdout=subprocess.PIPE
    )

    print("음성 명령 대기 중")
    print("예: 전체 시작 / 1번 시작 / 정지 / 종료")

    last_keepalive = time.monotonic()

    while True:
        data = record.stdout.read(4000)

        now = time.monotonic()
        if now - last_keepalive > 0.25:
            for motor_id, speed in current_speed.items():
                if speed != 0:
                    write4(motor_id, 104, speed)
            last_keepalive = now

        if recognizer.AcceptWaveform(data):
            text = json.loads(recognizer.Result()).get("text", "")

            if text and not handle_command(text):
                break

finally:
    if record is not None:
        record.terminate()

    stop_all()

    for motor_id in MOTOR_IDS:
        try:
            write1(motor_id, 98, 0)
            write1(motor_id, 11, 3)
        except Exception:
            pass

    port.closePort()
