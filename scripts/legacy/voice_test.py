import json
import subprocess
from vosk import Model, KaldiRecognizer, SetLogLevel

SetLogLevel(-1)

MODEL_PATH = "/home/wookmin/voice_models/vosk-model-small-ko-0.22"

COMMANDS = [
    "시작",
    "전체 시작",
    "모두 시작",
    "전부 시작",

    "일번 시작",
    "일 번 시작",
    "첫 번째 시작",
    "1번 시작",

    "이번 시작",
    "이 번 시작",
    "두 번째 시작",
    "2번 시작",

    "일번 반대",
    "일 번 반대",
    "첫 번째 반대",
    "1번 반대",

    "이번 반대",
    "이 번 반대",
    "두 번째 반대",
    "2번 반대",

    "왼쪽",
    "오른쪽",

    "정지",
    "멈춰",
    "멈춰라",
    "그만",

    "종료",
    "[unk]"
]

model = Model(MODEL_PATH)
grammar = json.dumps(COMMANDS, ensure_ascii=False)
recognizer = KaldiRecognizer(model, 16000, grammar)

record = subprocess.Popen(
    [
        "arecord",
        "-q",
        "-D", "plughw:1,0",
        "-f", "S16_LE",
        "-r", "16000",
        "-c", "1",
        "-t", "raw",
        "-"
    ],
    stdout=subprocess.PIPE
)

print("정해진 명령을 짧게 말하세요.")
print("예: 전체 시작 / 1번 시작 / 정지")

try:
    while True:
        data = record.stdout.read(4000)

        if not data:
            break

        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "")

            if text:
                print("인식:", text)

finally:
    record.terminate()
