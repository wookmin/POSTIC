#!/usr/bin/env python3
"""카메라 · 마이크 · 모터가 한 인터프리터에서 다 되는지 확인한다.

예전에는 mediapipe/vosk 는 venv 에만, dynamixel_sdk 는 ROS python 에만 있어서
자세를 보고 모터를 움직이는 조합이 불가능했다. 이 스크립트는 그 상태로 되돌아가지
않았는지 검증한다.

사용법:
    ~/dynamixel-venv/bin/python scripts/check_env.py
"""

import importlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

MODULES = [
    ("mediapipe", "자세 추정"),
    ("cv2", "카메라 캡처"),
    ("numpy", "수치 연산"),
    ("vosk", "STT"),
    ("sounddevice", "오디오 입출력"),
    ("google.genai", "Gemini"),
    ("pydantic", "스키마 검증"),
    ("dotenv", "환경변수"),
    ("yaml", "설정 로딩"),
    ("dynamixel_sdk", "모터 제어"),
    ("serial", "시리얼"),
]

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def check_modules():
    print("[모듈]")
    missing = []
    for name, purpose in MODULES:
        try:
            loaded = importlib.import_module(name)
            version = str(getattr(loaded, "__version__", "-"))
            print(f"  OK    {name:<16} {version:<12} {purpose}")
        except Exception as exc:
            missing.append(name)
            print(f"  FAIL  {name:<16} {'-':<12} {purpose}  <- {exc}")
    return missing


def check_project_files():
    print("\n[설정 파일]")
    problems = []
    required = [
        PROJECT_ROOT / "config" / "robot.yaml",
        PROJECT_ROOT / "config" / "joints.yaml",
        PROJECT_ROOT / "requirements.txt",
        PROJECT_ROOT / "models" / "pose" / "pose_landmarker_lite.task",
    ]
    for path in required:
        rel = path.relative_to(PROJECT_ROOT)
        if not path.exists():
            problems.append(f"{rel} 없음")
            print(f"  FAIL  {rel}  <- 파일이 없습니다")
        elif path.stat().st_size == 0:
            problems.append(f"{rel} 비어 있음")
            print(f"  WARN  {rel}  <- 0 byte")
        else:
            print(f"  OK    {rel}  ({path.stat().st_size:,} bytes)")

    env = PROJECT_ROOT / ".env"
    if not env.exists():
        problems.append(".env 없음")
        print("  WARN  .env  <- .env.example 을 복사해 채우세요")
    else:
        print("  OK    .env")
    return problems


def check_camera():
    print("\n[카메라]")
    devices = sorted(Path("/dev").glob("video*"))
    if not devices:
        print("  FAIL  /dev/video* 가 없습니다")
        return ["카메라 없음"]
    print(f"  OK    장치: {', '.join(d.name for d in devices)}")

    try:
        import cv2
    except ImportError:
        return ["cv2 없음"]

    capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
    try:
        if not capture.isOpened():
            print("  FAIL  video0 을 열 수 없습니다")
            return ["카메라 열기 실패"]
        ok, frame = capture.read()
        if not ok or frame is None:
            print("  FAIL  프레임을 읽지 못했습니다")
            return ["프레임 읽기 실패"]
        print(f"  OK    프레임 {frame.shape[1]}x{frame.shape[0]} 캡처")
    finally:
        capture.release()
    return []


def check_microphone():
    print("\n[마이크]")
    if shutil.which("arecord") is None:
        print("  FAIL  arecord 가 없습니다 (alsa-utils)")
        return ["arecord 없음"]

    # arecord -l 은 로케일에 따라 "card" / "카드" 로 출력이 달라진다.
    # 목록 파싱에 의존하지 않고 실제 캡처가 되는지로 판정한다.
    listing = subprocess.run(
        ["arecord", "-l"], capture_output=True, text=True,
        env={**os.environ, "LC_ALL": "C"},
    )
    cards = [line.strip() for line in listing.stdout.splitlines()
             if line.lower().startswith("card")]
    for line in cards:
        print(f"  장치  {line}")

    working = []
    for device in ("plughw:0,0", "plughw:0,6", "default"):
        probe = subprocess.run(
            ["arecord", "-q", "-D", device, "-f", "S16_LE",
             "-r", "16000", "-c", "1", "-d", "1", "/dev/null"],
            capture_output=True, text=True,
            env={**os.environ, "LC_ALL": "C"},
        )
        if probe.returncode == 0:
            working.append(device)
            print(f"  OK    {device} 캡처 성공")
        else:
            first = (probe.stderr or "").strip().splitlines()[:1]
            print(f"  WARN  {device} 실패: {first[0] if first else '알 수 없음'}")

    if not working:
        print("  FAIL  캡처 가능한 마이크가 없습니다")
        return ["마이크 캡처 실패"]

    vosk_model = Path.home() / "voice_models" / "vosk-model-small-ko-0.22"
    if vosk_model.exists():
        print(f"  OK    vosk 한국어 모델: {vosk_model}")
    else:
        print(f"  WARN  vosk 모델이 없습니다: {vosk_model}")
    return []


def check_motor_bus():
    print("\n[모터 버스]")
    try:
        from src.robot.dynamixel_driver import load_robot_config, open_bus, ping_all
    except ImportError as exc:
        print(f"  FAIL  드라이버 모듈 로드 실패: {exc}")
        return ["드라이버 로드 실패"]

    config = load_robot_config()
    bus = config["bus"]
    expected = (config.get("motors") or {}).get("ids") or []
    print(f"  설정  {bus['port']} @ {bus['baudrate']} bps, 기대 ID {expected}")

    if not Path(bus["port"]).exists():
        print(f"  WARN  {bus['port']} 가 없습니다 — U2D2 미연결")
        return []

    try:
        with open_bus(config) as (packet, port):
            found = sorted(ping_all(packet, port))
    except Exception as exc:
        print(f"  FAIL  버스 접속 실패: {exc}")
        return ["버스 접속 실패"]

    print(f"  OK    응답 ID {found}")
    if set(found) != set(expected):
        print(f"  WARN  설정과 불일치. 누락 {sorted(set(expected) - set(found))}, "
              f"예상외 {sorted(set(found) - set(expected))}")
    return []


def main():
    print(f"인터프리터: {sys.executable}")
    print(f"파이썬    : {sys.version.split()[0]}\n")

    problems = []
    problems += check_modules()
    problems += check_project_files()
    problems += check_camera()
    problems += check_microphone()
    problems += check_motor_bus()

    print()
    if problems:
        print(f"확인 필요: {len(problems)}건")
        for item in problems:
            print(f"  - {item}")
        return 1
    print("카메라 · 마이크 · 모터가 모두 한 인터프리터에서 동작합니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
