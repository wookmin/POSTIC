# Notifyi

탁상용 로봇 Notifyi의 자세 반응 프로토타입이다. 카메라로 사용자의 자세를 관찰하고, 나쁜 자세가 3초 지속될 때만 미리 정의한 과장 포즈를 실행한다.

## 현재 동작

- 정상 자세: 로봇은 시작 시점의 기준 자세를 유지하고 움직이지 않는다.
- 나쁜 자세 3초 지속: 목·상체·어깨선 중 어느 항목이든 나쁘면 하나의 고정 bad_posture 포즈를 실행한다.
- 개입 중: 사용자가 정상 자세로 돌아올 때까지 포즈를 유지한다.
- 정상 복귀: 로봇이 중립으로 돌아온 뒤 토크를 해제한다.
- 재개입: 정상 복귀 후 다시 나쁜 자세가 3초 지속되면 다시 반응한다.
- 로그: 자세 에피소드, 개입, 회복, 관측 중단, 안전 정지를 JSONL로 저장한다.

현재 프로토타입은 사용자 자세를 계속 따라 하는 미러링이 아니다. 정상일 때는 미러링하지 않고, 나쁜 자세에 대해서만 고정된 반응을 보여주는 구조다.

## 현재 범위와 제외 범위

현재 포함:

- 로봇 탑재 USB 카메라 중심의 MediaPipe 자세 인식
- 노트북 웹캠 보조 입력
- Dynamixel 2XL430-W250 5축 pitch 컬럼 제어
- 카메라 탑재를 고려한 neck_pitch 짐벌 보정
- SafetyGate, slew 제한, safe stop
- 실험 효과 평가용 JSONL 이벤트와 CSV 요약

현재 제외:

- 마이크·스피커·음성 안내
- 디스플레이 표정
- 바퀴 이동
- ROS 2 런타임
- Gemini 또는 기타 LLM 판단
- 자동 초기화·복구 모드
- 학습 기반 모터 정책

## 실행

가상환경을 활성화한 뒤 실행한다.

~~~bash
source .venv/bin/activate

# 카메라·인식만 확인하는 dry-run
python -m src.main --no-preview

# 로봇 카메라와 모터를 함께 사용하는 실험
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --participant-id P01

# 노트북 웹캠을 사용할 때는 탑재 카메라 짐벌 보정을 끈다
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --no-camera-gimbal \
  --participant-id P01
~~~

--move가 없으면 모터를 구동하지 않는다. --no-correction은 자세 개입 판단을 끄고 인식과 로그 확인에 사용할 수 있다.

## 하드웨어 매핑

| 관절 | ID | 용도 |
| --- | ---: | --- |
| base_pitch | 1 | 하체·상단 하중 |
| waist_pitch | 4 | 허리 |
| spine_lower_pitch | 5 | 척추 하단 |
| spine_upper_pitch | 8 | 척추 상단, 방향 반전 |
| neck_pitch | 9 | 목·카메라 짐벌 |
| 보조축 | 2, 3, 6, 7, 10 | 현재 미사용 |

자세와 모터의 기준값은 config/joints.yaml과 config/posture.yaml에서 관리한다. zero_position은 세워진 기준 자세의 tick이지, 어떤 누운 상태에서도 자동 복구할 수 있는 만능 원점이 아니다.

모터를 연결하기 전에는 다음을 먼저 확인한다.

~~~bash
python scripts/scan_dynamixel.py --port /dev/ttyUSB0 --baud 1000000
python scripts/check_env.py
~~~

ID 누락, 속도 제어 모드, hardware error, 기계적 범위 이탈이 있으면 --move로 실행하지 않는다.

## 로그와 효과 평가

익명 참가자 ID를 함께 주면 세션 메타데이터에 기록된다.

~~~bash
python scripts/summarize_posture_logs.py \
  --input data/runs \
  --output-dir data/summaries
~~~

로그로 다음을 계산할 수 있다.

- 개입 후 정상 자세 확인까지의 시간
- 회복률과 미완료 비율
- 회복 후 재발까지 걸린 시간
- 세션 전반·후반의 개입 빈도 변화
- 관측 중단과 안전 정지 횟수

자세 로그의 정의와 해석상 한계는 docs/EXPERIMENT_LOGGING.md에 정리했다.

## 문서

- 아키텍처와 현재 하드웨어: docs/ARCHITECTURE.md
- 빠른 시작: docs/QUICKSTART.md
- 명령어 모음: docs/COMMANDS.md
- 문제 해결: docs/TROUBLESHOOTING.md
- 하드웨어 연결: docs/HARDWARE_WIRING.md
- 카메라 장착: docs/CAMERA_MOUNTING.md
- 모터 한계와 기준 위치: docs/MOTOR_LIMITS.md
- 설정 안내: docs/CONFIGURATION.md
- 모듈 구조: docs/API_AND_MODULES.md
- 상태 기계: docs/STATE_MACHINE.md
- 안전 운용 절차: docs/SAFETY_RUNBOOK.md
- 자세 인식과 카메라 배치: docs/PERCEPTION_AND_CAMERA.md
- 실험 프로토콜: docs/EXPERIMENT_PROTOCOL.md
- 로그 기반 효과 평가: docs/EXPERIMENT_LOGGING.md
- 로그 스키마: docs/LOG_SCHEMA.md
- 로그 분석: docs/ANALYSIS_GUIDE.md
- 개인정보 원칙: docs/PRIVACY.md
- 초기화와 복구의 현재 상태: docs/INITIALIZATION_AND_RECOVERY.md
- LeRobot 참고와 적용 범위: docs/LEROBOT_INTEGRATION.md
- 의사결정 기록: docs/decisions/

## 개발 원칙

1. 정상 자세에서는 로봇이 움직이지 않는다.
2. 판단 모듈은 모터 packet을 직접 만들지 않는다.
3. 모든 목표는 SafetyGate와 관절별 운용 범위를 통과한다.
4. 사람 미검출과 카메라 이동은 정상이나 나쁜 자세로 추정하지 않는다.
5. 실제 개입과 출력 큐 수락을 로그에서 구분한다.
6. 새 하드웨어 기능은 현재 자세 반응 루프를 깨지 않도록 별도 행동 어댑터로 추가한다.
