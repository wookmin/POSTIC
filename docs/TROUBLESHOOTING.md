# Notifyi 문제 해결

## 공통 원칙

모터가 이상하게 움직이거나 과부하가 의심되면 실행을 반복하지 않는다. 먼저 토크를 해제하고 전원·배선·현재 위치·hardware error를 확인한다.

## 카메라를 열 수 없음

확인:

~~~bash
ls -l /dev/video*
v4l2-ctl --list-devices
fuser /dev/video0
~~~

조치:

1. 다른 카메라 프로그램을 종료한다.
2. 실행 명령의 camera 경로를 실제 장치로 바꾼다.
3. 카메라 권한과 USB 연결을 확인한다.
4. 프리뷰 창을 사용하는 환경에서 먼저 dry-run으로 확인한다.

카메라가 노트북에 있는데 /dev/video0이 아니라면 장치 번호가 달라질 수 있다.

## 사람을 인식하지 못함

- 얼굴과 상체가 프레임 안에 있는지 확인한다.
- 카메라가 너무 낮거나 위에 있지 않은지 확인한다.
- 역광과 강한 그림자를 줄인다.
- 카메라 해상도는 우선 640 x 480으로 고정한다.
- 자세가 계속 unknown이면 짐벌이 움직이는 동안 판정하고 있지 않은지 확인한다.

현재 시스템은 unknown을 good이나 bad로 바꾸지 않는다. 인식이 끊긴 상태에서 개입하지 않는 것은 정상 동작이다.

## 모터 ID가 누락됨

~~~bash
python scripts/scan_dynamixel.py \
  --port /dev/ttyUSB0 \
  --baud 1000000
~~~

전원, U2D2 또는 직렬 어댑터, 데이지체인 배선, baudrate를 확인한다. 현재 메인 제어에 필요한 ID는 1, 4, 5, 8, 9다. 전체 스캔에서 10개가 보이더라도 실제 사용축은 5개다.

## ID 9가 응답하지 않거나 과부하임

1. 모터에 토크를 계속 걸지 않는다.
2. 로봇의 하중을 손으로 줄인다.
3. 스캔으로 hardware error와 현재 위치를 확인한다.
4. 오류가 래치되어 있으면 torque_off.py를 사용한다.

~~~bash
python scripts/torque_off.py --ids 9
python scripts/torque_off.py --ids 9 --clear-errors
~~~

오류가 계속되면 전원을 껐다 켜기 전에 기구적 걸림, 케이블, 브라켓 간섭을 확인한다.

## 모터는 응답하지만 움직이지 않음

- 실행 명령에 move가 있는지 확인한다.
- 대상 모터가 위치 제어 모드인지 확인한다.
- ID 8의 방향이 joints.yaml과 실제 조립 방향에 맞는지 확인한다.
- 현재 위치가 운용 범위 안인지 확인한다.
- 모터가 유휴축 ID 2, 3, 6, 7, 10인지 확인한다.
- safe stop이 발생했는지 터미널의 SAFE_STOP 메시지를 확인한다.

## 실행 직후 고꾸라짐

자동 초기화 기능이 현재 메인 경로에 포함되어 있지 않은지 확인한다. 로봇을 손으로 기준 자세에 세운 뒤 스캔하고, dry-run 이후 move를 사용한다. zero_position으로 자동 복구된다고 가정하지 않는다.

## 로그가 없음

로그 경로는 config/posture.yaml의 experiment.log_dir이다. 기본값은 data/runs다.

~~~bash
ls -l data/runs
python scripts/summarize_posture_logs.py \
  --input data/runs \
  --output-dir data/summaries
~~~

로그 디스크 오류는 로봇 제어를 중단시키지 않도록 설계되어 있으므로, 로그 파일 권한도 별도로 확인한다.
