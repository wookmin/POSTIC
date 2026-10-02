# Notifyi 빠른 시작

## 대상

삼성 노트북에서 로봇 카메라 또는 USB 웹캠으로 Notifyi 자세 반응 프로토타입을 실행하는 절차다.

## 1. 프로젝트 준비

~~~bash
cd ~/Desktop/posture_robot_proto
source .venv/bin/activate
git status
git pull --ff-only origin minwook
~~~

원격 저장소 이름이 notifyi라면 마지막 명령의 origin을 notifyi로 바꾼다. 로컬 변경이 있으면 먼저 저장하거나 별도 브랜치로 옮긴다.

## 2. 환경 점검

~~~bash
python --version
python scripts/check_env.py
~~~

check_env.py에서 카메라와 모터 버스가 모두 연결되지 않아도, 현재 테스트 단계에서는 실패 원인을 확인하는 용도로 사용할 수 있다. 실제 모터 구동 전에는 Dynamixel 스캔을 별도로 통과해야 한다.

## 3. 카메라 확인

~~~bash
ls -l /dev/video*
v4l2-ctl --list-devices
~~~

장치 경로를 확인한 뒤 실행 명령의 camera 값을 실제 경로로 바꾼다.

## 4. 모터 읽기 전용 확인

~~~bash
python scripts/scan_dynamixel.py \
  --port /dev/ttyUSB0 \
  --baud 1000000
~~~

ID 1, 4, 5, 8, 9가 응답하고, 사용 모터가 위치 제어 모드이며, hardware error가 0인지 확인한다.

## 5. dry-run

모터에 명령을 쓰지 않고 카메라와 자세 판정만 확인한다.

~~~bash
python -m src.main --no-preview \
  --condition posture_trigger \
  --camera /dev/video0 \
  --no-camera-gimbal
~~~

노트북 웹캠은 로봇과 함께 움직이지 않으므로 no-camera-gimbal을 사용한다.

## 6. 실제 실행

모터가 세워진 상태이고 스캔 결과가 정상일 때만 실행한다.

~~~bash
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --participant-id P01
~~~

로봇 탑재 카메라가 아니라 노트북 웹캠을 사용하면 no-camera-gimbal을 추가한다.

## 7. 종료

프리뷰 모드에서는 q 또는 Esc를 누른다. 헤드리스 모드에서는 SIGINT로 종료한다.

종료 후 토크 해제와 중립 복귀 메시지를 확인한다. 오류가 발생하면 다시 실행하기 전에 TROUBLESHOOTING.md의 안전 절차를 따른다.
