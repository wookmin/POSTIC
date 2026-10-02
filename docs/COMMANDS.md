# Notifyi 명령어 모음

아래 명령은 프로젝트 루트에서 가상환경을 활성화한 뒤 실행한다.

## 환경

~~~bash
source .venv/bin/activate
python --version
python scripts/check_env.py
~~~

## 카메라

~~~bash
ls -l /dev/video*
v4l2-ctl --list-devices
fuser /dev/video0
~~~

현재 scripts/check_camera.py는 비어 있는 placeholder이므로 카메라 확인에는 v4l2 명령과 src.main dry-run을 사용한다.

## Dynamixel 스캔

~~~bash
python scripts/scan_dynamixel.py \
  --port /dev/ttyUSB0 \
  --baud 1000000
~~~

읽기 전용 스캔이며 토크나 목표 위치를 변경하지 않는다.

## 토크 해제와 오류 확인

~~~bash
python scripts/torque_off.py --ids 9
python scripts/torque_off.py --ids 9 --clear-errors
~~~

과부하가 발생한 상태에서 바로 재부팅하지 않는다. 먼저 하중을 줄이고 모터가 움직일 수 있는 상태인지 확인한다.

## Notifyi 실행

~~~bash
# 인식만
python -m src.main --no-preview --no-correction \
  --camera /dev/video0 --no-camera-gimbal

# 모터 포함
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --participant-id P01

# 노트북 웹캠
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --no-camera-gimbal \
  --participant-id P01
~~~

## 테스트와 정적 확인

~~~bash
pytest -q
python -m compileall -q src scripts
git diff --check
~~~

## 로그 요약

~~~bash
python scripts/summarize_posture_logs.py \
  --input data/runs \
  --output-dir data/summaries
~~~

결과:

- data/summaries/episodes.csv
- data/summaries/interventions.csv
