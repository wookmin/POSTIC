# Notifyi 설정 안내

## 설정 파일

| 파일 | 역할 |
| --- | --- |
| config/robot.yaml | 버스 포트, baudrate, 프로토콜, 전체·활성 모터 ID |
| config/joints.yaml | 관절 이름, ID, 방향, zero, 운용·기계적 범위 |
| config/posture.yaml | 카메라, 자세 판정, 개입 포즈, 안전, 로그 |

## config/robot.yaml

- bus.port: 직렬 장치 경로
- bus.baudrate: 현재 1000000
- bus.protocol: 현재 2.0
- motors.ids: 버스에 연결된 전체 ID
- motors.active_ids: 구조에 연결되어 제어할 ID
- motors.unused_ids: 물리적으로 사용하지 않는 축

전체 ID와 active_ids를 혼동하지 않는다. 현재 전체 모터는 10개지만 메인 제어는 5개다.

## config/joints.yaml

관절별 필수 항목:

- id
- direction
- zero_position
- min_position
- max_position
- mechanical_min
- mechanical_max

zero_position이 운용 범위 밖이면 매핑 초기화가 실패해야 한다. direction을 바꾸면 실제 조립 방향과 맞는지 단일 관절로 확인한다.

## config/posture.yaml

### perception

카메라 경로, 모델, 해상도, visibility 기준을 설정한다. 현재는 사용자별 캘리브레이션을 사용하지 않는다.

### angles

자세 proxy의 상한, 중앙값 필터, 평활 계수를 설정한다. 이 값은 사람의 의학적 각도를 의미하지 않는다.

### camera_gimbal

탑재 카메라 보정 여부와 보정 관절을 설정한다. 노트북 웹캠에서는 실행 옵션으로 끈다.

### motion

제어 주기당 변화량, 사람 이탈 유예, 중립 복귀 시간, 토크 해제 대기 시간을 설정한다.

### correction

- sustain_seconds: 나쁜 자세 지속 기준. 현재 3.0초 고정
- max_corrections_per_run: null이면 정상 복귀 후 에피소드마다 재호출
- cooldown_seconds: 현재 0
- check_interval_sec: 판단 스레드 주기

### safety

최종 안전 상한이다. 동작 코드가 이 값을 우회하지 않도록 유지한다.

### experiment

조건, 응답 제한시간, 로그 저장 경로를 설정한다.

### telemetry

회복 후보를 정상 회복으로 확정하는 관찰 시간을 설정한다. 현재 1.0초다.

## 설정 변경 후 확인

~~~bash
pytest -q
python -m src.main --help
git diff --check
~~~

실제 모터를 움직이는 설정 변경은 소프트웨어 테스트만으로 승인하지 않는다. 하드웨어 단일 관절 시험이 필요하다.
