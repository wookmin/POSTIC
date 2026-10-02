# Notifyi 안전 운용 절차

## 실행 전

1. 로봇을 손으로 세운다.
2. 관절과 브라켓의 걸림을 확인한다.
3. 전원과 데이지체인 배선을 확인한다.
4. scan_dynamixel.py로 ID, 모드, 위치, 오류를 읽는다.
5. 사용축이 운용 범위 안인지 확인한다.
6. 먼저 dry-run을 실행한다.
7. 비상 시 전원을 끌 수 있는 위치에 둔다.

## 실제 실행

~~~bash
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --participant-id P01
~~~

처음에는 로봇에서 손을 떼지 않고 짧게 확인한다. 포즈가 예상보다 빠르거나 한 축만 움직이면 즉시 종료한다.

## 즉시 중단 기준

- 모터에서 비정상 소음·진동·열이 발생함
- ID 9 또는 base_pitch에서 과부하가 발생함
- 목표 위치가 급격히 튐
- 케이블이나 브라켓이 당겨짐
- 사람이 프레임 밖으로 나갔는데 포즈가 유지됨
- SAFE_STOP 메시지가 출력됨

## 중단 절차

1. 프로그램을 SIGINT로 종료한다.
2. 토크 해제 메시지를 확인한다.
3. 반응하지 않으면 torque_off.py를 사용한다.
4. 과부하 상태에서는 전원 재인가 전에 하중을 줄인다.
5. 원인과 마지막 스캔 결과를 개인 또는 실험 로그에 기록한다.

## 변경 승인

다음 변경은 코드 리뷰와 단일 관절 하드웨어 시험 후 적용한다.

- zero_position
- min_position, max_position
- direction
- distribution
- max_step_deg
- intervention 포즈
- profile velocity, acceleration

자동 초기화·복구 모드는 이 runbook의 검증이 끝나기 전까지 메인 실행에 넣지 않는다.

## 자유축 로봇의 추가 안전 기준

자유도가 늘면 관절별 범위만으로 충분하지 않다. 다음 제한을 모든 trajectory와 motion primitive에 적용한다.

- 인접 링크 간 self-collision과 외장 끼임
- 카메라 시야를 가리는 자세와 케이블 비틀림
- 무게중심 이동과 테이블 지지면 이탈
- 관절 그룹 동시 구동 시 전류와 온도
- 관절별 속도·가속도·jerk 상한
- 통신 단절 시 모터 전원 차단과 상위 컴퓨터 로그 기록

판단 모듈에서 만든 목표도 같은 SafetyGate를 통과해야 하며, 관절을 직접 쓰는 예외 경로를 만들지 않는다. 표현력이 커질수록 비상정지와 저속 단일 관절 시험을 먼저 통과시킨다.
