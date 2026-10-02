# Notifyi 상태 기계

## 자세 분류 상태

~~~text
unknown <-> good
   |
   +-> bad
~~~

unknown은 사람 미검출, 가림, visibility 부족을 뜻한다. unknown을 good으로 간주하지 않는다.

## 개입 상태

~~~text
good
  |
  | bad 지속 3초
  v
intervening
  |
  | good 관측
  v
returning
  |
  | 중립 도달 + 대기
  v
idle
  |
  | bad 에피소드 재시작
  v
good
~~~

관측이 끊기면 intervening 포즈를 계속 유지하지 않고 취소·복귀 경로로 들어간다. 실제 구현에서 사람 이탈 유예시간은 motion.person_lost_grace_sec, 중립 복귀는 return_to_neutral_sec, 토크 해제 대기는 idle_release_sec가 담당한다.

## 동작별 모터 정책

| 상태 | 목표 | 토크 |
| --- | --- | --- |
| 시작·정상 | 시작 기준 | 유지 |
| bad 대기 | 시작 기준 | 유지 |
| intervening | 고정 과장 포즈 | 유지 |
| returning | 중립 | 유지 |
| idle | 중립 유지 | 해제 |
| safe_stop | 안전 종료 경로 | 종료 처리에 따름 |

## 로그 연결

- bad 시작: posture_episode_started
- 3초 초과: trigger_threshold_reached
- 출력 큐 수락: intervention
- good 후보: posture_recovery_candidate
- good 확정: posture_recovered
- 재발: posture_relapsed
- 관측 중단: observation_lost 또는 intervention_observation_lost
