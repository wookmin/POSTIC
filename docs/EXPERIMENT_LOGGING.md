# Notifyi 실험 로그와 효과 평가

## 1. 목적

이 문서는 설문 응답만으로 판단하기 어려운 즉각적인 행동 변화를 이벤트 로그로 측정하기 위한 기준이다. 현재 로봇 개입 시간은 실험 간 비교를 위해 3초로 고정한다.

- 나쁜 자세 지속 기준: 3초
- 고정 과장 포즈: 정상 자세로 돌아올 때까지 유지
- 회복 확정: good 관측이 1초 연속 유지
- 동작 시작점: 모터 명령이 출력 큐에 수락된 시각
- 설문: 로그를 대체하지 않고 인지·방해감·인상에 대한 보조 자료

## 2. 이벤트

| 이벤트 | 의미 |
| --- | --- |
| session_started | 실행 세션 시작 |
| posture_episode_started | 나쁜 자세 에피소드 시작 |
| trigger_threshold_reached | 나쁜 자세가 설정된 3초를 넘김 |
| intervention | 개입 이벤트가 출력 큐에 수락됨 |
| intervention_skipped | 조건·최신성·안전 문제로 개입하지 않음 |
| posture_recovery_candidate | 정상 자세 후보가 처음 관측됨 |
| posture_recovered | 정상 자세가 확인 시간만큼 유지됨 |
| posture_relapsed | 회복 뒤 다시 나쁜 자세가 됨 |
| posture_maintenance | 회복 뒤 정상 자세가 유지된 시간 |
| observation_lost | 사람 미검출·가림·신뢰도 부족 |
| observation_resumed | 관측 재개 |
| intervention_ignored | 제한 시간 안에 회복되지 않음 |
| intervention_outcome_incomplete | 관측 중단 또는 세션 종료로 미완료 |
| safe_stop | 안전 정지 발생 |
| session_finished | 세션 종료 |

모든 개입과 자세 에피소드는 episode_id, intervention_id, intervention_index로 연결한다.

## 3. 핵심 지표

### 즉시 효과

- confirmed_response_time_sec: 개입 시작부터 정상 자세가 확인될 때까지
- response_onset_sec: 첫 good 분류까지 걸린 시간
- recovery_rate: 제한 시간 안에 회복된 개입 비율
- ignored_rate: 제한 시간 안에 회복되지 않은 비율

### 반복 효과

- 세션별 개입 횟수
- 후반부 개입 빈도와 초반부 개입 빈도
- 회복 후 다시 나쁜 자세가 되기까지의 시간
- 회복 후 정상 자세 유지시간
- 관측 가능 시간 대비 나쁜 자세 시간

### 전달 신뢰성

- 모터 큐 수락 여부
- dry-run과 실제 --move 구분
- unknown 시간과 개입 판단 중단 횟수
- safe_stop 횟수와 원인

자유축·표현 동작을 도입할 때는 다음 필드도 기록한다.

- robot_profile
- motion_profile
- intervention_index
- time_since_last_intervention_sec
- cooldown_level
- suppression_reason
- user_activity_proxy
- trajectory_started
- trajectory_completed
- trajectory_cancelled
- actual_joint_error

주의할 점은 intervention이 물리적으로 사람이 로봇을 보고 반응했다는 뜻이 아니라, 현재 코드에서 출력 큐가 명령을 수락했다는 뜻이라는 것이다. 실제 모터 도달과 하드웨어 오류는 별도 텔레메트리로 보강해야 한다.

## 4. 로그 사용법

실행 시 익명 참가자 ID만 전달한다.

~~~bash
python -m src.main --no-preview --move \
  --condition posture_trigger \
  --camera /dev/video0 \
  --participant-id P01
~~~

로그는 설정의 experiment.log_dir 아래에 JSONL로 저장된다. 세션 종료 후 요약 CSV를 만든다.

~~~bash
python scripts/summarize_posture_logs.py \
  --input data/runs \
  --output-dir data/summaries
~~~

실험 로그에는 이름, 이메일, 영상 원본을 저장하지 않는다. 참가자 ID는 P01처럼 익명 코드로 관리하고, 설정 버전과 카메라 배치 조건은 메타데이터로 남긴다.

## 5. 해석상의 한계

- 정상 자세 분류가 실제 자세 회복을 완전히 의미하지 않을 수 있다.
- 단일 RGB 카메라는 깊이와 가림에 취약하다.
- 카메라가 로봇과 함께 움직이면 관측 품질과 개입 효과가 섞인다.
- 같은 참가자의 후반부 개입 감소는 학습 효과일 수도 있고, 피로·흥미 저하일 수도 있다.
- 한 조건만 반복하면 3초가 최적인지 비교할 수 없다.

따라서 현재 단계에서는 3초를 고정하고, 로그로 효과 측정 구조를 먼저 검증한다. 1초·3초·5초 비교는 이벤트 스키마가 안정된 뒤 별도 실험으로 진행한다.

외부 HRI 연구에서도 사용자의 작업을 언제 방해할 수 있는지를 고려하는 interruptibility가 로봇의 수행과 사회적 평가에 영향을 주는 것으로 보고된다. Notifyi에서는 초기 proxy로 키보드·마우스 활동, 최근 개입을 무시했는지, 직전 개입 후 회복했는지를 기록하고, 나중에 실제 작업 상태 센서로 교체한다.
