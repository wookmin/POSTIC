# Notifyi 로그 분석 가이드

## 생성

~~~bash
python scripts/summarize_posture_logs.py \
  --input data/runs \
  --output-dir data/summaries
~~~

주요 결과:

- episodes.csv: 나쁜 자세 에피소드 단위
- interventions.csv: 로봇 개입 단위

## 기본 지표

### 회복률

interventions.csv에서 outcome이 recovered인 행을 전체 완료 가능한 개입 수로 나눈다. incomplete와 hardware 오류 개입은 별도 보고한다.

### 회복시간

confirmed_response_time_sec를 사용한다. response_onset_sec는 첫 good 후보이므로 확정 회복시간과 구분한다.

### 재발

posture_relapsed 이벤트의 time_since_recovery_sec를 사용한다. 값이 없는 개입은 관측 종료나 세션 종료 여부를 확인한다.

### 개입 빈도

세션별 intervention 수를 세션 시간으로 나눈다. 단순 개입 수만 비교하면 세션 길이 차이를 반영하지 못한다.

### 관측 품질

observation_lost, observation_resumed 이벤트와 episode의 observation_complete를 함께 본다. unknown이 많은 세션은 효과가 낮았다고 바로 결론내리지 않는다.

## 전반·후반 비교

한 세션을 초반과 후반으로 나누어 다음을 비교한다.

- 분당 개입 수
- 평균 confirmed_response_time_sec
- 회복률
- 회복 후 정상 유지시간
- observation_lost 비율

후반 개입이 줄어도 학습 효과, 피로, 실험 인식, 카메라 이동을 분리해야 한다.

## 3초 조건 해석

현재 실험에서는 3초를 고정한다. 3초가 최적이라고 주장하지 않고, 먼저 동일 조건에서 로그가 안정적으로 수집되는지 확인한다.

다음 단계에서 1초·3초·5초를 비교할 때는 참가자, 카메라, 세션 길이, 포즈, 회복 확정시간을 동일하게 유지한다.
