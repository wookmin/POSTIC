# Notifyi 로그 스키마

## 공통 필드

모든 JSONL 행에는 다음 필드가 포함된다.

| 필드 | 의미 |
| --- | --- |
| event | 이벤트 이름 |
| session_id | 세션 식별자 |
| condition | 현재 개입 조건 |
| timestamp | UTC ISO 시각 |
| elapsed_sec | 세션 시작 이후 경과시간 |

## 세션 메타데이터

session_started의 metadata에는 현재 다음 값이 들어간다.

- participant_id
- move_enabled
- posture_trigger_sustain_sec
- response_timeout_sec
- recovery_confirm_sec
- camera_source
- camera_width
- camera_height

## 자세 에피소드 필드

posture_episode_started:

- episode_id
- episode_index
- posture_label
- torso_proxy
- neck_proxy
- lateral_proxy
- confidence

trigger_threshold_reached:

- episode_id
- posture_label
- configured_delay_sec
- observed_bad_duration_sec

posture_episode_ended:

- episode_id
- end_reason
- duration_sec
- observed_duration_sec
- triggered
- observation_complete

## 개입 필드

intervention:

- intervention_id
- intervention_index
- episode_id
- posture
- behavior
- delivery_status
- accepted_outputs

posture_recovered:

- intervention_id
- episode_id
- response_onset_sec
- confirmed_response_time_sec
- confirmation_observed_sec

intervention_ignored 또는 intervention_outcome_incomplete:

- intervention_id
- episode_id
- posture
- behavior
- reason
- response_onset_sec

## 해석 규칙

intervention은 출력 큐 수락을 의미한다. 모터가 실제 목표에 도달했다는 뜻이 아니므로 delivery_status와 하드웨어 오류 로그를 분리해 해석한다.

unknown 구간은 정상 자세 유지시간이나 회복시간에 포함하지 않는다. 관측이 중단된 세션은 observation_complete 값으로 필터링한다.
