# Notifyi 모듈 구조

## 실행 진입점

src/main.py가 카메라, 인식, 행동, 안전, 모터, 로그를 조합한다. ControlLoop는 최신 자세를 읽고 목표를 계산해 모터에 출력한다.

## 주요 모듈

| 모듈 | 책임 | 하드웨어 접근 |
| --- | --- | --- |
| camera_stream | 카메라 프레임 수집 | 카메라 |
| pose_estimator | MediaPipe 랜드마크와 프리뷰 | 없음 |
| posture_features | 랜드마크를 자세 proxy로 변환 | 없음 |
| classifier | good, bad, unknown 분류 | 없음 |
| policy | 나쁜 자세 지속시간과 재트리거 정책 | 없음 |
| behavior_manager | 개입 이벤트 생성 | 없음 |
| executor | 이벤트를 고정 포즈로 변환 | 없음 |
| joint_mapper | 포즈를 관절 tick으로 변환 | 없음 |
| safety_gate | 범위·속도·지속시간 제한 | 없음 |
| joint_writer | Dynamixel 토크·목표·현재 위치 IO | 모터 |
| control_loop | 제어 주기와 상태 전환 | 모터 |
| telemetry | JSONL 이벤트 기록과 요약 | 파일 |

## 데이터 경계

판단 모듈은 모터 packet을 만들지 않는다. Executor가 만드는 것은 PostureAngles 기반의 행동이고, JointMapper와 SafetyGate가 이를 tick으로 제한한 뒤 JointWriter가 전송한다.

## 확장 규칙

새 출력 장치를 추가할 때는 다음 경계를 유지한다.

1. 행동 요청 타입을 정의한다.
2. 장치별 executor 또는 adapter를 만든다.
3. 실행 시작·완료·실패·취소를 반환한다.
4. 안전 게이트를 통과한 결과만 출력한다.
5. 이벤트 로그에 요청과 실제 출력 결과를 연결한다.
