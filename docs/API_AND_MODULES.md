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

## 자유축 확장 예정 모듈

| 모듈 | 역할 | 현재 상태 |
| --- | --- | --- |
| RobotModel | 관절, 축, 부모-자식 관계, 기구 제한 정의 | 미구현 |
| JointGroup | 목·몸통·표정 등 함께 움직일 관절 그룹 | 미구현 |
| PosePlanner | 의미 기반 목표를 로봇별 목표 포즈로 변환 | 미구현 |
| MotionPrimitive | keyframe, 속도, pause, hold, 복귀를 표현 | 미구현 |
| TrajectoryExecutor | 부드러운 보간과 취소 가능한 실행 | 미구현 |
| InterruptibilityPolicy | 사용자의 작업 상태와 최근 개입 이력을 반영 | 미구현 |

새 모터 배치가 확정되기 전에는 위 모듈을 기존 5축 클래스에 억지로 덧붙이지 않는다. 먼저 로봇 프로파일과 실제 관절 의미를 확정한 뒤, 기존 고정 모션은 baseline primitive로 감싼다.
