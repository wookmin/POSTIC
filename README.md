# POSTIC

카메라로 본 사용자의 자세를 지연 재생하고, 필요할 때 로봇이 사용자의 자세를
미러링하거나 음성으로 알려주는 자세 개입 프로토타입입니다.

## 실행 구조

```text
Camera → PoseEstimator → PostureAngles → PoseBuffer
                                      ├─ ControlLoop (50 Hz)
                                      │    └─ SafetyGate → JointWriter
                                      └─ BehaviorManager → Gemini
                                           └─ BehaviorExecutor → SafetyGate
```

Gemini는 자세 상태를 판단하지만 각도, 속도, 토크 지령을 직접 생성하지 않습니다.
`mirror` 조건에서는 관측된 자세만 사용하고, 모든 모터 목표는
`src/behavior/executor.py`와 `src/safety/gate.py`를 거친 뒤에만 전달됩니다.

## 안전 계층

- `SafetyGate`: 관절 운용 범위, 제어 주기당 변화량, 행동 자세와 지속 시간을 제한
- `HealthMonitor`: 카메라 프레임과 제어 루프 heartbeat stale 상태를 감시
- `ControlLoop.safe_stop()`: 추적을 중지하고 종료 루틴에서 안전한 중립 자세로 복귀
- `IdlePolicy`: 사람 이탈 시 유예 후 중립 복귀, 안정화 뒤 토크 해제
- `BehaviorExecutor`: `voice`, `mirror` 실험 조건을 선택
- `ResponseTracker`: 개입 후 반응시간·무시율·자세 유지시간을 JSONL로 기록

정책은 `config/posture.yaml`의 `safety`와 `behavior` 섹션에서 관리합니다. 기존
`motion.max_step_deg` 설정도 호환을 위해 fallback으로 지원합니다.

## 동작 확인

```bash
pytest -q
python -m src.main --no-preview --no-correction
python -m src.main --move
```

기본 실행은 dry-run이며 실제 모터를 움직이려면 `--move`가 필요합니다.

## 개입 조건 실험

```bash
python -m src.main --no-preview --condition voice
python -m src.main --no-preview --condition mirror
```

세션 로그는 `data/runs/<session_id>.jsonl`에 저장됩니다. 원본 영상·음성·Gemini
대화 전문은 저장하지 않습니다. 실제 음성을 켜려면 `config/posture.yaml`의
`audio.tts.enabled`를 `true`로 바꾸고 장치에 `espeak-ng`를 설치합니다.

현재 실험 조건은 음성 알림과 자세 미러링 두 가지입니다. 디스플레이를 확보하면
`display` 표현 기능을, 바퀴를 확보하면 `locomotion` 이동 기능을 별도 capability로
추가할 수 있도록 행동 실행 계층을 분리해 두었습니다.
