# POSTIC

카메라로 본 사용자의 나쁜 자세가 일정 시간 지속될 때만, 로봇이 미리 정한
구부정한 자세를 실행하는 자세 반응 프로토타입입니다. 정상 자세에서는 로봇이
중립 자세를 유지하며 사용자의 자세를 미러링하지 않습니다. 나쁜 자세가
정상으로 돌아오면 중립으로 복귀하고, 이후 다시 나쁜 자세가 지속되면 다시
반응합니다.

## 실행 구조

```text
Camera → PoseEstimator → PostureAngles → posture classifier
                                      ├─ normal/unknown → no motion
                                      └─ bad posture sustained → fixed behavior
                                                            └─ SafetyGate → JointWriter
```

기본 `posture_trigger` 모드에서는 Gemini를 호출하지 않습니다. 로컬 분류기와
지속시간 정책이 이벤트를 만들고, `src/behavior/executor.py`가 자세 라벨을
설정된 고정 포즈로 변환합니다. 모든 모터 목표는
`src/behavior/executor.py`와 `src/safety/gate.py`를 거친 뒤에만 전달됩니다.

## 안전 계층

- `SafetyGate`: 관절 운용 범위, 제어 주기당 변화량, 행동 자세와 지속 시간을 제한
- `HealthMonitor`: 카메라 프레임과 제어 루프 heartbeat stale 상태를 감시
- `ControlLoop.safe_stop()`: 추적을 중지하고 종료 루틴에서 안전한 중립 자세로 복귀
- `IdlePolicy`: 사람 이탈 시 유예 후 중립 복귀, 안정화 뒤 토크 해제
- `BehaviorExecutor`: `posture_trigger`, `voice`, `mirror` 조건을 선택
- `ResponseTracker`: 개입 후 반응시간·무시율·자세 유지시간을 JSONL로 기록

정책은 `config/posture.yaml`의 `correction`, `intervention`, `safety` 섹션에서 관리합니다. 기존
`motion.max_step_deg` 설정도 호환을 위해 fallback으로 지원합니다.

## 동작 확인

```bash
pytest -q
python -m src.main --no-preview --no-correction
python -m src.main --move
```

기본 실행은 dry-run이며 실제 모터를 움직이려면 `--move`가 필요합니다.

실제 모터 실행:

```bash
python -m src.main \
  --no-preview \
  --move \
  --condition posture_trigger
```

## 현재 자세 반응 동작

`posture_trigger` 기본 동작은 다음과 같습니다.

1. 정상 자세에서는 로봇이 중립 자세를 유지합니다.
2. 목·상체·어깨선 중 하나라도 나쁜 상태가 3초 지속되면 반응합니다.
3. 미리 정의된 하나의 과장 포즈를 실행합니다.
4. 나쁜 자세가 유지되는 동안 과장 포즈를 유지합니다.
5. 정상 자세가 확인되면 중립으로 복귀합니다.
6. 다음 나쁜 자세 에피소드가 3초 지속되면 다시 반응합니다.

현재 시연용 고정 포즈는 다음과 같습니다.

```text
몸통 목표: 60도
목 목표: 35도
```

몸통 각도는 4개 pitch 관절에 분배됩니다. 목 50도는 현재 목 관절의
소프트 한계가 ±35도라 실제 동작에는 사용하지 않습니다.

## 모터 매핑

현재 실제 구동 축은 `config/robot.yaml`의 `active_ids`를 기준으로 합니다.

```text
ID 1 → base_pitch
ID 4 → waist_pitch
ID 5 → spine_lower_pitch
ID 8 → spine_upper_pitch, 방향 반전
ID 9 → neck_pitch
```

ID 3을 포함한 나머지 축은 유휴 모터로 취급하며 일반 제어 루프에서 토크를
인가하지 않습니다. ID 8은 실제 조립 방향에 맞춰 `direction: -1`로 설정되어
있습니다.

## 로봇 탑재 카메라 설계

카메라는 별도 외부 장치를 사용하지 않고 로봇에 1대만 탑재하는 방향입니다.
다만 몸통이나 목 관절에 직접 부착하지 않고, 하단 베이스에서 올라오는 고정
마스트에 장착합니다.

```text
카메라
  │
고정 마스트
  │
하단 베이스 ─ 구부러지는 몸통 ─ 목 관절
```

카메라가 로봇 동작과 함께 움직이면 사용자의 자세 변화와 카메라 시점 변화를
구분하기 어렵습니다. 향후 로봇 탑재 카메라를 사용할 때는 다음 상태를 추가할
예정입니다.

```text
OBSERVING → INTERVENING → REACQUIRING → RECOVERING
```

로봇이 움직이는 동안에는 자세 판정을 일시정지하고, 동작이 끝난 뒤 사용자를
다시 찾고 0.5~1초 동안 landmark 품질을 확인한 다음 복구 여부를 판단합니다.

자세 인식 카메라의 상세 안정화 계획은
[`CAMERA_POSITION_STABILIZATION.md`](CAMERA_POSITION_STABILIZATION.md),
YEGAM 전체 기획과 연구 확장안은
[`YEGAM_posture_detection_flow.md`](YEGAM_posture_detection_flow.md)에 정리되어 있습니다.

## 개입 조건 실험

```bash
python -m src.main --no-preview --condition posture_trigger
python -m src.main --no-preview --condition voice
python -m src.main --no-preview --condition mirror  # 기존 미러링 호환 모드
```

세션 로그는 `data/runs/<session_id>.jsonl`에 저장됩니다. 원본 영상·음성·Gemini
대화 전문은 저장하지 않습니다. 실제 음성을 켜려면 `config/posture.yaml`의
`audio.tts.enabled`를 `true`로 바꾸고 장치에 `espeak-ng`를 설치합니다.

현재 기본 실험 조건은 자세 반응이며, 음성 알림과 기존 미러링 모드는 호환용으로
남아 있습니다. 디스플레이를 확보하면
`display` 표현 기능을, 바퀴를 확보하면 `locomotion` 이동 기능을 별도 capability로
추가할 수 있도록 행동 실행 계층을 분리해 두었습니다.

## 검증 상태

- 단위 테스트: `107 passed`
- 기본 동작: 노트북 웹캠 기반
- 모터 제어: Dynamixel 위치 제어 모드
- 안전 기능: 관절 운용 범위, slew limit, 하드웨어 오류, 중립 복귀, 토크 해제
