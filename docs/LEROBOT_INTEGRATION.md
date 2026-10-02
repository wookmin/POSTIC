# LeRobot 참고와 Notifyi 적용 범위

## 결론

LeRobot의 데이터셋·정책 학습 구조는 향후 참고할 가치가 있지만, 현재 Notifyi의 3초 고정 자세 반응 프로토타입에 전체 스택을 바로 도입할 필요는 없다.

현재 우선순위는 다음과 같다.

1. 결정론적 자세 분류와 3초 트리거 유지
2. 안전한 Dynamixel 출력과 로그
3. 카메라 탑재 후 관측 안정화
4. 반복 시연 데이터를 쌓을 수 있는 공통 데이터 형식
5. 충분한 데이터가 생긴 뒤 학습 정책 검토

## 가져올 수 있는 패턴

### Robot 인터페이스 분리

LeRobot의 하드웨어 통합 방식처럼 카메라·관절·액추에이터를 애플리케이션 정책과 분리한다. Notifyi에서는 이미 JointMapper, JointWriter, SafetyGate가 이 경계의 시작점이다.

향후에는 다음과 같은 어댑터를 분리한다.

| 계층 | Notifyi 현재 | 향후 |
| --- | --- | --- |
| 관측 | MediaPipe 단일 카메라 | RobotCamera 어댑터 |
| 행동 | Dynamixel 직접 writer | OpenCR 또는 Raspberry Pi 어댑터 |
| 정책 | 3초 고정 규칙 | 데이터 기반 정책 선택 |
| 기록 | JSONL 이벤트 | LeRobot 호환 episode export |

### 데이터셋 구조

각 시점에 관측 이미지, 자세 feature, 분류 상태, 로봇 목표, 실제 출력 결과, 세션 메타데이터를 연결하면 나중에 imitation learning 데이터로 변환하기 쉽다. 지금은 영상 원본을 무조건 저장하지 않고, privacy와 저장 공간을 고려해 feature와 이벤트 로그를 우선 저장한다.

### OpenCR와 상위 컴퓨터의 역할

- OpenCR: 저수준 모터 제어, 전원·토크·즉시 정지 같은 안전 경계
- Raspberry Pi 또는 노트북: 카메라, 자세 인식, 정책, 로그, 네트워크
- Notifyi 앱: 행동 요청만 전달하고 모터 버스에 직접 접근하지 않음

하드웨어가 확정되면 OpenCR용 통신 어댑터를 추가하는 방식이 현재 Python Dynamixel writer를 전체 교체하는 것보다 안전하다.

## 지금 도입하지 않을 것

- ACT·SmolVLA를 이용한 학습 정책
- 모바일 로봇용 LeKiwi 구조
- 데이터셋 학습을 통한 모터 목표 생성
- 안전 검사를 우회하는 end-to-end 정책
- 음성·디스플레이·바퀴를 한 번에 묶는 통합 런타임

학습 정책이 추가되더라도 출력은 반드시 SafetyGate와 관절별 운용 한계를 통과해야 한다. 학습 모델이 직접 Dynamixel packet을 만드는 구조는 사용하지 않는다.

## 참고 링크

- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [Custom hardware integration](https://huggingface.co/docs/lerobot/integrate_hardware)
- [LeRobot dataset v3](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)
- [ACT policy](https://huggingface.co/docs/lerobot/act)
- [SmolVLA policy](https://huggingface.co/docs/lerobot/smolvla)
- [LeKiwi mobile robot](https://huggingface.co/docs/lerobot/lekiwi)
