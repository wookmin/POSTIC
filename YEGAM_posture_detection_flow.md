# YEGAM 탁상용 로봇의 재캘리브레이션 최소화 자세 판독 플로우

> 목적: 로봇이 노트북 옆 어디에 놓이든, 사용자가 로봇의 위치를 옮기더라도 별도의 사용자 캘리브레이션 없이 상체 자세 붕괴를 안정적으로 판독하고, 그 결과를 로봇의 잎·가지·몸통 등의 상태 변화로 연결하기 위한 설계안.

## 1. 문제 정의 및 제약조건

YEGAM은 사용자의 작업 중 자세 상태를 지속적으로 관찰하고, 일정 시간 동안 자세가 무너진 상태가 유지되면 로봇의 잎·가지·몸통 등의 형태를 변화시키는 탁상용 로봇을 목표로 한다.

판독 대상은 한 프레임의 자세가 아니라 다음과 같은 시간적 상태다.

- 정상적으로 앉아 있음
- 머리가 앞으로 숙여짐
- 상체가 앞으로 굽음
- 상체가 좌우로 기울어짐
- 여러 신호가 동시에 나빠져 자세가 붕괴됨
- 사람 또는 주요 관절이 가려져 판정할 수 없음
- 로봇이 이동 중이어서 카메라 관측이 불안정함

### 제약조건

1. 로봇은 노트북 옆의 임의 위치에 놓일 수 있다.
2. 사용자는 로봇을 책상 위에서 자유롭게 옮길 수 있다.
3. 카메라가 사용자 정면에 있다는 보장이 없다.
4. 카메라의 좌우 위치, 높이, 거리, 약간의 회전이 달라질 수 있다.
5. 로봇을 옮길 때마다 사용자가 특정 자세를 취하는 방식의 재캘리브레이션은 피한다.
6. 실시간에 가까운 반응이 필요하지만, 한두 프레임의 오검출에 로봇이 즉시 반응해서는 안 된다.
7. 결과는 의료적 진단이 아니라 작업 중 자세 상태에 대한 상호작용 신호로 해석한다.

### 설계 목표

모든 카메라 위치에서 완벽한 3D 자세를 복원하는 것보다 다음의 조합이 현실적이다.

- 카메라 위치 변화에 덜 민감한 신체 표현
- 관측이 불안정할 때 억지로 분류하지 않는 `Unknown` 상태
- 자세가 일정 시간 유지될 때만 상태를 전환하는 temporal logic
- 로봇 이동 중에는 영상 판단을 보류하는 measurement gating
- 실제 사용자 데이터가 적어도 합성 데이터와 소량의 실데이터로 개선 가능한 구조

## 2. 추천 핵심 아키텍처

권장 파이프라인은 다음과 같다.

```text
RGB Camera
    ↓
2D/3D upper-body landmarks + face pose
    ↓
body-centered / view-invariant canonicalization
    ↓
temporal posture classifier
    ↓
confidence gate
    ↓
posture state
    ↓
robot state mapping
```

구체적인 처리 흐름은 다음과 같다.

```text
카메라 프레임
  ├─ 사람/상체 검출
  ├─ 어깨·귀·코·눈·팔꿈치·골반 landmark 추출
  ├─ 얼굴 방향(head pose) 추정
  ├─ landmark 품질·visibility 계산
  └─ 로봇 IMU/encoder 상태 확인
          ↓
    measurement gating
          ↓
  신체 중심 좌표계로 정규화
          ↓
  카메라 시점에 덜 의존하는 canonical representation
          ↓
  짧은 시간 창의 시계열 분류
          ↓
  confidence + persistence + hysteresis
          ↓
  Normal / Warning / Collapsed / Unknown
```

| 단계 | 입력/처리 | 출력 | 목적 |
|---|---|---|---|
| RGB Camera | 로봇에 부착된 RGB 영상 | 프레임 | 별도 환경 센서 없이 시작 |
| Landmark 추출 | 2D 또는 monocular 3D pose | 상체 관절·얼굴 keypoint | 원본 픽셀보다 구조적인 표현 확보 |
| Face pose | 얼굴 yaw/pitch/roll 추정 | 머리 숙임·회전 특징 | forward head와 head-down 보강 |
| Body normalization | 어깨 중심, 어깨 폭, 몸통 축 기준 정규화 | 신체 중심 좌표 | 사람 거리·크기 변화 완화 |
| Canonicalization | 카메라 관점 또는 신체 방향 분리 | view-invariant feature | 좌우·사선 배치의 영향 감소 |
| Temporal classifier | 최근 수 초의 feature sequence | 다중 자세 확률 | 순간 노이즈와 지속적 자세 구분 |
| Confidence gate | visibility·추적 품질·로봇 상태 검사 | accept/reject/unknown | 불확실한 프레임의 오판 방지 |
| Posture state | 확률과 시간 누적 적용 | 상태 전이 | 로봇 표현과 연결 가능한 상태 생성 |

## 3. 왜 단순 MediaPipe 2D 각도 임계값만으로는 부족한가

MediaPipe/BlazePose의 landmark는 MVP의 매우 좋은 출발점이지만, `2D landmark 사이의 각도 하나 → 고정 임계값` 방식은 최종 구조로는 부족하다.

### 3.1 같은 자세도 카메라 위치에 따라 2D 각도가 달라진다

사용자가 같은 자세를 유지해도 카메라가 정면, 좌측 사선, 우측 사선에 놓이면 다음 값이 달라진다.

- 귀–어깨–골반의 2D 각도
- 어깨선의 기울기
- 코와 어깨의 화면상 거리
- 좌우 어깨의 상대적 길이
- 머리와 몸통이 겹쳐 보이는 정도

따라서 “귀와 어깨의 화면상 각도가 20도보다 크면 구부정” 같은 규칙은 카메라 시점에 종속된다.

### 3.2 깊이 정보가 2D 투영에서 사라진다

앞으로 숙이는 동작은 카메라와 몸의 상대적 방향에 따라 화면상에서는 작은 각도 변화로 보일 수 있다. 반대로 정상 자세에서도 몸이 카메라를 비스듬히 향하면 큰 2D 각도로 보일 수 있다.

### 3.3 사람마다 체형과 기준 자세가 다르다

어깨 폭, 목 길이, 상체 길이, 앉은 높이, 모니터 높이가 다르다. 고정 픽셀 거리나 고정 각도는 사용자 간 편차에 취약하다.

### 3.4 한 프레임의 confidence와 가림을 반영하기 어렵다

손, 머리카락, 안경, 마스크, 노트북 화면, 조명, 옷에 의해 관절이 흔들릴 수 있다. 2D 임계값만 사용하면 관측 실패를 자세 붕괴로 오해하기 쉽다.

### 3.5 시간적 맥락을 표현하지 못한다

잠깐 몸을 앞으로 움직이는 것은 작업 동작일 수 있지만, 수십 초 동안 지속되는 자세 붕괴는 다른 상태다. 최종 판정은 프레임 단위가 아니라 시계열 단위여야 한다.

### 결론

MediaPipe 2D landmark와 규칙 기반 각도는 다음 용도로는 유효하다.

- 초기 프로토타입
- feature sanity check
- 데이터 수집 중 빠른 라벨 보조
- 학습 모델이 고장났을 때의 fallback

그러나 주 판정기는 body-centered normalization, view-invariant representation, temporal classifier, confidence gate를 포함해야 한다.

## 4. 명시적 사용자 calibration 없이 동작시키는 방법

“캘리브레이션이 없다”는 것은 아무 기준도 쓰지 않는다는 뜻이 아니라, 사용자가 별도의 행동을 하지 않아도 영상에서 기준을 추정한다는 뜻으로 정의하는 것이 현실적이다.

### 4.1 Body normalization

각 프레임의 landmark를 다음과 같이 신체 중심 좌표로 바꾼다.

1. 원점: 좌우 어깨 중심 또는 어깨–골반 중심
2. 크기: 어깨 폭 또는 몸통 길이
3. 좌우 기준축: 좌우 어깨를 잇는 축
4. 상하 기준축: 어깨 중심과 골반 중심을 잇는 축
5. 각 관절 좌표: 원점 이동 및 크기 정규화

```text
normalized_joint = (joint - shoulder_center) / torso_or_shoulder_scale
```

이렇게 하면 사용자가 카메라에 가까워지거나 멀어져도 픽셀 크기에 덜 영향을 받는다.

### 4.2 Body-centered canonicalization

어깨선과 몸통 축으로 신체 좌표계를 만들고, 관측된 포즈를 이 좌표계로 회전시킨다. 카메라 기준의 화면 좌우를 그대로 사용하지 않고 사람 기준의 좌우·앞뒤 정보를 추정한다.

단안 영상만으로 앞뒤 방향이 완전히 결정되지 않는 경우가 있으므로 다음 정보를 함께 사용한다.

- 얼굴 yaw/pitch/roll
- 좌우 어깨의 상대적 깊이 또는 3D landmark
- 좌우 landmark의 비대칭 패턴
- 짧은 시계열의 움직임
- 학습된 view-invariant embedding

CanonPose는 여러 시점의 2D 관측에서 underlying 3D pose와 camera rotation을 분리하는 방향을 제시했고, 카메라 보정 파라미터 없이 canonical pose를 학습하는 가능성을 보였다. V-VIPE는 3D pose를 canonical coordinate space의 view-invariant embedding으로 표현하고 2D pose에서 해당 표현을 추정한다. 두 연구는 로봇을 옮겨도 사용자 보정을 반복하지 않는 YEGAM의 방향과 잘 맞는다.

### 4.3 Implicit personalization

명시적으로 “바른 자세를 취해 주세요”라고 요청하지 않고도 사용자가 실제로 안정적으로 앉아 있는 구간을 약한 기준으로 활용한다.

- 초기 며칠 동안 관측되는 high-confidence, low-motion 구간을 수집
- 가장 오래 유지되는 상체 feature 분포를 robust median/MAD 또는 이동 분위수로 요약
- 절대적인 정상 자세가 아니라 개인별 baseline으로 사용
- baseline은 아주 느리게 업데이트
- 이미 경고 상태인 구간은 baseline 업데이트에서 제외
- 변화가 크거나 관측 품질이 낮으면 personalization 중단

예를 들어 사용자의 몸통 굽힘 feature를 `f_t`라고 하면 다음처럼 천천히 갱신할 수 있다.

```text
baseline_t = (1 - α) * baseline_(t-1) + α * f_t
```

단, `α`는 작게 두고 `normal/high-confidence/low-motion` 조건을 만족할 때만 적용한다. “구부정한 상태를 정상으로 학습하는 것”을 막기 위해 업데이트 조건이 중요하다.

### 4.4 기대 가능한 범위

이 구조로 카메라 위치 이동에 대한 민감도는 크게 줄일 수 있다. 다만 단일 RGB 카메라로 완전히 가려진 관절, 화면 밖으로 나간 상체, 심한 역광, 정면과 후면이 완전히 모호한 경우까지 항상 정확하게 복원할 수는 없다. 이때는 분류를 강행하지 않고 `Unknown`으로 보내는 것이 올바르다.

## 5. 로봇 이동 감지와 measurement gating

로봇이 이동하면 카메라 영상의 변화가 사용자 자세 변화인지 로봇 자체의 이동인지 구분하기 어렵다. 따라서 로봇 움직임을 감지하는 별도 gating을 넣는다.

### 5.1 사용할 수 있는 신호

- 로봇 본체 IMU: 가속도, 각속도
- 베이스 회전 encoder
- 잎/가지 구동 모터 encoder
- 이동 명령의 시작·종료 이벤트
- 영상 기반 전역 optical flow: 보조 신호

### 5.2 권장 gating 상태

```text
ROBOT_STABLE
  └─ 측정 허용

ROBOT_MOVING
  └─ 새 자세 판정 보류
  └─ 마지막 확정 상태를 유지하거나 Unknown

SETTLING
  └─ 이동 종료 후 짧은 안정화 시간
  └─ detector 재획득 및 landmark 품질 확인

ROBOT_STABLE
  └─ 측정 재개
```

### 5.3 동작 원칙

1. IMU 각속도 또는 가속도가 임계값을 넘으면 measurement gate를 닫는다.
2. 베이스 encoder가 움직이는 동안에는 자세 상태를 업데이트하지 않는다.
3. 이동 종료 후 일정 시간 동안 프레임을 버리고 사람 검출과 landmark track을 다시 안정화한다.
4. 안정화 후에도 visibility criterion을 만족하지 않으면 `Unknown`으로 둔다.
5. 로봇 이동은 사용자 재캘리브레이션 이벤트가 아니다. tracker를 재획득할 뿐 사용자가 기준 자세를 취할 필요는 없다.

## 6. Visibility criterion과 Unknown 상태

자세 판독 시스템은 `정상/나쁨`만 출력해서는 안 된다. 관측할 수 없는 상황을 분리해야 신뢰성이 올라간다.

### 6.1 Visibility criterion 예시

- 상체 bounding box가 화면 안에 충분히 존재하는가
- 어깨 좌우 중 최소 한 쌍이 안정적으로 추적되는가
- 귀·코·눈 중 얼굴 방향에 필요한 landmark가 충분한가
- 어깨 폭 또는 몸통 길이가 최소 픽셀 크기 이상인가
- landmark confidence 중앙값이 기준 이상인가
- 최근 프레임에서 tracking ID가 안정적인가
- 손·노트북·물체에 의한 가림이 심하지 않은가
- 로봇 IMU/encoder가 안정 상태인가

예시 품질 점수:

```text
q = w1 * landmark_confidence
  + w2 * visible_joint_ratio
  + w3 * track_stability
  + w4 * body_size_score
  + w5 * robot_stability
```

`q`가 낮으면 자세 분류기를 호출하더라도 결과를 확정하지 않는다.

### 6.2 Unknown을 내보내야 하는 상황

- 사용자 얼굴 또는 상체가 화면 밖에 있음
- 주요 관절이 여러 프레임 연속으로 가려짐
- 카메라가 이동 중이거나 이동 직후임
- 조명 변화로 landmark가 불안정함
- 사람 검출이 끊김
- 3D canonicalization의 좌우/앞뒤 방향이 모호함
- 모델 confidence가 정상과 붕괴 사이에서 계속 충돌함

### 6.3 Unknown의 로봇 표현

`Unknown`은 “나쁜 자세”가 아니다. 로봇은 다음처럼 관측 불확실성을 표현하는 것이 좋다.

- 잎을 크게 움직이지 않고 중립 위치 유지
- 작은 호흡성 움직임만 수행
- 사용자에게 즉시 경고하지 않음
- 일정 시간 이상 Unknown이면 카메라 재획득 또는 부드러운 안내 고려

## 7. Synthetic data / Sim2Real 데이터 생성 전략

실제 자세 데이터는 사람 수, 카메라 배치, 조명, 옷, 책상 환경의 다양성을 모두 확보하기 어렵다. 합성 데이터로 view 변화와 자세 변형을 넓게 만들고 소량의 실제 데이터로 보정하는 전략이 적합하다.

### 7.1 SMPL-X 기반 생성

SMPL-X 또는 유사한 3D body model로 다음 자세 파라미터를 샘플링한다.

- 정상 upright sitting
- forward head
- upper-body flexion
- lateral lean left/right
- shoulder rounding
- 팔을 책상 위에 둔 상태
- 키보드·마우스·스마트폰 사용 동작
- 정상과 붕괴 사이의 경계 자세

각 3D pose를 다양한 카메라 위치로 렌더링한다.

- 정면, 좌우 15/30/45/60도
- 높이·거리 변화
- 카메라 roll/pitch/yaw 변화
- 다양한 focal length와 crop
- 노트북·모니터·책상에 의한 부분 가림
- 조명·피부톤·옷·배경 변화

### 7.2 BEDLAM 활용

BEDLAM은 SMPL-X 형식의 ground-truth 3D body를 포함하는 합성 RGB 영상 데이터셋으로, synthetic-only 또는 synthetic-heavy 3D human pose 학습의 근거로 활용할 수 있다. YEGAM에서는 BEDLAM을 그대로 자세 분류 데이터로 쓰기보다 상체 feature extractor의 사전학습, 시점 변화 검증, 도메인 다양성 확보에 활용하는 것이 현실적이다.

### 7.3 PoseAug 활용

PoseAug의 핵심 아이디어처럼 pose augmentation을 단순한 이미지 변형에 한정하지 않고 3D skeleton/pose 공간에서 생성한다.

- 몸통 굽힘 각도 변화
- 머리 pitch 변화
- 좌우 lean 변화
- 어깨 비대칭
- 관절 noise와 landmark dropout
- 시간축에서 천천히 무너지는 transition

특히 `정상 → 서서히 붕괴 → 회복` sequence를 만들면 temporal classifier 학습에 유용하다.

### 7.4 AdaptPose / domain adaptation 활용

AdaptPose 계열의 cross-dataset adaptation 아이디어를 적용해 synthetic pose distribution과 실제 책상 환경의 차이를 줄인다.

- synthetic와 real의 feature distribution 정렬
- 실제 target domain의 unlabeled landmark sequence 활용
- 실제 landmark noise를 synthetic skeleton에 주입
- 실제 카메라에서 자주 나타나는 가림·누락 패턴 재현
- domain-specific batch normalization 또는 작은 adapter layer

전체 대형 모델을 다시 학습하기보다 다음의 작은 모듈부터 adaptation하는 것이 효율적이다.

1. landmark quality/noise model
2. canonicalization adapter
3. temporal posture classifier
4. confidence calibration layer

### 7.5 권장 학습 순서

```text
SMPL-X/BEDLAM 기반 합성 pose pretraining
        ↓
PoseAug로 자세·시간·관절 noise 확장
        ↓
실제 영상에서 추출한 unlabeled landmark noise 주입
        ↓
소량의 실제 labeled sequence fine-tuning
        ↓
사용자별 implicit personalization
```

### 7.6 합성 데이터의 한계

합성 데이터는 실제 사람의 자연스러운 미세 움직임, 옷의 주름, 노트북 가림, landmark detector의 편향을 완벽히 재현하지 못한다. 합성 성능만 보고 실제 성능을 판단하면 안 되고, 실제 환경의 held-out participant와 held-out camera placement에서 검증해야 한다.

## 8. 실제 데이터 수집 프로토콜

### 8.1 촬영 환경

실제 노트북 작업 환경과 유사하게 구성한다.

- 노트북 또는 모니터를 책상 위에 배치
- 로봇 위치를 사용자 기준 좌측·우측·정면 사선에 각각 배치
- 카메라 높이와 거리도 여러 조건으로 변경
- 가능하면 RGB 카메라 1대와 보조 카메라 2대를 동시 촬영
- 보조 카메라는 ground-truth 라벨 확인용이며 제품에는 필수 아님

다각도 동시 촬영은 실제 시스템의 단일 카메라 view를 평가하면서도 라벨러가 3D에 가까운 관점에서 자세 전환을 확인할 수 있게 한다.

### 8.2 참가자와 세션

- 서로 다른 체형·키·헤어스타일·안경·옷 포함
- 참가자별 train/test가 섞이지 않도록 사람 기준 분할
- 참가자별 여러 날 또는 여러 세션 촬영
- 카메라 위치를 세션마다 바꾸되 재캘리브레이션 동작을 요구하지 않음
- 자연스러운 작업 구간과 지시된 자세 구간을 모두 포함

### 8.3 권장 자세 클래스

1. `normal_upright`: 편안한 정상 작업 자세
2. `forward_head`: 머리만 앞으로 빠짐 또는 고개가 오래 숙여짐
3. `forward_flexion`: 상체 전체가 앞으로 굽음
4. `lateral_lean_left`
5. `lateral_lean_right`
6. `mixed_collapse`: 머리 숙임과 상체 굽힘이 동시에 나타남
7. `transition`: 자세를 바꾸는 중
8. `unknown/occluded`: 판정 불가

### 8.4 지시 과제와 자연 과제

지시 과제만 사용하면 모델이 과장된 자세에만 강해진다. 다음을 함께 수집한다.

- 1~2분의 자연스러운 타이핑·읽기·마우스 조작
- 의도적으로 10~30초 동안 서서히 구부정해지는 sequence
- 잠깐 앞으로 손을 뻗는 정상 동작
- 옆 사람 또는 물체를 보는 동작
- 안경 조정, 머리 만지기, 물 마시기
- 노트북 화면에 의해 얼굴 일부가 가려지는 상황
- 로봇 위치를 옮기는 상황

### 8.5 라벨링 방식

프레임 하나씩만 라벨링하지 말고 구간 기반으로 라벨링한다.

- 시작 시점: 자세가 임계 상태에 들어간 시점
- 안정화 시점: 해당 자세가 일정 시간 유지된 시점
- 종료 시점: 정상으로 돌아오기 시작한 시점
- 애매한 구간: `transition` 또는 `unknown`
- 라벨러 2명 이상이 독립적으로 라벨링
- 의견 불일치 구간은 별도 adjudication

가능하면 다음 정보를 함께 저장한다.

```text
participant_id
session_id
camera_placement
robot_placement
lighting_condition
posture_label
transition_interval
visibility_quality
occlusion_type
robot_motion_interval
```

### 8.6 데이터 분할 원칙

가장 중요한 평가는 “새로운 사람 + 새로운 카메라 위치”다.

- participant-disjoint split
- camera-placement-disjoint split
- robot-placement-disjoint split
- session-disjoint split

랜덤 프레임 분할은 같은 사람과 같은 영상의 인접 프레임이 train/test에 함께 들어가 실제 일반화 성능을 과대평가할 수 있다.

## 9. Multi-task 출력과 시간 누적 로직

단일 `good/bad` 출력 대신 해석 가능한 여러 신호를 동시에 예측한다.

### 9.1 출력 예시

```text
forward_head:       0.00 ~ 1.00
upper_body_flexion: 0.00 ~ 1.00
lateral_lean:       -1.00 ~ 1.00   (-: left, +: right)
head_down:          0.00 ~ 1.00
visibility_quality: 0.00 ~ 1.00
model_confidence:   0.00 ~ 1.00
```

추가로 classification head를 둘 수 있다.

```text
posture_class = {
  normal,
  forward_head,
  flexion,
  lateral_left,
  lateral_right,
  mixed_collapse,
  transition,
  unknown
}
```

### 9.2 Feature window

한 프레임만 분류하지 않고 최근 `T`초의 sequence를 사용한다.

- landmark를 10~30 FPS로 입력
- 내부 temporal window는 초기 2~5초 권장
- EMA, 1D temporal convolution, GRU, temporal transformer 중 하나 선택
- 시작은 EMA + 작은 GRU 또는 1D TCN으로 충분

### 9.3 시간 누적 로직 예시

```text
if robot_moving:
    state = previous_confirmed_state 또는 UNKNOWN
elif visibility < threshold:
    state = UNKNOWN
elif confidence < threshold:
    state = UNKNOWN 또는 previous_confirmed_state 유지
else:
    bad_score = weighted_sum(
        forward_head,
        upper_body_flexion,
        abs(lateral_lean),
        head_down
    )

    bad_score = temporal_smooth(bad_score)

    if bad_score > warning_threshold for warning_duration:
        state = WARNING

    if bad_score > collapse_threshold for collapse_duration:
        state = COLLAPSED

    if bad_score < recovery_threshold for recovery_duration:
        state = NORMAL
```

### 9.4 Hysteresis와 persistence

상태 전환에 진입 임계값과 회복 임계값을 다르게 둔다.

- Warning 진입: 높은 점수가 일정 시간 지속
- Normal 복귀: 낮은 점수가 일정 시간 지속
- Unknown 복귀: 관측 품질이 회복된 뒤 일정 수의 연속 프레임 필요
- 한두 프레임의 이상치로 로봇 상태가 바뀌지 않도록 최소 지속시간 설정

초기 실험값의 방향은 다음과 같다.

- 0.5~2초: 프레임 노이즈 제거
- 5~15초: 사용자에게 자세 경고를 표현할 기준
- 20~60초: 자세 붕괴가 지속됨으로 보고 로봇의 큰 형태 변화
- 3~10초: 정상 복귀 후 회복 상태 확인

정확한 시간은 사용성 테스트로 조정한다. 짧을수록 민감하지만 귀찮고, 길수록 조용하지만 피드백이 늦다.

### 9.5 로봇 상태 매핑 예시

| 자세 상태 | 로봇 표현 |
|---|---|
| Normal | 잎이 세워진 중립 상태, 작은 생동감 |
| Warning | 잎이 약간 기울거나 움직임이 줄어듦 |
| Collapsed | 잎이 처지고 가지가 몸통 쪽으로 들어감 |
| Recovering | 사용자가 회복한 뒤 천천히 잎이 펴짐 |
| Unknown | 중립 유지, 작은 호흡성 움직임 |
| Robot moving | 현재 표현 고정 또는 이동 전환 애니메이션 |

## 10. MVP → 고도화 단계별 구현 우선순위

### Phase 0: 측정 가능성 확인

- MediaPipe/BlazePose 기반 upper-body landmark 추출
- 어깨 중심·어깨 폭·몸통 길이 정규화
- forward head, torso flexion, lateral lean의 수동 feature 계산
- 여러 카메라 위치에서 feature plot 비교
- visibility와 tracking confidence 기록

단순 각도 임계값의 한계를 실제 영상으로 확인한다.

### Phase 1: 실시간 규칙 기반 MVP

- 2D/가능하면 3D landmark
- body normalization
- face pose 또는 코·귀·어깨 조합
- EMA와 persistence logic
- Normal / Warning / Unknown 3상태
- 로봇 이동 중 gating
- 로봇 상태 매핑

목적은 최종 정확도보다 end-to-end 상호작용을 검증하는 것이다.

### Phase 2: 데이터 기반 temporal classifier

- 실제 촬영 데이터 수집
- 2~5초 landmark sequence 구성
- GRU, TCN 또는 소형 transformer 학습
- multi-task head 적용
- participant-disjoint / camera-disjoint 평가
- confidence calibration과 Unknown threshold 조정

### Phase 3: view-invariant canonicalization

- body-centered coordinate 개선
- 3D landmark 또는 monocular 3D lifting 도입
- CanonPose/V-VIPE 계열의 canonical representation 아이디어 적용
- 좌우·사선 카메라에서 성능 검증
- camera placement를 학습 분포에서 분리해 일반화 측정

### Phase 4: Sim2Real 확장

- SMPL-X 기반 자세 시퀀스 생성
- BEDLAM 참고 합성 RGB/3D pose 활용
- PoseAug로 자세·시간·관절 noise 확장
- AdaptPose 계열의 target-domain adaptation
- 실제 unlabeled landmark로 detector noise 모델 구축

### Phase 5: implicit personalization과 장기 상호작용

- 사용자의 자연스러운 정상 baseline 추정
- baseline을 천천히 업데이트
- 붕괴 상태는 baseline에 반영하지 않음
- 사용자별 민감도 조정
- 자세 알림 빈도와 로봇 표현의 피로도 평가

### Phase 6: 연구·제품 수준 검증

- 사용자 수와 세션 수 확대
- 새로운 책상, 조명, 옷, 노트북 모델에서 검증
- 위치 변경 전후 상태 유지·재획득 시간 측정
- false warning, missed collapse, unknown rate, recovery latency 측정
- 개인정보·영상 저장 정책 검토

## 11. 리스크와 한계

### 11.1 단안 RGB의 깊이 모호성

앞으로 숙인 자세와 카메라에 가까이 있는 정상 자세가 유사하게 보일 수 있다. 3D lifting이나 시계열로 완화할 수 있지만 완전히 제거할 수는 없다.

### 11.2 카메라 이동이 너무 큰 경우

gating으로 오검출을 줄일 수 있지만, 이동 후 사용자가 화면 밖으로 나가거나 시점이 완전히 달라지면 재획득 시간이 필요하다. 이것은 사용자 calibration이 아니라 tracking re-acquisition으로 설계해야 한다.

### 11.3 개인 baseline의 위험

사용자가 장시간 나쁜 자세로 있던 초기에 이를 정상으로 학습할 수 있다. baseline 업데이트는 high-confidence·low-motion·반복 관측·보수적 속도 제한을 적용해야 한다.

### 11.4 자연스러운 동작과 붕괴의 경계

타이핑, 책 보기, 물건 집기, 옆 보기 등은 잠깐 자세가 나빠 보여도 정상적인 작업 동작일 수 있다. 지속시간과 transition 상태가 중요하다.

### 11.5 landmark detector의 편향

피부색, 옷, 조명, 안경, 머리카락, 체형, 보조기구 등에 따라 품질이 달라질 수 있다. 참가자와 환경을 다양하게 구성하고, 품질이 낮을 때 Unknown을 사용해야 한다.

## 최신 카메라 탑재 및 제어 결정 (2026-09-21)

### 12.1 카메라는 분리하지 않고 로봇 한 대로 처리

YEGAM은 별도의 외부 자세 인식 카메라를 사용하지 않는다. 카메라 1대를 로봇에 탑재하고, 사용자 자세 인식과 로봇 상호작용을 하나의 장치에서 처리한다.

다만 카메라가 로봇의 움직이는 관절에 직접 연결되면 로봇 동작이 카메라 영상에 영향을 준다. 사용자의 자세 변화와 로봇 자체의 시점 변화를 구분할 수 없으므로, 카메라 장착 위치를 다음처럼 제한한다.

```text
카메라
  │
  │  고정 마스트 또는 고정 프레임
  │
하단 베이스 ─ 구부러지는 몸통 ─ 목/표정 관절
```

카메라는 로봇에 장착하지만 몸통·목과 함께 움직이지 않아야 한다. 하단 베이스에 카메라를 너무 낮게 직접 달하면 사용자를 올려다보는 시점이 되어 목·어깨 비율이 왜곡될 수 있으므로, 하단 베이스에서 고정 마스트를 세우고 사용자의 가슴~눈높이 부근에 카메라를 배치한다.

### 12.2 현재 권장 장착 조건

- 카메라와 몸통·목 구동축을 기계적으로 분리
- 카메라 높이: 사용자 상체 중심 또는 눈높이 부근
- 사용자의 머리·어깨·상체가 한 화면에 들어오는 화각
- 카메라가 좌우로 약간 이동해도 사용자를 계속 정면에 가깝게 바라보는 구조
- 카메라 이동이 감지되면 자세 이벤트를 일시정지

카메라가 사람 기준으로 앞뒤로 가까워지거나 멀어지는 것은 어깨 너비 기반 정규화로 어느 정도 대응한다. 좌우 이동으로 시점이 크게 바뀌거나 정면에서 측면으로 전환되는 경우에는 `world landmarks`, 깊이 센서 또는 별도 시점 보정이 필요하다.

### 12.3 로봇 탑재 카메라용 상태 흐름

로봇이 움직이는 순간의 카메라 프레임을 즉시 사용자 자세 판정에 사용하지 않는다.

```text
OBSERVING
  └─ 사용자 자세 측정
  └─ 나쁜 자세가 3초 지속되면 개입 시작

INTERVENING
  └─ 미리 정의된 과장 포즈 실행
  └─ 로봇 자체 움직임으로 인한 프레임은 자세 판정에서 제외

REACQUIRING
  └─ 동작 종료 또는 카메라 안정화 후 사용자 재검출
  └─ 약 0.5~1초 동안 landmark 품질 확인

RECOVERING
  └─ 정상 자세가 여러 프레임 지속되면 중립 복귀
  └─ 관측 불가이면 정상으로 간주하지 않고 안전 상태 유지
```

### 12.4 단일 카메라 피드백 루프 방지

다음과 같은 순환을 피해야 한다.

```text
사용자 나쁜 자세
  → 로봇 구부정 포즈
  → 카메라 시점 변화
  → 사용자 자세가 변한 것으로 오인식
  → 잘못된 복귀 또는 재개입
```

따라서 로봇 동작 중에는 다음 신호를 사용해 measurement gate를 닫는다.

- 모터 목표·현재 위치 변화
- IMU 각속도와 가속도
- 카메라 프레임 전체의 optical flow
- 로봇 동작 시작·종료 이벤트

동작 종료 후에는 사용자를 다시 찾고, `Unknown`이나 추적 실패를 정상 자세로 해석하지 않는다.

### 12.5 현재 POSTIC 프로토타입과의 연결

현재 POSTIC은 노트북 웹캠으로 다음 흐름을 검증한다.

- 정상 자세에서는 로봇을 미러링하지 않고 중립 유지
- 목·상체·어깨 중 하나라도 나쁜 자세가 3초 지속되면 하나의 과장 포즈 호출
- 몸통 60도, 목 35도 범위의 고정 포즈 유지
- 사용자가 정상 자세로 돌아오면 중립 복귀
- 다음 나쁜 자세 에피소드에서 다시 반응

로봇 탑재 카메라로 전환할 때는 먼저 카메라를 고정 마스트에 장착하고, 현재 `posture_trigger` 로직에 `ROBOT_MOVING`과 `REACQUIRING` 상태를 추가한다. 카메라를 목 관절에 직접 장착하는 방식은 IMU·관절 엔코더·시점 보정이 준비된 이후 단계로 미룬다.

### 12.6 구현 우선순위 갱신

1. 하단 베이스 고정 마스트와 카메라 시야 검증
2. 앞뒤 거리 변화에서 normalized feature 안정성 측정
3. 카메라 이동 중 posture event 차단
4. 로봇 동작 종료 후 사용자 재획득 상태 추가
5. 정상 자세 자동 기준값 갱신
6. 필요 시 IMU·encoder 기반 카메라 시점 보정
7. 측면 시점 지원을 위한 world landmark 또는 깊이 센서 검토

### 11.6 프라이버시와 사용자 피로

가능하면 원본 영상을 장기 저장하지 않고 landmark와 상태만 저장한다. 자세 피드백이 지나치게 자주 나오면 로봇이 돌봄 장치가 아니라 감시 장치처럼 느껴질 수 있으므로 알림 빈도와 표현 강도를 제한한다.

### 11.7 연구 주장 범위

이 시스템은 “자세 붕괴를 감지하고 상호작용하는 시스템”이지 통증·근골격계 질환·의학적 위험을 진단하는 시스템이 아니다. 논문과 사용자 인터페이스에서 이 범위를 명확히 구분해야 한다.

## 12. 관련 선행연구 목록

아래 연구들은 YEGAM의 전체 시스템을 그대로 제공한다기보다 각 구성요소의 설계 근거로 참고할 수 있다.

### View-invariant / canonical pose

1. **CanonPose: Self-Supervised Monocular 3D Human Pose Estimation in the Wild**, CVPR 2021<br>
   여러 시점의 2D 관측에서 3D pose와 camera rotation을 분리하고 canonical pose를 학습하는 방향.<br>
   https://openaccess.thecvf.com/content/CVPR2021/html/Wandt_CanonPose_Self-Supervised_Monocular_3D_Human_Pose_Estimation_in_the_Wild_CVPR_2021_paper.html

2. **V-VIPE: Variational View Invariant Pose Embedding**, CVPR Workshop 2024<br>
   3D pose를 canonical coordinate space의 view-invariant embedding으로 표현하는 접근.<br>
   https://openaccess.thecvf.com/content/CVPR2024W/Rhobin/papers/Levy_V-VIPE_Variational_View_Invariant_Pose_Embedding_CVPRW_2024_paper.pdf

### 실시간 landmark와 3D human motion

3. **BlazePose GHUM Holistic: Real-time 3D Human Landmarks and Pose Estimation**<br>
   실시간 human landmark와 pose 추정의 실용적 출발점.<br>
   https://arxiv.org/abs/2206.11678

4. **WHAM: Reconstructing World-grounded Humans with Accurate 3D Motion**, CVPR 2024<br>
   monocular video에서 사람의 3D motion과 전역 움직임을 함께 복원하려는 연구. YEGAM에서는 이동 카메라·시계열·월드 기준 움직임을 이해하는 참고로 사용.<br>
   https://openaccess.thecvf.com/content/CVPR2024/papers/Shin_WHAM_Reconstructing_World-grounded_Humans_with_Accurate_3D_Motion_CVPR_2024_paper.pdf

### Synthetic data와 Sim2Real

5. **BEDLAM: A Synthetic Dataset of Bodies Exhibiting Detailed Lifelike Animated Motion**<br>
   SMPL-X 형식의 ground-truth 3D body를 포함한 합성 RGB 영상 데이터셋.<br>
   https://bedlam.is.tue.mpg.de/

6. **PoseAug: A Differentiable Pose Augmentation Framework for 3D Human Pose Estimation**, CVPR 2021<br>
   pose 공간에서의 학습 가능한 증강과 3D pose 다양화에 대한 참고.<br>
   https://openaccess.thecvf.com/content/CVPR2021/papers/Gong_PoseAug_A_Differentiable_Pose_Augmentation_Framework_for_3D_Human_Pose_CVPR_2021_paper.pdf

7. **AdaptPose: Cross-Dataset Adaptation for 3D Human Pose Estimation by Learnable Motion Generation**<br>
   데이터셋·도메인 차이를 줄이고 제한된 target 정보를 활용하는 방향.<br>
   https://arxiv.org/abs/2112.11593

8. **SMPL-X: A new 3D body model with expressive hands and face**<br>
   사람의 몸·손·얼굴을 포함하는 파라메트릭 3D body model.<br>
   https://smpl-x.is.tue.mpg.de/

### Sitting posture 및 응용 참고

9. **USSP: University Student Sitting Posture Dataset**<br>
   실제 환경에서 세밀한 sitting posture와 head orientation을 다루는 데이터셋 계열의 참고. YEGAM의 학생·사무 작업 환경 데이터 수집 설계와 비교 가능.<br>
   https://pmc.ncbi.nlm.nih.gov/articles/PMC12732004/

## 최종 권장안

YEGAM의 첫 구현은 다음 조합으로 시작하는 것이 가장 안전하다.

```text
BlazePose/MediaPipe 기반 upper-body landmark
  + face pose
  + shoulder/torso body normalization
  + EMA 또는 소형 GRU temporal model
  + visibility/confidence gate
  + IMU/encoder measurement gating
  + Normal / Warning / Collapsed / Unknown 상태
```

그 다음 실제 다각도 데이터를 모아 temporal classifier를 학습하고, 3D lifting·canonicalization·합성 데이터·implicit personalization을 단계적으로 추가한다. 이 순서라면 처음부터 거대한 3D 시스템을 만들지 않고도 로봇의 핵심 경험을 빠르게 검증하면서, 이후 연구적으로도 “카메라 위치 변화에 강한 사용자 보정 없는 자세-로봇 동기화”라는 명확한 주제로 확장할 수 있다.
