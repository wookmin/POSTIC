# Notifyi 모터 한계와 기준 위치

## 용어

- zero_position: 세워진 기준 자세에서 사용할 tick
- min_position, max_position: 코드가 허용하는 운용 범위
- mechanical_min, mechanical_max: 기구적으로 측정한 잠정 범위
- direction: 자세의 양의 방향과 tick 증가 방향의 관계

zero_position은 누워 있는 로봇을 자동으로 세우는 만능 복구점이 아니다.

## 현재 매핑

| 관절 | ID | zero | 운용 범위 | 방향 |
| --- | ---: | ---: | ---: | ---: |
| base_pitch | 1 | 2033 | 1862 - 2204 | + |
| waist_pitch | 4 | 2041 | 1813 - 2269 | + |
| spine_lower_pitch | 5 | 2044 | 1816 - 2272 | + |
| spine_upper_pitch | 8 | 2046 | 1818 - 2274 | - |
| neck_pitch | 9 | 3018 | 2620 - 3416 | + |

각도와 tick 변환은 4096 tick = 360도 기준이다. 실제 목표는 JointMapper와 SafetyGate에서 다시 제한된다.

## 포즈 분배

몸통 목표 60도는 다음 비율로 네 관절에 나뉜다.

| 관절 | 비율 |
| --- | ---: |
| base_pitch | 0.15 |
| waist_pitch | 0.25 |
| spine_lower_pitch | 0.30 |
| spine_upper_pitch | 0.30 |

목 목표는 neck_pitch 하나가 받는다. 현재 intervention 설정은 몸통 60도, 목 35도이며 SafetyGate가 최종 상한을 적용한다.

## 점검 절차

~~~bash
python scripts/scan_dynamixel.py \
  --port /dev/ttyUSB0 \
  --baud 1000000
~~~

스캔 출력의 현재 위치와 모터 자체 위치 한계를 config/joints.yaml의 설정과 비교한다. 위치 제어 모드가 아닌 축에는 현재 목표 위치를 쓰지 않는다.

## 변경 규칙

zero, min, max, direction을 변경하면 다음을 함께 수행한다.

1. 변경 이유를 커밋 메시지 또는 decision 문서에 남긴다.
2. JointMapper와 SafetyGate 테스트를 실행한다.
3. 실제 모터 없이 계산 결과를 확인한다.
4. 토크를 낮춘 단일 관절 시험을 한다.
5. 과부하·기구 간섭·케이블 당김을 확인한다.
