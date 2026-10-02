# ADR 004: OpenCR과 상위 컴퓨터의 역할을 분리함

## 결정

OpenCR은 향후 저수준 모터·안전 경계를 담당하고, Raspberry Pi 또는 노트북은 카메라·인식·정책·로그를 담당한다.

## 현재 상태

현재 코드는 Dynamixel SDK를 통한 직렬 버스 제어를 사용한다. OpenCR용 상위 통신 어댑터는 아직 구현하지 않았다.

## 재검토 조건

전원·배선·통신 방식이 확정되면 OpenCR transport를 JointWriter와 별도 어댑터로 구현한다.
