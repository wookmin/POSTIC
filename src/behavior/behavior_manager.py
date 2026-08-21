"""행동 관리자 — Gemini 판단 결과를 로봇 동작과 TTS 로 실행한다.

TODO: 실시간 루프에서 비동기로 Gemini 를 호출하고, 결과를 제어 스레드와
TTS 에 분배하는 오케스트레이터 구현 예정.

예상 흐름:
  자세 분류 (주기적) → Gemini 비동기 호출 → BehaviorCommand 수신
  → JointWriter 에 행동 목표 전달 + TTS 로 안내 음성 출력
"""
