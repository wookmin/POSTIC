"""Gemini API 클라이언트 — 자세 교정 행동 판단.

싱글턴 Client 로 TCP 재사용. 판단 스레드에서 호출된다.
"""

import os
from pathlib import Path
from typing import Literal

from dotenv import load_dotenv
from pydantic import BaseModel, Field
from google import genai
from google.genai import types


PROJECT_ROOT = Path(__file__).resolve().parents[2]
load_dotenv(PROJECT_ROOT / ".env")


class GeminiDecision(BaseModel):
    """Gemini 가 돌려주는 교정 판단."""
    action: Literal["ignore", "gentle_remind", "strong_remind", "praise"] = "gentle_remind"
    behavior: Literal[
        "neutral",
        "mimic_slouch",
        "mimic_forward_head",
        "mimic_tilted_shoulders",
    ] = "neutral"
    speech: str = Field(default="", max_length=100)


_client = None


def _ensure_client():
    global _client
    if _client is None:
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY가 설정되지 않았습니다.")
        _client = genai.Client(api_key=api_key)
    return _client


def decide_posture(posture_label: str, features: dict,
                   urgency: int = 1) -> GeminiDecision:
    """자세 상태와 특징값을 보고 교정 행동을 결정한다.

    Args:
        posture_label: "slouch", "forward_head", "slouch_and_forward", "good"
        features: torso_pitch_deg, neck_pitch_deg, duration_sec 등
        urgency: 1~3, 반복 교정일수록 높아짐
    """
    model_name = os.getenv("GEMINI_MODEL", "gemini-2.5-flash-lite")
    client = _ensure_client()

    urgency_desc = {1: "부드럽게", 2: "분명하게", 3: "강하게"}

    prompt = f"""너는 탁상형 자세 교정 로봇의 행동 판단 모듈이다.

현재 사용자 자세:
- 분류: {posture_label}
- 수치: 상체 기울기 {features.get('torso_pitch_deg', 0):.1f}도, 목 기울기 {features.get('neck_pitch_deg', 0):.1f}도
- 나쁜 자세 지속: {features.get('duration_sec', 0):.0f}초
- 교정 강도: {urgency_desc.get(urgency, '부드럽게')} (레벨 {urgency}/3)
- 이전 교정 후 경과: {features.get('since_last_correction', 999):.0f}초

규칙:
1. action 을 선택한다:
   - ignore: 무시해도 되는 상황 (거의 안 씀)
   - gentle_remind: 부드러운 알림
   - strong_remind: 강한 알림 (반복 교정 시)
   - praise: 자세가 좋아졌을 때 칭찬
2. behavior 는 로봇이 취할 자세:
   - neutral: 바른 자세 시범
   - mimic_slouch: 사용자의 구부린 자세를 과장해 보여줌
   - mimic_forward_head: 거북목을 과장해 보여줌
   - mimic_tilted_shoulders: 어깨 기울임 (현재 미지원)
3. speech 는 짧고 친절한 한국어 안내 (20자 이내 권장).
4. 모터 각도, 속도, 토크값은 절대 생성하지 않는다.
5. 강도가 높을수록 speech 를 단호하게, behavior 를 과장되게 선택한다.
"""

    response = client.models.generate_content(
        model=model_name,
        contents=prompt,
        config=types.GenerateContentConfig(
            response_mime_type="application/json",
            response_schema=GeminiDecision,
        ),
    )

    if not response.text:
        # Gemini 실패 시 규칙 기반 fallback
        return _fallback_decision(posture_label, urgency)

    try:
        return GeminiDecision.model_validate_json(response.text)
    except Exception:
        return _fallback_decision(posture_label, urgency)


def _fallback_decision(posture_label: str, urgency: int) -> GeminiDecision:
    """Gemini 호출 실패 시 규칙 기반 대체 판단."""
    speeches = {
        "slouch": ["등을 펴보세요.", "허리를 세워주세요!", "자세가 많이 굽었어요!"],
        "forward_head": ["턱을 당겨보세요.", "고개를 뒤로!", "거북목 주의!"],
        "slouch_and_forward": ["등과 고개를 함께 펴세요.", "자세를 바로잡아주세요!", "지금 자세가 안 좋아요!"],
    }
    behaviors = {
        "slouch": "mimic_slouch",
        "forward_head": "mimic_forward_head",
        "slouch_and_forward": "mimic_slouch",
    }

    idx = min(urgency - 1, 2)
    return GeminiDecision(
        action="gentle_remind" if urgency <= 1 else "strong_remind",
        behavior=behaviors.get(posture_label, "neutral"),
        speech=speeches.get(posture_label, ["자세를 확인해보세요."])[idx],
    )
