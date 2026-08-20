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
    behavior: Literal[
        "neutral",
        "mimic_slouch",
        "mimic_forward_head",
        "mimic_tilted_shoulders",
    ]
    speech: str = Field(max_length=100)


def decide_posture(posture: str, features: dict) -> GeminiDecision:
    api_key = os.getenv("GEMINI_API_KEY")
    model_name = os.getenv(
        "GEMINI_MODEL",
        "gemini-2.5-flash-lite",
    )

    if not api_key:
        raise RuntimeError("GEMINI_API_KEY가 설정되지 않았습니다.")

    client = genai.Client(api_key=api_key)

    prompt = f"""
너는 탁상형 자세 교정 로봇의 행동 판단 모듈이다.

현재 사용자 자세:
- 분류: {posture}
- 특징값: {features}

허용된 behavior 중 하나만 선택한다.
- neutral
- mimic_slouch
- mimic_forward_head
- mimic_tilted_shoulders

모터 ID, 모터 각도, 속도, 토크값은 절대 생성하지 않는다.
speech는 짧고 친절한 한국어 안내 문장으로 작성한다.
좋은 자세라면 neutral을 선택한다.
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
        raise RuntimeError("Gemini 응답이 비어 있습니다.")

    return GeminiDecision.model_validate_json(response.text)
