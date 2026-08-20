import json

from gemini.client import decide_posture


features = {
    "torso_lean_deg": 18,
    "head_forward_ratio": 0.08,
    "shoulder_tilt_deg": 2,
}

decision = decide_posture("slouch", features)

print(json.dumps(
    decision.model_dump(),
    ensure_ascii=False,
    indent=2,
))
