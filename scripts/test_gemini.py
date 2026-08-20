import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.gemini.client import decide_posture  # noqa: E402


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
