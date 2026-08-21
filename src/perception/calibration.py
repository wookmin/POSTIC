"""바른 자세 기준값의 저장과 로딩.

정면 카메라에서 상체각은 골반을 추정으로 채우기 때문에 사람과 자리마다
일정한 치우침이 생긴다. 그 값을 한 번 재서 빼면 "내 바른 자세 대비 얼마나
굽었는가"가 남는다. 자리나 카메라 위치가 바뀌면 다시 잡아야 한다.
"""

from datetime import datetime, timezone
from pathlib import Path

import yaml

from src.perception.posture_features import PostureReference

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CALIBRATION_PATH = PROJECT_ROOT / "data" / "calibration.yaml"


def save_reference(reference, samples, path=CALIBRATION_PATH):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "torso_pitch_deg": round(reference.torso_pitch_deg, 3),
        "neck_pitch_deg": round(reference.neck_pitch_deg, 3),
        "samples": samples,
        "captured_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    path.write_text(yaml.safe_dump(document, allow_unicode=True,
                                   sort_keys=False), encoding="utf-8")
    return path


def load_reference(path=CALIBRATION_PATH):
    """저장된 기준값. 없으면 None."""
    path = Path(path)
    if not path.exists():
        return None
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if "torso_pitch_deg" not in document or "neck_pitch_deg" not in document:
        return None
    return PostureReference(
        torso_pitch_deg=float(document["torso_pitch_deg"]),
        neck_pitch_deg=float(document["neck_pitch_deg"]),
    )


def average_reference(samples):
    """수집한 각도들의 평균으로 기준값을 만든다."""
    if not samples:
        raise ValueError("표본이 없습니다")
    count = len(samples)
    return PostureReference(
        torso_pitch_deg=sum(s.torso_pitch_deg for s in samples) / count,
        neck_pitch_deg=sum(s.neck_pitch_deg for s in samples) / count,
    )
