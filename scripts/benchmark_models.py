#!/usr/bin/env python3
"""자세 추정 모델 변형을 같은 조건에서 비교한다.

정확도의 정답은 없으므로 여기서 재는 것은 안정성이다. 가만히 앉아 있을 때
각도가 얼마나 떨리는지(표준편차)와 초당 몇 프레임이 나오는지를 본다.
사람이 움직이면 비교가 무의미하므로 측정 중에는 정지해 있어야 한다.

사용법:
    ~/dynamixel-venv/bin/python scripts/benchmark_models.py
    ~/dynamixel-venv/bin/python scripts/benchmark_models.py --seconds 15
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import yaml  # noqa: E402

from src.camera.camera_stream import CameraStream  # noqa: E402
from src.perception.pose_estimator import (  # noqa: E402
    VARIANTS, PoseEstimator, resolve_model,
)
from src.perception.posture_features import extract_angles  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
POSTURE_CONFIG = PROJECT_ROOT / "config" / "posture.yaml"


def measure(variant, source, seconds, min_visibility):
    angles = []
    inference_ms = []
    frames = 0
    start = time.monotonic()

    with CameraStream(source) as camera, PoseEstimator(variant) as estimator:
        while time.monotonic() - start < seconds:
            frame = camera.read()
            if frame is None:
                continue
            frames += 1
            mark = time.monotonic()
            landmarks, world = estimator.detect(frame, (mark - start) * 1000)
            inference_ms.append((time.monotonic() - mark) * 1000)
            if not landmarks or not world:
                continue
            measured = extract_angles(world, landmarks, mark, min_visibility)
            if measured is not None:
                angles.append(measured)

    elapsed = time.monotonic() - start
    if len(angles) < 20:
        return None
    torso = [a.torso_pitch_deg for a in angles]
    neck = [a.neck_pitch_deg for a in angles]
    return {
        "fps": frames / elapsed,
        "detected": len(angles) / frames if frames else 0.0,
        "inference_ms": statistics.mean(inference_ms),
        "torso_sd": statistics.pstdev(torso),
        "torso_range": max(torso) - min(torso),
        "torso_mean": statistics.mean(torso),
        "neck_sd": statistics.pstdev(neck),
    }


def main():
    parser = argparse.ArgumentParser(description="자세 모델 비교")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--variants", nargs="+", default=list(VARIANTS))
    parser.add_argument("--camera")
    args = parser.parse_args()

    config = yaml.safe_load(POSTURE_CONFIG.read_text(encoding="utf-8"))
    perception = config["perception"]
    source = args.camera or perception.get("camera", 0)
    min_visibility = perception["min_visibility"]

    missing = [v for v in args.variants if not resolve_model(v).exists()]
    if missing:
        sys.exit(f"모델 파일이 없습니다: {missing}")

    print(f"카메라 {source} · 변형당 {args.seconds:.0f}초")
    print("측정 중에는 움직이지 마세요. 움직이면 비교가 무의미해집니다.\n")

    results = {}
    for variant in args.variants:
        print(f"[{variant}] 측정 중...", flush=True)
        results[variant] = measure(variant, source, args.seconds,
                                   min_visibility)

    print()
    header = ("{:<8}{:>8}{:>10}{:>10}{:>12}{:>12}{:>11}{:>11}".format(
        "모델", "fps", "추론ms", "검출률", "상체SD", "상체범위", "상체평균",
        "목SD"))
    print(header)
    print("-" * len(header))
    for variant, data in results.items():
        if data is None:
            print(f"{variant:<8}  표본 부족")
            continue
        print("{:<8}{:>8.1f}{:>10.1f}{:>9.0f}%{:>12.2f}{:>12.2f}"
              "{:>11.2f}{:>11.2f}".format(
                  variant, data["fps"], data["inference_ms"],
                  data["detected"] * 100, data["torso_sd"],
                  data["torso_range"], data["torso_mean"], data["neck_sd"]))

    usable = {k: v for k, v in results.items() if v}
    if len(usable) > 1:
        best = min(usable, key=lambda k: usable[k]["torso_sd"])
        print(f"\n가장 안정적: {best} (상체 표준편차 "
              f"{usable[best]['torso_sd']:.2f}도)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
