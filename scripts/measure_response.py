#!/usr/bin/env python3
"""자세를 바꿔가며 각 신호가 실제로 얼마나 반응하는지 잰다.

노이즈(안정성)와 감도(민감도)는 다른 문제다. 값이 아주 안정적이면서 동시에
굽힘에 거의 반응하지 않을 수 있다. 그런 신호는 쓸모가 없다.

지시대로 자세를 잡고 유지하면 자세별 평균값을 표로 내놓는다. 자세 간 차이가
노이즈보다 충분히 크지 않은 신호는 쓸 수 없다는 뜻이다.

사용법:
    ~/dynamixel-venv/bin/python scripts/measure_response.py
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import yaml  # noqa: E402

from src.camera.camera_stream import CameraStream  # noqa: E402
from src.perception.pose_estimator import PoseEstimator  # noqa: E402
from src.perception.posture_features import (  # noqa: E402
    LEFT_EAR, LEFT_HIP, LEFT_SHOULDER, NOSE, RIGHT_EAR, RIGHT_HIP,
    RIGHT_SHOULDER, midpoint, sagittal_angle,
)

LEFT_EYE = 2
RIGHT_EYE = 5

PROJECT_ROOT = Path(__file__).resolve().parents[1]
POSTURE_CONFIG = PROJECT_ROOT / "config" / "posture.yaml"

# 책상 앞 직장인이 실제로 취하는 범위만 다룬다. "최대한" 자세는 화면을
# 벗어나 인식이 끊기므로 측정 자체가 성립하지 않는다.
POSES = [
    ("바른자세", "등을 펴고 바르게 앉으세요"),
    ("거북목", "상체는 그대로 두고 고개만 앞으로 빼세요"),
    ("구부정", "등을 말고 모니터 쪽으로 살짝 숙이세요"),
    ("뒤로기댐", "등받이에 편하게 기대세요"),
    ("바른자세2", "다시 바르게 앉으세요 (재현성 확인)"),
]

# 비율 신호는 카메라와의 거리에 불변이다. 의자를 앞뒤로 미는 것과 자세가
# 바뀌는 것을 구분하려면 이런 신호가 필요하다.
SIGNALS = (
    "torso_z_angle",        # 지금 쓰는 것. 추측된 골반에 의존한다
    "neck_z_angle",
    "shoulder_px_width",    # 거리 대용
    "ear_px_width",         # 머리 크기 대용
    "head_shoulder_ratio",  # 거리 불변. 머리가 어깨보다 앞에 나오면 커진다
    "ear_drop_norm",        # 귀가 어깨 대비 얼마나 내려왔나 (어깨폭으로 정규화)
    "nose_drop_norm",
    "eye_drop_norm",
    "shoulder_y",
)


def sample_signals(landmarks, world):
    shoulder = midpoint(world[LEFT_SHOULDER], world[RIGHT_SHOULDER])
    hip = midpoint(world[LEFT_HIP], world[RIGHT_HIP])
    torso = sagittal_angle(hip, shoulder)
    ear_world = midpoint(world[LEFT_EAR], world[RIGHT_EAR])
    neck = sagittal_angle(shoulder, ear_world) - torso

    ls, rs = landmarks[LEFT_SHOULDER], landmarks[RIGHT_SHOULDER]
    shoulder_width = abs(ls.x - rs.x)
    shoulder_y = (ls.y + rs.y) / 2
    ear_width = abs(landmarks[LEFT_EAR].x - landmarks[RIGHT_EAR].x)
    ear_y = (landmarks[LEFT_EAR].y + landmarks[RIGHT_EAR].y) / 2
    eye_y = (landmarks[LEFT_EYE].y + landmarks[RIGHT_EYE].y) / 2
    scale = max(shoulder_width, 1e-6)

    return {
        "torso_z_angle": torso,
        "neck_z_angle": neck,
        "shoulder_px_width": shoulder_width,
        "ear_px_width": ear_width,
        # 머리가 카메라 쪽으로 나오면 어깨보다 빨리 커진다. 가까운 거리일수록
        # 원근 효과가 크므로 책상 거리에서 오히려 유리한 신호다.
        "head_shoulder_ratio": ear_width / scale,
        "ear_drop_norm": (shoulder_y - ear_y) / scale,
        "nose_drop_norm": (shoulder_y - landmarks[NOSE].y) / scale,
        "eye_drop_norm": (shoulder_y - eye_y) / scale,
        "shoulder_y": shoulder_y,
    }


def hold(camera, estimator, seconds, start):
    """자세를 유지하는 동안 표본을 모은다. 인식률도 함께 돌려준다.

    프레임이 좁으면 크게 움직일 때 몸이 화면을 벗어나 인식이 끊긴다. 그때
    표본만 세면 원인을 놓치므로, 몇 프레임 중 몇 개를 잡았는지 같이 본다.
    """
    collected = []
    frames = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame = camera.read()
        if frame is None:
            continue
        frames += 1
        now = time.monotonic()
        landmarks, world = estimator.detect(frame, (now - start) * 1000)
        if landmarks and world:
            collected.append(sample_signals(landmarks, world))
            mark = "잡힘"
        else:
            mark = "놓침"
        remaining = deadline - time.monotonic()
        print(f"    {remaining:4.1f}초 남음  [{mark}]  "
              f"인식률 {len(collected)}/{frames}", end="\r", flush=True)
    print(" " * 60, end="\r")
    return collected, frames


def main():
    parser = argparse.ArgumentParser(description="신호 감도 측정")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--prepare", type=float, default=5.0)
    args = parser.parse_args()

    config = yaml.safe_load(POSTURE_CONFIG.read_text(encoding="utf-8"))
    perception = config["perception"]

    results = {}
    with CameraStream(perception.get("camera", 0)) as camera, \
            PoseEstimator(perception.get("model")) as estimator:
        start = time.monotonic()
        for label, instruction in POSES:
            print(f"\n>>> {instruction}", flush=True)
            for remaining in range(int(args.prepare), 0, -1):
                print(f"    준비 {remaining}...  ", end="\r", flush=True)
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    camera.read()
            print(f"    측정 중 {args.seconds:.0f}초, 유지하세요     ")
            samples, frames = hold(camera, estimator, args.seconds, start)
            rate = len(samples) / frames * 100 if frames else 0.0
            if len(samples) < 15:
                print(f"    표본 부족 {len(samples)}/{frames} "
                      f"(인식률 {rate:.0f}%) — 몸이 화면을 벗어났을 가능성")
                continue
            results[label] = samples
            print(f"    표본 {len(samples)}/{frames}  인식률 {rate:.0f}%")

    if len(results) < 2:
        sys.exit("비교할 자세가 부족합니다.")

    print("\n\n=== 자세별 평균 (괄호는 표준편차) ===")
    labels = list(results)
    header = f"{'신호':<20}" + "".join(f"{l:>18}" for l in labels)
    print(header)
    print("-" * len(header))
    for signal in SIGNALS:
        row = f"{signal:<20}"
        for label in labels:
            column = [s[signal] for s in results[label]]
            row += "{:>11.3f}({:.2f})".format(statistics.mean(column),
                                              statistics.pstdev(column))
        print(row)

    print("\n=== 감도: 자세 간 변화폭 / 노이즈 ===")
    print("이 값이 3 미만이면 그 신호로는 자세를 구분할 수 없습니다.\n")
    print(f"{'신호':<20}{'변화폭':>12}{'평균노이즈':>12}{'비율':>10}")
    print("-" * 54)
    for signal in SIGNALS:
        means = [statistics.mean([s[signal] for s in results[l]])
                 for l in labels]
        noises = [statistics.pstdev([s[signal] for s in results[l]])
                  for l in labels]
        spread = max(means) - min(means)
        noise = statistics.mean(noises) or 1e-9
        ratio = spread / noise
        flag = "" if ratio >= 3 else "   <- 부족"
        print(f"{signal:<20}{spread:>12.3f}{noise:>12.3f}{ratio:>10.1f}{flag}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
