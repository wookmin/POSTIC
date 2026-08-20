import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.offline_pipeline import PoseFeatures, classify_posture  # noqa: E402


NOSE = 0
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_HIP = 23
RIGHT_HIP = 24


def midpoint(a, b):
    return (
        (a.x + b.x) / 2,
        (a.y + b.y) / 2,
        (a.z + b.z) / 2,
    )


def distance(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2
        + (a.y - b.y) ** 2
        + (a.z - b.z) ** 2
    )


def angle_from_vertical(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return abs(math.degrees(math.atan2(dx, dy)))


def extract_features(result):
    if not result.pose_landmarks:
        return None

    image_points = result.pose_landmarks[0]

    required = [
        NOSE,
        LEFT_SHOULDER,
        RIGHT_SHOULDER,
        LEFT_HIP,
        RIGHT_HIP,
    ]

    for index in required:
        visibility = getattr(image_points[index], "visibility", 1.0)

        if visibility < 0.5:
            return None

    left_shoulder = image_points[LEFT_SHOULDER]
    right_shoulder = image_points[RIGHT_SHOULDER]
    left_hip = image_points[LEFT_HIP]
    right_hip = image_points[RIGHT_HIP]

    shoulder_center = midpoint(
        left_shoulder,
        right_shoulder,
    )

    hip_center = midpoint(
        left_hip,
        right_hip,
    )

    torso_lean_deg = angle_from_vertical(
        shoulder_center,
        hip_center,
    )

    # 어깨 선의 각도를 0~90도 범위로 정상화
    shoulder_angle = abs(
        math.degrees(
            math.atan2(
                right_shoulder.y - left_shoulder.y,
                right_shoulder.x - left_shoulder.x,
            )
        )
    )

    shoulder_tilt_deg = min(
        shoulder_angle,
        180.0 - shoulder_angle,
    )

    # 3D 월드 좌표를 이용해 머리가 몸통보다 앞으로 나온 정도 계산
    if result.pose_world_landmarks:
        world = result.pose_world_landmarks[0]

        world_shoulder = midpoint(
            world[LEFT_SHOULDER],
            world[RIGHT_SHOULDER],
        )

        shoulder_width = distance(
            world[LEFT_SHOULDER],
            world[RIGHT_SHOULDER],
        )

        head_forward_ratio = abs(
            world[NOSE].z - world_shoulder[2]
        ) / max(shoulder_width, 0.001)
    else:
        head_forward_ratio = 0.0

    return PoseFeatures(
        torso_lean_deg=torso_lean_deg,
        head_forward_ratio=head_forward_ratio,
        shoulder_tilt_deg=shoulder_tilt_deg,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="0")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parents[1]

    model_path = (
        project_root
        / "models"
        / "pose"
        / "pose_landmarker_lite.task"
    )

    if not model_path.exists():
        raise FileNotFoundError(
            f"자세 모델이 없습니다: {model_path}"
        )

    camera_source = (
        int(args.camera)
        if args.camera.isdigit()
        else args.camera
    )

    cap = cv2.VideoCapture(
        camera_source,
        cv2.CAP_V4L2,
    )

    if not cap.isOpened():
        raise RuntimeError(
            f"카메라를 열 수 없습니다: {camera_source}"
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    base_options = python.BaseOptions(
        model_asset_path=str(model_path)
    )

    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.IMAGE,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    last_posture = None

    print("카메라 자세 인식 시작")
    print("종료: Ctrl+C")

    try:
        with vision.PoseLandmarker.create_from_options(
            options
        ) as landmarker:

            while True:
                ok, frame = cap.read()

                if not ok:
                    print("카메라 프레임을 읽지 못했습니다.")
                    break

                frame = cv2.flip(frame, 1)

                rgb = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2RGB,
                )

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=rgb,
                )

                result = landmarker.detect(mp_image)
                features = extract_features(result)

                if features is None:
                    posture = "no_person"
                    display_text = "사람을 찾는 중..."

                    payload = {
                        "posture": posture,
                        "features": {},
                    }

                else:
                    posture = classify_posture(features)

                    display_text = (
                        f"{posture} | "
                        f"torso={features.torso_lean_deg:.1f} | "
                        f"head={features.head_forward_ratio:.2f} | "
                        f"shoulder={features.shoulder_tilt_deg:.1f}"
                    )

                    payload = {
                        "posture": posture,
                        "features": {
                            "torso_lean_deg": round(
                                features.torso_lean_deg,
                                2,
                            ),
                            "head_forward_ratio": round(
                                features.head_forward_ratio,
                                3,
                            ),
                            "shoulder_tilt_deg": round(
                                features.shoulder_tilt_deg,
                                2,
                            ),
                        },
                    }

                if posture != last_posture:
                    print(
                        json.dumps(
                            payload,
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                    last_posture = posture

                if not args.headless:
                    cv2.putText(
                        frame,
                        display_text,
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2,
                    )

                    cv2.imshow(
                        "Posture Robot Camera",
                        frame,
                    )

                    key = cv2.waitKey(1) & 0xFF

                    if key == ord("q") or key == 27:
                        break

    except KeyboardInterrupt:
        print("\n종료합니다.")

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

