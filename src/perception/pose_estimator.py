"""MediaPipe PoseLandmarker 래퍼와 프리뷰 오버레이."""

from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = PROJECT_ROOT / "models" / "pose"
VARIANTS = ("lite", "full", "heavy")
DEFAULT_MODEL = MODEL_DIR / "pose_landmarker_lite.task"


def resolve_model(name):
    """"full" 같은 변형 이름이나 파일 경로를 모두 받는다."""
    if name is None:
        return DEFAULT_MODEL
    if name in VARIANTS:
        return MODEL_DIR / f"pose_landmarker_{name}.task"
    return Path(name)

# 상체 위주. 하체는 이 로봇이 표현할 수 없으므로 그리지 않는다.
CONNECTIONS = (
    (11, 12), (11, 23), (12, 24), (23, 24),   # 몸통
    (11, 13), (13, 15), (12, 14), (14, 16),   # 팔
    (7, 8),                                   # 귀 - 귀
)
HEAD_POINTS = (0, 7, 8)


class PoseEstimator:
    def __init__(self, model_path=None):
        self.model_path = resolve_model(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"자세 모델이 없습니다: {self.model_path}")
        self._landmarker = None

    def __enter__(self):
        options = vision.PoseLandmarkerOptions(
            base_options=python.BaseOptions(
                model_asset_path=str(self.model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._landmarker = vision.PoseLandmarker.create_from_options(options)
        return self

    def __exit__(self, *exc):
        if self._landmarker is not None:
            self._landmarker.close()
        self._landmarker = None
        return False

    def detect(self, frame_bgr, timestamp_ms):
        """(image_landmarks, world_landmarks). 사람이 없으면 (None, None)."""
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, int(timestamp_ms))
        if not result.pose_landmarks:
            return None, None
        world = result.pose_world_landmarks[0] if result.pose_world_landmarks else None
        return result.pose_landmarks[0], world


def draw_skeleton(frame, landmarks, color=(0, 220, 0)):
    if not landmarks:
        return frame
    height, width = frame.shape[:2]

    def point(index):
        mark = landmarks[index]
        return int(mark.x * width), int(mark.y * height)

    for start, end in CONNECTIONS:
        cv2.line(frame, point(start), point(end), color, 2)
    for index in HEAD_POINTS:
        cv2.circle(frame, point(index), 4, color, -1)
    for index in (11, 12, 23, 24):
        cv2.circle(frame, point(index), 5, (0, 140, 255), -1)
    return frame


def draw_overlay(frame, lines, color=(255, 255, 255)):
    """왼쪽 위에 상태 문자열을 쌓아 그린다."""
    for row, text in enumerate(lines):
        origin = (14, 30 + row * 26)
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    color, 1, cv2.LINE_AA)
    return frame


def draw_angle_bar(frame, label, value, limit, row, width=220):
    """각도를 중앙 기준 막대로 그린다. 부호와 크기를 눈으로 확인하기 쉽다."""
    height = frame.shape[0]
    left = 14
    top = height - 70 + row * 28
    center = left + width // 2
    cv2.rectangle(frame, (left, top), (left + width, top + 16), (60, 60, 60), -1)
    cv2.line(frame, (center, top), (center, top + 16), (150, 150, 150), 1)

    ratio = max(-1.0, min(1.0, value / limit if limit else 0.0))
    end = int(center + ratio * (width // 2))
    cv2.rectangle(frame, (center, top), (end, top + 16), (0, 200, 255), -1)
    cv2.putText(frame, f"{label} {value:+6.1f}", (left + width + 10, top + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return frame
