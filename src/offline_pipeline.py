from dataclasses import dataclass


@dataclass
class PoseFeatures:
    torso_lean_deg: float
    head_forward_ratio: float
    shoulder_tilt_deg: float


@dataclass
class BehaviorCommand:
    behavior: str
    speech: str
    speed: int
    duration_ms: int


def classify_posture(features: PoseFeatures) -> str:
    if features.torso_lean_deg > 12:
        return "slouch"

    if features.head_forward_ratio > 0.12:
        return "forward_head"

    if abs(features.shoulder_tilt_deg) > 10:
        return "tilted_shoulders"

    return "good"


def gemini_mock(posture: str) -> BehaviorCommand:
    commands = {
        "slouch": BehaviorCommand(
            behavior="mimic_slouch",
            speech="등을 펴고 어깨를 뒤로 해보세요.",
            speed=20,
            duration_ms=1000,
        ),
        "forward_head": BehaviorCommand(
            behavior="mimic_forward_head",
            speech="고개가 앞으로 나와 있어요. 턱을 살짝 당겨보세요.",
            speed=15,
            duration_ms=800,
        ),
        "tilted_shoulders": BehaviorCommand(
            behavior="mimic_tilted_shoulders",
            speech="어깨 높이를 맞춰보세요.",
            speed=15,
            duration_ms=800,
        ),
        "good": BehaviorCommand(
            behavior="neutral",
            speech="좋은 자세입니다.",
            speed=0,
            duration_ms=0,
        ),
    }

    return commands[posture]


def safety_validate(command: BehaviorCommand) -> BehaviorCommand:
    command.speed = max(0, min(command.speed, 30))
    command.duration_ms = max(0, min(command.duration_ms, 1500))

    allowed_behaviors = {
        "mimic_slouch",
        "mimic_forward_head",
        "mimic_tilted_shoulders",
        "neutral",
    }

    if command.behavior not in allowed_behaviors:
        return BehaviorCommand(
            behavior="neutral",
            speech="",
            speed=0,
            duration_ms=0,
        )

    return command


def mock_robot_execute(command: BehaviorCommand):
    print(f"[ROBOT MOCK] behavior={command.behavior}")
    print(f"[ROBOT MOCK] speed={command.speed}")
    print(f"[ROBOT MOCK] duration={command.duration_ms} ms")
    print(f"[TTS MOCK] {command.speech}")
    print()


def main():
    test_samples = [
        PoseFeatures(
            torso_lean_deg=18,
            head_forward_ratio=0.08,
            shoulder_tilt_deg=2,
        ),
        PoseFeatures(
            torso_lean_deg=5,
            head_forward_ratio=0.18,
            shoulder_tilt_deg=3,
        ),
        PoseFeatures(
            torso_lean_deg=3,
            head_forward_ratio=0.05,
            shoulder_tilt_deg=1,
        ),
    ]

    for features in test_samples:
        posture = classify_posture(features)
        command = gemini_mock(posture)
        safe_command = safety_validate(command)

        print(f"[POSTURE] {posture}")
        mock_robot_execute(safe_command)


if __name__ == "__main__":
    main()
