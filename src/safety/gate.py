"""모터 지령을 하드웨어 직전에 통과시키는 결정론적 안전 게이트.

행동 판단은 모터 각도를 만들지 않고, 실제 목표 tick은 이 모듈을 통과한 값만
쓴다. 이 파일의 계산은 IO를 하지 않아서 하드웨어 없이 테스트할 수 있다.
"""

from __future__ import annotations

import math

from src.perception.posture_features import PostureAngles
from src.robot.joint_mapper import TICKS_PER_DEG
from src.safety.supervisor import SlewLimiter


class SafetyViolation(ValueError):
    """안전 계약을 만족하지 않는 지령."""


class SafetyGate:
    """목표 위치·행동 자세·동작 지속 시간을 공통으로 제한한다."""

    def __init__(self, mapper, config: dict):
        self.mapper = mapper
        safety = config.get("safety") or {}
        schema = safety.get("schema")
        if schema is not None and schema != "postic.safety.v1":
            raise SafetyViolation(
                f"지원하지 않는 safety schema: {schema!r} "
                "(postic.safety.v1 필요)"
            )
        motion = safety.get("motion") or {}
        legacy_motion = config.get("motion") or {}
        angles = config.get("angles") or {}

        # safety.motion 이 없던 기존 설정도 그대로 동작하도록 이전 값을
        # fallback 으로 사용한다. 새 설정에서는 safety 가 단일 정책 소스다.
        max_step_deg = motion.get("max_step_deg",
                                 legacy_motion.get("max_step_deg", 1.5))
        self.max_step_deg = float(max_step_deg)
        if self.max_step_deg <= 0:
            raise SafetyViolation("max_step_deg 는 0 보다 커야 합니다")
        self.max_step_ticks = self.max_step_deg * TICKS_PER_DEG
        self.limiter = SlewLimiter(self.max_step_ticks)

        self.max_angles = {
            "torso_pitch": float(angles.get("max_torso_pitch_deg", 30.0)),
            "torso_yaw": float(angles.get("max_torso_yaw_deg", 30.0)),
            "torso_roll": float(angles.get("max_torso_roll_deg", 30.0)),
            "neck_pitch": float(angles.get("max_neck_pitch_deg", 25.0)),
            "neck_yaw": float(angles.get("max_neck_yaw_deg", 25.0)),
            "neck_roll": float(angles.get("max_neck_roll_deg", 25.0)),
        }
        if any(value < 0 for value in self.max_angles.values()):
            raise SafetyViolation("각도 안전 범위는 0 이상이어야 합니다")
        self.max_behavior_duration_sec = float(
            motion.get("behavior_max_duration_sec", 1.5))
        if self.max_behavior_duration_sec <= 0:
            raise SafetyViolation("behavior_max_duration_sec 는 0 보다 커야 합니다")

    def clamp_pose(self, pose: PostureAngles) -> PostureAngles:
        """행동용 자세를 각도 안전 범위 안으로 제한한다."""
        if not pose.valid:
            raise SafetyViolation("유효하지 않은 자세는 행동으로 실행할 수 없습니다")
        fields = (
            "torso_pitch_deg", "torso_yaw_deg", "torso_roll_deg",
            "neck_pitch_deg", "neck_yaw_deg", "neck_roll_deg",
        )
        if not all(math.isfinite(getattr(pose, field)) for field in fields):
            raise SafetyViolation("자세 각도에 유한하지 않은 값이 있습니다")
        limited = {}
        for field in fields:
            limit = self.max_angles[field[:-4]]
            limited[field] = max(-limit, min(limit, getattr(pose, field)))
        return PostureAngles(
            timestamp=pose.timestamp,
            torso_pitch_deg=limited["torso_pitch_deg"],
            neck_pitch_deg=limited["neck_pitch_deg"],
            confidence=pose.confidence,
            torso_yaw_deg=limited["torso_yaw_deg"],
            torso_roll_deg=limited["torso_roll_deg"],
            neck_yaw_deg=limited["neck_yaw_deg"],
            neck_roll_deg=limited["neck_roll_deg"],
        )

    def clamp_duration(self, duration_sec: float) -> float:
        """행동 지속 시간을 0~정책 상한으로 제한한다."""
        if not math.isfinite(duration_sec):
            raise SafetyViolation("행동 지속 시간이 유한하지 않습니다")
        return max(0.0, min(self.max_behavior_duration_sec, duration_sec))

    def clamp_targets(self, targets: dict) -> dict:
        """관절별 운용 한계 안으로 tick을 제한한다.

        알 수 없는 관절은 조용히 버리지 않고 실패시킨다. 잘못된 이름을
        무시하면 일부 축만 움직인 상태를 정상으로 오인할 수 있다.
        """
        known = set(self.mapper.joints)
        unknown = set(targets) - known
        if unknown:
            raise SafetyViolation(f"알 수 없는 관절 지령: {sorted(unknown)}")

        bounded = {}
        for name, target in targets.items():
            if not isinstance(target, (int, float)) or not math.isfinite(target):
                raise SafetyViolation(f"{name} 목표 tick이 유효하지 않습니다: {target!r}")
            bounded[name] = self.mapper.clamp_target(name, target)
        return bounded

    def limit_targets(self, current: dict, targets: dict) -> dict:
        """관절 한계와 제어 주기당 변화량을 모두 적용한다."""
        bounded = self.clamp_targets(targets)
        limited = {}
        for name, target in bounded.items():
            if name not in current:
                limited[name] = target
                continue
            delta = self.mapper.position_delta(name, current[name], target)
            if delta > self.max_step_ticks:
                delta = self.max_step_ticks
            elif delta < -self.max_step_ticks:
                delta = -self.max_step_ticks
            limited[name] = self.mapper.normalize_position(
                name, current[name] + delta)
        return limited
