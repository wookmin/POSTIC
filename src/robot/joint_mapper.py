"""사람의 굽힘각을 로봇 관절의 목표 tick 으로 옮긴다.

하드웨어를 모르는 순수 계산이다. 관절 정의(config/joints.yaml)와 분배 비율
(config/posture.yaml)만 받는다.

상체각 하나를 4축에 나누고, 목각은 목 축 하나가 받는다. 축마다 운용 한계로
자르므로 상체각이 커도 개별 관절이 한계를 넘지 않는다.
"""

TICKS_PER_REV = 4096
TICKS_PER_DEG = TICKS_PER_REV / 360.0

NECK_ROLE = "neck"


class MappingError(ValueError):
    pass


class JointMapper:
    def __init__(self, joints, distribution):
        if not joints:
            raise MappingError("관절 정의가 비어 있습니다")

        self._joints = joints
        self._distribution = dict(distribution)

        unknown = set(self._distribution) - set(joints)
        if unknown:
            raise MappingError(f"분배 비율에 없는 관절이 있습니다: {sorted(unknown)}")

        neck = [name for name, spec in joints.items()
                if spec.get("role") == NECK_ROLE]
        if len(neck) != 1:
            raise MappingError(f"목 관절은 정확히 하나여야 합니다. 현재 {neck}")
        self._neck = neck[0]

        torso = set(joints) - {self._neck}
        missing = torso - set(self._distribution)
        if missing:
            raise MappingError(f"분배 비율이 빠진 관절: {sorted(missing)}")

        total = sum(self._distribution.values())
        if abs(total - 1.0) > 1e-6:
            raise MappingError(f"분배 비율 합이 1.0 이 아닙니다: {total}")

        for name, spec in joints.items():
            for field in ("zero_position", "min_position", "max_position"):
                if spec.get(field) is None:
                    raise MappingError(f"{name}: {field} 가 없습니다")
            if not spec["min_position"] <= spec["zero_position"] <= spec["max_position"]:
                raise MappingError(
                    f"{name}: zero_position 이 운용 범위 밖입니다")

    @property
    def neck_joint(self):
        return self._neck

    @property
    def joints(self):
        """관절 이름 -> 설정 딕셔너리. 읽기 전용 참조."""
        return self._joints

    def joint_ids(self):
        return {name: spec["id"] for name, spec in self._joints.items()}

    def neutral_targets(self):
        return {name: spec["zero_position"]
                for name, spec in self._joints.items()}

    def degrees_for(self, angles):
        """관절별 목표 각도(도). tick 변환 전 값이라 디버깅에 쓴다."""
        result = {name: angles.torso_pitch_deg * weight
                  for name, weight in self._distribution.items()}
        result[self._neck] = angles.neck_pitch_deg
        return result

    def to_targets(self, angles):
        """관절 이름 -> 목표 tick. 각 축의 운용 한계로 자른다."""
        targets = {}
        for name, degrees in self.degrees_for(angles).items():
            spec = self._joints[name]
            direction = spec.get("direction", 1)
            raw = spec["zero_position"] + direction * degrees * TICKS_PER_DEG
            targets[name] = int(round(max(spec["min_position"],
                                          min(spec["max_position"], raw))))
        return targets
