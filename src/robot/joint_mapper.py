"""자세 입력을 다축 DYNAMIXEL 목표 위치로 옮기는 순수 매핑 계층.

기존 설정은 ``torso_pitch`` 하나를 여러 관절에 분배하고 ``neck_pitch``를
목 관절 하나에 보내는 형태였다. 새 구조에서는 각 관절이
``torso_pitch``, ``torso_yaw``, ``neck_roll`` 같은 입력 축을 직접 선택한다.
기존 평면형 설정도 계속 읽을 수 있도록 legacy 분배 형식을 유지한다.
"""

import math

TICKS_PER_REV = 4096
TICKS_PER_DEG = TICKS_PER_REV / 360.0
POSITION_TICKS = TICKS_PER_REV
NECK_ROLE = "neck"


class MappingError(ValueError):
    pass


class JointMapper:
    def __init__(self, joints, distribution):
        if not joints:
            raise MappingError("관절 정의가 비어 있습니다")
        if not distribution:
            raise MappingError("관절 분배 설정이 비어 있습니다")

        self._joints = joints
        self._distribution = dict(distribution)
        self._multi_axis = any(
            isinstance(value, dict) for value in self._distribution.values())

        if self._multi_axis:
            self._validate_multi_axis_distribution()
            neck = [name for name, spec in joints.items()
                    if spec.get("group", "torso") == "neck"]
            self._neck = next(
                (name for name in neck if spec_axis(joints[name]) == "pitch"),
                neck[0] if neck else next(iter(joints)),
            )
        else:
            # 이전 설정은 neck role 하나와 torso 분배 하나를 요구한다.
            unknown = set(self._distribution) - set(joints)
            if unknown:
                raise MappingError(
                    f"분배 비율에 없는 관절이 있습니다: {sorted(unknown)}")
            neck = [name for name, spec in joints.items()
                    if spec.get("role") == NECK_ROLE]
            if len(neck) != 1:
                raise MappingError(
                    f"목 관절은 정확히 하나여야 합니다. 현재 {neck}")
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
            if not 0 <= spec["min_position"] <= spec["max_position"] < POSITION_TICKS:
                raise MappingError(f"{name}: 위치 범위가 0~4095 밖입니다")
            if not spec["min_position"] <= spec["zero_position"] <= spec["max_position"]:
                raise MappingError(f"{name}: zero_position 이 운용 범위 밖입니다")
            direction = spec.get("direction", 1)
            if direction not in (-1, 1):
                raise MappingError(f"{name}: direction 은 1 또는 -1 이어야 합니다")
            for field in ("relative_min_deg", "relative_max_deg"):
                if field in spec and not math.isfinite(float(spec[field])):
                    raise MappingError(f"{name}: {field} 가 유효하지 않습니다")
            if ("relative_min_deg" in spec and "relative_max_deg" in spec
                    and float(spec["relative_min_deg"])
                    > float(spec["relative_max_deg"])):
                raise MappingError(f"{name}: 상대 위치 범위가 뒤집혀 있습니다")

    def _validate_multi_axis_distribution(self):
        expected = set(self._joints)
        covered = set()
        for input_axis, weights in self._distribution.items():
            if not isinstance(weights, dict) or not weights:
                raise MappingError(f"{input_axis}: 관절 분배가 비어 있습니다")
            unknown = set(weights) - expected
            if unknown:
                raise MappingError(
                    f"{input_axis} 분배에 없는 관절: {sorted(unknown)}")
            total = sum(float(value) for value in weights.values())
            if abs(total - 1.0) > 1e-6:
                raise MappingError(
                    f"{input_axis} 분배 비율 합이 1.0 이 아닙니다: {total}")
            covered.update(weights)

        missing = expected - covered
        if missing:
            raise MappingError(f"분배가 빠진 관절: {sorted(missing)}")

        for name, spec in self._joints.items():
            input_axis = spec.get("input_axis")
            if input_axis is None:
                raise MappingError(f"{name}: input_axis 가 없습니다")
            if input_axis not in self._distribution:
                raise MappingError(
                    f"{name}: input_axis {input_axis!r} 분배가 없습니다")
            if name not in self._distribution[input_axis]:
                raise MappingError(
                    f"{name}: input_axis 분배에 관절이 없습니다")

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

    def _legacy_degrees(self, angles):
        result = {name: angles.torso_pitch_deg * weight
                  for name, weight in self._distribution.items()}
        result[self._neck] = angles.neck_pitch_deg
        return result

    def degrees_for(self, angles):
        """관절별 목표 각도(도)를 반환한다."""
        if not self._multi_axis:
            return self._legacy_degrees(angles)

        result = {}
        for input_axis, weights in self._distribution.items():
            value = float(getattr(angles, f"{input_axis}_deg", 0.0))
            for name, weight in weights.items():
                result[name] = value * float(weight)
        return result

    @staticmethod
    def _shortest_delta(current, target):
        return ((target - current + POSITION_TICKS / 2)
                % POSITION_TICKS - POSITION_TICKS / 2)

    def _relative_bounds(self, spec):
        minimum = spec.get("relative_min_deg")
        maximum = spec.get("relative_max_deg")
        if minimum is None or maximum is None:
            return None
        return float(minimum) * TICKS_PER_DEG, float(maximum) * TICKS_PER_DEG

    def _target_for(self, name, degrees):
        spec = self._joints[name]
        direction = spec.get("direction", 1)
        delta = direction * float(degrees) * TICKS_PER_DEG
        bounds = self._relative_bounds(spec)
        if bounds is not None:
            delta = max(bounds[0], min(bounds[1], delta))
        raw = spec["zero_position"] + delta
        if spec.get("wrap_position", False):
            return int(round(raw)) % POSITION_TICKS
        return int(round(max(spec["min_position"],
                             min(spec["max_position"], raw))))

    def to_targets(self, angles):
        """자세를 관절별 목표 tick으로 변환한다."""
        return {name: self._target_for(name, degrees)
                for name, degrees in self.degrees_for(angles).items()}

    def clamp_target(self, name, target):
        """절대/상대 운용 범위 안에서 목표 위치를 제한한다."""
        spec = self._joints[name]
        target = float(target)
        if not math.isfinite(target):
            raise MappingError(f"{name}: 목표 위치가 유효하지 않습니다")
        if spec.get("wrap_position", False):
            target %= POSITION_TICKS
            delta = self._shortest_delta(spec["zero_position"], target)
            bounds = self._relative_bounds(spec)
            if bounds is not None:
                delta = max(bounds[0], min(bounds[1], delta))
            return int(round(spec["zero_position"] + delta)) % POSITION_TICKS
        return int(round(max(spec["min_position"],
                             min(spec["max_position"], target))))

    def normalize_position(self, name, position):
        if self._joints[name].get("wrap_position", False):
            return int(round(position)) % POSITION_TICKS
        return int(round(position))

    def position_delta(self, name, current, target):
        if self._joints[name].get("wrap_position", False):
            return self._shortest_delta(current, target)
        return target - current

    def within_limits(self, name, position):
        spec = self._joints[name]
        if spec.get("wrap_position", False):
            delta = self._shortest_delta(spec["zero_position"], position)
            bounds = self._relative_bounds(spec)
            return bounds is None or bounds[0] <= delta <= bounds[1]
        return spec["min_position"] <= position <= spec["max_position"]


def spec_axis(spec):
    """새 설정에서 input_axis의 축 이름을 추출한다."""
    input_axis = spec.get("input_axis", "")
    return input_axis.rsplit("_", 1)[-1]
