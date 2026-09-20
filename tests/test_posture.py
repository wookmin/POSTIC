"""2D 비율 기반 자세 추출과 지연 버퍼 테스트. 카메라도 모터도 필요 없다."""

from dataclasses import dataclass

import pytest

from src.perception.posture_features import (
    LEFT_EAR, LEFT_HIP, LEFT_SHOULDER, NOSE, RIGHT_EAR, RIGHT_HIP,
    RIGHT_SHOULDER, PostureAngles, clamp_angles, extract_angles, smooth,
)
from src.posture.pose_buffer import PoseBuffer


@dataclass
class Point:
    """이미지 좌표 mock. x, y 는 0~1 범위."""
    x: float = 0.5
    y: float = 0.5
    z: float = 0.0
    visibility: float = 1.0


def build_upright(visibility=1.0):
    """바로 앉은 자세: 코(0.3) → 어깨(0.5) → 골반(0.8) 수직 정렬."""
    points = [Point() for _ in range(33)]
    # 골반: 이미지 하단
    points[LEFT_HIP] = Point(x=0.43, y=0.8, visibility=visibility)
    points[RIGHT_HIP] = Point(x=0.57, y=0.8, visibility=visibility)
    # 어깨: 중간
    points[LEFT_SHOULDER] = Point(x=0.4, y=0.5, visibility=visibility)
    points[RIGHT_SHOULDER] = Point(x=0.6, y=0.5, visibility=visibility)
    # 머리: 상단
    points[LEFT_EAR] = Point(x=0.47, y=0.3, visibility=visibility)
    points[RIGHT_EAR] = Point(x=0.53, y=0.3, visibility=visibility)
    points[NOSE] = Point(x=0.5, y=0.3, visibility=visibility)
    return points


def build_slouch():
    """상체를 숙인 자세: 어깨 X가 오른쪽으로 치우짐 (정면에서 앞으로 숙이면)."""
    points = build_upright()
    # 숙이면 어깨가 골반 대비 X로 치우침
    for idx in (LEFT_SHOULDER, RIGHT_SHOULDER):
        points[idx].y = 0.62
    return points


def build_forward_head():
    """거북목: 머리가 어깨 대비 앞으로 (X로 치우짐)."""
    points = build_upright()
    for idx in (LEFT_EAR, RIGHT_EAR):
        points[idx].y = 0.42
    points[NOSE].y = 0.42
    return points


def build_lateral_tilt():
    points = build_upright()
    points[LEFT_SHOULDER].y = 0.42
    points[RIGHT_SHOULDER].y = 0.58
    points[LEFT_HIP].y = 0.74
    points[RIGHT_HIP].y = 0.86
    return points


class TestExtractAngles2D:
    def test_upright_gives_small_angle(self):
        points = build_upright()
        angles = extract_angles(None, points, timestamp=1.0)
        # 완전 수직이면 torso ≈ 0, neck ≈ 0
        assert abs(angles.torso_pitch_deg) < 1.0
        assert abs(angles.neck_pitch_deg) < 1.0

    def test_slouch_increases_torso(self):
        points = build_slouch()
        angles = extract_angles(None, points, timestamp=1.0)
        assert angles.torso_pitch_deg > 15.0  # 확실히 기울어짐

    def test_forward_head_increases_neck(self):
        points = build_forward_head()
        angles = extract_angles(None, points, timestamp=1.0)
        assert angles.neck_pitch_deg > 15.0

    def test_lateral_tilt_is_separate_from_torso_pitch(self):
        points = build_lateral_tilt()
        angles = extract_angles(None, points, timestamp=1.0)
        assert angles.lateral_tilt_deg > 15.0
        assert angles.torso_pitch_deg < 12.0

    def test_small_center_offset_does_not_become_lateral_during_slouch(self):
        points = build_slouch()
        for idx in (LEFT_SHOULDER, RIGHT_SHOULDER):
            points[idx].x += 0.01

        angles = extract_angles(None, points, timestamp=1.0)

        assert angles.torso_pitch_deg > 15.0
        assert angles.lateral_tilt_deg < 12.0

    def test_mirrored_horizontal_landmarks_are_not_lateral_tilt(self):
        points = build_upright()
        for left, right in ((LEFT_SHOULDER, RIGHT_SHOULDER),
                            (LEFT_HIP, RIGHT_HIP)):
            points[left].x, points[right].x = (
                points[right].x, points[left].x)

        angles = extract_angles(None, points, timestamp=1.0)

        assert angles.lateral_tilt_deg < 1.0

    def test_low_visibility_returns_none(self):
        points = build_upright(visibility=0.2)
        assert extract_angles(None, points, timestamp=1.0) is None

    def test_empty_input_returns_none(self):
        assert extract_angles(None, None, timestamp=1.0) is None
        assert extract_angles(None, [], timestamp=1.0) is None


class TestClampAndSmooth:
    def test_clamp_limits_both_axes(self):
        angles = PostureAngles(1.0, 90.0, -80.0, 1.0)
        clamped = clamp_angles(angles, max_torso_deg=45, max_neck_deg=35)
        assert clamped.torso_pitch_deg == 45.0
        assert clamped.neck_pitch_deg == -35.0

    def test_smooth_without_previous_returns_current(self):
        current = PostureAngles(1.0, 10.0, 5.0, 1.0)
        assert smooth(None, current, 0.25) is current

    def test_smooth_blends(self):
        previous = PostureAngles(0.0, 0.0, 0.0, 1.0)
        current = PostureAngles(1.0, 10.0, 20.0, 1.0)
        blended = smooth(previous, current, 0.25)
        assert blended.torso_pitch_deg == pytest.approx(2.5)
        assert blended.neck_pitch_deg == pytest.approx(5.0)

    def test_alpha_one_disables_filter(self):
        previous = PostureAngles(0.0, 0.0, 0.0, 1.0)
        current = PostureAngles(1.0, 10.0, 20.0, 1.0)
        assert smooth(previous, current, 1.0) is current

    def test_neck_alpha_separate(self):
        previous = PostureAngles(0.0, 0.0, 0.0, 1.0)
        current = PostureAngles(1.0, 10.0, 20.0, 1.0)
        blended = smooth(previous, current, 0.5, neck_alpha=0.1)
        assert blended.torso_pitch_deg == pytest.approx(5.0)
        assert blended.neck_pitch_deg == pytest.approx(2.0)


class TestPoseBuffer:
    def test_rejects_non_positive_span(self):
        with pytest.raises(ValueError):
            PoseBuffer(0)

    def test_sample_before_first_returns_none(self):
        buffer = PoseBuffer(5.0)
        buffer.push(PostureAngles(10.0, 0.0, 0.0, 1.0))
        assert buffer.sample(9.0) is None

    def test_sample_after_last_returns_none(self):
        buffer = PoseBuffer(5.0)
        buffer.push(PostureAngles(10.0, 0.0, 0.0, 1.0))
        assert buffer.sample(11.0) is None

    def test_exact_hit(self):
        buffer = PoseBuffer(5.0)
        buffer.push(PostureAngles(10.0, 7.0, 3.0, 1.0))
        got = buffer.sample(10.0)
        assert got.torso_pitch_deg == 7.0

    def test_linear_interpolation(self):
        buffer = PoseBuffer(5.0)
        buffer.push(PostureAngles(10.0, 0.0, 0.0, 1.0))
        buffer.push(PostureAngles(11.0, 10.0, 20.0, 1.0))
        got = buffer.sample(10.25)
        assert got.torso_pitch_deg == pytest.approx(2.5)
        assert got.neck_pitch_deg == pytest.approx(5.0)

    def test_out_of_order_push_is_ignored(self):
        buffer = PoseBuffer(5.0)
        assert buffer.push(PostureAngles(10.0, 0.0, 0.0, 1.0)) is True
        assert buffer.push(PostureAngles(9.0, 0.0, 0.0, 1.0)) is False
        assert len(buffer) == 1

    def test_old_samples_are_dropped(self):
        buffer = PoseBuffer(1.0)
        for n in range(30):
            buffer.push(PostureAngles(10.0 + n * 0.1, 0.0, 0.0, 1.0))
        oldest, newest = buffer.span()
        assert newest - oldest <= 1.0 + 1e-9


from src.perception.calibration import (
    average_reference, load_reference, save_reference,
)
from src.perception.posture_features import (
    MedianFilter, PostureReference, apply_reference,
)


class TestReference:
    def test_reference_subtracts_bias(self):
        angles = PostureAngles(1.0, 20.0, 5.0, 1.0)
        reference = PostureReference(torso_pitch_deg=20.0, neck_pitch_deg=5.0)
        corrected = apply_reference(angles, reference)
        assert corrected.torso_pitch_deg == pytest.approx(0.0)
        assert corrected.neck_pitch_deg == pytest.approx(0.0)

    def test_none_reference_is_identity(self):
        angles = PostureAngles(1.0, 20.0, 5.0, 1.0)
        assert apply_reference(angles, None) is angles

    def test_average_reference(self):
        samples = [PostureAngles(0.0, 10.0, 2.0, 1.0),
                   PostureAngles(1.0, 20.0, 4.0, 1.0)]
        reference = average_reference(samples)
        assert reference.torso_pitch_deg == pytest.approx(15.0)
        assert reference.neck_pitch_deg == pytest.approx(3.0)

    def test_save_and_load_roundtrip(self, tmp_path):
        target = tmp_path / "calibration.yaml"
        reference = PostureReference(19.812, -3.25)
        save_reference(reference, samples=42, path=target)
        loaded = load_reference(target)
        assert loaded.torso_pitch_deg == pytest.approx(19.812)
        assert loaded.neck_pitch_deg == pytest.approx(-3.25)


class TestMedianFilter:
    def test_rejects_zero_window(self):
        with pytest.raises(ValueError):
            MedianFilter(0)

    def test_single_sample_passes_through(self):
        filt = MedianFilter(5)
        got = filt.apply(PostureAngles(1.0, 7.0, 3.0, 1.0))
        assert got.torso_pitch_deg == 7.0

    def test_outlier_is_rejected(self):
        filt = MedianFilter(5)
        for value in (10.0, 10.0, 10.0, 10.0):
            filt.apply(PostureAngles(1.0, value, 0.0, 1.0))
        spiked = filt.apply(PostureAngles(1.0, 95.0, 0.0, 1.0))
        assert spiked.torso_pitch_deg == pytest.approx(10.0)

    def test_reset_clears_history(self):
        filt = MedianFilter(3)
        filt.apply(PostureAngles(1.0, 100.0, 0.0, 1.0))
        filt.reset()
        got = filt.apply(PostureAngles(1.0, 5.0, 0.0, 1.0))
        assert got.torso_pitch_deg == 5.0
