"""자세 각 추출과 지연 버퍼 테스트. 카메라도 모터도 필요 없다."""

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.perception.posture_features import (  # noqa: E402
    LEFT_EAR, LEFT_HIP, LEFT_SHOULDER, NOSE, RIGHT_EAR, RIGHT_HIP,
    RIGHT_SHOULDER, PostureAngles, clamp_angles, extract_angles, smooth,
)
from src.posture.pose_buffer import PoseBuffer  # noqa: E402


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    visibility: float = 1.0


def build(shoulder_z=0.0, head_z=None, visibility=1.0):
    """골반 원점, 어깨는 위로 0.5, 머리는 어깨에서 위로 0.3 인 상체."""
    head_z = shoulder_z if head_z is None else head_z
    points = [Point() for _ in range(33)]
    for index in (LEFT_HIP, RIGHT_HIP):
        points[index] = Point(y=0.0, z=0.0, visibility=visibility)
    for index in (LEFT_SHOULDER, RIGHT_SHOULDER):
        points[index] = Point(y=-0.5, z=shoulder_z, visibility=visibility)
    for index in (LEFT_EAR, RIGHT_EAR):
        points[index] = Point(y=-0.8, z=head_z, visibility=visibility)
    points[NOSE] = Point(y=-0.8, z=head_z, visibility=visibility)
    return points


class TestExtractAngles:
    def test_upright_gives_zero(self):
        points = build()
        angles = extract_angles(points, points, timestamp=1.0)
        assert angles.torso_pitch_deg == pytest.approx(0.0, abs=1e-9)
        assert angles.neck_pitch_deg == pytest.approx(0.0, abs=1e-9)

    def test_leaning_forward_is_positive(self):
        # z 가 작을수록 카메라에 가깝다. 앞으로 숙이면 어깨 z 가 내려간다.
        points = build(shoulder_z=-0.5)
        angles = extract_angles(points, points, timestamp=1.0)
        assert angles.torso_pitch_deg == pytest.approx(45.0)

    def test_leaning_back_is_negative(self):
        points = build(shoulder_z=0.5)
        angles = extract_angles(points, points, timestamp=1.0)
        assert angles.torso_pitch_deg == pytest.approx(-45.0)

    def test_neck_is_relative_to_torso(self):
        # 상체와 머리가 같은 각도로 기울면 목 상대각은 0 이어야 한다.
        points = build(shoulder_z=-0.5, head_z=-0.8)
        angles = extract_angles(points, points, timestamp=1.0)
        assert angles.torso_pitch_deg == pytest.approx(45.0)
        assert angles.neck_pitch_deg == pytest.approx(0.0, abs=1e-9)

    def test_head_forward_relative_to_torso(self):
        points = build(shoulder_z=0.0, head_z=-0.3)
        angles = extract_angles(points, points, timestamp=1.0)
        assert angles.torso_pitch_deg == pytest.approx(0.0, abs=1e-9)
        assert angles.neck_pitch_deg == pytest.approx(45.0)

    def test_low_visibility_returns_none(self):
        points = build(visibility=0.2)
        assert extract_angles(points, points, timestamp=1.0) is None

    def test_empty_input_returns_none(self):
        assert extract_angles(None, None, timestamp=1.0) is None
        assert extract_angles([], [], timestamp=1.0) is None

    def test_invert_flags(self):
        points = build(shoulder_z=-0.5, head_z=-1.1)
        plain = extract_angles(points, points, timestamp=1.0)
        flipped = extract_angles(points, points, timestamp=1.0,
                                 invert_torso=True, invert_neck=True)
        assert flipped.torso_pitch_deg == pytest.approx(-plain.torso_pitch_deg)
        assert flipped.neck_pitch_deg == pytest.approx(-plain.neck_pitch_deg)


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
        assert got.timestamp == 10.25

    def test_confidence_is_the_lower_of_the_two(self):
        buffer = PoseBuffer(5.0)
        buffer.push(PostureAngles(10.0, 0.0, 0.0, 0.9))
        buffer.push(PostureAngles(11.0, 0.0, 0.0, 0.4))
        assert buffer.sample(10.5).confidence == pytest.approx(0.4)

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

    def test_delay_playback_matches_pushed_value(self):
        # 에코의 핵심 성질: t 에 넣은 값이 t+delay 에 그대로 나와야 한다.
        buffer = PoseBuffer(10.0)
        delay = 1.5
        for n in range(50):
            buffer.push(PostureAngles(n * 0.1, float(n), 0.0, 1.0))
        now = 4.0
        got = buffer.sample(now - delay)
        assert got.torso_pitch_deg == pytest.approx(25.0)


from src.perception.calibration import (  # noqa: E402
    average_reference, load_reference, save_reference,
)
from src.perception.posture_features import (  # noqa: E402
    MedianFilter, PostureReference, apply_reference,
)


class TestHipsAreOptional:
    def test_hips_out_of_frame_still_extracts(self):
        # 책상에 앉으면 골반은 프레임 밖으로 나간다. 그래도 각이 나와야 한다.
        points = build(shoulder_z=-0.5)
        for index in (LEFT_HIP, RIGHT_HIP):
            points[index].visibility = 0.01
        angles = extract_angles(points, points, timestamp=1.0)
        assert angles is not None
        assert angles.torso_pitch_deg == pytest.approx(45.0)

    def test_hidden_shoulders_still_reject(self):
        points = build()
        for index in (LEFT_SHOULDER, RIGHT_SHOULDER):
            points[index].visibility = 0.1
        assert extract_angles(points, points, timestamp=1.0) is None


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

    def test_average_reference_rejects_empty(self):
        with pytest.raises(ValueError):
            average_reference([])

    def test_save_and_load_roundtrip(self, tmp_path):
        target = tmp_path / "calibration.yaml"
        reference = PostureReference(19.812, -3.25)
        save_reference(reference, samples=42, path=target)
        loaded = load_reference(target)
        assert loaded.torso_pitch_deg == pytest.approx(19.812)
        assert loaded.neck_pitch_deg == pytest.approx(-3.25)

    def test_load_missing_file_returns_none(self, tmp_path):
        assert load_reference(tmp_path / "nope.yaml") is None

    def test_load_incomplete_file_returns_none(self, tmp_path):
        target = tmp_path / "calibration.yaml"
        target.write_text("captured_at: 2026-01-01\n", encoding="utf-8")
        assert load_reference(target) is None


class TestMedianFilter:
    def test_rejects_zero_window(self):
        with pytest.raises(ValueError):
            MedianFilter(0)

    def test_single_sample_passes_through(self):
        filt = MedianFilter(5)
        got = filt.apply(PostureAngles(1.0, 7.0, 3.0, 1.0))
        assert got.torso_pitch_deg == 7.0

    def test_outlier_is_rejected(self):
        # 단안 깊이는 가끔 크게 튄다. 중앙값은 그 한 프레임에 끌려가지 않는다.
        filt = MedianFilter(5)
        for value in (10.0, 10.0, 10.0, 10.0):
            filt.apply(PostureAngles(1.0, value, 0.0, 1.0))
        spiked = filt.apply(PostureAngles(1.0, 95.0, 0.0, 1.0))
        assert spiked.torso_pitch_deg == pytest.approx(10.0)

    def test_window_slides(self):
        filt = MedianFilter(3)
        for value in (1.0, 2.0, 3.0, 100.0, 100.0):
            got = filt.apply(PostureAngles(1.0, value, 0.0, 1.0))
        assert got.torso_pitch_deg == pytest.approx(100.0)

    def test_reset_clears_history(self):
        filt = MedianFilter(3)
        filt.apply(PostureAngles(1.0, 100.0, 0.0, 1.0))
        filt.reset()
        got = filt.apply(PostureAngles(1.0, 5.0, 0.0, 1.0))
        assert got.torso_pitch_deg == 5.0

    def test_timestamp_and_confidence_pass_through(self):
        filt = MedianFilter(3)
        got = filt.apply(PostureAngles(12.5, 1.0, 2.0, 0.77))
        assert got.timestamp == 12.5
        assert got.confidence == pytest.approx(0.77)
