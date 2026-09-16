import numpy as np
import pytest

from i2rt.utils.viser_control_interface import parse_joint_target_degrees

JOINT_RANGES_DEG = [(-180.0, 180.0)] * 6


def test_parse_joint_target_degrees_accepts_spaces_and_commas() -> None:
    target = parse_joint_target_degrees("0, 20  30,0 10 0", JOINT_RANGES_DEG)

    np.testing.assert_array_equal(target, [0.0, 20.0, 30.0, 0.0, 10.0, 0.0])


def test_parse_joint_target_degrees_rejects_wrong_angle_count() -> None:
    with pytest.raises(ValueError, match="expected 6 joint angles"):
        parse_joint_target_degrees("0 20 30", JOINT_RANGES_DEG)


@pytest.mark.parametrize("text", ["0 20 bad 0 10 0", "0 20 nan 0 10 0", "0 20 inf 0 10 0"])
def test_parse_joint_target_degrees_rejects_non_finite_angles(text: str) -> None:
    with pytest.raises(ValueError, match="joint angles must be finite numbers"):
        parse_joint_target_degrees(text, JOINT_RANGES_DEG)


def test_parse_joint_target_degrees_rejects_angles_outside_joint_limits() -> None:
    ranges = [(-180.0, 180.0), (-45.0, 90.0)] + JOINT_RANGES_DEG[2:]

    with pytest.raises(ValueError, match=r"j2 must be between -45\.0 and 90\.0 deg"):
        parse_joint_target_degrees("0 100 30 0 10 0", ranges)
