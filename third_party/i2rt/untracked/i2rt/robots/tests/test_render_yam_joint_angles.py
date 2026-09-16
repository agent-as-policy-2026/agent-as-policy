import os
import subprocess
import sys
from pathlib import Path

import imageio.v3 as iio
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "render_yam_joint_angles.py"
sys.path.insert(0, str(SCRIPT.parent))
import render_yam_joint_angles


def _render(
    target: list[str],
    output: Path,
    gripper_pos: str | None = None,
    camera_elevation_deg: str | None = None,
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop("MUJOCO_GL", None)
    command = [
        sys.executable,
        str(SCRIPT),
        "--target-joints-deg",
        *target,
        "--output",
        str(output),
        "--width",
        "320",
        "--height",
        "240",
    ]
    if gripper_pos is not None:
        command.extend(["--gripper-pos", gripper_pos])
    if camera_elevation_deg is not None:
        command.extend(["--camera-elevation-deg", camera_elevation_deg])
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )


def test_render_camera_spec_uses_zoomed_out_left_rear_perspective() -> None:
    assert hasattr(render_yam_joint_angles, "render_camera_spec")

    assert render_yam_joint_angles.render_camera_spec() == ("left-rear", 315.0, -35.0, 1.07)


def test_render_camera_spec_accepts_elevation_override() -> None:
    assert render_yam_joint_angles.render_camera_spec(-50.0) == ("left-rear", 315.0, -50.0, 1.07)


def test_render_yam_joint_angles_applies_camera_elevation_override(tmp_path: Path) -> None:
    default_output = tmp_path / "default-elevation.png"
    elevated_output = tmp_path / "elevated.png"

    default_result = _render(["0"] * 6, default_output)
    elevated_result = _render(["0"] * 6, elevated_output, camera_elevation_deg="-50")

    assert default_result.returncode == 0, default_result.stderr
    assert elevated_result.returncode == 0, elevated_result.stderr
    assert not (iio.imread(default_output) == iio.imread(elevated_output)).all()


def test_render_yam_joint_angles_rejects_invalid_camera_elevation(tmp_path: Path) -> None:
    result = _render(["0"] * 6, tmp_path / "invalid.png", camera_elevation_deg="91")

    assert result.returncode != 0
    assert "camera_elevation_deg must be within [-90, 90]" in result.stderr


def test_render_yam_joint_angles_writes_png(tmp_path: Path) -> None:
    output = tmp_path / "pose.png"

    result = _render(["0", "20", "30", "0", "10", "0"], output)

    assert result.returncode == 0, result.stderr
    image = iio.imread(output)
    assert image.shape == (240, 320, 3)


def test_render_yam_joint_angles_outputs_one_view_without_grid_dividers(tmp_path: Path) -> None:
    output = tmp_path / "single-view.png"

    result = _render(["0", "20", "30", "0", "10", "0"], output)

    assert result.returncode == 0, result.stderr
    image = iio.imread(output)
    vertical_center = image[:, 159:161]
    horizontal_center = image[119:121, :]
    assert np.unique(vertical_center.reshape(-1, 3), axis=0).shape[0] > 1
    assert np.unique(horizontal_center.reshape(-1, 3), axis=0).shape[0] > 1


def test_render_yam_joint_angles_applies_requested_angles(tmp_path: Path) -> None:
    zero_output = tmp_path / "zero.png"
    target_output = tmp_path / "target.png"

    zero_result = _render(["0", "0", "0", "0", "0", "0"], zero_output)
    target_result = _render(["0", "20", "30", "0", "10", "0"], target_output)

    assert zero_result.returncode == 0, zero_result.stderr
    assert target_result.returncode == 0, target_result.stderr
    assert not (iio.imread(zero_output) == iio.imread(target_output)).all()


def test_render_yam_joint_angles_applies_normalized_gripper_position(tmp_path: Path) -> None:
    closed_output = tmp_path / "closed.png"
    open_output = tmp_path / "open.png"

    closed_result = _render(["0"] * 6, closed_output, gripper_pos="0")
    open_result = _render(["0"] * 6, open_output, gripper_pos="1")

    assert closed_result.returncode == 0, closed_result.stderr
    assert open_result.returncode == 0, open_result.stderr
    assert not (iio.imread(closed_output) == iio.imread(open_output)).all()


def test_render_yam_joint_angles_rejects_out_of_range_gripper_position(tmp_path: Path) -> None:
    result = _render(["0"] * 6, tmp_path / "invalid.png", gripper_pos="1.1")

    assert result.returncode != 0
    assert "gripper_pos must be within [0, 1]" in result.stderr
