#!/usr/bin/env python3
"""Render a YAM joint-angle pose to an image without CAN or a web server."""

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v3 as iio
import mujoco
import numpy as np
import tyro
from yam_mujoco_passive_preview import FALLBACK_GRIPPER, build_preview_model

JointAngles = tuple[float, float, float, float, float, float]


def render_camera_spec(elevation_deg: float = -35.0) -> tuple[str, float, float, float]:
    """Return the elevated left-rear camera used for rendering."""
    if not np.isfinite(elevation_deg) or not -90.0 <= elevation_deg <= 90.0:
        raise ValueError("camera_elevation_deg must be within [-90, 90]")
    return ("left-rear", 315.0, elevation_deg, 1.07)


def main(
    target_joints_deg: JointAngles,
    output: Path = Path("yam_joint_angles.png"),
    width: int = 1280,
    height: int = 960,
    gripper_pos: float = FALLBACK_GRIPPER,
    camera_elevation_deg: float = -35.0,
) -> None:
    """Render one YAM pose from six arm joint angles and a normalized gripper position.

    Args:
        target_joints_deg: Target [j1, j2, j3, j4, j5, j6] in degrees.
        output: Destination image path.
        width: Output image width in pixels.
        height: Output image height in pixels.
        gripper_pos: Normalized gripper position, where 0 is closed and 1 is open.
        camera_elevation_deg: Camera elevation in degrees; more negative values look down from higher above.
    """
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be greater than zero")
    if not np.isfinite(gripper_pos) or not 0.0 <= gripper_pos <= 1.0:
        raise ValueError("gripper_pos must be within [0, 1]")

    model = build_preview_model()
    data = mujoco.MjData(model)
    target_rad = np.deg2rad(np.asarray(target_joints_deg, dtype=float))

    for joint_number, angle in enumerate(target_rad, start=1):
        joint = model.joint(f"joint{joint_number}")
        lo, hi = joint.range
        if angle < lo or angle > hi:
            raise ValueError(
                f"joint{joint_number} target {np.degrees(angle):g} deg is outside "
                f"[{np.degrees(lo):g}, {np.degrees(hi):g}] deg"
            )
        data.qpos[joint.qposadr[0]] = angle

    if model.njnt > 6:
        gripper_joint = model.joint(6)
        lo, hi = gripper_joint.range
        data.qpos[gripper_joint.qposadr[0]] = lo + gripper_pos * (hi - lo)

    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.30]
    _, camera.azimuth, camera.elevation, camera.distance = render_camera_spec(camera_elevation_deg)
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.update_scene(data, camera=camera)
        frame = renderer.render()

    iio.imwrite(output, frame)
    print(output)


if __name__ == "__main__":
    tyro.cli(main)
