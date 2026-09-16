"""Offline constant joint-zero-offset fit for the LEFT arm.

Replicates the right arm's joint-4 encoder-zero diagnosis (yam_v1.yml
``motor_offsets_deg_by_channel: can_follower_r: [0,0,0,5.4674...,0,0]``) for
``can_follower_l``: given the left_pose*.npz captures, the board geometry and
the left D405 distortion json, re-solve the eye-in-hand calibration while
optimizing constant zero offsets on joints 2..5.

Joint 1 is EXCLUDED: a constant base-yaw offset only rotates the world frame
about z, which co-rotates the solved fixed target - a gauge freedom the
fixed-target consistency residual cannot observe.

Joint 6 is ALSO EXCLUDED (verified by this script's --self-test): joint 6 is
the last joint, so a constant offset d right-multiplies the flange pose by a
fixed rotation about the flange's roll axis, and the solved hand-eye
``gripper_from_camera`` absorbs it EXACTLY (residual identically zero for any
d).  A joint-6 zero error therefore cannot hurt camera_to_world accuracy - it
is silently compensated by the solved extrinsic - but it also cannot be
diagnosed from fixed-target hand-eye data.

Model:  q_true = q_measured + delta   (delta = fitted correction, radians)
For a candidate delta the per-capture ``T_world_gripper`` is recomputed by FK
on the combined model, the hand-eye is re-solved (cv2.calibrateHandEye PARK
via the bridge's ``solve_hand_eye``), and the residual is the fixed-target
consistency: per capture, deviation of ``T_world_target_i`` from the mean
(translation in meters + rotation-vector scaled by --length-scale-m-per-rad).
Checkerboard detection/undistortion reuses ``agp_yam_bridge.camera_acceptance``
functions; ``T_camera_target`` per capture does not depend on delta and is
computed once.

Install convention (printed with the result): i2rt reports
``joint = raw - offset_yml`` (dm_driver, directions are +1 on yam v1), so the
fitted CORRECTION to the yml row is ``-delta_deg``.  The printed final row is
``--installed-offsets-deg + correction``, where --installed-offsets-deg is the
``can_follower_l`` row that was ALREADY in yam_v1.yml when the captures were
recorded (zeros on a first-round fit).

Run under the hardware-bridge uv project:
    cd hardware-bridge
    uv run --locked python ../calib/fit_joint_offsets.py \
        --captures ../calib/out/left_pose*.npz

Offline self-check (synthetic data, no captures needed):  --self-test
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from left_handeye_common import (  # noqa: E402
    INTRINSICS_JSON,
    LEFT_CAMERA_SERIAL,
    LeftArmFK,
    load_intrinsics_json,
    station_left_flange_from_camera,
)

# zero-based joint indices eligible for fitting (joints 2..5; 1 and 6 are
# gauge freedoms, see module docstring)
DEFAULT_FIT_JOINTS = (1, 2, 3, 4)


# --------------------------------------------------------------------------
# Data loading / detection (reuses the bridge solver's internals)
# --------------------------------------------------------------------------
def load_capture_entries(
    paths: list[Path],
    *,
    frame_index: int | None,
    columns: int,
    rows: int,
    distortion_model: str,
    distortion_coefficients: np.ndarray,
) -> list[dict]:
    from agp_yam_bridge.camera_acceptance import (
        _detect_checkerboard,
        undistort_realsense_pixels,
    )

    entries = []
    for path in paths:
        with np.load(path, allow_pickle=False) as capture:
            frame_count = len(capture["rgb"])
            index = frame_count // 2 if frame_index is None else frame_index
            if not 0 <= index < frame_count:
                raise ValueError(
                    f"{path}: frame index {index} outside 0..{frame_count - 1}"
                )
            rgb = capture["rgb"][index]
            K = np.asarray(capture["intrinsics"][index], dtype=np.float64)
            joints = np.asarray(
                capture["robot_joint_pos_0"][index], dtype=np.float64
            )
            serial = str(capture["serial"])
        if serial != LEFT_CAMERA_SERIAL:
            raise ValueError(
                f"{path}: serial {serial!r} is not the left D405 "
                f"{LEFT_CAMERA_SERIAL!r}"
            )
        if joints.shape != (7,):
            raise ValueError(f"{path}: robot_joint_pos_0 must be (7,), got {joints.shape}")
        pixels = _detect_checkerboard(
            rgb, columns=columns, rows=rows, label=str(path)
        )
        pixels = undistort_realsense_pixels(
            pixels,
            K,
            width=rgb.shape[1],
            height=rgb.shape[0],
            distortion_model=distortion_model,
            distortion_coefficients=distortion_coefficients,
        )
        entries.append(
            {
                "label": path.stem,
                "pixels": pixels,
                "intrinsics": K,
                "q6": joints[:6],
            }
        )
    return entries


def attach_camera_from_target(
    entries: list[dict],
    fk: LeftArmFK,
    flange_from_camera: np.ndarray,
    *,
    columns: int,
    rows: int,
    square_size_m: float,
) -> None:
    """Resolve the 180-deg corner-order ambiguity once (at delta = 0).

    ``T_camera_target`` does not depend on the joint offsets, so it is fixed
    for the whole fit.  Mirrors ``calibrate_checkerboard_records``'s
    orientation pass.
    """
    from agp_yam_bridge.camera_acceptance import (
        choose_checkerboard_orientation,
        estimate_checkerboard_pose,
    )

    zeros5 = np.zeros(5)
    first = entries[0]
    first_cft = estimate_checkerboard_pose(
        first["pixels"],
        first["intrinsics"],
        columns=columns,
        rows=rows,
        square_size_m=square_size_m,
        distortion_coefficients=zeros5,
    )
    reference_world_from_target = (
        fk.base_from_gripper(first["q6"]) @ flange_from_camera
    ) @ first_cft
    for entry in entries:
        world_from_camera = fk.base_from_gripper(entry["q6"]) @ flange_from_camera
        pose = choose_checkerboard_orientation(
            entry["pixels"],
            entry["intrinsics"],
            world_from_camera=world_from_camera,
            reference_world_from_target=reference_world_from_target,
            columns=columns,
            rows=rows,
            square_size_m=square_size_m,
            distortion_coefficients=zeros5,
        )
        entry["camera_from_target"] = pose.camera_from_target
        entry["reversed"] = pose.reversed


# --------------------------------------------------------------------------
# Fit
# --------------------------------------------------------------------------
def apply_delta(
    q6: np.ndarray, delta: np.ndarray, joint_indices: np.ndarray
) -> np.ndarray:
    corrected = np.asarray(q6, dtype=np.float64).copy()
    corrected[joint_indices] += delta
    return corrected


def hand_eye_for_delta(
    entries: list[dict], fk: LeftArmFK, delta: np.ndarray, joint_indices: np.ndarray
):
    from agp_yam_bridge.camera_acceptance import solve_hand_eye

    world_from_gripper = [
        fk.base_from_gripper(apply_delta(entry["q6"], delta, joint_indices))
        for entry in entries
    ]
    camera_from_target = [entry["camera_from_target"] for entry in entries]
    return solve_hand_eye(world_from_gripper, camera_from_target), world_from_gripper


def residual_vector(
    delta: np.ndarray,
    entries: list[dict],
    fk: LeftArmFK,
    length_scale_m_per_rad: float,
    joint_indices: np.ndarray,
) -> np.ndarray:
    hand_eye, world_from_gripper = hand_eye_for_delta(entries, fk, delta, joint_indices)
    world_from_target = [
        wfg @ hand_eye.gripper_from_camera @ entry["camera_from_target"]
        for wfg, entry in zip(world_from_gripper, entries, strict=True)
    ]
    translations = np.stack([pose[:3, 3] for pose in world_from_target])
    mean_translation = translations.mean(axis=0)
    rotations = Rotation.from_matrix(
        np.stack([pose[:3, :3] for pose in world_from_target])
    )
    mean_rotation = rotations.mean()
    rotation_deviation = (mean_rotation.inv() * rotations).as_rotvec()
    return np.concatenate(
        [
            (translations - mean_translation).reshape(-1),
            (length_scale_m_per_rad * rotation_deviation).reshape(-1),
        ]
    )


def fit_offsets(
    entries: list[dict],
    fk: LeftArmFK,
    *,
    joint_indices: np.ndarray,
    max_offset_rad: float,
    length_scale_m_per_rad: float,
    return_jacobian: bool = False,
):
    result = least_squares(
        residual_vector,
        x0=np.zeros(len(joint_indices)),
        bounds=(-max_offset_rad, max_offset_rad),
        args=(entries, fk, length_scale_m_per_rad, joint_indices),
        method="trf",
        diff_step=1e-5,
        xtol=1e-14,
        ftol=1e-14,
        gtol=1e-14,
    )
    if not result.success:
        raise RuntimeError(f"least_squares did not converge: {result.message}")
    near_bound = np.abs(result.x) >= 0.95 * max_offset_rad
    if near_bound.any():
        names = ", ".join(
            f"joint{joint_indices[i] + 1}" for i in np.nonzero(near_bound)[0]
        )
        print("!" * 72)
        print(f"!! WARNING: fitted offset for {names} lands within 5% of the")
        print(f"!! +/-{np.degrees(max_offset_rad):.2f} deg bound. least_squares")
        print("!! pins at the bound SILENTLY - the true offset may be larger.")
        print("!! Re-run with a larger --max-offset-deg before trusting this fit.")
        print("!" * 72)
    if return_jacobian:
        return result.x, result.jac, result.fun
    return result.x


def consistency_metrics(
    entries: list[dict], fk: LeftArmFK, delta: np.ndarray, joint_indices: np.ndarray
) -> dict:
    hand_eye, _ = hand_eye_for_delta(entries, fk, delta, joint_indices)
    return {
        "target_translation_rms_m": hand_eye.target_translation_rms_m,
        "target_translation_max_m": hand_eye.target_translation_max_m,
        "target_rotation_rms_deg": hand_eye.target_rotation_rms_deg,
        "target_rotation_max_deg": hand_eye.target_rotation_max_deg,
        "gripper_from_camera": hand_eye.gripper_from_camera,
    }


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------
def parameter_sigmas_deg(jacobian: np.ndarray, residual: np.ndarray) -> np.ndarray:
    """Per-joint 1-sigma (deg) from the linearized LS covariance at the fit.

    The residual re-solves the hand-eye per evaluation, so this covariance
    already accounts for the compensation between joint offsets and the
    jointly-estimated hand-eye + target: a joint whose effect the hand-eye can
    nearly absorb for this pose set gets a large sigma, even when
    leave-one-out looks repeatable (the degeneracy is structural, not
    data-split dependent).
    """
    m, k = jacobian.shape
    dof = max(m - k, 1)
    noise_variance = float(residual @ residual) / dof
    covariance = noise_variance * np.linalg.inv(jacobian.T @ jacobian)
    return np.degrees(np.sqrt(np.diag(covariance)))


def print_conditioning(
    jacobian: np.ndarray, sigmas_deg: np.ndarray, joint_names: list[str]
) -> None:
    _, singular_values, v_rows = np.linalg.svd(jacobian, full_matrices=False)
    print(
        "conditioning: per-joint 1-sigma (deg): "
        + ", ".join(
            f"{name} {sigma:.2f}"
            for name, sigma in zip(joint_names, sigmas_deg, strict=True)
        )
    )
    if (sigmas_deg > 0.5).any():
        weakest = v_rows[-1]
        combo = " ".join(
            f"{coefficient:+.2f}*{name}"
            for coefficient, name in zip(weakest, joint_names, strict=True)
        )
        print("  WARNING: this pose set determines some joints poorly (sigma >")
        print("  0.5 deg): their values trade against the jointly-solved")
        print(f"  hand-eye/target. Weakest direction: {combo}")
        print("  Prefer a targeted re-run (e.g. --joints 4) and/or a capture")
        print("  round with stronger wrist tilts and distance variation.")
    print()


def print_report(
    entries: list[dict],
    fk: LeftArmFK,
    delta_full: np.ndarray,
    loo_deltas: list[np.ndarray] | None,
    *,
    joint_indices: np.ndarray,
    jacobian: np.ndarray,
    fit_residual: np.ndarray,
    stable_threshold_deg: float,
    flange_from_camera: np.ndarray,
    length_scale_m_per_rad: float,
    installed_offsets_deg: np.ndarray,
) -> None:
    np.set_printoptions(suppress=True, precision=6)
    joint_names = [f"joint{i + 1}" for i in joint_indices]
    baseline = consistency_metrics(
        entries, fk, np.zeros_like(delta_full), joint_indices
    )
    fitted = consistency_metrics(entries, fk, delta_full, joint_indices)
    delta_deg = np.degrees(delta_full)

    print()
    print(f"captures used: {len(entries)}")
    for entry in entries:
        print(
            f"  {entry['label']}  joints(deg)="
            f"{np.degrees(entry['q6']).round(2).tolist()}"
            f"  reversed_corners={entry['reversed']}"
        )
    print()
    print("fixed-target consistency (bridge hand-eye gates: 15 mm / 2 deg RMS):")
    print(
        f"  baseline (delta=0): {baseline['target_translation_rms_m'] * 1000:.2f} mm RMS / "
        f"{baseline['target_rotation_rms_deg']:.3f} deg RMS "
        f"(max {baseline['target_translation_max_m'] * 1000:.2f} mm / "
        f"{baseline['target_rotation_max_deg']:.3f} deg)"
    )
    print(
        f"  fitted offsets:     {fitted['target_translation_rms_m'] * 1000:.2f} mm RMS / "
        f"{fitted['target_rotation_rms_deg']:.3f} deg RMS "
        f"(max {fitted['target_translation_max_m'] * 1000:.2f} mm / "
        f"{fitted['target_rotation_max_deg']:.3f} deg)"
    )
    print()

    if loo_deltas:
        loo_matrix_deg = np.degrees(np.stack(loo_deltas))
        loo_std_deg = loo_matrix_deg.std(axis=0)
        loo_range_deg = loo_matrix_deg.max(axis=0) - loo_matrix_deg.min(axis=0)
    else:
        loo_std_deg = np.full(len(delta_deg), np.nan)
        loo_range_deg = np.full(len(delta_deg), np.nan)

    sigmas_deg = parameter_sigmas_deg(jacobian, fit_residual)
    print_conditioning(jacobian, sigmas_deg, joint_names)

    print("fitted zero-offset corrections (q_true = q_measured + delta):")
    print("  joint    delta_deg   sigma_deg   LOO_std_deg   LOO_range_deg   verdict")
    stable = np.zeros(len(delta_deg), dtype=bool)
    for i, name in enumerate(joint_names):
        magnitude_ok = abs(delta_deg[i]) > stable_threshold_deg
        significant = abs(delta_deg[i]) > 2.0 * sigmas_deg[i]
        if loo_deltas:
            stability_ok = loo_std_deg[i] <= max(0.1, 0.3 * abs(delta_deg[i]))
        else:
            stability_ok = False
        stable[i] = magnitude_ok and significant and stability_ok
        if not magnitude_ok:
            verdict = f"below {stable_threshold_deg:g} deg threshold"
        elif not significant:
            verdict = "NOT SIGNIFICANT (< 2 sigma; do not install)"
        elif not loo_deltas:
            verdict = "no leave-one-out run (needs >=4 captures, without --no-loo)"
        elif stability_ok:
            verdict = "STABLE -> install"
        else:
            verdict = "UNSTABLE across leave-one-out (do not install)"
        print(
            f"  {name}   {delta_deg[i]:+9.4f}   {sigmas_deg[i]:9.4f}"
            f"   {loo_std_deg[i]:11.4f}   {loo_range_deg[i]:13.4f}   {verdict}"
        )
    print("  (joint1 excluded: base-yaw offset is a world-frame gauge freedom;")
    print("   joint6 excluded: a flange-roll offset is absorbed exactly by the")
    print("   solved hand-eye and is unobservable from fixed-target data)")
    print()

    # Parsimony check: one well-conditioned single-joint fit per candidate.
    # A single genuine encoder-zero error (the expected failure mode, like the
    # right arm's joint 4) shows up as ONE single-joint fit recovering most of
    # the multi-joint fit's residual improvement; spread-out multi-joint
    # "solutions" that no single joint reproduces are usually the pose set
    # trading offsets against the hand-eye/target and should not be installed.
    print("single-joint fits (parsimony check; install the simplest model):")
    print("  joints fit          delta_deg   consistency after fit")
    print(
        f"  {'(none)':14s}   {'':>9s}   "
        f"{baseline['target_translation_rms_m'] * 1000:.2f} mm / "
        f"{baseline['target_rotation_rms_deg']:.3f} deg RMS"
    )
    for index in joint_indices:
        single = np.asarray([index])
        delta_single = fit_offsets(
            entries,
            fk,
            joint_indices=single,
            max_offset_rad=np.radians(
                max(np.max(np.abs(delta_deg)) * 2.0 + 1.0, 3.0)
            ),
            length_scale_m_per_rad=length_scale_m_per_rad,
        )
        single_metrics = consistency_metrics(entries, fk, delta_single, single)
        print(
            f"  joint{index + 1} only       {np.degrees(delta_single[0]):+9.4f}   "
            f"{single_metrics['target_translation_rms_m'] * 1000:.2f} mm / "
            f"{single_metrics['target_rotation_rms_deg']:.3f} deg RMS"
        )
    print(
        f"  {'all fitted':14s}   {'':>9s}   "
        f"{fitted['target_translation_rms_m'] * 1000:.2f} mm / "
        f"{fitted['target_rotation_rms_deg']:.3f} deg RMS"
    )
    print()

    delta_hand_eye = np.linalg.inv(flange_from_camera) @ fitted["gripper_from_camera"]
    print("hand-eye at fitted offsets, solved_from_nominal(CAD): "
          f"{np.linalg.norm(delta_hand_eye[:3, 3]) * 1000:.2f} mm / "
          f"{np.degrees(Rotation.from_matrix(delta_hand_eye[:3, :3]).magnitude()):.3f} deg")
    print()

    if stable.any():
        installed = np.asarray(installed_offsets_deg, dtype=np.float64)
        correction_yml = np.zeros(6)
        correction_yml[joint_indices[stable]] = -delta_deg[stable]
        final_yml = installed + correction_yml
        installed_values = ", ".join(f"{value:.12g}" for value in installed)
        correction_values = ", ".join(f"{value:.12g}" for value in correction_yml)
        final_values = ", ".join(f"{value:.12g}" for value in final_yml)
        print("install snippet for i2rt/i2rt/robots/config/yam_v1.yml")
        print("(sign: i2rt reports joint = raw - offset, so correction = -delta;")
        print(" unstable/below-threshold joints get correction 0.0.")
        print(" ASSUMES the can_follower_l row already in yam_v1.yml when these")
        print(f" captures were recorded was [{installed_values}]")
        print(" (--installed-offsets-deg); final row = installed + correction):")
        print()
        print(f"  correction (deg): [{correction_values}]")
        print()
        print("motor_offsets_deg_by_channel:")
        print("  can_follower_r: [0.0, 0.0, 0.0, 5.467439674492972, 0.0, 0.0]")
        print(f"  can_follower_l: [{final_values}]")
        print()
        print("AFTER installing: re-run a fresh capture round and solve_left.sh;")
        print("the existing npz files embed the OLD measured joints.")
    else:
        print("no stable offset above the threshold - do not modify yam_v1.yml.")


# --------------------------------------------------------------------------
# Self test (synthetic, no hardware, no captures)
# --------------------------------------------------------------------------
def run_self_test(max_offset_deg: float, length_scale: float) -> int:
    print("self-test: synthetic offset recovery on the real FK model")
    fk = LeftArmFK()
    flange_from_camera = station_left_flange_from_camera()
    rng = np.random.default_rng(20260831)

    # True zero errors on the FITTED joints 2..5 ...
    delta_true_deg = np.array([0.9, -0.6, 2.4, 0.45])
    delta_true = np.radians(delta_true_deg)
    # ... plus an UNMODELED joint-6 zero error that must be absorbed exactly
    # by the solved hand-eye (gauge freedom), leaving joints 2..5 recoverable.
    joint6_error_rad = np.radians(-0.8)

    # True hand-eye = CAD perturbed by ~3 mm / ~0.6 deg.
    perturbation = np.eye(4)
    perturbation[:3, :3] = Rotation.from_rotvec(
        np.radians([0.3, -0.4, 0.25])
    ).as_matrix()
    perturbation[:3, 3] = [0.002, -0.0015, 0.001]
    gripper_from_camera_true = flange_from_camera @ perturbation

    # Fixed target in the world.
    world_from_target = np.eye(4)
    world_from_target[:3, :3] = Rotation.from_euler(
        "xyz", [5.0, 175.0, 20.0], degrees=True
    ).as_matrix()
    world_from_target[:3, 3] = [0.34, 0.05, 0.02]

    joint_indices = np.asarray(DEFAULT_FIT_JOINTS)
    nominal = np.radians([10.0, 25.0, -30.0, 45.0, 60.0, 15.0])
    entries = []
    for index in range(10):
        q_true = nominal + rng.uniform(-0.35, 0.35, size=6)
        q_measured = q_true.copy()
        q_measured[joint_indices] -= delta_true  # q_true = q_meas + delta
        q_measured[5] -= joint6_error_rad  # unmodeled flange-roll zero error
        world_from_camera_true = (
            fk.base_from_gripper(q_true) @ gripper_from_camera_true
        )
        camera_from_target = np.linalg.inv(world_from_camera_true) @ world_from_target
        entries.append(
            {
                "label": f"synthetic_{index:02d}",
                "q6": q_measured,
                "camera_from_target": camera_from_target,
                "reversed": False,
            }
        )

    delta_fit = fit_offsets(
        entries,
        fk,
        joint_indices=joint_indices,
        max_offset_rad=np.radians(max_offset_deg),
        length_scale_m_per_rad=length_scale,
    )
    error_deg = np.degrees(delta_fit) - delta_true_deg
    print(f"  true  joints2..5 (deg): {delta_true_deg.round(4).tolist()}"
          f"  (+ unmodeled joint6 zero error {np.degrees(joint6_error_rad):+.2f} deg)")
    print(f"  fitted joints2..5 (deg): {np.degrees(delta_fit).round(4).tolist()}")
    print(f"  error (deg): {error_deg.round(6).tolist()}")
    metrics = consistency_metrics(entries, fk, delta_fit, joint_indices)
    print(
        f"  residual consistency: {metrics['target_translation_rms_m'] * 1000:.4f} mm / "
        f"{metrics['target_rotation_rms_deg']:.5f} deg RMS"
    )
    # The joint-6 error must be absorbed by the solved hand-eye: the solved
    # extrinsic should differ from the true one by ~|joint6_error| about the
    # flange roll axis while the target residual stays ~zero.
    absorbed = np.linalg.inv(gripper_from_camera_true) @ metrics["gripper_from_camera"]
    absorbed_deg = np.degrees(Rotation.from_matrix(absorbed[:3, :3]).magnitude())
    print(
        f"  joint6 absorption: solved hand-eye differs from true by "
        f"{absorbed_deg:.3f} deg (expected ~{abs(np.degrees(joint6_error_rad)):.2f})"
    )
    ok = (
        bool(np.all(np.abs(error_deg) < 0.02))
        and metrics["target_translation_rms_m"] < 1e-5
        and abs(absorbed_deg - abs(np.degrees(joint6_error_rad))) < 0.05
    )
    print(f"self-test {'PASS' if ok else 'FAIL'} (tolerance 0.02 deg per joint)")
    return 0 if ok else 1


# --------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit constant zero offsets on left-arm joints 2..5 from "
        "hand-eye captures (offline; joints 1 and 6 are gauge freedoms)"
    )
    parser.add_argument("--captures", type=Path, nargs="+", help="left_pose*.npz files")
    parser.add_argument(
        "--intrinsics-json",
        type=Path,
        default=INTRINSICS_JSON,
        help="left_d405_intrinsics.json from capture_left_handeye.py",
    )
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--square-size-m", type=float, default=0.022)
    parser.add_argument("--frame-index", type=int, default=None)
    parser.add_argument(
        "--max-offset-deg",
        type=float,
        default=8.0,
        help="fit bound per joint (deg); must exceed the largest plausible "
        "zero error (the right arm's joint4 was +5.47 deg) - a fit pinned "
        "near the bound prints a warning",
    )
    parser.add_argument(
        "--installed-offsets-deg",
        type=float,
        nargs=6,
        default=[0.0] * 6,
        metavar="DEG",
        help="the can_follower_l motor_offsets_deg_by_channel row that was "
        "ALREADY in yam_v1.yml when these captures were recorded (default: "
        "six zeros = first-round fit); the printed install row is "
        "installed + fitted correction",
    )
    parser.add_argument(
        "--length-scale-m-per-rad",
        type=float,
        default=0.1,
        help="weight of rotational vs translational target deviation",
    )
    parser.add_argument("--stable-threshold-deg", type=float, default=0.3)
    parser.add_argument(
        "--joints",
        default="2,3,4,5",
        help="comma-separated joints to fit, subset of 2,3,4,5 (joints 1 and "
        "6 are gauge freedoms and cannot be fitted); a targeted single-joint "
        "fit (e.g. --joints 4) is much better conditioned",
    )
    parser.add_argument("--no-loo", action="store_true", help="skip leave-one-out")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="synthetic offset-recovery check (no captures needed)",
    )
    args = parser.parse_args()

    if args.self_test:
        return run_self_test(args.max_offset_deg, args.length_scale_m_per_rad)

    if not args.captures:
        parser.error("--captures is required (or use --self-test)")
    if len(args.captures) < 3:
        parser.error("the hand-eye solve needs at least 3 captures")

    try:
        joints = sorted({int(token) for token in args.joints.split(",") if token})
    except ValueError:
        parser.error("--joints must be comma-separated integers, e.g. 2,3,4,5")
    if not joints or any(j not in (2, 3, 4, 5) for j in joints):
        parser.error(
            "--joints must be a non-empty subset of 2,3,4,5 (joint 1 is a "
            "world-yaw gauge; joint 6 is absorbed exactly by the hand-eye)"
        )
    joint_indices = np.asarray([j - 1 for j in joints])

    payload = load_intrinsics_json(args.intrinsics_json)
    distortion_model = payload["distortion_model"]
    distortion_coefficients = np.asarray(
        payload["distortion_coefficients"], dtype=np.float64
    )
    print(f"distortion: {distortion_model} {distortion_coefficients.tolist()}")

    fk = LeftArmFK()
    flange_from_camera = station_left_flange_from_camera()
    entries = load_capture_entries(
        args.captures,
        frame_index=args.frame_index,
        columns=args.columns,
        rows=args.rows,
        distortion_model=distortion_model,
        distortion_coefficients=distortion_coefficients,
    )
    attach_camera_from_target(
        entries,
        fk,
        flange_from_camera,
        columns=args.columns,
        rows=args.rows,
        square_size_m=args.square_size_m,
    )

    max_offset_rad = np.radians(args.max_offset_deg)
    delta_full, jacobian, fit_residual = fit_offsets(
        entries,
        fk,
        joint_indices=joint_indices,
        max_offset_rad=max_offset_rad,
        length_scale_m_per_rad=args.length_scale_m_per_rad,
        return_jacobian=True,
    )

    loo_deltas: list[np.ndarray] | None = None
    if not args.no_loo and len(entries) >= 4:
        loo_deltas = []
        for leave_out in range(len(entries)):
            subset = [
                entry for index, entry in enumerate(entries) if index != leave_out
            ]
            loo_deltas.append(
                fit_offsets(
                    subset,
                    fk,
                    joint_indices=joint_indices,
                    max_offset_rad=max_offset_rad,
                    length_scale_m_per_rad=args.length_scale_m_per_rad,
                )
            )

    print_report(
        entries,
        fk,
        delta_full,
        loo_deltas,
        joint_indices=joint_indices,
        jacobian=jacobian,
        fit_residual=fit_residual,
        stable_threshold_deg=args.stable_threshold_deg,
        flange_from_camera=flange_from_camera,
        length_scale_m_per_rad=args.length_scale_m_per_rad,
        installed_offsets_deg=np.asarray(args.installed_offsets_deg),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
