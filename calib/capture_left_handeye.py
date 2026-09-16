"""Standalone LEFT-arm hand-eye capture tool (no bridge required).

Opens the left wrist D405 (serial 353322271204) and the i2rt robot on
``can_follower_l``, then runs a three-state interactive loop:

    g          gravity-comp idle: the operator drags the arm by hand
    h          hold: freeze at the CURRENT measured joint position
    c [label]  capture (hold only): 30 frames at 0.1 s into an auto-numbered
               npz whose schema exactly mirrors the bridge's
               camera_acceptance._save_capture, so the offline
               calibrate-checkerboard solver consumes the files unchanged
    p          print the current checkerboard-detection status (one-shot)
    q          quit, leaving the arm in HOLD

Because this tool holds the left D405 EXCLUSIVELY (pyrealsense2 allows one
owner per device), no external viewer can attach while it runs.  A built-in
MJPEG preview server (``--preview-port``, default 8766; ``--no-preview`` to
disable) therefore streams the live wrist color image with a checkerboard
overlay to any browser at http://<host-ip>:8766/ so the operator can confirm
the board is fully in frame.  All frame acquisition goes through a single
FramePump background thread that owns the pipeline; preview/detection threads
only ever READ camera frames and NEVER touch the robot.

SAFETY INVARIANTS
  * The only position command this script ever sends is the current measured
    joint position (robot.command_joint_pos of the just-read state).
  * A hold is REFUSED (no command sent) when any measured arm joint sits more
    than 0.02 rad outside the range i2rt clips position commands to (model XML
    limits widened by 0.15 rad, get_robot.py): the "hold" would otherwise
    command the CLIPPED position and the arm would jump toward it.  A dragged
    joint can legally rest up to a further 0.1 rad outside that range
    (MotorChainRobot._check_current_qpos_in_joint_limits buffer_rad).
  * Before every hold, capture, and quit the liveness of the i2rt driver
    threads is checked (after a CAN/motor fail-fast they die silently while
    get_observations keeps returning cached joints); a dead chain prints an
    unmissable warning and exits - the arm is NOT being held in that case.
  * No trajectories, no cartesian moves, no gain changes (i2rt defaults).
  * The gripper auto-calibration wiggle is SKIPPED via i2rt's supported
    ``gripper_limits_override`` (docstring: "If provided, skips calibration").
    The override [-20, 20] rad is deliberately wider than any reachable motor
    position so MotorChainRobot's gripper command clip can never move the
    gripper; the recorded gripper "fraction" is therefore NOT calibrated
    (harmless: the solver and the offset fit only use arm joints 1..6).
  * On ANY exception the script prints the current mode and exits WITHOUT
    commanding motion (os._exit).  DM motors keep executing the last MIT
    command onboard, so a held arm stays held; the operator supervises with
    the e-stop.

Run from the hardware-bridge uv project:
    cd hardware-bridge
    uv run --locked python ../calib/capture_left_handeye.py
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from left_handeye_common import (  # noqa: E402
    CAD_FLANGE_CAMERA_WXYZ,
    CAD_FLANGE_CAMERA_XYZ,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    INTRINSICS_JSON,
    LEFT_CAMERA_SERIAL,
    LEFT_CAN_CHANNEL,
    MAX_DEPTH_M,
    MIN_DEPTH_M,
    OUT_DIR,
    LeftArmFK,
    next_pose_path,
    station_left_flange_from_camera,
)

# Wider than any reachable DM motor position (startup normalizes to [-pi, pi];
# the linear_4310 stroke is ~6.6 rad), so the hold command's gripper clip is
# inert and a hold at the measured position is exact.
GRIPPER_LIMITS_OVERRIDE = np.array([-20.0, 20.0])


def say(message: str) -> None:
    print(message, flush=True)


def die_without_motion(state: str, exc: BaseException | None) -> None:
    """Print the current mode and exit WITHOUT commanding any motion."""
    say("")
    say("=" * 72)
    say("!! capture_left_handeye is exiting WITHOUT sending any new command.")
    say(f"!! Last commanded arm mode: {state.upper()}")
    if state == "hold":
        say("!! The motors keep holding the last position onboard.")
    else:
        say("!! The arm was in GRAVITY-COMP: the last (gravity) torque stays")
        say("!! frozen onboard - SUPPORT THE ARM BY HAND or press the e-stop.")
    if exc is not None:
        say(f"!! Reason: {type(exc).__name__}: {exc}")
    say("!! You may now press the e-stop or power off the arm.")
    say("=" * 72)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)


# --------------------------------------------------------------------------
# Camera
# --------------------------------------------------------------------------
class LeftD405:
    """Minimal standalone D405 reader: aligned color+depth 640x360@30.

    ``serial`` selects the wrist device (default: the LEFT D405, unchanged
    behaviour); capture_top_pairs.py passes the rig's wrist serial so the
    RIGHT rig's D405 (353322271910) is opened - and its OWN factory
    intrinsics dumped - when calibrating that rig.
    """

    def __init__(self, serial: str = LEFT_CAMERA_SERIAL) -> None:
        import pyrealsense2 as rs

        expected_serial = serial
        self._rs = rs
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(expected_serial)
        config.enable_stream(
            rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.rgb8, CAMERA_FPS
        )
        config.enable_stream(
            rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, CAMERA_FPS
        )
        profile = self._pipeline.start(config)
        device = profile.get_device()
        serial = device.get_info(rs.camera_info.serial_number)
        if serial != expected_serial:
            self._pipeline.stop()
            raise RuntimeError(
                f"opened camera serial {serial!r}, expected {expected_serial!r}"
            )
        self.serial = serial
        self.depth_scale_m = float(device.first_depth_sensor().get_depth_scale())
        self._align = rs.align(rs.stream.color)
        # Factory color intrinsics of THIS device (per-device, do not reuse the
        # right arm's values).
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self._color_intrinsics = color_stream.get_intrinsics()

    def intrinsics_payload(self) -> dict:
        intr = self._color_intrinsics
        model_name = str(intr.model).rsplit(".", 1)[-1]
        K = [
            [float(intr.fx), 0.0, float(intr.ppx)],
            [0.0, float(intr.fy), float(intr.ppy)],
            [0.0, 0.0, 1.0],
        ]
        return {
            "camera_model": "Intel RealSense D405",
            "serial": self.serial,
            "width": int(intr.width),
            "height": int(intr.height),
            "fps": CAMERA_FPS,
            "camera_matrix": K,
            "distortion_model": model_name,
            "distortion_coefficients": [float(c) for c in intr.coeffs],
            "depth_scale_m": self.depth_scale_m,
        }

    def read(self) -> dict:
        """One aligned frame: rgb, depth_m, K, device_frame_number, monotonic_ns.

        The RealSense frame counter is kept for diagnostics only: on this
        D405 the aligned color frame's ``get_frame_number()`` has been seen
        to STOP ADVANCING while images keep flowing, so nothing may sequence
        on it — FramePump stamps its own ``pump_sequence`` instead.
        """
        frames = self._pipeline.wait_for_frames(timeout_ms=2000)
        aligned = self._align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("D405 frame is missing aligned RGB or depth")
        monotonic_ns = time.monotonic_ns()
        intr = color.profile.as_video_stream_profile().get_intrinsics()
        K = np.array(
            [
                [intr.fx, 0.0, intr.ppx],
                [0.0, intr.fy, intr.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        rgb = np.ascontiguousarray(np.asanyarray(color.get_data()))
        raw_depth = np.asanyarray(depth.get_data())
        depth_m = raw_depth.astype(np.float32) * np.float32(self.depth_scale_m)
        invalid = (raw_depth == 0) | (depth_m < MIN_DEPTH_M) | (depth_m > MAX_DEPTH_M)
        depth_m[invalid] = 0.0
        return {
            "rgb": rgb,
            "depth_m": np.ascontiguousarray(depth_m),
            "intrinsics": K,
            "device_frame_number": int(color.get_frame_number()),
            "frame_monotonic_ns": monotonic_ns,
        }

    def close(self) -> None:
        self._pipeline.stop()


# --------------------------------------------------------------------------
# Frame pump (single owner of all pipeline reads)
# --------------------------------------------------------------------------
class FramePump(threading.Thread):
    """Background thread that OWNS all reads from the D405 pipeline.

    Continuously drains ``camera.read()`` (wait_for_frames + depth-to-color
    align) and stores the latest frame dict under a condition.  Captures and
    the preview both consume from here, so the pipeline never accumulates
    stale buffered framesets (this replaces the old ``drain()`` calls) and
    only one thread ever touches pyrealsense2.  Read-only with respect to the
    robot: this thread never sends any command.
    """

    def __init__(self, camera: LeftD405) -> None:
        super().__init__(name="frame-pump", daemon=True)
        self._camera = camera
        self.serial = camera.serial
        self._condition = threading.Condition()
        self._latest: dict | None = None
        self._frames_seen = 0
        self._sequence = 0
        self._last_error: str | None = None
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._camera.read()
            except Exception as exc:  # noqa: BLE001  camera hiccup: keep trying
                with self._condition:
                    self._last_error = f"{type(exc).__name__}: {exc}"
                self._stop_event.wait(0.1)
                continue
            with self._condition:
                # The pump's own counter is the ONLY sequence anything may
                # wait on: the D405's device frame number can stop advancing
                # while images keep flowing (seen in the field), which made
                # wait_for_frame_after time out with a healthy camera.
                self._sequence += 1
                frame["pump_sequence"] = self._sequence
                self._latest = frame
                self._frames_seen += 1
                self._last_error = None
                self._condition.notify_all()

    def latest(self) -> dict | None:
        """The most recent frame dict (never mutated; a new dict per frame)."""
        with self._condition:
            return self._latest

    def latest_sequence(self) -> int:
        with self._condition:
            return -1 if self._latest is None else int(self._latest["pump_sequence"])

    def wait_for_frame_after(self, sequence: int, timeout_s: float = 2.0) -> dict:
        """Next frame whose PUMP sequence is STRICTLY greater than ``sequence``.

        This is what guarantees the capture burst's strictly increasing frame
        numbers (duplicates are skipped by construction).  The pump sequence,
        not the device frame number, is compared — see run().
        """
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                if (
                    self._latest is not None
                    and int(self._latest["pump_sequence"]) > sequence
                ):
                    return self._latest
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    detail = (
                        f" (last camera error: {self._last_error})"
                        if self._last_error
                        else ""
                    )
                    raise RuntimeError(
                        f"no new camera frame within {timeout_s:.1f} s{detail}"
                    )
                self._condition.wait(remaining)

    def wait_until_ready(self, min_frames: int, timeout_s: float) -> None:
        """Block until the pump has produced ``min_frames`` frames (warmup)."""
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._frames_seen < min_frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    detail = (
                        f" (last camera error: {self._last_error})"
                        if self._last_error
                        else ""
                    )
                    raise RuntimeError(
                        f"frame pump produced only {self._frames_seen}/"
                        f"{min_frames} frames in {timeout_s:.1f} s{detail}"
                    )
                self._condition.wait(remaining)

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=3.0)


# --------------------------------------------------------------------------
# Board detection + MJPEG preview (read-only; never touches the robot)
# --------------------------------------------------------------------------
# Checkerboard used by the offline solver (camera_acceptance defaults
# --columns 9 --rows 7; solve_left.sh passes the same): INNER corner grid.
# detect_board below uses IDENTICAL detection params to the solver's
# _calibrate_checkerboard_captures / _detect_checkerboard_images
# (findChessboardCornersSB on RGB->GRAY with the same flags).
BOARD_COLUMNS = 9
BOARD_ROWS = 7
BOARD_CORNER_COUNT = BOARD_COLUMNS * BOARD_ROWS  # 63


def detect_board(rgb: np.ndarray) -> dict:
    """One-shot checkerboard detection, identical params to the solver."""
    import cv2

    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    detected, corners = cv2.findChessboardCornersSB(
        gray,
        (BOARD_COLUMNS, BOARD_ROWS),
        flags=(
            cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_EXHAUSTIVE
            | cv2.CALIB_CB_ACCURACY
        ),
    )
    count = 0 if corners is None else int(len(corners))
    found = bool(detected) and corners is not None and count == BOARD_CORNER_COUNT
    return {"found": found, "count": count, "corners": corners, "error": None}


def render_preview_jpeg(rgb: np.ndarray, detection: dict | None) -> bytes:
    """Latest color frame + cached board overlay + status banner, as JPEG."""
    import cv2

    # cvtColor makes a copy: never draw on the pump's shared array.
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if detection is not None and detection.get("corners") is not None:
        cv2.drawChessboardCorners(
            bgr,
            (BOARD_COLUMNS, BOARD_ROWS),
            detection["corners"],
            detection["found"],
        )
    if detection is None:
        text, color = "board: detecting ...", (0, 170, 255)
    elif detection.get("error"):
        text = f"detection ERROR: {detection['error']}"[:70]
        color = (0, 0, 220)
    elif detection["found"]:
        text = f"board OK: {detection['count']}/{BOARD_CORNER_COUNT} corners"
        color = (0, 160, 0)
    else:
        text = f"board NOT fully detected ({detection['count']}/{BOARD_CORNER_COUNT})"
        color = (0, 0, 220)
    cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 26), color, thickness=-1)
    cv2.putText(
        bgr,
        text,
        (8, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    encoded, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not encoded:
        raise RuntimeError("cv2.imencode failed on the preview frame")
    return buffer.tobytes()


class DetectionCache:
    """Latest board-detection result, shared between threads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._result: dict | None = None

    def set(self, result: dict) -> None:
        with self._lock:
            self._result = result

    def get(self) -> dict | None:
        with self._lock:
            return self._result


class PreviewState:
    """Latest preview JPEG + a condition so MJPEG clients can wait on it."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0

    def update(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg = jpeg
            self._sequence += 1
            self._condition.notify_all()

    def wait_for_jpeg(
        self, previous_sequence: int, timeout: float = 1.0
    ) -> tuple[bytes | None, int]:
        with self._condition:
            if not self._condition.wait_for(
                lambda: self._sequence > previous_sequence, timeout=timeout
            ):
                return None, previous_sequence
            return self._jpeg, self._sequence


class DetectionWorker(threading.Thread):
    """~2 Hz checkerboard detection on the source's latest frame (read-only).

    Runs in its own low-rate thread so the (expensive, EXHAUSTIVE) detection
    can never block the capture burst or the command loop.
    """

    def __init__(self, source, cache: DetectionCache, period_s: float = 0.5) -> None:
        super().__init__(name="board-detect", daemon=True)
        self._source = source
        self._cache = cache
        self._period_s = period_s
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            frame = self._source.latest()
            if frame is not None:
                try:
                    result = detect_board(frame["rgb"])
                except Exception as exc:  # noqa: BLE001  preview must not die
                    result = {
                        "found": False,
                        "count": 0,
                        "corners": None,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                result["frame_sequence"] = int(frame["pump_sequence"])
                result["checked_monotonic"] = time.monotonic()
                self._cache.set(result)
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.05, self._period_s - elapsed))

    def stop(self) -> None:
        self._stop_event.set()


class EncoderWorker(threading.Thread):
    """~8 fps JPEG encoding of the latest frame with the cached overlay."""

    def __init__(
        self,
        source,
        cache: DetectionCache,
        state: PreviewState,
        period_s: float = 0.125,
    ) -> None:
        super().__init__(name="preview-encode", daemon=True)
        self._source = source
        self._cache = cache
        self._state = state
        self._period_s = period_s
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            frame = self._source.latest()
            if frame is not None:
                try:
                    self._state.update(
                        render_preview_jpeg(frame["rgb"], self._cache.get())
                    )
                except Exception:  # noqa: BLE001  keep serving; retry next tick
                    pass
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.01, self._period_s - elapsed))

    def stop(self) -> None:
        self._stop_event.set()


class _PreviewHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], state: PreviewState) -> None:
        self.preview_state = state
        super().__init__(address, _PreviewRequestHandler)


class _PreviewRequestHandler(BaseHTTPRequestHandler):
    server: _PreviewHTTPServer

    def do_GET(self) -> None:  # noqa: N802
        if urlparse(self.path).path != "/":
            body = b"Not found - the preview stream is at /\n"
            self.send_response(404)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.end_headers()
        sequence = 0
        try:
            while True:
                jpeg, sequence = self.server.preview_state.wait_for_jpeg(sequence)
                if jpeg is None:
                    continue  # no new frame within the timeout; keep waiting
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

    def log_message(self, format: str, *args) -> None:  # noqa: A002
        return


def start_preview_server(
    host: str, port: int, state: PreviewState
) -> _PreviewHTTPServer:
    return _PreviewHTTPServer((host, port), state)


def best_effort_host_ips() -> list[str]:
    """Non-loopback host IPv4 addresses, ``hostname -I`` style (best effort)."""
    ips: list[str] = []
    try:
        result = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, timeout=2.0
        )
        ips = [token for token in result.stdout.split() if token]
    except Exception:  # noqa: BLE001
        ips = []
    if not ips:
        try:
            ips = [
                info[4][0]
                for info in socket.getaddrinfo(
                    socket.gethostname(), None, socket.AF_INET
                )
            ]
        except Exception:  # noqa: BLE001
            ips = []
    deduped: list[str] = []
    for ip in ips:
        if ip.startswith("127.") or ip in deduped:
            continue
        deduped.append(ip)
    return deduped


def announce_preview_urls(port: int) -> None:
    say("")
    say("=" * 72)
    say("LIVE WRIST-CAMERA PREVIEW (MJPEG) - open in a browser:")
    ips = best_effort_host_ips()
    for ip in ips:
        say(f"    http://{ip}:{port}/")
    if not ips:
        say(f"    http://<this-host-ip>:{port}/   (could not auto-detect host IPs)")
    say(f"    http://127.0.0.1:{port}/   (on this host)")
    say("use the tailscale IP from another machine.  Green banner")
    say(
        f"'board OK: {BOARD_CORNER_COUNT}/{BOARD_CORNER_COUNT} corners' = "
        "the checkerboard is fully in frame."
    )
    say("=" * 72)
    say("")


def print_board_status(cache: DetectionCache | None, pump: FramePump) -> None:
    """One-shot terminal report of the checkerboard-detection status ('p')."""
    if cache is not None:
        result = cache.get()
        if result is None:
            say("board status: no detection result yet (preview still warming up)")
            return
    else:  # --no-preview: run one detection inline on the latest pump frame
        frame = pump.latest()
        if frame is None:
            say("board status: no camera frame available yet")
            return
        result = detect_board(frame["rgb"])
        result["frame_sequence"] = int(frame["pump_sequence"])
        result["checked_monotonic"] = time.monotonic()
    age_s = time.monotonic() - result["checked_monotonic"]
    where = f"(frame #{result['frame_sequence']}, checked {age_s:.1f} s ago)"
    if result.get("error"):
        say(f"board status: detection ERROR {where}: {result['error']}")
    elif result["found"]:
        say(f"board OK: {result['count']}/{BOARD_CORNER_COUNT} corners {where}")
    else:
        say(
            f"board NOT fully detected ({result['count']}/{BOARD_CORNER_COUNT}) "
            f"{where}"
        )


# --------------------------------------------------------------------------
# Robot helpers
# --------------------------------------------------------------------------
def read_command_space_joints(robot) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Measured (pos, vel, eff) 7-vectors in command space + read timestamp.

    pos = [joint1..joint6 rad, gripper fraction under the override limits].
    """
    monotonic_ns = time.monotonic_ns()
    obs = robot.get_observations()
    pos = np.concatenate([obs["joint_pos"], obs["gripper_pos"]]).astype(np.float64)
    vel = np.concatenate([obs["joint_vel"], obs["gripper_vel"]]).astype(np.float64)
    eff = np.concatenate([obs["joint_eff"], obs["gripper_eff"]]).astype(np.float64)
    if pos.shape != (7,):
        raise RuntimeError(f"expected 7-DOF joint state, got shape {pos.shape}")
    return pos, vel, eff, monotonic_ns


# Max tolerated measured-vs-clip-range excess before a hold is refused (rad).
HOLD_CLIP_TOLERANCE_RAD = 0.02


def hold_command_range(robot) -> np.ndarray:
    """The exact (6, 2) arm-joint range i2rt clips position commands to.

    MotorChainRobot._clip_robot_joint_pos_command clips arm joints to the
    robot's ``joint_limits``, which get_yam_robot builds as the model XML
    ranges widened by its 0.15 rad safety buffer (get_robot.py:216-217).
    ``get_robot_info()["joint_limits"]`` returns that very array, so no
    constant is duplicated here.
    """
    limits = np.asarray(robot.get_robot_info()["joint_limits"], dtype=np.float64)
    if limits.shape != (6, 2):
        raise RuntimeError(f"expected (6, 2) arm joint limits, got {limits.shape}")
    return limits


def command_hold_at_measured(robot) -> np.ndarray | None:
    """The ONLY position command this script may send: hold the measured pose.

    Returns None WITHOUT sending any command when a measured arm joint sits
    more than HOLD_CLIP_TOLERANCE_RAD outside the command clip range: the
    "hold" would silently command the CLIPPED position and the arm would jump
    toward it.  While dragging, a joint can legally rest up to a further
    0.1 rad outside the clip range (the runtime limit check's buffer_rad), so
    this is reachable in normal use near a soft limit.
    """
    pos, _, _, _ = read_command_space_joints(robot)
    limits = hold_command_range(robot)
    lower_excess = limits[:, 0] - pos[:6]
    upper_excess = pos[:6] - limits[:, 1]
    excess = np.maximum(np.maximum(lower_excess, upper_excess), 0.0)
    if np.any(excess > HOLD_CLIP_TOLERANCE_RAD):
        say("REFUSING the hold: measured joints outside the command clip range:")
        for index in np.nonzero(excess > HOLD_CLIP_TOLERANCE_RAD)[0]:
            side = "lower" if lower_excess[index] > upper_excess[index] else "upper"
            say(
                f"  joint{index + 1}: {np.degrees(excess[index]):.2f} deg beyond "
                f"the {side} command limit (clip range "
                f"{np.degrees(limits[index, 0]):.2f} .. "
                f"{np.degrees(limits[index, 1]):.2f} deg)"
            )
        say("a hold here would command the CLIPPED position and the arm would")
        say("jump toward it.  NO hold was installed - staying in gravity mode:")
        say("drag the listed joint(s) slightly away from the limit by hand,")
        say("then press 'h' again.")
        return None
    robot.command_joint_pos(pos.copy())
    return pos


def _chain_alive(robot) -> bool:
    """True iff the i2rt threads that actually talk to the motors are alive.

    After a CAN/motor fail-fast the driver threads die silently but
    get_observations keeps returning cached joints.  Attribute paths verified
    against the vendored i2rt sources:
      * DMChainCanInterface.running          (dm_driver.py: set False on any
        control-loop/motor error just before the thread dies)
      * DMChainCanInterface._control_thread  (dm_driver.py start_thread)
      * MotorChainRobot._server_thread       (motor_chain_robot.py; its
        start_server raises as soon as motor_chain.running goes False)
    Every access uses getattr so a missing attribute degrades to a warning,
    never a crash.
    """
    checks: list[bool] = []
    chain = getattr(robot, "motor_chain", None)
    if chain is None:
        say("WARNING: robot has no motor_chain attribute; cannot verify chain liveness.")
    else:
        running = getattr(chain, "running", None)
        if running is None:
            say("WARNING: motor chain has no 'running' flag; cannot verify it.")
        else:
            checks.append(bool(running))
        control_thread = getattr(chain, "_control_thread", None)
        if control_thread is None:
            say("WARNING: motor chain has no _control_thread; cannot verify it.")
        else:
            checks.append(bool(control_thread.is_alive()))
    server_thread = getattr(robot, "_server_thread", None)
    if server_thread is None:
        say("WARNING: robot has no _server_thread; cannot verify it.")
    else:
        checks.append(bool(server_thread.is_alive()))
    return all(checks) if checks else True


def ensure_chain_alive(robot, moment: str) -> None:
    """os._exit(1) with an unmissable message if the motor chain is dead."""
    if _chain_alive(robot):
        return
    say("")
    say("!" * 72)
    say(f"!! MOTOR CHAIN DEAD (checked {moment}).")
    say("!! The driver threads are no longer running: joint readings are")
    say("!! CACHED and the arm is NOT being held.")
    say("!! SUPPORT THE ARM BY HAND / PRESS THE E-STOP NOW.")
    say("!" * 72)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)


# --------------------------------------------------------------------------
# Capture
# --------------------------------------------------------------------------
def capture_npz(
    pump: FramePump,
    robot,
    fk: LeftArmFK,
    flange_from_camera: np.ndarray,
    *,
    frames: int,
    interval_s: float,
    label: str | None,
) -> Path:
    from agp_yam_bridge.camera_acceptance import _pose_jitter, _save_capture

    # The pump continuously drains the pipeline, so nothing stale is buffered
    # (this replaces the old drain(3)).  Starting from the PUMP sequence at
    # burst entry, every sample waits for a STRICTLY greater pump sequence,
    # so frames are distinct by construction.  The device frame number is
    # NOT used: this D405's aligned frame counter can freeze mid-stream.
    last_sequence = pump.latest_sequence()
    observations = []
    for index in range(frames):
        frame = pump.wait_for_frame_after(last_sequence)
        last_sequence = int(frame["pump_sequence"])
        pos, vel, eff, joint_monotonic_ns = read_command_space_joints(robot)
        base_from_gripper = fk.base_from_gripper(pos[:6])
        # world == left_base (identity): camera_to_world = T_world_gripper @ CAD
        camera_to_world = base_from_gripper @ flange_from_camera
        observations.append(
            {
                "camera_0": {
                    "serial": pump.serial,
                    "images": {"rgb": frame["rgb"], "depth": frame["depth_m"]},
                    "intrinsics": frame["intrinsics"],
                    "camera_to_world": camera_to_world,
                    "frame_sequence": frame["pump_sequence"],
                    "frame_monotonic_ns": frame["frame_monotonic_ns"],
                    "joint_time_delta_ns": abs(
                        frame["frame_monotonic_ns"] - joint_monotonic_ns
                    ),
                },
                "robot_joint_pos_0": pos.astype(np.float32),
                "robot_joint_vel_0": vel.astype(np.float32),
                "robot_joint_effort_0": eff.astype(np.float32),
            }
        )
        if index + 1 < frames:
            time.sleep(interval_s)

    path = next_pose_path(OUT_DIR, label)
    try:
        _save_capture(path, observations)
        with np.load(path, allow_pickle=False) as saved:
            jitter = _pose_jitter(saved["camera_to_world"])
            joint_delta_ms = float(saved["joint_time_delta_ns"].max() / 1e6)
    except Exception:
        path.unlink(missing_ok=True)  # never leave a partial npz behind
        raise
    say(f"saved {path}")
    say(
        f"  frames={jitter['frame_count']}  "
        f"pose jitter: {jitter['max_translation_jitter_m'] * 1000:.3f} mm / "
        f"{jitter['max_rotation_jitter_deg']:.4f} deg  "
        f"(stillness gate: 2 mm / 0.5 deg)  "
        f"max camera-joint skew: {joint_delta_ms:.1f} ms"
    )
    if (
        jitter["max_translation_jitter_m"] > 0.002
        or jitter["max_rotation_jitter_deg"] > 0.5
    ):
        say("  WARNING: jitter above the stillness gate; consider re-capturing this pose.")
    return path


# --------------------------------------------------------------------------
# Dry run
# --------------------------------------------------------------------------
def run_dry_run() -> int:
    """Exercise the FK math offline against a hand computation; no hardware.

    Hand computation at zero joints:
      * combined-model flange pose: R_bg = [[0,0,-1],[0,-1,0],[-1,0,0]],
        t_bg = (0.110597, 0, 0.173502)  (chain sums of the yam v1 MJCF)
      * CAD T_leftgripper_leftcamera: t_gc = (-0.070435, 0, -0.077006),
        R_gc = [[0, 0.906303, 0.422628], [1, 0, 0], [0, 0.422628, -0.906303]]
        (from quat wxyz (0.153051, 0.690345, 0.690345, 0.153048))
      * camera_to_world = T_bg @ T_gc:
        R rows: (0,-0.422628,0.906303), (-1,0,0), (0,-0.906303,-0.422628)
        t = R_bg @ t_gc + t_bg = (0.077006+0.110597, 0, 0.070435+0.173502)
          = (0.187603, 0, 0.243937)

    The GATED check runs FK against the CAD constants so the hand computation
    stays valid forever.  The station XML transform (what captures actually
    use) is printed INFORMATIONALLY next to CAD: since the measured 2026-08-31
    hand-eye was installed into the station model, a ~5 mm / ~2.1 deg
    difference from CAD is EXPECTED there, not an error.
    """
    import mujoco
    from scipy.spatial.transform import Rotation

    fk = LeftArmFK()
    flange_from_camera = station_left_flange_from_camera()

    # 1. INFORMATIONAL: station transform vs the CAD constants.
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, flange_from_camera[:3, :3].reshape(-1))
    if quaternion[0] < 0:
        quaternion = -quaternion
    cad = np.eye(4)
    cad_rotation = np.empty(9)
    mujoco.mju_quat2Mat(
        cad_rotation, CAD_FLANGE_CAMERA_WXYZ / np.linalg.norm(CAD_FLANGE_CAMERA_WXYZ)
    )
    cad[:3, :3] = cad_rotation.reshape(3, 3)
    cad[:3, 3] = CAD_FLANGE_CAMERA_XYZ
    station_from_cad = np.linalg.inv(cad) @ flange_from_camera
    delta_translation_mm = float(np.linalg.norm(station_from_cad[:3, 3])) * 1000.0
    delta_rotation_deg = float(
        np.degrees(Rotation.from_matrix(station_from_cad[:3, :3]).magnitude())
    )
    say(f"station T_leftgripper_leftcamera translation: {flange_from_camera[:3, 3]}")
    say(f"station T_leftgripper_leftcamera quat (wxyz): {quaternion}")
    say(f"CAD     T_leftgripper_leftcamera translation: {CAD_FLANGE_CAMERA_XYZ}")
    say(f"CAD     T_leftgripper_leftcamera quat (wxyz): {CAD_FLANGE_CAMERA_WXYZ}")
    say(
        f"station-vs-CAD delta: {delta_translation_mm:.2f} mm / "
        f"{delta_rotation_deg:.3f} deg  (INFORMATIONAL: ~4.85 mm / ~2.15 deg "
        "expected since the measured 2026-08-31 hand-eye install)"
    )

    # 2. GATED: camera_to_world at zero joints (FK @ CAD constants) vs the
    #    hand computation above.
    zero_joints = np.zeros(6)
    camera_to_world = fk.base_from_gripper(zero_joints) @ cad
    expected = np.array(
        [
            [0.0, -0.422628, 0.906303, 0.187603],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -0.906303, -0.422628, 0.243937],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    np.set_printoptions(suppress=True, precision=6)
    say("camera_to_world at zero joints (FK @ CAD constants):")
    say(str(camera_to_world))
    say("expected (hand computation):")
    say(str(expected))
    max_abs_error = float(np.max(np.abs(camera_to_world - expected)))
    say(f"max |difference|: {max_abs_error:.2e}")
    ok = max_abs_error < 1e-3
    say(f"DRY-RUN {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# Preview selftest (no camera, no robot)
# --------------------------------------------------------------------------
def _synthetic_board_rgb(size: tuple[int, int] | None = None) -> np.ndarray:
    """RGB test card with a full 10x8-square (9x7 inner) checkerboard.

    ``size`` = (height, width); default = the wrist D405 frame (360x640).
    The square size scales with the width so the board keeps its layout at
    the 1920x1080 top-camera profile.
    """
    height, width = size if size is not None else (CAMERA_HEIGHT, CAMERA_WIDTH)
    square = max(8, int(round(36 * width / CAMERA_WIDTH)))
    columns_squares = BOARD_COLUMNS + 1
    rows_squares = BOARD_ROWS + 1
    image = np.full((height, width, 3), 220, dtype=np.uint8)
    x0 = (width - columns_squares * square) // 2
    y0 = (height - rows_squares * square) // 2
    for row in range(rows_squares):
        for column in range(columns_squares):
            if (row + column) % 2 == 0:
                image[
                    y0 + row * square : y0 + (row + 1) * square,
                    x0 + column * square : x0 + (column + 1) * square,
                ] = 15
    return image


class _SyntheticSource:
    """Stands in for FramePump in --preview-selftest (no camera)."""

    def __init__(self, rgb: np.ndarray) -> None:
        self._rgb = rgb
        self._lock = threading.Lock()
        self._sequence = 0

    def latest(self) -> dict:
        with self._lock:
            self._sequence += 1
            return {
                "rgb": self._rgb,
                "pump_sequence": self._sequence,
                "frame_monotonic_ns": time.monotonic_ns(),
            }


def run_preview_selftest() -> int:
    """Exercise detection, overlay, workers, and the MJPEG endpoint offline.

    No camera, no robot, no fixed port: the server binds an ephemeral
    127.0.0.1 port so this can run while a real preview is up elsewhere.
    """
    import http.client

    checks: list[tuple[str, bool]] = []
    board_rgb = _synthetic_board_rgb()
    blank_rgb = np.full((CAMERA_HEIGHT, CAMERA_WIDTH, 3), 128, dtype=np.uint8)

    detection = detect_board(board_rgb)
    say(
        f"synthetic board: found={detection['found']} "
        f"corners={detection['count']}/{BOARD_CORNER_COUNT}"
    )
    checks.append(
        (
            f"board detected {BOARD_CORNER_COUNT}/{BOARD_CORNER_COUNT}",
            detection["found"] and detection["count"] == BOARD_CORNER_COUNT,
        )
    )
    blank_detection = detect_board(blank_rgb)
    say(
        f"blank frame:     found={blank_detection['found']} "
        f"corners={blank_detection['count']}/{BOARD_CORNER_COUNT}"
    )
    checks.append(("blank frame not detected", not blank_detection["found"]))

    jpeg = render_preview_jpeg(board_rgb, detection)
    checks.append(("overlay renders a JPEG", jpeg[:2] == b"\xff\xd8"))
    checks.append(
        ("overlay of a blank frame renders too",
         render_preview_jpeg(blank_rgb, blank_detection)[:2] == b"\xff\xd8")
    )

    # The real worker threads + real HTTP handler against the synthetic source.
    source = _SyntheticSource(board_rgb)
    cache = DetectionCache()
    state = PreviewState()
    detection_worker = DetectionWorker(source, cache)
    encoder_worker = EncoderWorker(source, cache, state)
    detection_worker.start()
    encoder_worker.start()
    deadline = time.monotonic() + 10.0
    cached: dict | None = None
    while time.monotonic() < deadline:
        cached = cache.get()
        if cached is not None and state.wait_for_jpeg(0, timeout=0.25)[0] is not None:
            break
    checks.append(
        (
            "detection worker cached a full-board result",
            cached is not None and cached.get("found") is True,
        )
    )

    server = start_preview_server("127.0.0.1", 0, state)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5.0)
    connection.request("GET", "/")
    response = connection.getresponse()
    content_type = response.getheader("Content-Type", "") or ""
    checks.append(("HTTP 200 on /", response.status == 200))
    checks.append(
        ("multipart/x-mixed-replace stream", "multipart/x-mixed-replace" in content_type)
    )
    body = b""
    stream_deadline = time.monotonic() + 5.0
    while time.monotonic() < stream_deadline and len(body) < 262144:
        try:
            chunk = response.read(4096)
        except TimeoutError:
            break
        if not chunk:
            break
        body += chunk
        if b"\xff\xd8" in body and body.count(b"--frame") >= 2:
            break
    checks.append(
        (
            "stream carries JPEG parts",
            b"Content-Type: image/jpeg" in body and b"\xff\xd8" in body,
        )
    )
    connection.close()

    not_found = http.client.HTTPConnection("127.0.0.1", port, timeout=5.0)
    not_found.request("GET", "/nope")
    checks.append(("404 on unknown path", not_found.getresponse().status == 404))
    not_found.close()

    server.shutdown()
    server.server_close()
    detection_worker.stop()
    encoder_worker.stop()

    ok = all(passed for _, passed in checks)
    for name, passed in checks:
        say(f"  [{'ok' if passed else 'FAIL'}] {name}")
    say(f"PREVIEW-SELFTEST {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# Robot startup (shared with capture_top_pairs.py - import, do not copy)
# --------------------------------------------------------------------------
def port_in_use(port: int, host: str = "127.0.0.1") -> bool:
    """True iff something accepts TCP connections on host:port (a bridge)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.3)
        return sock.connect_ex((host, port)) == 0


def refuse_if_bridge_listening(port: int, side: str) -> bool:
    """ONE CONTROLLER PER CAN BUS / ONE OWNER PER CAMERA.

    A running bridge holds the arm's CAN channel and both of its cameras.
    Returns True (after printing an unmissable message) when something is
    listening on the side's bridge port; the caller must then exit WITHOUT
    touching any hardware.
    """
    if not port_in_use(port):
        return False
    say("")
    say("!" * 72)
    say(f"!! something is listening on 127.0.0.1:{port} - the {side.upper()} bridge appears")
    say("!! to be running.  It owns the CAN channel and the cameras of that rig.")
    if side == "right":
        say(f"!! This is the RIGHT-arm bridge (port {port}): stop it first")
        say("!! (support the arm, Ctrl-C its serve process); never kill it from here.")
    else:
        say("!! Stop the bridge first (support the arm, Ctrl-C), then retry.")
    say("!! REFUSING to start; no hardware was touched.")
    say("!" * 72)
    return True


def enable_arm(channel: str):
    """Enable the arm on ``channel``, entering HOLD at the measured position
    (zero_gravity_mode=False) with the gripper auto-calibration wiggle
    skipped via GRIPPER_LIMITS_OVERRIDE.

    i2rt applies the per-channel encoder-zero offsets (yam_v1.yml
    ``motor_offsets_deg_by_channel``) by channel name, so the RIGHT rig's
    joint-4 offset row is picked up automatically for can_follower_r.

    On ANY startup failure this never returns: it exits through
    die_without_motion WITHOUT commanding motion, reporting whether an
    emergency hold is active on the motors.
    """
    from i2rt.robots.get_robot import YamStartupHoldingError, get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType

    try:
        return get_yam_robot(
            channel=channel,
            arm_type=ArmType.YAM,
            gripper_type=GripperType.LINEAR_4310,
            # False => __init__ installs a hold at the measured position
            # instead of dropping straight into gravity comp.
            zero_gravity_mode=False,
            gripper_limits_override=GRIPPER_LIMITS_OVERRIDE,
        )
    except YamStartupHoldingError as exc:
        say("!! i2rt startup failed while an emergency position hold "
            f"{'IS' if exc.hold_active else 'is NOT'} active on the motors.")
        die_without_motion("hold" if exc.hold_active else "unknown", exc)
    except BaseException as exc:  # noqa: BLE001
        die_without_motion("unknown (startup failed before any hold)", exc)


def enable_left_arm():
    """Enable the LEFT arm on can_follower_l (wrapper kept for existing callers)."""
    return enable_arm(LEFT_CAN_CHANNEL)


# --------------------------------------------------------------------------
# Main interactive loop
# --------------------------------------------------------------------------
HELP_TEXT = """commands (press Enter after each):
  g           gravity-comp idle - drag the arm by hand
  h           hold - freeze at the CURRENT measured joint position
  c [label]   capture 30 frames (hold only), e.g.: c tilt_left
  p           print the current checkerboard-detection status
  q           quit (arm stays in HOLD; then e-stop / take over)
"""


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Standalone left-arm hand-eye capture (no bridge)"
    )
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--interval-s", type=float, default=0.1)
    parser.add_argument(
        "--preview-port",
        type=int,
        default=8766,
        help="MJPEG preview port (default 8766; an earlier camera_preview.py, "
        "not shipped here, used 8765)",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="disable the browser preview server ('p' still detects inline)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="skip robot+camera; verify FK math on a fixed joint vector",
    )
    parser.add_argument(
        "--preview-selftest",
        action="store_true",
        help="skip robot+camera; exercise the MJPEG server and board overlay "
        "on synthetic frames (binds an ephemeral localhost port)",
    )
    args = parser.parse_args()
    if args.frames <= 0 or args.interval_s < 0:
        parser.error("--frames must be positive and --interval-s non-negative")
    if not 1 <= args.preview_port <= 65535:
        parser.error("--preview-port must be in 1..65535")

    if args.dry_run:
        return run_dry_run()
    if args.preview_selftest:
        return run_preview_selftest()

    say("== left-arm hand-eye capture ==")
    say("loading kinematics (combined YAM+linear_4310 model, station CAD) ...")
    fk = LeftArmFK()
    flange_from_camera = station_left_flange_from_camera()

    say(f"opening left D405 {LEFT_CAMERA_SERIAL} "
        f"({CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_FPS}, depth aligned to color) ...")
    camera = LeftD405()
    payload = camera.intrinsics_payload()
    INTRINSICS_JSON.parent.mkdir(parents=True, exist_ok=True)
    INTRINSICS_JSON.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    say(f"factory color intrinsics written to {INTRINSICS_JSON}")
    say(f"  distortion_model: {payload['distortion_model']}")
    say(f"  distortion_coefficients: {payload['distortion_coefficients']}")
    if payload["distortion_model"] != "inverse_brown_conrady":
        say("  WARNING: distortion model is not inverse_brown_conrady; "
            "check solve_left.sh before solving.")

    pump = FramePump(camera)
    pump.start()
    pump.wait_until_ready(min_frames=10, timeout_s=10.0)  # warmup (was drain(10))

    detection_cache: DetectionCache | None = None
    if args.no_preview:
        say("browser preview disabled (--no-preview); the 'p' command still")
        say("runs a one-shot board detection in the terminal.")
    else:
        detection_cache = DetectionCache()
        preview_state = PreviewState()
        DetectionWorker(pump, detection_cache).start()
        EncoderWorker(pump, detection_cache, preview_state).start()
        try:
            server = start_preview_server("0.0.0.0", args.preview_port, preview_state)
        except OSError as exc:
            say("")
            say(f"WARNING: preview server could not bind port {args.preview_port} "
                f"({exc}).")
            say("Continuing WITHOUT the browser preview (is another preview")
            say("already using the port?); the 'p' command still reports the")
            say("board-detection status in the terminal.")
        else:
            threading.Thread(
                target=server.serve_forever, name="preview-http", daemon=True
            ).start()
            announce_preview_urls(args.preview_port)

    say("")
    say("=" * 72)
    say("ABOUT TO ENABLE THE LEFT ARM ON can_follower_l")
    say("  * The arm will IMMEDIATELY HOLD its current position (no motion).")
    say("  * The gripper auto-calibration wiggle is SKIPPED "
        "(gripper_limits_override).")
    say("  * Keep a hand near the e-stop.")
    say("=" * 72)
    answer = input("type 'yes' to enable the arm (anything else aborts): ").strip()
    if answer.lower() != "yes":
        say("aborted before touching the robot.")
        pump.stop()
        camera.close()
        return 1

    state = "hold"
    robot = enable_left_arm()

    try:
        dofs = robot.num_dofs()
        if dofs != 7:
            raise RuntimeError(f"expected a 7-DOF YAM+gripper, got {dofs}")
        pos, _, _, _ = read_command_space_joints(robot)
        say(f"arm enabled, HOLDING at joints (deg): "
            f"{np.degrees(pos[:6]).round(2).tolist()}")
        say(HELP_TEXT)

        while True:
            try:
                line = input(f"[{state}] g/h/c [label]/p/q > ").strip()
            except EOFError:
                line = "q"
            if not line:
                say(HELP_TEXT)
                continue
            parts = line.split(maxsplit=1)
            command = parts[0].lower()
            argument = parts[1] if len(parts) > 1 else None

            if command == "g":
                if state == "hold":
                    input("请先扶住手臂，按回车后释放为重力补偿模式 > ")
                robot.enter_gravity_comp_idle()
                state = "gravity"
                say("gravity-comp idle: you can drag the arm by hand now.")
            elif command == "h":
                ensure_chain_alive(robot, "before entering hold")
                held = command_hold_at_measured(robot)
                if held is None:
                    continue  # refused near a soft limit; stay in gravity mode
                state = "hold"
                say(f"holding at joints (deg): "
                    f"{np.degrees(held[:6]).round(2).tolist()}")
            elif command == "c":
                if state != "hold":
                    say("capture is only allowed in HOLD - press 'h' first.")
                    continue
                ensure_chain_alive(robot, "before capture")
                try:
                    capture_npz(
                        pump,
                        robot,
                        fk,
                        flange_from_camera,
                        frames=args.frames,
                        interval_s=args.interval_s,
                        label=argument,
                    )
                except Exception as exc:  # camera hiccup etc.; hold untouched
                    say(f"capture FAILED ({type(exc).__name__}: {exc})")
                    say("partial capture discarded; the arm HOLD is untouched.")
                    say("fix the camera/board and press 'c' again.")
            elif command == "p":
                # Read-only status print; never touches the robot.
                print_board_status(detection_cache, pump)
            elif command == "q":
                ensure_chain_alive(robot, "on quit")
                if state != "hold":
                    held = command_hold_at_measured(robot)
                    if held is None:
                        say("quit aborted: no hold could be installed - still in")
                        say("gravity mode; move away from the limit, then retry.")
                        continue
                    state = "hold"
                    say(f"switched to HOLD at joints (deg): "
                        f"{np.degrees(held[:6]).round(2).tolist()}")
                say("")
                say("quit: the arm REMAINS IN HOLD (motors keep executing the")
                say("last position command onboard). You may now press the")
                say("e-stop or take over the arm, then power off when done.")
                sys.stdout.flush()
                os._exit(0)
            else:
                say(HELP_TEXT)
    except BaseException as exc:  # noqa: BLE001  (includes KeyboardInterrupt)
        die_without_motion(state, exc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
