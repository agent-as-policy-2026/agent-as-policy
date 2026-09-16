#!/usr/bin/env python3
"""AgP — REAL YAM ARM server (file bridge; left arm by default).

Watches <session>/bridge/ for req_NNNNNN.json written by robot_client.py,
executes them ONE AT A TIME against the real left arm through the external real
connector (gap.connector.real("yam_left") -> hardware bridge on :9021), and
writes resp_NNNNNN.json. Responses contain only images, calibration, robot
proprioception and command outcomes — no task knowledge, no scaffolding.

Two arms (2026-09-08) — one server process PER ARM, both watching the same session:
  --arm left       (default) exactly the server above: bridge/, frames/, server.log,
                   connector "yam_left", port 9021, poses in left_base. Unchanged code path.
  --arm right      connector "yam_right" (port 9022), bridge_right/, frames_right/,
                   server_right.log. The right bridge reports poses in right_base; this
                   server converts EVERY pose it returns (state / move responses / calib
                   camera poses / _meta) into the WORLD frame = left_base and EVERY target
                   it accepts (move_ee, move_delta) from the world frame back into
                   right_base, where the r/step guards are evaluated (the z/table guard
                   is checked on the world-frame target: the table plane is a left_base
                   measurement). The transform
                   T (pose of right_base in left_base) comes from --base-transform-file,
                   else agp/config/right_base_in_left_base.json if present, else
                   the nominal station offset (0, -0.61, 0) / identity rotation.
                   Cameras: 'wrist' only — 'top'/'side' answer ok:false (the overhead
                   camera is served by the left arm). SERVER_STOP is shared.
  --arm right --standalone   (2026-09-09) the right arm as the ONLY arm of its session, run
                   concurrently with an independent left session: the single-arm layout
                   (bridge/, frames/, server.log), NO base transform (poses in right_base,
                   which is this session's "robot base frame"), and BOTH cameras — the
                   right bridge's own overhead BRIO (B8C7F203, 640x360, calibrated in
                   right_base by the bridge's acceptance calibration) is served as 'top'.
                   Every text is the single-arm server's; status/READY name the arm.

Modes
  default          OBSERVATION-ONLY: frames / state / deproject / status / help.
                   Every motion command is refused locally with READ_ONLY
                   (and a read-only bridge would refuse it too).
  --allow-motion   motion enabled; refuses to start unless the bridge itself
                   was started with --enable-motion.
  --bare           ablation interface (2026-09-07): the convenience commands
                   deproject / home / move_delta / reset do not exist (they answer
                   like any unknown command), help/status/notes/error texts carry
                   no interpretation or advice, and status omits table_z. The
                   physics guards, budget, frames/state/move_ee/move_joints/gripper
                   and every safety behaviour are IDENTICAL to the full interface.
                   Default off = the interface used by every experiment before.

Only physics guards are enforced here (all CLI args): reach annulus, floor,
ceiling, max straight-line step. Everything else (joint limits, speed caps,
heartbeat, e-stop) is the bridge's job.

Launch (from the session dir; the bridge venv python, which imports the vendored connector; no
sim env vars; both paths are relative to the repo root):
  PYTHONDONTWRITEBYTECODE=1 hardware-bridge/.venv/bin/python \
      agp/server_real.py --session <dir> [--allow-motion]

Connector facts this file relies on (gap/connector/real.py, gap/envs/yam_real_env.py):
  * ONLY conn._tool_go_to_pose / _tool_go_to_pose_cartesian / _tool_move_to_joints /
    open_gripper / close_gripper / env._set_gripper reach the bridge. The public
    conn.go_to_pose* / move_to_joints are sim base-class paths that raise.
  * EE poses (in and out) are in the sim `tcp_gap` convention: (w,x,y,z)=(0,1,0,0)
    = tool straight down, +z out of the fingers, fingers close along tool y.
  * Camera poses are camera_to_world in the bridge world == that arm's OWN base
    frame (left: no conversion; right: converted here into left_base by BaseTransform).
    top_brio has NO depth key.
  * Exactly ONE bridge observation client at a time (never conn.start_video).
"""
import argparse
import itertools
import json
import math
import os
import sys
import time
import traceback
import uuid

import numpy as np

import gap.connector  # noqa: E402
from gap.connector.real import bridge_quat_wxyz_to_sim  # noqa: E402
from gap.envs.robot_specs import OBSERVE_MAIN_JOINTS, YAM_REAL_LEFT_SPEC  # noqa: E402
from gap.envs.yam_real_env import YamBridgeError, YamDisconnectedError  # noqa: E402
from gap_core.errors import ToolError  # noqa: E402

ALIAS = {"wrist": "wrist_d405", "top": "top_brio"}          # agent name -> bridge camera name
SYNONYM = {"side": "top", "wrist_d405": "wrist", "top_brio": "top"}
RALIAS = {v: k for k, v in ALIAS.items()}
FREE_CMDS = {"status", "help", "state", "deproject", "preview_program", "program_report"}
MOTION_CMDS = {"move_ee", "move_delta", "move_joints", "home", "gripper", "run_program"}
# 2026-09-14 buffered joint programs (--programs, throwing sessions on a bridge started with
# config/left_arm_throw.yaml): these three commands exist ONLY then; elsewhere they are unknown.
PROGRAM_CMDS = {"preview_program", "run_program", "program_report"}
BARE_HIDDEN = {"deproject", "home", "move_delta", "reset"}       # --bare: not part of the interface
MX_CMDS = {"episode", "experience"}   # multiple-experience sessions only (free; agp/mx/), unknown elsewhere

SPEC = YAM_REAL_LEFT_SPEC
MAX_OPEN = float(SPEC.gripper.max_opening_m)                   # 0.0955 m (calipers)
TABLE_Z = float(SPEC.workspace.table_z)                        # -0.045 m (board-plane fit)
CART_V = 0.03                                                  # bridge cap, m/s
CART_W = math.radians(10.0)                                    # bridge cap, rad/s

# ---------- two-arm additions (2026-09-08; left arm = defaults, untouched) ----------
ARM_DEFAULT_PORT = {"left": 9021, "right": 9022}
WORLD_FRAME = "left_base"                                      # the agents' one world frame
NOMINAL_RIGHT_BASE_IN_LEFT_BASE = {                            # station README: left_base -> right_base
    "translation": [0.0, -0.61, 0.0],
    "rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
    "source": "nominal station offset (i2rt station README: left_base -> right_base = 0 -0.61 0 / quat 1 0 0 0)",
}
DEFAULT_BASE_TRANSFORM_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                           "config", "right_base_in_left_base.json")
RIGHT_TOP_ERROR = "the overhead camera is served by the left arm (use --arm left frames)"


# ---------- small helpers (sim server verbatim unless noted) ----------
def _f(x):
    return float(x)


def _pose_dict(p):
    """Tolerant pose -> {position:{xyz}, rotation:{wxyz}}."""
    if p is None:
        return None
    if isinstance(p, dict):
        d = p.get("pose") or p
        pos, rot = d.get("position"), d.get("rotation")
    else:
        pos, rot = getattr(p, "position", None), getattr(p, "rotation", None)

    def _xyz(v, keys):
        if v is None:
            return None
        if isinstance(v, dict):
            return {k: _f(v[k]) for k in keys}
        arr = np.asarray(v, dtype=float).ravel()
        return {k: _f(arr[i]) for i, k in enumerate(keys)}
    return {"position": _xyz(pos, "xyz"), "rotation": _xyz(rot, "wxyz")}


def _quat_arr(rot):
    return np.array([rot["w"], rot["x"], rot["y"], rot["z"]], dtype=float)


def _rot_delta_mat(deg_xyz):
    rx, ry, rz = [math.radians(d) for d in deg_xyz]
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _wxyz_to_mat(q):
    """Unit quaternion (w,x,y,z) -> 3x3 rotation matrix. Self-contained (no skills-tree import)."""
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_wxyz(R):
    """3x3 rotation matrix -> unit quaternion (w,x,y,z) (Shepperd's method)."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    return q / (np.linalg.norm(q) or 1.0)


def _quat_mul(a, b):
    """Hamilton product of two scalar-first quaternions (w,x,y,z): rotation a AFTER b."""
    aw, ax, ay, az = [float(v) for v in a]
    bw, bx, by, bz = [float(v) for v in b]
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw], dtype=float)


def _quat_conj(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([w, -x, -y, -z], dtype=float)


class BaseTransform:
    """Rigid transform T = pose of an arm's base in the WORLD frame (= left_base).

    Pure numpy, no gap import. Both directions live here so they cannot drift apart:
        to world   : p_w = R @ p_b + t          q_w = q_T * q_b        (scalar-first, Hamilton)
        from world : p_b = R^T @ (p_w - t)      q_b = conj(q_T) * q_w
    Free vectors (move_delta dpos) rotate without the translation. Pose dicts are the
    server's {"position": {x,y,z}, "rotation": {w,x,y,z}} shape; None passes through.
    Only the RIGHT server instantiates one — the left server keeps self.T = None and its
    conversion wrappers return the very same object (no arithmetic, no rounding).
    """

    def __init__(self, translation, rotation_wxyz, source="nominal", date=None, path=None):
        t = np.asarray(translation, dtype=float).ravel()
        q = np.asarray(rotation_wxyz, dtype=float).ravel()
        if t.shape != (3,) or not np.isfinite(t).all():
            raise ValueError("translation must be 3 finite numbers [x, y, z] (metres)")
        if q.shape != (4,) or not np.isfinite(q).all():
            raise ValueError("rotation_wxyz must be 4 finite numbers [w, x, y, z]")
        n = float(np.linalg.norm(q))
        if abs(n - 1.0) > 1e-3:
            raise ValueError(f"rotation_wxyz must be a unit quaternion (norm {n:.6f})")
        self.t = t
        self.q = q / n
        self.R = _wxyz_to_mat(self.q)
        self.source = str(source)
        self.date = None if date is None else str(date)
        self.path = None if path is None else str(path)

    @classmethod
    def nominal(cls):
        d = NOMINAL_RIGHT_BASE_IN_LEFT_BASE
        return cls(d["translation"], d["rotation_wxyz"], source=d["source"])

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            d = json.load(f)
        if not isinstance(d, dict) or "translation" not in d:
            raise ValueError('expected a JSON object with "translation" and "rotation_wxyz"')
        rot = d.get("rotation_wxyz")
        if rot is None and isinstance(d.get("rotation"), dict):
            rot = [d["rotation"][k] for k in "wxyz"]
        if rot is None:
            raise ValueError('missing "rotation_wxyz": [w, x, y, z]')
        return cls(d["translation"], rot, source=d.get("source", os.path.basename(path)),
                   date=d.get("date"), path=os.path.abspath(path))

    def describe(self):
        out = {"translation": [float(v) for v in self.t], "rotation_wxyz": [float(v) for v in self.q],
               "source": self.source}
        if self.date is not None:
            out["date"] = self.date
        if self.path is not None:
            out["file"] = self.path
        return out

    def angle_to_deg(self, other):
        """Rotation angle between this transform's rotation and another's."""
        d = _quat_mul(_quat_conj(other.q), self.q)
        return math.degrees(2.0 * math.acos(min(1.0, abs(float(d[0])))))

    # vectors / points / quaternions
    def point_to_world(self, p):
        return self.R @ np.asarray(p, dtype=float) + self.t

    def point_from_world(self, p):
        return self.R.T @ (np.asarray(p, dtype=float) - self.t)

    def vector_to_world(self, v):
        return self.R @ np.asarray(v, dtype=float)

    def vector_from_world(self, v):
        return self.R.T @ np.asarray(v, dtype=float)

    def quat_to_world(self, q):
        return _quat_mul(self.q, q)

    def quat_from_world(self, q):
        return _quat_mul(_quat_conj(self.q), q)

    # server pose dicts
    def _pose(self, pose, fp, fq):
        if pose is None:
            return None
        out = {}
        pos, rot = pose.get("position"), pose.get("rotation")
        if pos is not None:
            p = fp(np.array([pos["x"], pos["y"], pos["z"]], dtype=float))
            out["position"] = {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}
        else:
            out["position"] = None
        if rot is not None:
            q = fq(np.array([rot["w"], rot["x"], rot["y"], rot["z"]], dtype=float))
            out["rotation"] = {"w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])}
        else:
            out["rotation"] = None
        for k, v in pose.items():                      # keep any extra keys untouched
            if k not in ("position", "rotation"):
                out[k] = v
        return out

    def pose_to_world(self, pose):
        return self._pose(pose, self.point_to_world, self.quat_to_world)

    def pose_from_world(self, pose):
        return self._pose(pose, self.point_from_world, self.quat_from_world)


def _jsonable(o):
    """json.dump default: bridge results carry numpy scalars/arrays."""
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def _alias(c):
    return SYNONYM.get(c, c)


def _r4(x):
    return round(float(x), 4)


class BootError(RuntimeError):
    def __init__(self, msg, code):
        super().__init__(msg)
        self.code = code


class Server:
    def __init__(self, a):
        self.arm = str(getattr(a, "arm", None) or "left")
        if self.arm not in ARM_DEFAULT_PORT:
            raise BootError(f"--arm must be left or right, got {self.arm!r}", 2)
        # --standalone (2026-09-09): the right arm as the only arm of its session -> the single-arm
        # layout and texts of the left server, its own base as the frame, its own top camera
        self.standalone = bool(getattr(a, "standalone", False)) and self.arm == "right"
        self.solo = self.arm == "left" or self.standalone     # serves both cameras; frame = own base
        self.world_frame = "right_base" if self.standalone else WORLD_FRAME
        sfx = "" if self.solo else "_" + self.arm          # left (and standalone right) keeps today's names
        self.session = os.path.abspath(a.session)
        self.bridge = os.path.join(self.session, "bridge" + sfx)
        self.frames = os.path.join(self.session, "frames" + sfx)
        os.makedirs(self.bridge, exist_ok=True)
        os.makedirs(self.frames, exist_ok=True)
        os.makedirs(os.path.join(self.session, "scratch"), exist_ok=True)
        self.log_path = os.path.join(self.session, f"server{sfx}.log")
        # frame conversion: None for the left arm (its base IS the world frame; wrappers pass
        # objects through untouched); a BaseTransform (right_base in left_base) for the right arm
        self.T = None
        self.pose_frame = "robot_base"
        if self.arm == "right" and not self.standalone:
            self.T = self._load_base_transform(getattr(a, "base_transform_file", None))
            self.pose_frame = WORLD_FRAME
        elif self.standalone and getattr(a, "base_transform_file", None):
            print("NOTE: --base-transform-file is ignored with --standalone (right_base is this session's frame)", flush=True)
        elif getattr(a, "base_transform_file", None):
            print("NOTE: --base-transform-file is ignored for --arm left (left_base is the world frame)", flush=True)
        self.capture_seq = itertools.count(1)
        self.counted = 0
        # 2026-09-11 multiple-experience sessions (session.json "method": "multiple_experience", written by
        # run_real_probe.sh for --knowledge mx): the multiple-experience method's episode / experience commands, cycle guard
        # and command metrics (agp/mx/). Every other session has no session.json and is unchanged.
        self.mx = None
        _mx_path = os.path.join(self.session, "session.json")
        if os.path.exists(_mx_path):
            _mx_info = json.load(open(_mx_path))
            if _mx_info.get("method") == "multiple_experience":
                if type(_mx_info.get("cycle")) is not int or not 1 <= _mx_info["cycle"] <= 5:
                    raise BootError("session.json: multiple_experience needs an integer cycle 1..5", 2)
                # read_only (--knowledge mx-ro): another model reuses a finished sequence's memory; commits refused
                self.mx = {"cycle": _mx_info["cycle"], "read_only": _mx_info.get("read_only") is True}
        self.mx_capture, self.mx_capture_time = 0, 0.0
        self.mx_motion_revision, self.mx_capture_revision = 0, None
        self.allow_motion = bool(a.allow_motion)
        self.bare = bool(getattr(a, "bare", False))
        self.budget = int(a.budget)
        self.r_min, self.r_max = float(a.r_min), float(a.r_max)
        self.z_min, self.z_max = float(a.z_min), float(a.z_max)
        self.max_step = float(a.max_step_m)
        self.observe_joints = [float(x) for x in (a.observe_joints or OBSERVE_MAIN_JOINTS)]
        self.host = a.host
        self.port = int(a.port) if a.port is not None else ARM_DEFAULT_PORT[self.arm]
        # periodic top/wrist recorder (this process is the sole bridge observer, so a
        # second continuous stream is impossible; instead the single-threaded loop saves
        # a frame every --record-interval s BETWEEN agent commands — it pauses during a
        # blocking motion, which the side camera covers)
        self.record_dir = os.path.abspath(a.record_dir) if a.record_dir else None
        self.record_interval = float(a.record_interval)
        self._last_record = 0.0
        self._rec_i = itertools.count(1)
        if self.record_dir:
            for c in ("top", "wrist"):
                os.makedirs(os.path.join(self.record_dir, c), exist_ok=True)
            self._rec_ts = open(os.path.join(self.record_dir, "timestamps.csv"), "a")
            self._rec_ts.write("idx,wall_s,top,wrist\n")
        self.conn = None
        self._connect(float(a.wait_timeout_s))
        # 2026-09-14 buffered joint programs: --programs is honoured only by a bridge that answers the
        # program requests (started with program J4 limits); probe with an id that cannot exist
        self.programs = bool(getattr(a, "programs", False))
        if self.programs:
            try:
                self.conn.env.program_report("boot-probe")
            except YamBridgeError as e:
                if getattr(e, "code", "") != "PROGRAM_NOT_FOUND":
                    raise BootError(f"--programs requested but the bridge offers no joint programs ({e}); "
                                    "start it with config/left_arm_throw.yaml (bash start_bridges.sh start left throw)", 2)
            else:
                raise BootError("bridge answered a report for an unknown program id", 2)
        h = self._health()
        self.bridge_motion = bool(h.get("motion_enabled"))
        self.bridge_state = str(h.get("state"))
        self.safety_state = str(h.get("safety_state"))
        if self.allow_motion and not self.bridge_motion:
            raise BootError("--allow-motion requested but the bridge is read-only (mode: read-only); "
                            "restart the bridge with --enable-motion --acknowledge-first-motion-checklist", 3)
        self._log(f"BOOT allow_motion={self.allow_motion} bare={self.bare} bridge_motion={self.bridge_motion} "
                  f"state={self.bridge_state} safety={self.safety_state} clamps r[{self.r_min},{self.r_max}] "
                  f"z[{self.z_min},{self.z_max}] step<={self.max_step}")
        if self.standalone:
            self._log(f"ARM right STANDALONE bridge={self.host}:{self.port} frame=right_base (own base, no transform; "
                      "'top' = the right bridge's own overhead camera)")
        if self.T is not None:
            self._log(f"ARM {self.arm} bridge={self.host}:{self.port} world_frame={WORLD_FRAME} "
                      f"base_in_world t={[round(float(v), 5) for v in [round(v, 5) for v in self.T.t]]} q_wxyz={[round(float(v), 6) for v in [round(v, 6) for v in self.T.q]]} "
                      f"source={self.T.source!r} file={self.T.path}")

    # ---------- infrastructure ----------
    def _log(self, line):
        with open(self.log_path, "a") as f:
            f.write(f"{time.strftime('%H:%M:%S')} {line}\n")

    def _load_base_transform(self, path):
        """Right arm only: explicit file > config/right_base_in_left_base.json > nominal."""
        src = path or (DEFAULT_BASE_TRANSFORM_FILE if os.path.exists(DEFAULT_BASE_TRANSFORM_FILE) else None)
        if src is None:
            return BaseTransform.nominal()
        try:
            T = BaseTransform.from_file(src)
        except Exception as e:  # noqa: BLE001
            raise BootError(f"base transform file {src}: {type(e).__name__}: {e}", 4)
        nom = BaseTransform.nominal()
        dt, dang = float(np.linalg.norm(T.t - nom.t)), T.angle_to_deg(nom)
        if dt > 0.10 or dang > 15.0:
            self._log(f"WARNING base transform {src} differs from the nominal station offset by "
                      f"{dt * 1000:.0f} mm / {dang:.1f} deg — check the file")
        return T

    def _to_world(self, pose):
        """Arm-base pose dict -> world (left_base). Left arm: the same object, untouched."""
        return pose if self.T is None else self.T.pose_to_world(pose)

    def _from_world(self, pose):
        """World (left_base) pose dict -> this arm's base frame. Left arm: the same object, untouched."""
        return pose if self.T is None else self.T.pose_from_world(pose)

    def _ee_world(self):
        return self._to_world(self._ee())

    def _connect(self, timeout_s):
        robot = "yam_left" if self.arm == "left" else "yam_right"
        self.conn = gap.connector.real(robot, host=self.host, port=self.port, wait_timeout_s=timeout_s)

    def _ensure_conn(self):
        if self.conn is None:
            self._log("reconnecting to the bridge ...")
            self._connect(10.0)
            self._log("reconnected")

    def _health(self):
        self._ensure_conn()
        return dict(self.conn.env.get_observation()["bridge_health"])

    def _obs(self):
        self._ensure_conn()
        return self.conn.get_observation()

    def _ee(self):
        self._ensure_conn()
        return _pose_dict(self.conn.get_ee_pose())

    def _err(self, e):
        if isinstance(e, YamDisconnectedError):
            try:
                self.conn.close()
            except Exception:  # noqa: BLE001
                pass
            self.conn = None
            return {"error": f"DISCONNECTED: {str(e)[:300]}", "error_code": "DISCONNECTED", "retryable": True}
        if isinstance(e, YamBridgeError):
            return {"error": str(e)[:300], "error_code": str(getattr(e, "code", "BRIDGE")),
                    "retryable": bool(getattr(e, "retryable", False))}
        if isinstance(e, ToolError):
            return {"error": f"{getattr(e, 'tool', 'tool')}: {getattr(e, 'detail', str(e))}"[:300],
                    "error_code": "TOOL_ERROR", "retryable": False}
        return {"error": f"{type(e).__name__}: {str(e)[:300]}", "error_code": "INTERNAL", "retryable": False}

    def _refuse_motion(self):
        try:
            ee = self._ee_world()
        except Exception:  # noqa: BLE001
            ee = None
        return {"ok": False,
                "error": "READ_ONLY: this session is observation-only (server started without --allow-motion); "
                         "motion commands are refused locally and the bridge would reject them too",
                "error_code": "READ_ONLY", "motion_enabled": False, "ee_pose": ee}

    # ---------- commands ----------
    def cmd_status(self, a):
        try:
            h = self._health()
        except Exception as e:  # noqa: BLE001
            h = {"state": "unknown", "detail": self._err(e)["error"]}
        # 2026-09-08 (shared two-arm spec): "arm" and "world_frame" are reported by BOTH arms, "base_in_world"
        # by the right arm only. Versus the pre-09-08 server the LEFT status gains exactly these two keys;
        # every other left-arm response is unchanged (deliberate, documented deviation from byte-identity).
        return {"ok": True, "robot": f"yam_real_{self.arm}", "arm": self.arm, "world_frame": self.world_frame,
                **({"base_in_world": self.T.describe()} if self.T is not None else {}),
                "bridge": f"{self.host}:{self.port}",
                "motion_enabled": self.allow_motion, "bridge_motion_enabled": bool(h.get("motion_enabled", self.bridge_motion)),
                "bridge_state": h.get("state"), "safety_state": h.get("safety_state"),
                "commands_used": self.counted, "budget_hard": self.budget,
                **({"joint_programs": True} if self.programs else {}),
                "clamps": {"r_min": self.r_min, "r_max": self.r_max, "z_min": self.z_min, "z_max": self.z_max,
                           "max_step_m": self.max_step},
                **({} if self.bare else {"table_z": TABLE_Z}),
                "max_opening_m": MAX_OPEN, "time": time.strftime("%H:%M:%S")}

    def cmd_help(self, a):
        ro = " (refused with READ_ONLY in observation-only sessions)" if not self.allow_motion else ""
        # 2026-09-08: the right arm reports/accepts world-frame poses and serves 'wrist' only; its help says
        # so. For the left arm every string below is byte-identical to the single-arm server.
        left = self.solo
        fr = "base frame" if left else "world frame (the left arm's base)"
        axes = "base-frame axes" if left else "world-frame axes (the left arm's base)"
        cams = '["wrist", "top"]' if left else '["wrist"]'
        wrist_only = "the wrist camera (the overhead camera is served by the left arm: --arm left frames)"
        fdir = os.path.basename(self.frames) + "/"
        topnote = 'the only option for "top"' if left else '"top" is served by the left arm'
        if self.bare:
            return {"ok": True, "commands": {
                "status": "server, bridge and budget info (free)",
                "state": "{} -> joints (rad), ee_pose (" + fr + "), gripper_fraction, gripper_width_m, bridge health (free)",
                "frames": '{"cams": ' + cams + ', "depth": true} -> captures ' + ("both cameras" if left else wrist_only)
                          + ' from one instant; saves ' + fdir
                          + 'NNNN_<cam>.png (+ NNNN_wrist_depth.npy float32 metres, wrist only) + NNNN_calib.json; returns paths',
                "move_ee": '{"position": {"x","y","z"}, "rotation": {"w","x","y","z"}, "mode": "linear"|"plan"} -> '
                           '"linear" = straight line holding orientation (default), "plan" = joint-space move to the IK solution; '
                           'returns achieved ee_pose, target_error_mm; ok:false + error_code if refused or failed' + ro,
                "move_joints": '{"joints": [6 floats radians]} -> joint-space move; returns joints, ee_pose' + ro,
                "gripper": '{"action": "open"|"close"|0..1} -> returns fraction (0 closed .. 1 open) and width_m' + ro,
            }}
        return {"ok": True, "commands": {
            "status": "server, bridge and budget info (free)",
            "state": "{} -> joints, ee_pose (" + fr + "), gripper_fraction, gripper_width_m, bridge health (free)",
            "frames": '{"cams": ' + cams + ', "depth": true} -> captures ' + ("BOTH cameras" if left else wrist_only)
                      + ' from one instant; saves ' + fdir
                      + 'NNNN_<cam>.png (+ NNNN_wrist_depth.npy float32 metres, wrist only) + NNNN_calib.json; returns paths',
            "deproject": '{"capture": N, "cam": "wrist", "u": px, "v": px} -> 3D point (' + fr + ') from the saved wrist depth '
                         '(5x5 median); add "plane_z": <m> for ray-plane intersection instead of depth (' + topnote + ') (free)',
            "move_ee": '{"position": {"x","y","z"}, "rotation": {"w","x","y","z"}, "mode": "linear"|"plan"} -> straight line holding '
                       'orientation (default) or planned joint move; returns achieved ee_pose + target_error_mm; ok:false + error '
                       '(IK_FAILED / JOINT_LIMIT / ACTION_TIMEOUT ...) if the robot could not do it' + ro,
            "move_delta": '{"dpos": [dx,dy,dz], "drot_deg": [rx,ry,rz]} -> relative straight-line move from the current pose '
                          '(' + axes + ')' + ro,
            "move_joints": '{"joints": [6 floats radians]} -> joint-space move (the robot chooses the timing)' + ro,
            "home": "{} -> move to the OBSERVE posture (wrist camera overlooking the table); not a folded park" + ro,
            "gripper": '{"action": "open"|"close"|0..1} -> returns fraction (0 closed..1 open) + width_m (= fraction x 0.0955); '
                       '"close" stops on contact, so a clearly nonzero fraction after close means something is held' + ro,
        }}

    def cmd_state(self, a):
        obs = self._obs()
        arm = obs["arms"][0]
        js = arm.get("joint_state", {})
        frac = float(arm.get("gripper_fraction", float("nan")))
        out = {"ok": True,
               "joints": [_r4(x) for x in np.asarray(js.get("positions")).ravel()],
               "ee_pose": self._to_world(_pose_dict(arm.get("ee_pose"))),
               "gripper_fraction": _r4(frac), "gripper_width_m": _r4(max(0.0, frac) * MAX_OPEN),
               "health": obs.get("health"), "sequence": obs.get("sequence"), "age_s": obs.get("age_s")}
        if js.get("velocities") is not None:
            out["joint_velocities"] = [_r4(x) for x in np.asarray(js["velocities"]).ravel()]
        return out

    def cmd_frames(self, a):
        want = a.get("cams") or (["wrist", "top"] if self.solo else ["wrist"])
        names = {}
        for c in want:
            al = _alias(str(c))
            if al not in ALIAS:
                return {"ok": False, "error": f"unknown camera {c!r}; valid: wrist, top (side = top)"}
            if al == "top" and not self.solo:
                return {"ok": False, "error": RIGHT_TOP_ERROR}
            names[ALIAS[al]] = al
        seq = next(self.capture_seq)
        obs = self._obs()                      # ONE observation: both cameras from the same instant
        from PIL import Image
        files, calib = {}, {}
        for cam in obs["cameras"]:
            alias = names.get(cam["name"])
            if alias is None:
                continue
            rgb = np.asarray(cam["rgb"])
            if rgb.dtype != np.uint8:
                rgb = (rgb * 255.0 if float(rgb.max()) <= 1.0 else rgb).clip(0, 255).astype(np.uint8)
            p = os.path.join(self.frames, f"{seq:04d}_{alias}.png")
            Image.fromarray(np.ascontiguousarray(rgb[..., :3])).save(p)
            entry = {"rgb": p}
            if a.get("depth", True) and cam.get("depth") is not None:
                dp = os.path.join(self.frames, f"{seq:04d}_{alias}_depth.npy")
                np.save(dp, np.asarray(cam["depth"], dtype=np.float32))
                entry["depth_npy"] = dp
            K = cam.get("intrinsics")
            dist = cam.get("distortion_coefficients")
            model = str(cam.get("distortion_model", "unknown"))
            calib[alias] = {"intrinsics": (np.asarray(K, dtype=float).tolist() if K is not None else None),
                            "pose": self._to_world(_pose_dict(cam.get("pose"))), "frame": self.pose_frame,
                            "image_size": [int(rgb.shape[1]), int(rgb.shape[0])],
                            "distortion_model": model,
                            "distortion_coefficients": (np.asarray(dist, dtype=float).tolist() if dist is not None else None),
                            "rectified": model == "none",
                            "serial": cam.get("serial"), "frame_sequence": cam.get("frame_sequence"),
                            "frame_wall_time_ns": cam.get("frame_wall_time_ns"), "age_s": cam.get("age_s")}
            files[alias] = entry
        arm = obs["arms"][0]
        calib["_meta"] = {"wall_time_ns": obs.get("wall_time_ns"), "sequence": obs.get("sequence"),
                          "joints": [_r4(x) for x in np.asarray(arm["joint_state"]["positions"]).ravel()],
                          "ee_pose": self._to_world(_pose_dict(arm.get("ee_pose"))),
                          "gripper_fraction": _r4(arm.get("gripper_fraction", float("nan"))),
                          "capture_time": time.strftime("%Y-%m-%d %H:%M:%S")}
        if self.T is not None:
            # provenance for tools/solve_base_transform.py: the poses above were converted with THIS
            # transform, so the right_base-frame originals can be recovered exactly
            calib["_meta"]["arm"] = self.arm
            calib["_meta"]["world_frame"] = WORLD_FRAME
            calib["_meta"]["base_in_world"] = self.T.describe()
        elif self.standalone:
            calib["_meta"]["arm"] = self.arm
            calib["_meta"]["world_frame"] = self.world_frame
        cp = os.path.join(self.frames, f"{seq:04d}_calib.json")
        with open(cp, "w") as f:
            json.dump(calib, f, indent=1, default=_jsonable)
        # latest capture and the motion revision it was taken at (episode end markers, mx sessions)
        self.mx_capture, self.mx_capture_time = seq, time.monotonic()
        self.mx_capture_revision = self.mx_motion_revision
        if not self.solo:
            return {"ok": True, "capture": seq, "files": files, "calibration_file": cp,
                    "note": "depth npy = float32 metres, 0 = invalid; calib pose = camera pose in the world frame "
                            "(the left arm's base); the overhead camera is served by the left arm"}
        return {"ok": True, "capture": seq, "files": files, "calibration_file": cp,
                "note": "depth npy = float32 metres, 0 = invalid, wrist only; calib pose = camera pose in the robot base "
                        "frame; the top image is rectified (its pinhole K applies directly to the PNG)"}

    def cmd_deproject(self, a):
        seq = int(a["capture"]); cam = _alias(str(a.get("cam", "wrist"))); u = int(a["u"]); v = int(a["v"])
        if cam == "top" and not self.solo:
            return {"ok": False, "error": RIGHT_TOP_ERROR}
        cp = os.path.join(self.frames, f"{seq:04d}_calib.json")
        if not os.path.exists(cp):
            return {"ok": False, "error": f"no calibration for capture {seq} (take frames first)"}
        allc = json.load(open(cp))
        if cam not in allc:
            return {"ok": False, "error": f"capture {seq} has no camera {cam!r}"}
        calib = allc[cam]
        K = np.asarray(calib["intrinsics"], dtype=float)
        pose = calib["pose"]
        R = _wxyz_to_mat(_quat_arr(pose["rotation"]))
        t = np.array([pose["position"][k] for k in "xyz"], dtype=float)
        w, h = [int(x) for x in calib["image_size"]]
        if not (0 <= v < h and 0 <= u < w):
            return {"ok": False, "error": f"pixel ({u},{v}) outside image {w}x{h}"}
        ray_c = np.linalg.inv(K) @ np.array([u, v, 1.0])
        if a.get("plane_z") is not None:
            zp = _f(a["plane_z"])
            r_w = R @ ray_c
            if abs(r_w[2]) < 1e-6:
                return {"ok": False, "error": "ray is parallel to the horizontal plane"}
            s = (zp - t[2]) / r_w[2]
            if s <= 0:
                return {"ok": False, "error": f"plane z={zp} is behind the camera along that ray"}
            X = t + s * r_w
            return {"ok": True, "mode": "plane", "plane_z": zp,
                    "point_base": {"x": _r4(X[0]), "y": _r4(X[1]), "z": _r4(X[2])},
                    "ray_length_m": _r4(s * float(np.linalg.norm(r_w)))}
        dp = os.path.join(self.frames, f"{seq:04d}_{cam}_depth.npy")
        if not os.path.exists(dp):
            return {"ok": False, "error": f"no depth for cam {cam!r} in capture {seq}; use \"plane_z\" (ray-plane) "
                                          f"or the wrist camera"}
        depth = np.load(dp)
        win = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        vals = win[win > 0]
        if vals.size == 0:
            return {"ok": False, "error": "no valid depth in the 5x5 window at that pixel; try a neighbouring pixel"}
        d = float(np.median(vals))
        X = R @ (d * ray_c) + t
        return {"ok": True, "mode": "depth", "point_base": {"x": _r4(X[0]), "y": _r4(X[1]), "z": _r4(X[2])},
                "depth_m": _r4(d), "valid_in_window": int(vals.size)}

    # ---------- motion ----------
    def _clamp_check(self, pos, cur=None, pos_world=None):
        """`pos`/`cur`: THIS arm's base frame (reach radius and step are per arm). `pos_world`: the
        same target in the WORLD frame for the z guard — z_min/z_max/TABLE_Z are left_base (= world)
        heights of the table plane, so they must not be tested on right_base numbers (2026-09-08 fix:
        a base transform with a non-zero z would otherwise shift the floor). Default None = `pos`
        (left arm: the identical object, behaviour unchanged)."""
        zp = pos if pos_world is None else pos_world
        r = math.hypot(pos["x"], pos["y"])
        if not (self.r_min <= r <= self.r_max):
            return (f"target radius {r:.3f} m outside [{self.r_min}, {self.r_max}]"
                    + ("" if self.T is None else " (r is measured from this arm's own base)"))
        if not (self.z_min <= zp["z"] <= self.z_max):
            return (f"target z {zp['z']:.3f} m outside [{self.z_min}, {self.z_max}]"
                    + ("" if self.bare else f" (table at {TABLE_Z})"))
        if cur is not None:
            step = math.dist((cur["x"], cur["y"], cur["z"]), (pos["x"], pos["y"], pos["z"]))
            if step > self.max_step:
                return (f"step {step:.3f} m exceeds max_step_m {self.max_step}"
                        + ("" if self.bare else "; split the move"))
        return None

    def _move_common(self, pose, mode):
        """`pose` arrives in the WORLD frame (left_base). The reach/step guards, the bridge call and
        the target error are evaluated in THIS arm's base frame; the z guard (table plane) on the
        world-frame target; every returned pose is world."""
        if not self.allow_motion:
            return self._refuse_motion()
        ee = self._ee()                              # this arm's base frame
        pose_w = pose                                # the agent's world-frame target (z guard)
        pose = self._from_world(pose)                # right: world -> right_base; left: same object
        bad = self._clamp_check(pose["position"], ee["position"], pose_w["position"])
        if bad:
            return {"ok": False, "error": bad, "error_code": "CLAMP", "ee_pose": self._to_world(ee)}
        t0 = time.time()
        err, res = None, {}
        try:
            if mode == "plan":
                res = self.conn._tool_go_to_pose(pose, timeout_s=0.0) or {}
            else:
                sp, tp = ee["position"], pose["position"]
                d = math.dist((sp["x"], sp["y"], sp["z"]), (tp["x"], tp["y"], tp["z"]))
                q0, q1 = _quat_arr(ee["rotation"]), _quat_arr(pose["rotation"])
                q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
                ang = 2.0 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))
                timeout = max(10.0, 2.0 * (d / CART_V + ang / CART_W) + 5.0)
                res = self.conn._tool_go_to_pose_cartesian(pose, timeout_s=timeout) or {}
            st = res.get("status")
            detail = str(res.get("detail") or "")
            if st == "completed" and detail.startswith("SETTLE_MISS"):
                # 2026-09-03: the bridge finished the trajectory but could not settle within
                # tolerance (object/fingers pressing on something, or load sag). It keeps the
                # arm STIFF at the achieved pose instead of going limp; surface it as a
                # failure so the agent re-decides from the frames.
                core = "target not reached: " + detail[len("SETTLE_MISS: "):][:220]
                err = {"error": (core if self.bare else
                                 core + ". The arm is HOLDING at ee_pose below (stiff, gripper unchanged): "
                                        "inspect fresh frames, then lift/retreat or retry from here"),
                       "error_code": "SETTLE_MISS", "retryable": True}
            elif st != "completed":
                err = {"error": f"bridge status {st}: {detail}"[:300],
                       "error_code": f"BRIDGE_{str(st).upper()}", "retryable": False}
        except Exception as e:  # noqa: BLE001
            self._log(traceback.format_exc())
            err = self._err(e)
        time.sleep(0.2)
        try:
            ee2 = self._ee()
        except Exception as e:  # noqa: BLE001
            ee2 = None
            err = err or self._err(e)
        d_mm = None
        if ee2 is not None:
            tp, ap = pose["position"], ee2["position"]
            d_mm = round(math.dist((tp["x"], tp["y"], tp["z"]), (ap["x"], ap["y"], ap["z"])) * 1000, 1)
        out = {"ok": err is None, "error": None, "ee_pose": self._to_world(ee2), "target_error_mm": d_mm,
               "duration_s": round(time.time() - t0, 1), "bridge_status": res.get("status")}
        if res.get("max_tracking_error_rad") is not None:
            out["max_tracking_error_deg"] = round(math.degrees(float(res["max_tracking_error_rad"])), 2)
        if err:
            out.update(err)
        return out

    def cmd_move_ee(self, a):
        pose = {"position": {k: _f(a["position"][k]) for k in "xyz"},
                "rotation": {k: _f(a["rotation"][k]) for k in "wxyz"}}
        return self._move_common(pose, a.get("mode", "linear"))

    def cmd_move_delta(self, a):
        if not self.allow_motion:
            return self._refuse_motion()
        ee = self._ee_world()                        # dpos / drot_deg are WORLD-frame axes
        dp = a.get("dpos") or [0, 0, 0]
        pos = {"x": ee["position"]["x"] + _f(dp[0]), "y": ee["position"]["y"] + _f(dp[1]),
               "z": ee["position"]["z"] + _f(dp[2])}
        rot = ee["rotation"]
        if a.get("drot_deg"):
            R = _rot_delta_mat([_f(x) for x in a["drot_deg"]]) @ _wxyz_to_mat(_quat_arr(rot))
            w, x, y, z = _mat_to_wxyz(R)
            rot = {"w": _f(w), "x": _f(x), "y": _f(y), "z": _f(z)}
        return self._move_common({"position": pos, "rotation": rot}, "linear")

    def _joint_move(self, joints, label):
        if not self.allow_motion:
            return self._refuse_motion()
        t0 = time.time()
        err, res = None, {}
        try:
            cur = np.asarray(self.conn.env.get_observation()["robot_joint_pos_0"][:6], dtype=float)
            disp = float(np.max(np.abs(np.asarray(joints, dtype=float) - cur)))
            timeout = float(self.conn._joint_move_timeout_s(disp))
            res = self.conn._tool_move_to_joints({"positions": list(joints)}, tolerance=0.03, timeout_s=timeout) or {}
            if res.get("status") != "completed":
                err = {"error": f"bridge status {res.get('status')}: {res.get('detail', '')}"[:300],
                       "error_code": f"BRIDGE_{str(res.get('status')).upper()}", "retryable": False}
        except Exception as e:  # noqa: BLE001
            self._log(traceback.format_exc())
            err = self._err(e)
        time.sleep(0.2)
        out = {"ok": err is None, "error": None, "move": label, "duration_s": round(time.time() - t0, 1),
               "bridge_status": res.get("status")}
        try:
            out["ee_pose"] = self._ee_world()
            fj = res.get("final_joint_pos_0")
            out["joints"] = ([_r4(x) for x in np.asarray(fj).ravel()[:6]] if fj is not None
                             else [_r4(x) for x in self.conn.env.get_observation()["robot_joint_pos_0"][:6]])
        except Exception as e:  # noqa: BLE001
            err = err or self._err(e)
            out["ok"] = False
        if err:
            out.update(err)
        return out

    def cmd_move_joints(self, a):
        joints = [float(x) for x in a["joints"]]
        if len(joints) != 6:
            return {"ok": False, "error": "joints must have 6 entries (radians)"}
        return self._joint_move(joints, "joints")

    def cmd_home(self, a):
        out = self._joint_move(self.observe_joints, "observe_posture")
        out.setdefault("note", "home = the observe posture (wrist camera overlooking the table)")
        return out

    # ---------- buffered joint programs (2026-09-14, --programs; port of the original throwing runtime) ----------
    @staticmethod
    def _program_help():
        return {
            "preview_program": '{"program": {"times_s": [...], "joint_deltas_rad": [[6 floats], ...], "gripper_events": '
                               '[{"time_s": t, "fraction": f}, ...]}} -> validates against the current arm state; returns the '
                               'compiled 50 Hz commands, TCP poses/velocities, scheduled events and peak joint rates; no motion (free)',
            "run_program": '{"program": {...}} -> executes exactly ONE authored joint/gripper program (J4 up to 180 deg/s and '
                           '360 deg/s^2 inside a program, other joints 20/40); returns program_id, status '
                           '(completed | settle_miss | rejected | fault), the achieved state and report_file',
            "program_report": '{"program_id": "..."} -> the full/partial per-tick report of an executed program, incl. faults (free)',
        }

    @staticmethod
    def _program_duration_s(program):
        try:
            return float(program["times_s"][-1])
        except Exception:  # noqa: BLE001
            return 0.0

    def _program_pose_out(self, pose7):
        """Bridge FK pose (grasp_site, this arm's base) -> the agent's convention (tcp_gap orientation, world frame)."""
        p = [float(v) for v in pose7]
        w, x, y, z = bridge_quat_wxyz_to_sim((p[3], p[4], p[5], p[6]))
        pose = self._to_world({"position": {"x": p[0], "y": p[1], "z": p[2]},
                               "rotation": {"w": float(w), "x": float(x), "y": float(y), "z": float(z)}})
        return [_r4(pose["position"][k]) for k in "xyz"] + [_r4(pose["rotation"][k]) for k in "wxyz"]

    def _save_program_report(self, program_id):
        report = self.conn.env.program_report(program_id)
        path = os.path.join("scratch", f"program_{program_id}_report.json")
        os.makedirs(os.path.join(self.session, "scratch"), exist_ok=True)
        with open(os.path.join(self.session, path), "w") as f:
            json.dump(report, f, default=_jsonable)
        return path, report

    def _program_state(self):
        try:
            st = self.cmd_state({})
            return {k: st[k] for k in ("joints", "ee_pose", "gripper_fraction") if k in st}
        except Exception as e:  # noqa: BLE001
            return {"state": None, "observation_error": self._err(e)["error"]}

    def cmd_preview_program(self, a):
        if not isinstance(a.get("program"), dict):
            return {"ok": False, "error": 'args must be {"program": {...}}', "error_code": "INVALID_ARGUMENT"}
        preview = self.conn.env.preview_program(a["program"])
        preview["tcp_poses_wxyz"] = [self._program_pose_out(p) for p in preview.get("tcp_poses_wxyz", [])]
        preview["orientation_frame"] = "the frame and orientation convention of state/ee_pose"
        return {"ok": True, "preview": preview}

    def cmd_program_report(self, a):
        program_id = a.get("program_id")
        if not isinstance(program_id, str):
            return {"ok": False, "error": 'args must be {"program_id": "..."}', "error_code": "INVALID_ARGUMENT"}
        path, report = self._save_program_report(program_id)
        return {"ok": True, "report_file": path, "report": report}

    def cmd_run_program(self, a):
        if not self.allow_motion:
            return self._refuse_motion()
        program = a.get("program")
        if not isinstance(program, dict):
            return {"ok": False, "error": 'args must be {"program": {...}}', "error_code": "INVALID_ARGUMENT", "retryable": True}
        program_id = uuid.uuid4().hex
        duration = self._program_duration_s(program)
        # the bridge's hard execution timeout is max(10 s, 2 x duration + 5 s); wait a little longer than that
        timeout = max(10.0, 2.0 * duration + 5.0) + 5.0
        t0 = time.time()
        error = None
        try:
            self.conn.env.run_joint_program(program, program_id=program_id, timeout_s=timeout)
        except Exception as e:  # noqa: BLE001
            self._log(traceback.format_exc())
            error = e
        if isinstance(error, YamBridgeError) and getattr(error, "code", "") in ("INVALID_ACTION", "READ_ONLY", "STALE_ACTION"):
            # refused before anything reached the controller: no report exists
            return {"ok": False, "program_id": program_id, **self._err(error), "retryable": True,
                    "duration_s": round(time.time() - t0, 1)}
        try:
            path, report = self._save_program_report(program_id)
        except Exception as report_error:  # noqa: BLE001
            failure = error or report_error
            return {"ok": False, "program_id": program_id, **self._err(failure), "motion_stopped": True,
                    "duration_s": round(time.time() - t0, 1)}
        status = report.get("status")
        base = {"program_id": program_id, "status": status, "report_file": path,
                "duration_s": round(time.time() - t0, 1)}
        if status == "rejected":
            # validation failed before any motion target was sent: the arm stays where it is
            return {"ok": False, **base, "error_code": report.get("error_code") or "INVALID_ARGUMENT",
                    "error": report.get("error"), "retryable": True, "motion_stopped": False, **self._program_state()}
        if error is not None or status == "fault":
            err = self._err(error) if error is not None else {"error": report.get("error"),
                                                             "error_code": report.get("error_code") or "FAULT"}
            return {"ok": False, **base, **err, "physical_status": status, "motion_stopped": True,
                    **self._program_state()}
        out = {"ok": status == "completed", **base,
               "max_tracking_error_rad": _r4(report.get("max_tracking_error_rad", 0.0)),
               "final_gripper_fraction": _r4(report.get("final_gripper_fraction", -1.0)),
               "events": report.get("events", []), **self._program_state()}
        if status == "settle_miss":
            out.update(error_code="SETTLE_MISS", retryable=True, motion_stopped=False,
                       error="the program finished but the measured endpoint or jaw did not converge; "
                             "the arm holds its last command; inspect the report and fresh frames")
        return out

    def cmd_gripper(self, a):
        if not self.allow_motion:
            return self._refuse_motion()
        act = a.get("action", "close")
        t0 = time.time()
        try:
            if act == "open":
                r = self.conn.open_gripper(timeout_s=8.0)
            elif act == "close":
                r = self.conn.close_gripper(timeout_s=8.0)
            else:
                frac_t = min(1.0, max(0.0, _f(act)))
                r = self.conn.env._set_gripper(frac_t, timeout_s=8.0, tolerance=0.02, arm_id=0, stop_on_contact=True)
                r = {**r, "position": float(np.asarray(r["final_joint_pos_0"]).ravel()[6])}
        except Exception as e:  # noqa: BLE001
            self._log(traceback.format_exc())
            return {"ok": False, **self._err(e), "duration_s": round(time.time() - t0, 1)}
        frac = float(r.get("position", -1.0))
        return {"ok": r.get("status") == "completed", "fraction": _r4(frac), "width_m": _r4(max(0.0, frac) * MAX_OPEN),
                "bridge_status": r.get("status"), "duration_s": round(time.time() - t0, 1),
                "note": ("fraction 0 = fully closed, 1 = fully open" if self.bare else
                         "fraction 0 = fully closed, 1 = fully open; after close, fraction >> 0 means something is between the pads")}

    def cmd_reset(self, a):
        return {"ok": False, "error_code": "NO_RESET",
                "error": "reset is not available: this is a real robot and a real table; recover by re-observing "
                         "and dealing with the scene as it now is"}

    # ---------- multiple-experience commands (mx sessions only; port of the multiple-experience method's server) ----------
    def cmd_episode(self, a):
        """The original cmd_episode: fresh-capture and empty-gripper checks, previous checkpoint before a new cycle."""
        from mx.cycle_dataset import mark
        from mx.experience import load
        if not self.allow_motion:
            return {"ok": False, "error_code": "READ_ONLY"}
        if a.get("event") == "end" and (
            type(a.get("capture")) is not int
            or a["capture"] != self.mx_capture
            or self.mx_capture_revision != self.mx_motion_revision
            or time.monotonic() - self.mx_capture_time > 30
        ):
            return {"ok": False, "error_code": "FRESH_CAPTURE_REQUIRED"}
        if (
            a.get("event") == "start"
            and a.get("stage") == "assemble"
            and type(a.get("cycle")) is int
            and a["cycle"] > 1
        ):
            memory = load(self.session)["latest"]
            if memory is None or memory["cycle"] != a["cycle"] - 1:
                return {"ok": False, "error_code": "EXPERIENCE_REQUIRED"}
        state = self.cmd_state({})
        if a.get("event") == "end" and a.get("outcome") == "success" and state["gripper_fraction"] < 0.95:
            return {"ok": False, "error_code": "EMPTY_GRIPPER_REQUIRED"}
        # the stage record keeps exactly the state fields the original server records (our `state` has 3 more)
        state = {k: state[k] for k in ("ok", "joints", "ee_pose", "gripper_fraction", "health", "sequence") if k in state}
        return mark(self.session, actor="coordinator", state=state, revision=self.mx_motion_revision, **a)

    def cmd_experience(self, a):
        """The original cmd_experience: no arguments = history; otherwise commit (server supplies times and evidence)."""
        from mx.experience import commit, load
        if not a:
            return load(self.session)
        if self.mx.get("read_only"):
            return {"ok": False, "error_code": "READ_ONLY_EXPERIENCE",
                    "error": "this session reuses earlier experience read-only; checkpoints are not saved"}
        if not self.allow_motion:
            return {"ok": False, "error_code": "READ_ONLY"}
        return commit(self.session, cycle=a["cycle"], lesson=a["lesson"], artifacts=a["artifacts"])

    def _mx_guard(self, cmd, a):
        """The original cycle_agents.guard for a session that holds exactly one assigned cycle."""
        if cmd not in MOTION_CMDS | MX_CMDS or (cmd == "experience" and not a):
            return None
        cycle = self.mx["cycle"]
        if cmd in MX_CMDS and a.get("cycle") != cycle:
            return "CYCLE_ASSIGNMENT"
        # read-only reuse: the session holds ANOTHER sequence's checkpoints and never commits one itself,
        # so "motion after this cycle's checkpoint" cannot arise there
        if cmd in MOTION_CMDS and cycle < 5 and not self.mx.get("read_only"):
            from mx.experience import load
            latest = load(self.session)["latest"]
            if latest and latest["cycle"] >= cycle:
                return "CYCLE_FINISHED"
        return None

    def _mx_dispatch(self, req):
        started = time.monotonic_ns()
        resp = self._mx_dispatch_inner(req)
        row = dict(command=req.get("cmd"), started_monotonic_ns=started, finished_monotonic_ns=time.monotonic_ns(),
                   ok=resp.get("ok", False), error_code=resp.get("error_code"), request_id=req.get("id"))
        with open(os.path.join(self.session, "command_metrics.jsonl"), "a") as stream:
            stream.write(json.dumps(row) + "\n")
        return resp

    def _mx_dispatch_inner(self, req):
        cmd = req.get("cmd", "")
        a = req.get("args") or {}
        fn = getattr(self, f"cmd_{cmd}", None)
        if fn is None or (self.bare and cmd in BARE_HIDDEN) or (cmd in PROGRAM_CMDS and not self.programs):
            return {"ok": False, "error": f"unknown command {cmd!r}; run help"}
        if cmd not in FREE_CMDS | MX_CMDS:
            if self.counted >= self.budget:
                return {"ok": False, "error": f"hard command budget ({self.budget}) exhausted", "error_code": "BUDGET"}
            self.counted += 1
        err = self._mx_guard(cmd, a)
        if err:
            return {"ok": False, "error_code": err}
        if cmd in MOTION_CMDS:
            self.mx_motion_revision += 1
        try:
            resp = fn(a)
        except Exception as e:  # noqa: BLE001
            self._log(traceback.format_exc())
            if cmd in MX_CMDS:   # the original error shape for validation failures of these two commands
                return {"ok": False, "error_code": getattr(e, "code", "INVALID_ARGUMENT"), "error": str(e),
                        "motion_stopped": False}
            resp = {"ok": False, **self._err(e)}
        if cmd == "help" and self.programs and resp.get("ok"):
            resp["commands"].update(self._program_help())
        if cmd == "help" and resp.get("ok"):
            resp["commands"]["episode"] = ('{"cycle": n, "stage": "assemble", "event": "start"|"end", "note": "...", '
                                           '"outcome": "success"|"failure"|"uncertain", "capture": N} -> stage marker (free)')
            resp["commands"]["experience"] = ('{} -> checkpoint history; {"cycle": n, "lesson": {...}, "artifacts": [...]} '
                                              '-> commit this cycle (free)')
        return resp

    # ---------- loop (sim server verbatim, minus truth/video) ----------
    def dispatch(self, req):
        if self.mx is not None:
            return self._mx_dispatch(req)
        cmd = req.get("cmd", "")
        fn = getattr(self, f"cmd_{cmd}", None)
        if fn is None or (self.bare and cmd in BARE_HIDDEN) or cmd in MX_CMDS or (cmd in PROGRAM_CMDS and not self.programs):
            return {"ok": False, "error": f"unknown command {cmd!r}; run help"}
        if cmd not in FREE_CMDS:
            if self.counted >= self.budget:
                return {"ok": False, "error": f"hard command budget ({self.budget}) exhausted", "error_code": "BUDGET"}
            self.counted += 1
        try:
            resp = fn(req.get("args") or {})
        except Exception as e:  # noqa: BLE001
            self._log(traceback.format_exc())
            resp = {"ok": False, **self._err(e)}
        if cmd == "help" and self.programs and resp.get("ok"):
            resp["commands"].update(self._program_help())
        return resp

    def _record_tick(self):
        """Save one top + wrist JPEG for the run video, if the interval has elapsed. Never
        raises into the loop. Top is downscaled to keep the sequence small."""
        if not self.record_dir or self.conn is None:
            return
        now = time.time()
        if now - self._last_record < self.record_interval:
            return
        self._last_record = now
        try:
            from PIL import Image
            obs = self.conn.get_observation()
            i = next(self._rec_i)
            paths = {}
            for cam in obs["cameras"]:
                alias = {"wrist_d405": "wrist", "top_brio": "top"}.get(cam["name"])
                if alias is None:
                    continue
                rgb = np.asarray(cam["rgb"])[..., :3]
                if rgb.dtype != np.uint8:
                    rgb = (rgb * 255.0 if float(rgb.max()) <= 1.0 else rgb).clip(0, 255).astype(np.uint8)
                im = Image.fromarray(np.ascontiguousarray(rgb))
                if alias == "top" and im.width > 960:
                    im = im.resize((960, round(im.height * 960 / im.width)))
                p = os.path.join(self.record_dir, alias, f"{i:06d}.jpg")
                im.save(p, quality=80)
                paths[alias] = os.path.basename(p)
            if hasattr(self, "_rec_ts"):
                self._rec_ts.write(f"{i},{now:.3f},{paths.get('top','')},{paths.get('wrist','')}\n")
                self._rec_ts.flush()
        except Exception as e:  # noqa: BLE001
            self._log(f"record tick failed: {type(e).__name__}: {e}")

    def ready_line(self):
        return (f"READY session={self.session} motion_enabled={self.allow_motion} bare={self.bare} "
                f"bridge_motion_enabled={self.bridge_motion} bridge_state={self.bridge_state} "
                f"safety_state={self.safety_state}"
                + ("" if self.arm == "left" else f" arm={self.arm} world_frame={self.world_frame}"))

    def run(self):
        print(self.ready_line(), flush=True)
        self._log(self.ready_line())
        if self.mx is not None:   # the start of this cycle's session (mx/experience.py metrics boundary)
            with open(os.path.join(self.session, "rounds.jsonl"), "a") as stream:
                stream.write(json.dumps({"cycle": self.mx["cycle"], "session": os.path.basename(self.session),
                                         "ready_monotonic_ns": time.monotonic_ns(),
                                         "ready_wall_time_ns": time.time_ns()}) + "\n")
            self._log(f"MX multiple-experience session: cycle {self.mx['cycle']} (episode / experience enabled"
                      + (", experience READ-ONLY)" if self.mx.get("read_only") else ")"))
        stop = os.path.join(self.session, "SERVER_STOP")
        try:
            while True:
                if os.path.exists(stop):
                    print("SERVER_STOP seen, exiting", flush=True)
                    self._log("SERVER_STOP seen, exiting")
                    return
                reqs = []
                for f in os.listdir(self.bridge):
                    if f.startswith("req_") and f.endswith(".json"):
                        rid = f[4:-5]
                        if not os.path.exists(os.path.join(self.bridge, f"resp_{rid}.json")):
                            try:
                                reqs.append(int(rid))
                            except ValueError:
                                pass
                for rid in sorted(reqs):
                    rp = os.path.join(self.bridge, f"req_{rid:06d}.json")
                    try:
                        req = json.load(open(rp))
                    except Exception:  # noqa: BLE001
                        time.sleep(0.1)
                        try:
                            req = json.load(open(rp))
                        except Exception as e:  # noqa: BLE001
                            req = {"cmd": "_unreadable", "error": str(e)}
                    t0 = time.time()
                    resp = self.dispatch(req)
                    resp["id"] = rid
                    out = os.path.join(self.bridge, f"resp_{rid:06d}.json")
                    with open(out + ".tmp", "w") as f:
                        json.dump(resp, f, default=_jsonable)
                    os.replace(out + ".tmp", out)
                    self._log(f"#{rid} {req.get('cmd')} ok={resp.get('ok')} {time.time()-t0:.1f}s used={self.counted}"
                              + (f" err={resp.get('error_code')}" if not resp.get('ok') else ""))
                self._record_tick()
                time.sleep(0.2)
        finally:
            try:
                if self.conn is not None:
                    self.conn.close()
            except Exception as e:  # noqa: BLE001
                self._log(f"close failed: {type(e).__name__}: {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--arm", choices=("left", "right"), default="left",
                    help="which arm this server drives (default left = today's server, unchanged). right: connector "
                         "yam_right, bridge_right/ frames_right/ server_right.log, port 9022, poses reported/accepted "
                         "in the world frame = left_base, cameras 'wrist' only")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None, help="bridge port (default 9021 for --arm left, 9022 for --arm right)")
    ap.add_argument("--standalone", action="store_true",
                    help="with --arm right: the right arm as the ONLY arm of its session (concurrent with an independent "
                         "left session): single-arm layout bridge/ frames/ server.log, poses in right_base (no base "
                         "transform), both cameras incl. the right bridge's own overhead BRIO as 'top'; no-op for --arm left")
    ap.add_argument("--base-transform-file", default=None,
                    help="--arm right only: JSON {translation:[x,y,z], rotation_wxyz:[w,x,y,z]} = pose of right_base in "
                         f"left_base (default: {DEFAULT_BASE_TRANSFORM_FILE} if it exists, else the nominal "
                         "(0,-0.61,0)/identity); ignored for --arm left")
    ap.add_argument("--allow-motion", action="store_true", help="enable motion commands (bridge must be --enable-motion)")
    ap.add_argument("--bare", action="store_true",
                    help="ablation interface: no deproject/home/move_delta/reset, no advice or interpretation in texts "
                         "(guards, budget and safety identical); default = the full interface used so far")
    ap.add_argument("--programs", action="store_true",
                    help="2026-09-14: offer preview_program / run_program / program_report (buffered 50 Hz joint+gripper "
                         "programs with the bridge's J4 program limits); the bridge must have been started with a config "
                         "that names program J4 limits (config/left_arm_throw.yaml), else the server refuses to boot")
    ap.add_argument("--budget", type=int, default=500, help="hard cap on counted commands")
    ap.add_argument("--r-min", type=float, default=0.12, help="min horizontal radius of a target [m]")
    ap.add_argument("--r-max", type=float, default=0.65, help="max horizontal radius of a target [m]")
    ap.add_argument("--z-min", type=float, default=TABLE_Z - 0.005, help="floor for the grasp point [m] (table -5 mm)")
    ap.add_argument("--z-max", type=float, default=0.60, help="ceiling for the grasp point [m]")
    ap.add_argument("--max-step-m", type=float, default=0.25, help="max distance of one move from the current pose [m]")
    ap.add_argument("--observe-joints", type=float, nargs=6, default=None,
                    help="'home' posture in radians (default: OBSERVE_MAIN_JOINTS)")
    ap.add_argument("--wait-timeout-s", type=float, default=60.0)
    ap.add_argument("--record-dir", default=None, help="save periodic top/wrist JPEGs here for the run video")
    ap.add_argument("--record-interval", type=float, default=2.0, help="seconds between recorded top/wrist frames")
    a = ap.parse_args()
    try:
        srv = Server(a)
    except BootError as e:
        print(f"BOOT_ERROR: {e}", flush=True)
        sys.exit(e.code)
    except Exception as e:  # noqa: BLE001
        print(f"BOOT_ERROR: {type(e).__name__}: {str(e)[:400]}", flush=True)
        traceback.print_exc()
        sys.exit(2)
    srv.run()


if __name__ == "__main__":
    main()
