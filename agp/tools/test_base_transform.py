#!/usr/bin/env python3
"""Hardware-free tests for the two-arm base transform.

Covers server_real.BaseTransform (nominal, rotated, identity; forward/inverse round trips;
move_delta rotation handling), the Server wiring with a stub connector (right arm: every
returned pose is world, every accepted target is converted, guards in the right base frame,
'top' refused, calib.json = wrist + _meta only; left arm: identity pass-through of the very
same objects), and tools/solve_base_transform.py (recovers a synthetic transform, incl. the
undo of a right-server calib.json written with the nominal offset).

Run (from the repo root; the bridge venv python has numpy):
      hardware-bridge/.venv/bin/python \\
          agp/tools/test_base_transform.py
No bridge, no robot, no server process. Writes only into a temp dir.
"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import types

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
FA = os.path.dirname(HERE)


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


srv = _load("server_real", os.path.join(FA, "server_real.py"))
sbt = _load("solve_base_transform", os.path.join(HERE, "solve_base_transform.py"))
BaseTransform = srv.BaseTransform

PASSED = []


def ok(name, cond, detail=""):
    if not cond:
        raise AssertionError(f"{name}: {detail}")
    PASSED.append(name)


def pose_d(p, q):
    return {"position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
            "rotation": {"w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])}}


def pose_arr(d):
    return (np.array([d["position"][k] for k in "xyz"]), np.array([d["rotation"][k] for k in "wxyz"]))


def q_axis(axis, deg):
    ax = np.asarray(axis, float)
    ax /= np.linalg.norm(ax)
    h = math.radians(deg) / 2
    return np.array([math.cos(h), *(math.sin(h) * ax)])


def same_rot(qa, qb, tol=1e-9):
    return abs(abs(float(np.dot(qa, qb))) - 1.0) < tol


rng = np.random.default_rng(7)


def rand_pose():
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return rng.uniform(-0.6, 0.6, 3), q


# ---------------------------------------------------------------- 1. nominal transform: exact arithmetic
T_nom = BaseTransform.nominal()
ok("nominal.values", np.array_equal(T_nom.t, [0.0, -0.61, 0.0]) and np.array_equal(T_nom.q, [1, 0, 0, 0]))
for _ in range(50):
    p, q = rand_pose()
    w = T_nom.pose_to_world(pose_d(p, q))
    pw, qw = pose_arr(w)
    ok("nominal.forward.exact", np.array_equal(pw, p + np.array([0.0, -0.61, 0.0])) and np.array_equal(qw, q),
       f"{pw - p} {qw - q}")
    b = T_nom.pose_from_world(w)
    pb, qb = pose_arr(b)
    ok("nominal.roundtrip", np.allclose(pb, p, atol=1e-15) and np.array_equal(qb, q), f"{pb - p}")

# ---------------------------------------------------------------- 2. identity transform: bitwise pass-through
T_id = BaseTransform([0, 0, 0], [1, 0, 0, 0], source="identity")
for _ in range(50):
    p, q = rand_pose()
    d = pose_d(p, q)
    ok("identity.forward.bitwise", T_id.pose_to_world(d) == d)
    ok("identity.inverse.bitwise", T_id.pose_from_world(d) == d)

# ---------------------------------------------------------------- 3. rotated transform vs homogeneous matrices
t_rot = np.array([0.03, -0.62, 0.015])
q_rot = sbt.quat_mul(q_axis([0, 0, 1], 12.0), q_axis([1, 0, 0], 2.5))
T_rot = BaseTransform(t_rot, q_rot, source="test")
M = sbt.make_T(t_rot, q_rot)
for _ in range(100):
    p, q = rand_pose()
    w = T_rot.pose_to_world(pose_d(p, q))
    pw, qw = pose_arr(w)
    Mw = M @ sbt.make_T(p, q)
    ok("rotated.forward.pos", np.allclose(pw, Mw[:3, 3], atol=1e-12))
    ok("rotated.forward.rot", np.allclose(sbt.wxyz_to_mat(qw), Mw[:3, :3], atol=1e-12))
    ok("rotated.forward.unit", abs(np.linalg.norm(qw) - 1.0) < 1e-12)
    b = T_rot.pose_from_world(w)
    pb, qb = pose_arr(b)
    ok("rotated.roundtrip.pos", np.allclose(pb, p, atol=1e-12))
    ok("rotated.roundtrip.rot", np.allclose(qb, q, atol=1e-12))            # conj(qT)*qT*q == q exactly up to fp
    v = rng.normal(size=3)
    ok("rotated.vector", np.allclose(T_rot.vector_from_world(T_rot.vector_to_world(v)), v, atol=1e-12)
       and np.allclose(T_rot.vector_to_world(v), M[:3, :3] @ v))
    ok("rotated.point_vs_vector", np.allclose(T_rot.point_to_world(v) - T_rot.point_to_world(np.zeros(3)),
                                              T_rot.vector_to_world(v), atol=1e-12))
ok("rotated.angle_to_nominal", abs(T_rot.angle_to_deg(T_nom) - math.degrees(2 * math.acos(abs(
    float(sbt.quat_mul(q_rot, [1, 0, 0, 0])[0]))))) < 1e-9)
ok("rotated.none_passthrough", T_rot.pose_to_world(None) is None and T_rot.pose_from_world(None) is None)

# ---------------------------------------------------------------- 4. move_delta: world-frame deltas via the server's path
# server path: ee_world = to_world(ee_right); target_world = ee_world + dp, R_delta @ R(ee_world); target_right = from_world
# direct path:  p_right + R^T dp ; R^T R_delta R R_right
for _ in range(50):
    p_r, q_r = rand_pose()
    dp = rng.uniform(-0.05, 0.05, 3)
    drot = rng.uniform(-20, 20, 3)
    ee_w = T_rot.pose_to_world(pose_d(p_r, q_r))
    pw, qw = pose_arr(ee_w)
    Rd = srv._rot_delta_mat(drot)
    tgt_w = pose_d(pw + dp, srv._mat_to_wxyz(Rd @ srv._wxyz_to_mat(qw)))
    tgt_r = T_rot.pose_from_world(tgt_w)
    p_t, q_t = pose_arr(tgt_r)
    R = T_rot.R
    ok("move_delta.pos", np.allclose(p_t, p_r + R.T @ dp, atol=1e-12), f"{p_t - (p_r + R.T @ dp)}")
    ok("move_delta.rot", np.allclose(srv._wxyz_to_mat(q_t), R.T @ Rd @ R @ srv._wxyz_to_mat(q_r), atol=1e-12))
    # pure translation with the NOMINAL (identity-rotation) transform: dpos passes straight through
    ee_wn = T_nom.pose_to_world(pose_d(p_r, q_r))
    pwn, qwn = pose_arr(ee_wn)
    tn = T_nom.pose_from_world(pose_d(pwn + dp, qwn))
    ok("move_delta.nominal", np.allclose(pose_arr(tn)[0], p_r + dp, atol=1e-15) and np.array_equal(pose_arr(tn)[1], q_r))

# ---------------------------------------------------------------- 5. Server wiring with a stub connector (no bridge)
CAM_K = np.array([[100.0, 0, 4.0], [0, 100.0, 4.0], [0, 0, 1.0]])


class StubConn:
    """Mimics the few connector methods server_real.py touches. Poses are in the ARM base frame."""

    def __init__(self, ee_p, ee_q, cam_p, cam_q):
        self.ee = pose_d(ee_p, ee_q)
        self.cam = pose_d(cam_p, cam_q)           # connector emits make_pose() dicts for camera poses and ee_pose
        self.sent = []

    def get_ee_pose(self, arm_id=0):
        return json.loads(json.dumps(self.ee))

    def get_observation(self):
        p, q = pose_arr(self.ee)
        cams = [{"name": "wrist_d405", "rgb": np.zeros((8, 8, 3), np.uint8), "depth": np.full((8, 8), 0.5, np.float32),
                 "intrinsics": CAM_K, "pose": self.cam, "serial": "353322271910", "frame_sequence": 1,
                 "frame_wall_time_ns": 0, "age_s": 0.0, "distortion_model": "none", "distortion_coefficients": np.zeros(5)},
                {"name": "top_brio", "rgb": np.zeros((6, 8, 3), np.uint8), "intrinsics": CAM_K, "pose": self.cam,
                 "serial": "B8C7F203", "frame_sequence": 1, "frame_wall_time_ns": 0, "age_s": 0.0,
                 "distortion_model": "none", "distortion_coefficients": np.zeros(5)}]
        return {"cameras": cams, "arms": [{"joint_state": {"positions": np.zeros(6)}, "gripper_fraction": 1.0,
                                           "ee_pose": pose_d(p, q)}],
                "health": {"state": "ok"}, "sequence": 1, "age_s": 0.0, "wall_time_ns": 0}

    def _tool_go_to_pose_cartesian(self, pose, timeout_s=10.0):
        self.sent.append(("linear", json.loads(json.dumps(pose)), timeout_s))
        self.ee = json.loads(json.dumps(pose))          # arrives exactly
        return {"status": "completed", "detail": ""}

    def _tool_go_to_pose(self, pose, timeout_s=0.0):
        self.sent.append(("plan", json.loads(json.dumps(pose)), timeout_s))
        self.ee = json.loads(json.dumps(pose))
        return {"status": "completed", "detail": ""}


def make_server(arm, T, tmp, conn, allow_motion=True):
    s = object.__new__(srv.Server)
    s.arm = arm
    s.T = T
    s.pose_frame = "robot_base" if arm == "left" else srv.WORLD_FRAME
    s.session = tmp
    sfx = "" if arm == "left" else "_right"
    s.bridge = os.path.join(tmp, "bridge" + sfx)
    s.frames = os.path.join(tmp, "frames" + sfx)
    os.makedirs(s.frames, exist_ok=True)
    s.log_path = os.path.join(tmp, f"server{sfx}.log")
    s.capture_seq = iter(range(1, 1000))
    s.counted, s.budget = 0, 500
    s.allow_motion, s.bare = allow_motion, False
    s.r_min, s.r_max, s.z_min, s.z_max, s.max_step = 0.12, 0.65, -0.05, 0.60, 0.25
    s.host, s.port = "127.0.0.1", 9022 if arm == "right" else 9021
    s.bridge_motion, s.bridge_state, s.safety_state = True, "SERVING", "ok"
    s.record_dir = None
    s.conn = conn
    return s


srv.time.sleep = lambda *_a, **_k: None                 # _move_common sleeps 0.2 s; not needed here

# camera in the arm base: 0.5 m above (0.3, 0, 0), optical axis straight down (q = (0,1,0,0): R = diag(1,-1,-1))
cam_p, cam_q = np.array([0.3, 0.0, 0.5]), np.array([0.0, 1.0, 0.0, 0.0])
ee_p, ee_q = np.array([0.30, -0.10, 0.15]), np.array([0.0, 1.0, 0.0, 0.0])

for label, T in (("nominal", T_nom), ("rotated", T_rot)):
    with tempfile.TemporaryDirectory() as tmp:
        conn = StubConn(ee_p, ee_q, cam_p, cam_q)
        s = make_server("right", T, tmp, conn)
        # status
        st = s.cmd_status({})
        ok(f"right.{label}.status", st["arm"] == "right" and st["world_frame"] == "left_base" and st["robot"] == "yam_real_right"
           and np.allclose(st["base_in_world"]["translation"], T.t) and np.allclose(st["base_in_world"]["rotation_wxyz"], T.q))
        # state: ee_pose in world
        stt = s.cmd_state({})
        ok(f"right.{label}.state_world", stt["ee_pose"] == T.pose_to_world(pose_d(ee_p, ee_q)))
        # frames: default cams wrist only; top/side refused; calib = wrist + _meta, poses world, provenance
        fr = s.cmd_frames({})
        ok(f"right.{label}.frames_default_wrist", fr["ok"] and list(fr["files"]) == ["wrist"] and "depth_npy" in fr["files"]["wrist"])
        calib = json.load(open(fr["calibration_file"]))
        ok(f"right.{label}.calib_keys", set(calib) == {"wrist", "_meta"}, str(list(calib)))
        ok(f"right.{label}.calib_cam_world", calib["wrist"]["frame"] == "left_base"
           and np.allclose(pose_arr(calib["wrist"]["pose"])[0], T.point_to_world(cam_p))
           and same_rot(pose_arr(calib["wrist"]["pose"])[1], T.quat_to_world(cam_q)))
        ok(f"right.{label}.calib_meta", calib["_meta"]["arm"] == "right" and calib["_meta"]["world_frame"] == "left_base"
           and np.allclose(calib["_meta"]["base_in_world"]["translation"], T.t)
           and calib["_meta"]["ee_pose"] == T.pose_to_world(pose_d(ee_p, ee_q)))
        for bad in ("top", "side", "top_brio"):
            r = s.cmd_frames({"cams": ["wrist", bad]})
            ok(f"right.{label}.frames_refuse_{bad}", r["ok"] is False and r["error"] == srv.RIGHT_TOP_ERROR, str(r))
        r = s.cmd_frames({"cams": ["nope"]})
        ok(f"right.{label}.frames_unknown", r["ok"] is False and "unknown camera" in r["error"])
        # deproject: points are world (calib pose is world); plane_z is a world height
        zp_world = float(T.point_to_world([0.3, 0.0, 0.0])[2])
        dpj = s.cmd_deproject({"capture": fr["capture"], "cam": "wrist", "u": 4, "v": 4, "plane_z": zp_world})
        exp = T.point_to_world([0.3, 0.0, 0.0])
        ok(f"right.{label}.deproject_plane_world", dpj["ok"] and np.allclose([dpj["point_base"][k] for k in "xyz"], exp, atol=2e-4), str(dpj))
        dpd = s.cmd_deproject({"capture": fr["capture"], "cam": "wrist", "u": 4, "v": 4})
        ok(f"right.{label}.deproject_depth_world", dpd["ok"] and np.allclose([dpd["point_base"][k] for k in "xyz"], exp, atol=2e-4), str(dpd))
        ok(f"right.{label}.deproject_top_refused", s.cmd_deproject({"capture": 1, "cam": "top", "u": 1, "v": 1})["error"] == srv.RIGHT_TOP_ERROR)
        # move_ee: world target -> right_base target at the bridge; response ee_pose world; error 0
        tgt_r = pose_d([0.35, -0.05, 0.12], q_axis([0, 0, 1], 30.0) if label == "rotated" else ee_q)
        tgt_w = T.pose_to_world(tgt_r)
        mv = s.cmd_move_ee({"position": tgt_w["position"], "rotation": tgt_w["rotation"], "mode": "linear"})
        ok(f"right.{label}.move_ee_ok", mv["ok"] and mv["target_error_mm"] == 0.0, str(mv))
        kind, sent, _ = conn.sent[-1]
        ok(f"right.{label}.move_ee_bridge_frame", kind == "linear"
           and np.allclose(pose_arr(sent)[0], pose_arr(tgt_r)[0], atol=1e-12) and np.allclose(pose_arr(sent)[1], pose_arr(tgt_r)[1], atol=1e-12))
        ok(f"right.{label}.move_ee_resp_world", np.allclose(pose_arr(mv["ee_pose"])[0], pose_arr(tgt_w)[0], atol=1e-12)
           and np.allclose(pose_arr(mv["ee_pose"])[1], pose_arr(tgt_w)[1], atol=1e-12))
        mvp = s.cmd_move_ee({"position": tgt_w["position"], "rotation": tgt_w["rotation"], "mode": "plan"})
        ok(f"right.{label}.move_ee_plan", mvp["ok"] and conn.sent[-1][0] == "plan")
        # guards in the RIGHT base frame: a world point that is fine for the left arm but 0.68 m from the right base
        far_w = T.pose_to_world(pose_d([0.30, 0.61, 0.10], ee_q))
        cl = s.cmd_move_ee({"position": far_w["position"], "rotation": far_w["rotation"]})
        ok(f"right.{label}.clamp_right_frame", cl["ok"] is False and cl["error_code"] == "CLAMP" and "radius 0.680" in cl["error"], str(cl))
        ok(f"right.{label}.clamp_resp_world", cl["ee_pose"] == T.pose_to_world(conn.ee))
        # step guard uses right-frame distances (rigid -> identical to world, but must not include the base offset)
        near_r = pose_d(np.array(pose_arr(conn.ee)[0]) + [0.0, 0.0, 0.05], pose_arr(conn.ee)[1])
        near_w = T.pose_to_world(near_r)
        ok(f"right.{label}.step_ok", s.cmd_move_ee({"position": near_w["position"], "rotation": near_w["rotation"]})["ok"])
        # z guard (2026-09-08 fix): z_min/z_max/TABLE_Z are WORLD (left_base) heights of the table plane, so the
        # test runs on the world-frame target, not on the right_base z (which differs by T's z / rotation)
        q_w = T.quat_to_world(ee_q)
        p_ok_w = T.point_to_world([0.35, -0.05, 0.0]); p_ok_w[2] = -0.048            # 2 mm above the floor, world z
        conn.ee = T.pose_from_world(pose_d(p_ok_w + [0.0, 0.0, 0.10], q_w))         # start 10 cm above it (step guard)
        r_ok = s.cmd_move_ee({"position": pose_d(p_ok_w, q_w)["position"], "rotation": pose_d(p_ok_w, q_w)["rotation"]})
        ok(f"right.{label}.zguard_world_accepts", r_ok["ok"], str(r_ok))
        ok(f"right.{label}.zguard_bridge_z_differs", label != "rotated" or abs(conn.sent[-1][1]["position"]["z"] - (-0.048)) > 0.005,
           str(conn.sent[-1][1]))                                                  # right_base z != world z for the rotated T
        p_bad_w = p_ok_w.copy(); p_bad_w[2] = -0.060                                # 15 mm below the table plane
        r_bad = s.cmd_move_ee({"position": pose_d(p_bad_w, q_w)["position"], "rotation": pose_d(p_bad_w, q_w)["rotation"]})
        ok(f"right.{label}.zguard_world_refuses", r_bad["ok"] is False and r_bad["error_code"] == "CLAMP"
           and "target z -0.060" in r_bad["error"], str(r_bad))                     # message quotes the WORLD z the agent sent
        # move_delta: dpos / drot about WORLD axes
        before_r = pose_arr(conn.ee)
        dp, drot = np.array([0.02, -0.03, 0.01]), [0.0, 0.0, 15.0]
        md = s.cmd_move_delta({"dpos": dp.tolist(), "drot_deg": drot})
        ok(f"right.{label}.move_delta_ok", md["ok"], str(md))
        sent = pose_arr(conn.sent[-1][1])
        ok(f"right.{label}.move_delta_pos", np.allclose(sent[0], before_r[0] + T.R.T @ dp, atol=1e-12))
        ok(f"right.{label}.move_delta_rot", np.allclose(srv._wxyz_to_mat(sent[1]),
                                                       T.R.T @ srv._rot_delta_mat(drot) @ T.R @ srv._wxyz_to_mat(before_r[1]), atol=1e-12))
        ok(f"right.{label}.move_delta_resp_world", np.allclose(pose_arr(md["ee_pose"])[0], T.point_to_world(sent[0]), atol=1e-12))
        # read-only refusal carries a world pose
        s.allow_motion = False
        rf = s.cmd_move_ee({"position": tgt_w["position"], "rotation": tgt_w["rotation"]})
        ok(f"right.{label}.readonly_world", rf["error_code"] == "READ_ONLY" and rf["ee_pose"] == T.pose_to_world(conn.ee))
        ok(f"right.{label}.ready_line", s.ready_line().endswith(" arm=right world_frame=left_base"))

# LEFT arm: T is None -> wrappers return the very same objects; behaviour = today's
with tempfile.TemporaryDirectory() as tmp:
    conn = StubConn(ee_p, ee_q, cam_p, cam_q)
    s = make_server("left", None, tmp, conn)
    d = pose_d(ee_p, ee_q)
    ok("left.to_world_is_same_object", s._to_world(d) is d and s._from_world(d) is d and s._to_world(None) is None)
    st = s.cmd_status({})
    ok("left.status", st["robot"] == "yam_real_left" and st["arm"] == "left" and st["world_frame"] == "left_base" and "base_in_world" not in st)
    ok("left.state_native", s.cmd_state({})["ee_pose"] == d)
    fr = s.cmd_frames({})
    ok("left.frames_both", fr["ok"] and set(fr["files"]) == {"wrist", "top"} and "rectified" in fr["note"])
    calib = json.load(open(fr["calibration_file"]))
    ok("left.calib", set(calib) == {"wrist", "top", "_meta"} and calib["top"]["frame"] == "robot_base"
       and "base_in_world" not in calib["_meta"] and np.allclose(pose_arr(calib["wrist"]["pose"])[0], cam_p))
    tgt = pose_d([0.35, -0.05, 0.12], ee_q)
    mv = s.cmd_move_ee({"position": tgt["position"], "rotation": tgt["rotation"]})
    ok("left.move_ee_passthrough", mv["ok"] and conn.sent[-1][1] == tgt and mv["ee_pose"] == tgt and mv["target_error_mm"] == 0.0)
    lowz = pose_d([0.35, -0.05, -0.06], ee_q)
    lz = s.cmd_move_ee({"position": lowz["position"], "rotation": lowz["rotation"]})
    ok("left.zguard_unchanged", lz["error_code"] == "CLAMP" and lz["error"] == "target z -0.060 m outside [-0.05, 0.6] (table at -0.045)", str(lz))
    farr = pose_d([0.70, 0.0, 0.10], ee_q)
    fz = s.cmd_move_ee({"position": farr["position"], "rotation": farr["rotation"]})
    ok("left.rguard_unchanged", fz["error"] == "target radius 0.700 m outside [0.12, 0.65]", str(fz))
    ok("left.ready_line", s.ready_line().endswith("safety_state=ok"))
    ok("left.frames_top_allowed", s.cmd_frames({"cams": ["side"]})["ok"])

# ---------------------------------------------------------------- 6. solve_base_transform recovers a synthetic T
# ground truth
T_true = sbt.make_T([0.012, -0.604, -0.006], sbt.quat_mul(q_axis([0, 0, 1], -3.0), q_axis([0, 1, 0], 1.2)))
T_lb_lt = sbt.make_T([0.062, -0.329, 0.784], [0.1019, -0.6913, 0.7097, -0.0897] / np.linalg.norm([0.1019, -0.6913, 0.7097, -0.0897]))
T_rb_rw = sbt.make_T([0.28, 0.05, 0.42], q_axis([1, 0, 0], 175.0))
T_lt_rw = sbt.inv_T(T_lb_lt) @ T_true @ T_rb_rw            # what the user's camera-camera calibration measures
T_sol = sbt.solve(T_lb_lt, T_lt_rw, T_rb_rw)
ok("solve.recovers", np.allclose(T_sol, T_true, atol=1e-12))
ok("solve.inverse_convention", np.allclose(sbt.solve(T_lb_lt, sbt.inv_T(sbt.inv_T(T_lt_rw)), T_rb_rw), T_true, atol=1e-12))

with tempfile.TemporaryDirectory() as tmp:
    # (a) left calib.json as the LEFT server writes it (top pose in robot_base == left_base)
    t, q = sbt.split_T(T_lb_lt)
    left_calib = {"top": {"intrinsics": CAM_K.tolist(), "pose": pose_d(t, q), "frame": "robot_base", "image_size": [1920, 1080]},
                  "_meta": {"capture_time": "2026-09-08 12:00:00", "sequence": 5}}
    lp = os.path.join(tmp, "0005_calib.json")
    json.dump(left_calib, open(lp, "w"))
    # (c) right calib.json as the RIGHT server writes it under the NOMINAL offset: world-frame pose + provenance
    T_used = sbt.make_T(sbt.NOMINAL_T, sbt.NOMINAL_Q)
    tw, qw = sbt.split_T(T_used @ T_rb_rw)
    right_calib = {"wrist": {"intrinsics": CAM_K.tolist(), "pose": pose_d(tw, qw), "frame": "left_base", "image_size": [640, 480]},
                   "_meta": {"arm": "right", "world_frame": "left_base", "capture_time": "2026-09-08 12:00:00", "sequence": 9,
                             "base_in_world": {"translation": sbt.NOMINAL_T.tolist(), "rotation_wxyz": sbt.NOMINAL_Q.tolist(),
                                               "source": "nominal"}}}
    rp = os.path.join(tmp, "0009_calib.json")
    json.dump(right_calib, open(rp, "w"))
    T_rec, info = sbt.camera_pose_from_calib(rp, "wrist", "right-calib")
    ok("solve.undo_right_server_conversion", np.allclose(T_rec, T_rb_rw, atol=1e-12) and "undone_base_in_world" in info)
    # (b) the user's calibration in the three accepted encodings
    tc, qc = sbt.split_T(T_lt_rw)
    c_str = ",".join(f"{v:.17g}" for v in np.concatenate([tc, qc]))
    c_json = os.path.join(tmp, "cam2cam.json")
    json.dump({"matrix": T_lt_rw.tolist()}, open(c_json, "w"))
    c_npz = os.path.join(tmp, "cam2cam.npz")
    np.savez(c_npz, camera_to_world=T_lt_rw)
    for enc in (c_str, c_json, c_npz):
        out = os.path.join(tmp, "right_base_in_left_base.json")
        if os.path.exists(out):
            os.remove(out)
        rc = sbt.main(["--left-calib", lp, "--cam-to-cam", enc, "--right-calib", rp, "--out", out, "--source", "test"])
        ok("solve.cli_rc", rc == 0, f"rc={rc} for {enc[:40]}")
        res = json.load(open(out))
        ok("solve.cli_file_keys", {"translation", "rotation_wxyz", "source", "date"} <= set(res))
        T_file = sbt.make_T(res["translation"], res["rotation_wxyz"])
        ok("solve.cli_file_values", np.allclose(T_file, T_true, atol=1e-9), str(res))
        # the server loads exactly this file shape
        Tl = BaseTransform.from_file(out)
        ok("solve.server_loads_file", np.allclose(Tl.t, T_true[:3, 3]) and np.allclose(Tl.R, T_true[:3, :3], atol=1e-9)
           and Tl.source == "test" and Tl.path == out)
    # refuses to overwrite, --force overwrites, --check / --dry-run never write
    rc = sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out])
    ok("solve.no_overwrite", rc == 3)
    ok("solve.force", sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out, "--force"]) == 0)
    os.remove(out)
    ok("solve.check_no_write", sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out, "--check"]) == 0
       and not os.path.exists(out))
    ok("solve.dryrun_no_write", sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out, "--dry-run"]) == 0
       and not os.path.exists(out))
    ok("solve.check_only_nofile", sbt.main(["--check", "--out", out]) == 0)
    # (finding 6) an existing file in the server's ALTERNATE key shape ("rotation": {w,x,y,z}) is recognised -> protected
    alt = {"translation": [0.001, -0.612, 0.002], "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, "source": "hand-calibrated"}
    json.dump(alt, open(out, "w"))
    ok("solve.alt_shape_server_loads", np.allclose(BaseTransform.from_file(out).t, alt["translation"]))
    rc = sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out])
    ok("solve.no_overwrite_alt_shape", rc == 3 and json.load(open(out)) == alt, f"rc={rc}")
    ok("solve.check_alt_shape", sbt.main(["--check", "--out", out]) == 0)
    # ... and an existing but UNPARSEABLE file is protected too (only --force replaces it)
    open(out, "w").write("{not json")
    rc = sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out])
    ok("solve.no_overwrite_unparseable", rc == 3 and open(out).read() == "{not json", f"rc={rc}")
    ok("solve.check_unparseable", sbt.main(["--check", "--out", out]) == 2)
    ok("solve.force_over_unparseable", sbt.main(["--left-calib", lp, "--cam-to-cam", c_str, "--right-calib", rp, "--out", out, "--force"]) == 0
       and np.allclose(sbt.make_T(*[json.load(open(out))[k] for k in ("translation", "rotation_wxyz")]), T_true, atol=1e-9))
    os.remove(out)
    # (finding 7) capture-time skew between the two calib.json inputs: stored, warned, and a write refused above 2 s
    def with_wall(src, dst, ns):
        c = json.load(open(src)); c["_meta"]["wall_time_ns"] = ns; json.dump(c, open(dst, "w"))
    lp_t = os.path.join(tmp, "0005_calib_t.json"); rp_t = os.path.join(tmp, "0009_calib_t.json")
    with_wall(lp, lp_t, 1_700_000_000_000_000_000); with_wall(rp, rp_t, 1_700_000_000_000_000_000 + 500_000_000)      # 0.5 s apart
    ok("solve.skew_small_ok", sbt.main(["--left-calib", lp_t, "--cam-to-cam", c_str, "--right-calib", rp_t, "--out", out]) == 0
       and json.load(open(out))["inputs"]["capture_skew_s"] == 0.5)
    os.remove(out)
    with_wall(rp, rp_t, 1_700_000_000_000_000_000 + 30_000_000_000)                                                    # 30 s apart
    ok("solve.skew_refused", sbt.main(["--left-calib", lp_t, "--cam-to-cam", c_str, "--right-calib", rp_t, "--out", out]) == 4
       and not os.path.exists(out))
    ok("solve.skew_dryrun_warns_only", sbt.main(["--left-calib", lp_t, "--cam-to-cam", c_str, "--right-calib", rp_t, "--out", out, "--dry-run"]) == 0
       and not os.path.exists(out))
    ok("solve.skew_allowed", sbt.main(["--left-calib", lp_t, "--cam-to-cam", c_str, "--right-calib", rp_t, "--out", out, "--allow-time-skew"]) == 0
       and json.load(open(out))["inputs"]["capture_skew_s"] == 30.0)
    os.remove(out)
    ok("solve.skew_threshold_arg", sbt.main(["--left-calib", lp_t, "--cam-to-cam", c_str, "--right-calib", rp_t, "--out", out, "--max-time-skew-s", "60"]) == 0)
    os.remove(out)
    ok("solve.skew_none_without_times", sbt.capture_skew_s({"left_top": {"file": "x"}, "right_wrist": {"spec": "y"}}) is None)
    # --invert accepts the opposite arrow convention
    inv_json = os.path.join(tmp, "cam2cam_inv.json")
    json.dump(sbt.inv_T(T_lt_rw).tolist(), open(inv_json, "w"))
    ok("solve.invert", sbt.main(["--left-calib", lp, "--cam-to-cam", inv_json, "--invert", "--right-calib", rp, "--out", out, "--force"]) == 0
       and np.allclose(sbt.make_T(*[json.load(open(out))[k] for k in ("translation", "rotation_wxyz")]), T_true, atol=1e-9))
    # the LEFT snapshot config/top_calib.json shape is accepted as (a)
    ok("solve.top_calib_snapshot", np.allclose(sbt.pose_dict_to_T(json.load(open(os.path.join(FA, "config", "top_calib.json"))), "x")[:3, 3],
                                               [0.06201349885528393, -0.32912758902618217, 0.7841590759717554]))
    # BaseTransform rejects garbage
    for bad in ({"translation": [0, 0], "rotation_wxyz": [1, 0, 0, 0]}, {"translation": [0, 0, 0], "rotation_wxyz": [2, 0, 0, 0]},
                {"translation": [0, 0, 0]}, {"translation": [0, 0, float("nan")], "rotation_wxyz": [1, 0, 0, 0]}):
        bp = os.path.join(tmp, "bad.json")
        json.dump(bad, open(bp, "w"))
        try:
            BaseTransform.from_file(bp)
            ok("basetransform.rejects", False, str(bad))
        except ValueError:
            ok("basetransform.rejects", True)

# the example config parses and equals the nominal transform; the REAL file must not exist in the repo
Tx = BaseTransform.from_file(os.path.join(FA, "config", "right_base_in_left_base.example.json"))
ok("example.nominal", np.array_equal(Tx.t, T_nom.t) and np.array_equal(Tx.q, T_nom.q))
ok("real_config_absent", not os.path.exists(srv.DEFAULT_BASE_TRANSFORM_FILE), srv.DEFAULT_BASE_TRANSFORM_FILE)

names = sorted(set(PASSED))
print(f"OK: {len(PASSED)} checks, {len(names)} distinct cases")
for n in names:
    print("  ", n)
