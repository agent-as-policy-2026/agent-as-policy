#!/usr/bin/env python3
"""Check a bridge's overhead-camera calibration with a checkerboard seen by BOTH cameras.

    <hardware-bridge>/.venv/bin/python3 tools/check_top_calib_board.py <session> <capture N>
        [--square 0.022] [--cols 9 --rows 7] [--alt-top-calib JSON [--alt-scale 3.0]]

Reads frames/NNNN_{top,wrist}.png + NNNN_calib.json of an agp session (either arm, any mode).
Board pose from the WRIST camera (PnP with its intrinsics/distortion, then the kinematic camera
pose from the calib JSON) is the reference; the same board from the TOP image (PnP, then the
top camera_to_world the bridge serves) is compared: centre offset [mm], normal angle and in-plane
angle [deg], plus the wrist DEPTH at the board centre versus the PnP distance. --alt-top-calib
evaluates another camera_to_world/camera_matrix file (e.g. a 1080p calibration; --alt-scale 3
divides fx,fy,cx,cy for a 640x360 stream) on the same image.
"""
import argparse, json, sys
import numpy as np, cv2

ap = argparse.ArgumentParser()
ap.add_argument("session"); ap.add_argument("capture", type=int)
ap.add_argument("--square", type=float, default=0.022)
ap.add_argument("--cols", type=int, default=9); ap.add_argument("--rows", type=int, default=7)
ap.add_argument("--alt-top-calib", default=None); ap.add_argument("--alt-scale", type=float, default=1.0)
a = ap.parse_args()
S = a.session.rstrip("/"); n = a.capture
calib = json.load(open(f"{S}/frames/{n:04d}_calib.json"))

def quat_R(q):
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def T_of(pose):
    T = np.eye(4); T[:3, :3] = quat_R(pose["rotation"]); p = pose["position"]; T[:3, 3] = [p["x"], p["y"], p["z"]]; return T

def detect(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    for (c, r) in [(a.cols-1, a.rows-1), (a.cols, a.rows), (a.rows-1, a.cols-1), (a.rows, a.cols)]:
        for fn, kw in ((getattr(cv2, "findChessboardCornersSB", None), dict(flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)),
                       (cv2.findChessboardCorners, dict(flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE))):
            if fn is None: continue
            ok, pts = fn(g, (c, r), **kw)
            if ok:
                if fn is cv2.findChessboardCorners:
                    pts = cv2.cornerSubPix(g, pts, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3))
                return (c, r), pts.reshape(-1, 2), fn.__name__
    return None, None, None

def board_pose(img, K, dist, T_base_cam, label):
    (c, r), pts, how = detect(img)
    if pts is None:
        print(f"[{label}] board NOT detected"); return None
    objp = np.zeros((c*r, 3)); objp[:, :2] = np.mgrid[0:c, 0:r].T.reshape(-1, 2) * a.square
    ok, rvec, tvec = cv2.solvePnP(objp, pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist); rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - pts)**2, axis=1))))
    R, _ = cv2.Rodrigues(rvec); T_cam_board = np.eye(4); T_cam_board[:3, :3] = R; T_cam_board[:3, 3] = tvec.ravel()
    T_bb = T_base_cam @ T_cam_board
    centre_board = np.array([(c-1)/2*a.square, (r-1)/2*a.square, 0, 1.0])
    centre = (T_bb @ centre_board)[:3]
    normal = T_bb[:3, 2]; xaxis = T_bb[:3, 0]
    print(f"[{label}] {how} {c}x{r} corners, PnP reproj rms {rms:.2f} px, cam-board dist {np.linalg.norm(tvec):.3f} m; "
          f"board centre in base {np.round(centre, 4)} normal {np.round(normal, 3)}")
    return dict(centre=centre, normal=normal, xaxis=xaxis, pts=pts, size=(c, r), T=T_bb, dist=float(np.linalg.norm(tvec)))

wr = calib["wrist"]; tp = calib["top"]
K_w = np.array(wr["intrinsics"]); d_w = np.array(wr["distortion_coefficients"] or [0]*5, dtype=float)
K_t = np.array(tp["intrinsics"]); d_t = np.zeros(5) if tp.get("rectified") else np.array(tp["distortion_coefficients"] or [0]*5, dtype=float)
img_w = cv2.imread(f"{S}/frames/{n:04d}_wrist.png"); img_t = cv2.imread(f"{S}/frames/{n:04d}_top.png")
print(f"frame {calib['top'].get('frame')} | wrist {img_w.shape[1]}x{img_w.shape[0]} dist model {wr.get('distortion_model')} | top {img_t.shape[1]}x{img_t.shape[0]} rectified={tp.get('rectified')}")
ref = board_pose(img_w, K_w, d_w, T_of(wr["pose"]), "wrist (reference)")
ref0 = board_pose(img_w, K_w, np.zeros(5), T_of(wr["pose"]), "wrist, distortion ignored")
if ref is not None:
    # independent check: wrist depth at the board centre pixel vs PnP distance
    depth = np.load(f"{S}/frames/{n:04d}_wrist_depth.npy"); cpx = ref["pts"].mean(axis=0); u, v = int(round(cpx[0])), int(round(cpx[1]))
    win = depth[max(0, v-2):v+3, max(0, u-2):u+3]; win = win[win > 0.05]
    T_cb = np.linalg.inv(T_of(wr["pose"])) @ np.r_[ref["centre"], 1.0]
    print(f"[wrist depth] centre pixel ({u},{v}) median depth {np.median(win) if win.size else float('nan'):.4f} m vs PnP centre z_cam {T_cb[2]:.4f} m  (n={win.size})")
def compare(label, res):
    if ref is None or res is None: return
    dc = (res["centre"] - ref["centre"]) * 1000
    ang_n = np.degrees(np.arccos(np.clip(abs(np.dot(res["normal"], ref["normal"])), -1, 1)))
    ang_x = np.degrees(np.arccos(np.clip(abs(np.dot(res["xaxis"], ref["xaxis"])), -1, 1)))
    print(f"==> {label}: centre offset {np.round(dc, 1)} mm (|{np.linalg.norm(dc):.1f}| mm), normal {ang_n:.2f} deg, in-plane axis {ang_x:.2f} deg")
top = board_pose(img_t, K_t, d_t, T_of(tp["pose"]), "top (as served)")
compare("top calibration AS SERVED vs wrist", top)
if a.alt_top_calib:
    alt = json.load(open(a.alt_top_calib)); Ka = np.array(alt["camera_matrix"], dtype=float)
    if a.alt_scale != 1.0: Ka[0, 0] /= a.alt_scale; Ka[1, 1] /= a.alt_scale; Ka[0, 2] /= a.alt_scale; Ka[1, 2] /= a.alt_scale
    Ta = np.array(alt["camera_to_world"], dtype=float)
    # the served image is rectified with the SERVED intrinsics; the alt K applies to a raw image — use the served K for pixels, alt only for the pose
    alt_pose = board_pose(img_t, K_t, d_t, Ta, f"top with ALT camera_to_world ({a.alt_top_calib.split('/')[-1]}), served K")
    compare("ALT extrinsics vs wrist", alt_pose)
    alt_pose2 = board_pose(img_t, Ka, np.zeros(5), Ta, "top with ALT K (scaled) + ALT camera_to_world")
    compare("ALT extrinsics + ALT K vs wrist", alt_pose2)
