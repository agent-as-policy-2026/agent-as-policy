#!/usr/bin/env python3
"""Grab one frame from the overhead camera (and optionally the wrist camera) through the
running bridge and save it as PNG. Observation-only: no motion, no lease.

Usage (bridge must be up in any mode; no other bridge observer running):
  capture_top.py --out /path/to/dir_or_file.png [--wrist] [--host 127.0.0.1] [--port 9021]

If --out is a directory (or ends with '/'), files are named top_camera.png (and
wrist_camera.png). The saved top image is the rectified 1920x1080 frame exactly as an
agent receives it from `frames`.
"""
import argparse
import os
import sys
import time

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output PNG path, or a directory")
    ap.add_argument("--wrist", action="store_true", help="also save the wrist RGB frame")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9021)
    ap.add_argument("--robot", choices=("yam_left", "yam_right"), default="yam_left",
                    help="connector robot name: yam_right for the right arm's bridge (its own overhead BRIO, 640x360)")
    ap.add_argument("--wait-timeout-s", type=float, default=30.0)
    a = ap.parse_args()

    out = a.out
    as_dir = out.endswith("/") or os.path.isdir(out) or not out.lower().endswith(".png")
    if as_dir:
        os.makedirs(out, exist_ok=True)
        top_path = os.path.join(out, "top_camera.png")
        wrist_path = os.path.join(out, "wrist_camera.png")
    else:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        top_path = out
        stem, _ = os.path.splitext(out)
        wrist_path = stem + "_wrist.png"

    import gap.connector  # noqa: E402  (the vendored connector, in the bridge venv)
    from PIL import Image

    t0 = time.time()
    try:
        conn = gap.connector.real(a.robot, host=a.host, port=a.port, wait_timeout_s=a.wait_timeout_s)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: cannot connect to the bridge at {a.host}:{a.port}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    try:
        obs = conn.get_observation()
        saved = []
        for cam in obs["cameras"]:
            rgb = np.asarray(cam["rgb"])[..., :3]
            if rgb.dtype != np.uint8:
                rgb = (rgb * 255.0 if float(rgb.max()) <= 1.0 else rgb).clip(0, 255).astype(np.uint8)
            if cam["name"] == "top_brio":
                Image.fromarray(np.ascontiguousarray(rgb)).save(top_path)
                saved.append(top_path)
            elif cam["name"] == "wrist_d405" and a.wrist:
                Image.fromarray(np.ascontiguousarray(rgb)).save(wrist_path)
                saved.append(wrist_path)
        h = obs.get("health", {})
        print(f"saved: {', '.join(saved)}")
        print(f"bridge: state={h.get('state')} motion_enabled={h.get('motion_enabled')} "
              f"observation age {float(obs.get('age_s', 0))*1000:.0f} ms, took {time.time()-t0:.1f}s")
        return 0 if saved else 3
    finally:
        try:
            conn.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
