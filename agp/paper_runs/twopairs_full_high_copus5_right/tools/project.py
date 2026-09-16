# tool: project.py
# category: geometry
# purpose: project base-frame 3D points into the top or wrist image of a capture and optionally draw them on a (cropped, 2x-upscaled) copy, to check alignment of a held part over a target or to sanity-check a position estimate against the image
# usage: <venv-python from README_interface.md> knowledge/tools/project.py <capture_N> <top|wrist> "x,y,z" ["x,y,z" ...] [--out scratch/file.png] [--crop u0,v0,u1,v1]    (run from the session directory; needs numpy+PIL)
# inputs/outputs: reads frames/NNNN_calib.json (and frames/NNNN_<cam>.png with --out); prints u,v per point; with --out writes the image with circles coloured red, lime, blue, yellow, magenta, cyan, orange, white in argument order
# assumptions: pinhole model Xc = R^T (X - t), pixel = K Xc/Xc_z as documented in README_interface.md; wrist lens distortion ignored (top image is already rectified)
# verified: used successfully in the session that wrote it
import sys, json, numpy as np
from PIL import Image, ImageDraw

N = int(sys.argv[1]); cam = sys.argv[2]
args = sys.argv[3:]; out = None; crop = None
if "--out" in args:
    i = args.index("--out"); out = args[i + 1]; args = args[:i] + args[i + 2:]
if "--crop" in args:
    i = args.index("--crop"); crop = [int(v) for v in args[i + 1].split(",")]; args = args[:i] + args[i + 2:]
cal = json.load(open(f"frames/{N:04d}_calib.json"))[cam]
K = np.array(cal["intrinsics"]); p = cal["pose"]
t = np.array([p["position"][k] for k in "xyz"]); q = p["rotation"]
w, x, y, z = q["w"], q["x"], q["y"], q["z"]
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
cols = ["red", "lime", "blue", "yellow", "magenta", "cyan", "orange", "white"]
pix = []
for a in args:
    X = np.array([float(v) for v in a.split(",")])
    Xc = R.T @ (X - t); uv = (K @ (Xc / Xc[2]))[:2]
    pix.append(uv); print(f"{a} -> u={uv[0]:.1f} v={uv[1]:.1f}")
if out:
    im = Image.open(f"frames/{N:04d}_{cam}.png").convert("RGB"); d = ImageDraw.Draw(im)
    for k, (u, v) in enumerate(pix):
        d.ellipse([u - 4, v - 4, u + 4, v + 4], outline=cols[k % len(cols)], width=2)
    if crop:
        im = im.crop(crop); im = im.resize((im.width * 2, im.height * 2))
    im.save(out)
