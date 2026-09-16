#!/usr/bin/env python3
"""Hardware-reset a wrist D405 that enumerates but does not stream, then prove it streams.

    <hardware-bridge>/.venv/bin/python3 tools/reset_wrist_d405.py --serial 353322271910 [--no-reset]

Refuses while any process holds the camera's /dev/video nodes (stop that bridge first:
bash start_bridges.sh stop right). Steps: hardware_reset() -> wait for the device to
re-enumerate (<= 20 s) -> start the bridge's wrist profile (640x360@30 RGB8 + Z16) ->
count frames for 3 s. Exit 0 only if >= 30 frames arrived. --no-reset = stream test only.
Seen 2026-09-08: after a dirty USB disconnect (uvcvideo -71) the D405 came back on the bus,
opened fine, but delivered no frames ("Frame didn't arrive within 1000") until reset.
"""
import argparse, glob, os, subprocess, sys, time
import pyrealsense2 as rs

ap = argparse.ArgumentParser()
ap.add_argument("--serial", required=True)
ap.add_argument("--no-reset", action="store_true")
ap.add_argument("--seconds", type=float, default=3.0)
a = ap.parse_args()

def find(serial):
    for d in rs.context().query_devices():
        if d.get_info(rs.camera_info.serial_number) == serial:
            return d
    return None

dev = find(a.serial)
if dev is None:
    print(f"[d405] serial {a.serial} not enumerated (check the USB cable / replug)"); sys.exit(2)
port = dev.get_info(rs.camera_info.physical_port)           # /sys/.../usb6/6-2/6-2:1.0/video4linux/videoN
usb_dir = port.split("/video4linux/")[0].rsplit("/", 1)[0]  # /sys/.../6-2
nodes = sorted(os.path.basename(p) for p in glob.glob(usb_dir + "/*/video4linux/video*"))
busy = subprocess.run(["fuser"] + [f"/dev/{n}" for n in nodes], capture_output=True, text=True).stdout.split()
if busy:
    print(f"[d405] refusing: /dev nodes of {os.path.basename(usb_dir)} are held by pid(s) {sorted(set(busy))} — stop that bridge first"); sys.exit(2)
print(f"[d405] {a.serial} at {os.path.basename(usb_dir)} usb {dev.get_info(rs.camera_info.usb_type_descriptor)} fw {dev.get_info(rs.camera_info.firmware_version)}")

if not a.no_reset:
    print("[d405] hardware_reset() ..."); dev.hardware_reset(); del dev
    t0 = time.time(); dev = None
    time.sleep(2.0)
    while time.time() - t0 < 20.0:
        dev = find(a.serial)
        if dev is not None: break
        time.sleep(0.5)
    if dev is None:
        print("[d405] did not re-enumerate within 20 s"); sys.exit(3)
    print(f"[d405] re-enumerated after {time.time() - t0:.1f} s (usb {dev.get_info(rs.camera_info.usb_type_descriptor)})")
    time.sleep(1.0)

cfg = rs.config(); cfg.enable_device(a.serial)
cfg.enable_stream(rs.stream.color, 640, 360, rs.format.rgb8, 30)
cfg.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
pipe = rs.pipeline()
try:
    pipe.start(cfg)
except Exception as e:
    print(f"[d405] pipeline.start failed: {e}"); sys.exit(3)
n = 0; miss = 0; t0 = time.time(); first = None
while time.time() - t0 < a.seconds:
    try:
        pipe.wait_for_frames(timeout_ms=1000); n += 1
        if first is None: first = time.time() - t0
    except Exception:
        miss += 1
pipe.stop()
print(f"[d405] {n} frames in {a.seconds:.0f} s (first after {first if first is None else round(first, 2)} s, {miss} timeouts)")
sys.exit(0 if n >= 30 else 3)
