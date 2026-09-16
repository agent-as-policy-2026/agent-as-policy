#!/usr/bin/env python3
"""Serve a MuJoCo mirror driven only by passive YAM CAN feedback."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import can
import imageio.v3 as iio
import mujoco
import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType


CHANNEL = "can_follower_r"
ARM_IDS = tuple(range(1, 7))
FALLBACK_GRIPPER = 0.998250346285631
INITIAL_ARM_QPOS = np.array(
    [
        0.010109102006561344,
        0.0024795910582131597,
        0.0017166399633783413,
        -0.14324406805523715,
        -1.5699626153963528,
        1.6451132982375825,
    ],
    dtype=float,
)
RENDER_WIDTH = 1280
RENDER_HEIGHT = 960
RENDER_MSAA = 8

state_lock = threading.Lock()
latest_arm_qpos = INITIAL_ARM_QPOS.copy()
latest_feedback_time = 0.0
latest_feedback_error = "waiting for controller feedback"
frame_lock = threading.Lock()
latest_jpeg = b""
stop_event = threading.Event()


def decode_position(data: bytes) -> float:
    encoded = (data[1] << 8) | data[2]
    return encoded * 25.0 / 65535.0 - 12.5


def wrap_arm_position(position: float) -> float:
    if position < -math.pi:
        return position + 2.0 * math.pi
    if position > math.pi:
        return position - 2.0 * math.pi
    return position


def can_state_loop() -> None:
    global latest_arm_qpos, latest_feedback_time, latest_feedback_error

    bus = None
    positions = INITIAL_ARM_QPOS.copy()
    try:
        bus = can.Bus(interface="socketcan", channel=CHANNEL)
        while not stop_event.is_set():
            message = bus.recv(timeout=0.1)
            if message is None:
                continue
            motor_id = message.arbitration_id - 0x10
            if motor_id not in ARM_IDS or len(message.data) < 3:
                continue
            positions[motor_id - 1] = wrap_arm_position(decode_position(message.data))
            with state_lock:
                latest_arm_qpos = positions.copy()
                latest_feedback_time = time.monotonic()
                latest_feedback_error = ""
    except Exception as exc:
        with state_lock:
            latest_feedback_error = f"{type(exc).__name__}: {exc}"
    finally:
        if bus is not None:
            bus.shutdown()


def build_preview_model() -> mujoco.MjModel:
    source = get_yam_robot(
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        sim=True,
    )
    try:
        tree = ET.parse(source.xml_path)
    finally:
        source.close()

    root = tree.getroot()
    asset = root.find("asset")
    ET.SubElement(
        asset,
        "texture",
        name="preview_skybox",
        type="skybox",
        builtin="gradient",
        rgb1="0.82 0.86 0.92",
        rgb2="0.68 0.74 0.82",
        width="512",
        height="3072",
    )
    worldbody = root.find("worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        name="preview_floor",
        type="plane",
        pos="0 0 -0.006",
        size="2 2 0.1",
        rgba="0.72 0.76 0.82 1",
    )
    ET.SubElement(
        worldbody,
        "light",
        name="preview_key",
        pos="-0.8 -0.8 1.5",
        dir="0.35 0.35 -1",
        directional="true",
        diffuse="0.8 0.8 0.8",
        specular="0.15 0.15 0.15",
        castshadow="false",
    )

    with tempfile.NamedTemporaryFile(suffix=".xml") as staged:
        tree.write(staged.name, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(staged.name)

    model.vis.global_.offwidth = RENDER_WIDTH
    model.vis.global_.offheight = RENDER_HEIGHT
    model.vis.quality.offsamples = RENDER_MSAA
    model.vis.headlight.ambient[:] = [0.35, 0.35, 0.35]
    model.vis.headlight.diffuse[:] = [0.75, 0.75, 0.75]
    model.vis.headlight.specular[:] = [0.12, 0.12, 0.12]
    return model


def render_loop() -> None:
    global latest_jpeg

    model = build_preview_model()
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.30]
    camera.distance = 0.75
    camera.azimuth = 135
    camera.elevation = -22

    with mujoco.Renderer(model, height=RENDER_HEIGHT, width=RENDER_WIDTH) as renderer:
        while not stop_event.is_set():
            with state_lock:
                arm = latest_arm_qpos.copy()
            for joint_index, position in enumerate(arm):
                qpos_address = model.jnt_qposadr[joint_index]
                lo, hi = model.jnt_range[joint_index]
                data.qpos[qpos_address] = np.clip(position, lo, hi)
            if model.njnt > 6:
                qpos_address = model.jnt_qposadr[6]
                lo, hi = model.jnt_range[6]
                data.qpos[qpos_address] = lo + FALLBACK_GRIPPER * (hi - lo)
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            frame = renderer.render()
            encoded = iio.imwrite("<bytes>", frame, extension=".jpg", quality=92)
            with frame_lock:
                latest_jpeg = encoded
            stop_event.wait(0.10)


PAGE = """<!doctype html>
<meta charset="utf-8"><title>YAM live MuJoCo mirror</title>
<style>
  :root { color-scheme:light; font-family:system-ui,sans-serif; }
  body { margin:0; background:#e9edf2; color:#17202a; }
  main { max-width:1280px; margin:auto; padding:20px; }
  h2 { margin:0 0 6px; }
  .status { margin:0 0 14px; color:#425466; }
  img { display:block; width:100%; background:#cdd3dc; border:1px solid #aeb7c4; border-radius:8px; }
  pre { background:white; padding:12px; border:1px solid #ccd3dc; border-radius:8px; overflow:auto; }
</style>
<main>
  <h2>YAM right arm — live MuJoCo mirror</h2>
  <p class="status" id="status">Waiting for controller feedback...</p>
  <img src="/stream.mjpg" alt="YAM live MuJoCo state">
  <pre id="joints"></pre>
</main>
<script>
async function updateState() {
  try {
    const state = await (await fetch('/state.json', {cache:'no-store'})).json();
    document.getElementById('status').textContent = state.live
      ? 'LIVE - passive CAN feedback - fixed camera - 1280x960 / 8x MSAA'
      : 'HOLDING LAST FEEDBACK - ' + state.error;
    document.getElementById('joints').textContent = state.arm_qpos_rad
      .map((v, i) => 'joint' + (i + 1) + ': ' + v.toFixed(6) + ' rad').join('\n');
  } catch (error) {
    document.getElementById('status').textContent = 'Preview state request failed: ' + error;
  }
}
updateState(); setInterval(updateState, 250);
</script>""".encode("utf-8")


def state_payload() -> dict:
    with state_lock:
        age = time.monotonic() - latest_feedback_time if latest_feedback_time else None
        arm = latest_arm_qpos.copy()
        error = latest_feedback_error
    return {
        "source": CHANNEL,
        "mode": "passive CAN feedback; sends no CAN frames",
        "live": age is not None and age < 1.0 and not error,
        "feedback_age_s": age,
        "error": error,
        "arm_qpos_rad": arm.tolist(),
    }


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: object) -> None:
        pass

    def send_bytes(self, content_type: str, body: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self.send_bytes("text/html; charset=utf-8", PAGE)
            return
        if self.path == "/healthz":
            self.send_bytes("text/plain; charset=utf-8", b"ok\n")
            return
        if self.path == "/state.json":
            self.send_bytes(
                "application/json; charset=utf-8",
                json.dumps(state_payload()).encode("utf-8"),
            )
            return
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Connection", "close")
            self.end_headers()
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            while not stop_event.is_set():
                with frame_lock:
                    jpeg = latest_jpeg
                if not jpeg:
                    time.sleep(0.02)
                    continue
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.flush()
                time.sleep(0.10)
        except (BrokenPipeError, ConnectionResetError):
            pass


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main() -> None:
    threading.Thread(target=can_state_loop, name="yam-can-passive", daemon=True).start()
    threading.Thread(target=render_loop, name="mujoco-render", daemon=True).start()
    print("YAM passive MuJoCo mirror: http://127.0.0.1:8090/", flush=True)
    try:
        Server(("127.0.0.1", 8090), Handler).serve_forever()
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
