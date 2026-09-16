#!/usr/bin/env python3
"""Generic read-only MJPEG preview for a V4L2 camera (any BRIO): open at
640x360@30 MJPG, serve http://<host>:<port>/ . Never touches the robot."""
import argparse, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("--device", required=True)
ap.add_argument("--port", type=int, default=8768)
ap.add_argument("--grid", action="store_true", help="draw a 3x3 grid + centre mark")
ap.add_argument("--width", type=int, default=640)
ap.add_argument("--height", type=int, default=360)
ap.add_argument("--fps", type=int, default=30)
a = ap.parse_args()

cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(3, a.width); cap.set(4, a.height); cap.set(5, a.fps)
print("negotiated", int(cap.get(3)), "x", int(cap.get(4)), "@", cap.get(5), flush=True)
latest = {"jpg": None}

def pump():
    while True:
        ok, f = cap.read()
        if not ok:
            time.sleep(0.05); continue
        if a.grid:
            h, w = f.shape[:2]
            for i in (1, 2):
                cv2.line(f, (w*i//3, 0), (w*i//3, h), (0, 255, 255), 1)
                cv2.line(f, (0, h*i//3), (w, h*i//3), (0, 255, 255), 1)
            cv2.drawMarker(f, (w//2, h//2), (0, 0, 255), cv2.MARKER_CROSS, 20, 1)
        ok2, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok2:
            latest["jpg"] = buf.tobytes()

class H(BaseHTTPRequestHandler):
    def log_message(self, *_): pass
    def do_GET(self):
        if self.path.startswith("/snap"):
            j = latest["jpg"] or b""
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(j))); self.end_headers(); self.wfile.write(j); return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
        try:
            while True:
                j = latest["jpg"]
                if j:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j))
                    self.wfile.write(j); self.wfile.write(b"\r\n")
                time.sleep(1/15)
        except (BrokenPipeError, ConnectionResetError):
            pass

threading.Thread(target=pump, daemon=True).start()
print(f"preview: http://0.0.0.0:{a.port}/  (device {a.device})", flush=True)
ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()
