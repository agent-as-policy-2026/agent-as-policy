"""Continuous camera capture + optional run recording for the production cameras.

Why: the bridge owns the wrist D405 and the overhead BRIO exclusively and, until
now, read them only when a client asked for an observation. A run video of those
two views therefore had to be pieced together from the client's own captures.
This module makes a dedicated thread per camera the ONLY hardware reader: it
captures continuously, keeps the latest frame in a slot for observations, and —
while a recording is active — pipes every frame to an ffmpeg encoder.

Observation semantics are preserved: ``StreamingCamera.read()`` blocks until a
frame captured AFTER the call arrives (like ``wait_for_frames`` did), so the
camera/joint skew stays as small as before, and every delivered frame carries a
strictly increasing sequence. Freshness gates are unchanged: a frame older than
``stale_after_s`` is never handed out, and a dead capture thread fails closed.

Recording is toggled from the bridge process by SIGUSR1 (start) / SIGUSR2 (stop),
wired in ``main.py``. The output directory is read from ``<record_dir>/target.txt``
when that file exists (one line, an absolute path), else ``<record_dir>/<ts>/``.
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from agp_yam_bridge.camera import CameraFrameError

logger = logging.getLogger(__name__)


class RunRecorder:
    """ffmpeg-backed writer for named raw-frame streams (one mp4 per camera)."""

    def __init__(self, record_dir: Path | None, *, live_readable: bool = False) -> None:
        self._record_dir = Path(record_dir) if record_dir else None
        # 2026-09-14 (throwing sessions): fragmented MP4 with a keyframe about every second, so
        # the file being written can be decoded while the recording runs (an agent aligning the
        # camera frames with a program's execution trace). Default = the plain MP4 written so far
        # (moov atom only at the end: unreadable until the recording stops).
        self.live_readable = bool(live_readable)
        self._lock = threading.Lock()
        self._procs: dict[str, subprocess.Popen[bytes]] = {}
        self._specs: dict[str, tuple[int, int, str, int, int]] = {}  # name -> (w, h, pixfmt, fps, every)
        self._counts: dict[str, int] = {}
        self._written: dict[str, int] = {}
        self._out_dir: Path | None = None
        self._ffmpeg = shutil.which("ffmpeg")
        # Side data written next to the videos while a recording is active (2026-09-04, for
        # later policy-learning datasets): joints.csv = the arm state at the logger's rate,
        # <stream>_frames.csv = wall/monotonic capture time of every frame pushed to ffmpeg.
        self._csv_lock = threading.Lock()
        self._csv: dict[str, Any] = {}
        self._running = False

    @property
    def enabled(self) -> bool:
        return self._record_dir is not None and self._ffmpeg is not None

    @property
    def active(self) -> bool:
        return self._running

    def register(self, name: str, *, width: int, height: int, pixfmt: str, fps: int, every: int = 1) -> None:
        """Declare a stream. ``every`` keeps one frame in N (e.g. 2 -> half rate)."""
        self._specs[name] = (int(width), int(height), pixfmt, int(fps), max(1, int(every)))

    def start(self, out_dir: Path | None = None) -> Path | None:
        if not self.enabled:
            logger.warning("recording requested but the recorder is disabled (no --record-dir or no ffmpeg)")
            return None
        with self._lock:
            if self._procs:
                logger.warning("recording already active in %s", self._out_dir)
                return self._out_dir
            target = out_dir
            if target is None:
                marker = self._record_dir / "target.txt"
                if marker.exists():
                    line = marker.read_text(encoding="utf-8").strip()
                    if line:
                        target = Path(line)
            if target is None:
                target = self._record_dir / time.strftime("%Y%m%d_%H%M%S")
            target.mkdir(parents=True, exist_ok=True)
            self._out_dir = target
            for name, (w, h, pixfmt, fps, every) in self._specs.items():
                out = target / f"{name}.mp4"
                # Timestamps come from the wall clock at pipe-read time, not from the
                # nominal rate: a capture thread that achieves fewer fps than the
                # device's nominal rate (the D405 path runs ~15 fps) would otherwise
                # produce a video that plays too fast.
                rate = max(1, fps // every)
                if self.live_readable:
                    # zerolatency: no encoder lookahead / B-frame delay, so a frame is on disk
                    # within ~0.1 s of capture (measured 2026-09-14; ~1.5 s without it)
                    container = ["-tune", "zerolatency", "-g", str(rate),
                                 "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
                                 "-frag_duration", "500000", "-flush_packets", "1"]
                else:
                    container = ["-movflags", "+faststart"]
                cmd = [
                    self._ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
                    "-use_wallclock_as_timestamps", "1",
                    "-f", "rawvideo", "-pix_fmt", pixfmt, "-s", f"{w}x{h}", "-r", str(rate),
                    "-i", "-",
                    "-fps_mode", "vfr",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p",
                    *container, str(out),
                ]
                log = open(target / f"{name}.ffmpeg.log", "ab")  # noqa: SIM115
                self._procs[name] = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=log, stderr=log)
                self._counts[name] = 0
                self._written[name] = 0
            with self._csv_lock:
                for name in self._specs:
                    fh = open(target / f"{name}_frames.csv", "w", encoding="utf-8")  # noqa: SIM115
                    fh.write("frame_index,wall_time_ns,monotonic_ns\n")
                    self._csv[name] = fh
                fh = open(target / "joints.csv", "w", encoding="utf-8")  # noqa: SIM115
                fh.write("wall_time_ns,monotonic_ns,sequence,"
                         + ",".join(f"q{i}" for i in range(7)) + ","
                         + ",".join(f"v{i}" for i in range(7)) + ","
                         + ",".join(f"eff{i}" for i in range(7)) + "\n")
                self._csv["joints"] = fh
            self._running = True
            logger.info("recording started -> %s (%s + joints.csv)", target, ", ".join(self._specs))
            (target / "recording_started.txt").write_text(f"{time.time():.3f}\n", encoding="utf-8")
            return target

    def push(self, name: str, array: np.ndarray, wall_ns: int | None = None, mono_ns: int | None = None) -> None:
        """Called from a capture thread for every frame; drops silently if not recording."""
        proc = self._procs.get(name)
        if proc is None:
            return
        spec = self._specs.get(name)
        if spec is None:
            return
        with self._lock:
            proc = self._procs.get(name)
            if proc is None or proc.stdin is None:
                return
            n = self._counts.get(name, 0)
            self._counts[name] = n + 1
            if n % spec[4]:
                return
            try:
                proc.stdin.write(np.ascontiguousarray(array).tobytes())
            except (BrokenPipeError, OSError) as exc:
                logger.error("recorder %s pipe failed: %s; stopping that stream", name, exc)
                self._procs.pop(name, None)
                return
            idx = self._written.get(name, 0)
            self._written[name] = idx + 1
        with self._csv_lock:
            fh = self._csv.get(name)
            if fh is not None:
                fh.write(f"{idx},{wall_ns if wall_ns is not None else time.time_ns()},"
                         f"{mono_ns if mono_ns is not None else time.monotonic_ns()}\n")

    def log_joints(self, state: Any) -> None:
        """Append one arm-state row (called by JointLogger); no-op when not recording."""
        if not self._running:
            return
        pos = np.asarray(state.position, dtype=float).ravel()
        vel = np.asarray(state.velocity, dtype=float).ravel()
        eff = np.asarray(state.effort, dtype=float).ravel()
        row = (f"{time.time_ns()},{int(state.monotonic_ns)},{int(state.sequence)},"
               + ",".join(f"{x:.6f}" for x in pos) + ","
               + ",".join(f"{x:.6f}" for x in vel) + ","
               + ",".join(f"{x:.6f}" for x in eff) + "\n")
        with self._csv_lock:
            fh = self._csv.get("joints")
            if fh is not None:
                fh.write(row)

    def stop(self) -> Path | None:
        with self._lock:
            if not self._running and not self._procs:
                return None
            self._running = False
            procs, self._procs = self._procs, {}
            out = self._out_dir
        with self._csv_lock:
            for fh in self._csv.values():
                try:
                    fh.close()
                except Exception:  # noqa: BLE001
                    pass
            self._csv = {}
        for name, proc in procs.items():
            try:
                if proc.stdin:
                    proc.stdin.close()
                proc.wait(timeout=30)
            except Exception as exc:  # noqa: BLE001
                logger.error("recorder %s did not finish cleanly: %s", name, exc)
                proc.kill()
        if out is not None:
            (out / "recording_stopped.txt").write_text(f"{time.time():.3f}\n", encoding="utf-8")
            logger.info("recording stopped -> %s (frames: %s)", out, dict(self._counts))
            # Re-time streams whose capture thread ran slower than the nominal rate (the
            # D405 path achieves ~15 fps of a nominal 30): wallclock timestamps on a raw
            # pipe proved unreliable, so scale the container timestamps after the fact
            # (stream copy, no re-encode) to the real elapsed duration.
            try:
                started = float((out / "recording_started.txt").read_text().strip())
                elapsed = max(1e-3, time.time() - started)
                for name, (w, h, pixfmt, fps, every) in self._specs.items():
                    n = self._counts.get(name, 0) // max(1, every)
                    nominal = max(1, fps // every)
                    if n < 10:
                        continue
                    ratio = (elapsed * nominal) / n          # >1 when frames came slower than nominal
                    if abs(ratio - 1.0) > 0.1 and self._ffmpeg:
                        src = out / f"{name}.mp4"; tmp = out / f"{name}.retimed.mp4"
                        r = subprocess.run([self._ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
                                            "-itsscale", f"{ratio:.4f}", "-i", str(src), "-c", "copy", str(tmp)],
                                           capture_output=True, text=True)
                        if r.returncode == 0 and tmp.exists() and tmp.stat().st_size > 0:
                            tmp.replace(src)
                            logger.info("recording %s re-timed x%.3f (%d frames over %.0fs)", name, ratio, n, elapsed)
                        else:
                            logger.warning("re-timing %s failed: %s", name, r.stderr[-200:])
            except Exception:  # noqa: BLE001
                logger.exception("recording re-timing failed")
        return out


class StreamingCamera:
    """Wrap a camera adapter with a dedicated capture thread and a latest-frame slot.

    ``inner.read()`` becomes the only hardware call site (from the thread). ``read()``
    hands out the first frame captured AFTER the call, within ``timeout_s``.
    """

    def __init__(
        self,
        inner: Any,
        *,
        name: str,
        recorder: RunRecorder | None,
        frame_attr: str,
        timeout_s: float,
        stale_after_s: float,
        monotonic_ns: Any = time.monotonic_ns,
    ) -> None:
        self._inner = inner
        self._name = name
        self._recorder = recorder
        self._frame_attr = frame_attr          # "rgb" for both adapters
        self._timeout_s = float(timeout_s)
        self._stale_after_s = float(stale_after_s)
        self._monotonic_ns = monotonic_ns
        self._cv = threading.Condition()
        self._latest: Any = None
        self._latest_captured_ns = 0
        self._captures = 0
        self._error: BaseException | None = None
        self._closed = False
        self._thread = threading.Thread(target=self._loop, name=f"capture-{name}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        consecutive_errors = 0
        while not self._closed:
            try:
                frame = self._inner.read()
                consecutive_errors = 0
            except Exception as exc:  # noqa: BLE001
                consecutive_errors += 1
                if consecutive_errors >= 30:
                    with self._cv:
                        self._error = exc
                        self._cv.notify_all()
                    logger.error("capture thread %s giving up: %s", self._name, exc)
                    return
                time.sleep(0.01)
                continue
            with self._cv:
                self._latest = frame
                self._latest_captured_ns = self._monotonic_ns()
                self._captures += 1
                self._cv.notify_all()
            if self._recorder is not None and self._recorder.active:
                try:
                    self._recorder.push(self._name, getattr(frame, self._frame_attr),
                                        wall_ns=time.time_ns(), mono_ns=self._latest_captured_ns)
                except Exception as exc:  # noqa: BLE001
                    logger.error("recorder push failed for %s: %s", self._name, exc)

    def read(self) -> Any:
        if self._closed:
            raise CameraFrameError(f"{self._name} camera is closed")
        deadline = time.monotonic() + self._timeout_s
        with self._cv:
            since = self._captures
            while True:
                if self._error is not None:
                    raise CameraFrameError(f"{self._name} capture thread failed: {self._error}")
                if self._captures > since and self._latest is not None:
                    age_s = (self._monotonic_ns() - self._latest_captured_ns) / 1e9
                    if age_s <= self._stale_after_s:
                        return self._latest
                    since = self._captures            # too old (thread stalled); wait for a newer one
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise CameraFrameError(
                        f"{self._name} camera produced no fresh frame within {self._timeout_s:.2f}s"
                    )
                self._cv.wait(timeout=remaining)

    def __getattr__(self, item: str) -> Any:       # e.g. .calibration on the top camera
        return getattr(self._inner, item)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._cv:
            self._cv.notify_all()
        self._thread.join(timeout=max(2.0, self._timeout_s + 1.0))
        self._inner.close()


class JointLogger:
    """Sample the arm state at a fixed rate while a recording is active and append it to
    the recorder's joints.csv. Read-only with respect to the robot (``read_motion_state``
    is the same call the motion controller and its watchdog make); a failing read is
    logged (rate-limited) and never propagates.
    """

    def __init__(self, source: Any, recorder: RunRecorder, hz: float = 50.0) -> None:
        self._source = source
        self._recorder = recorder
        self._period = 1.0 / float(hz)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="joint-logger", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        last_err = 0.0
        while not self._stop.is_set():
            if not self._recorder.active:
                self._stop.wait(0.1)
                continue
            t0 = time.monotonic()
            try:
                self._recorder.log_joints(self._source.read_motion_state())
            except Exception as exc:  # noqa: BLE001
                if t0 - last_err > 60.0:
                    logger.warning("joint logger read failed (suppressing repeats for 60 s): %s", exc)
                    last_err = t0
            self._stop.wait(max(0.0, self._period - (time.monotonic() - t0)))

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)


_RECORDER: RunRecorder | None = None


def install_recorder(record_dir: Path | None, *, live_readable: bool = False) -> RunRecorder:
    global _RECORDER
    _RECORDER = RunRecorder(record_dir, live_readable=live_readable)
    return _RECORDER


def get_recorder() -> RunRecorder | None:
    return _RECORDER


def install_signal_handlers(recorder: RunRecorder) -> None:
    """SIGUSR1 -> start recording, SIGUSR2 -> stop. Must be called from the main thread."""
    import signal

    def _start(signum: int, frame: Any) -> None:  # noqa: ARG001
        try:
            recorder.start()
        except Exception:  # noqa: BLE001
            logger.exception("recording start failed")

    def _stop(signum: int, frame: Any) -> None:  # noqa: ARG001
        try:
            recorder.stop()
        except Exception:  # noqa: BLE001
            logger.exception("recording stop failed")

    signal.signal(signal.SIGUSR1, _start)
    signal.signal(signal.SIGUSR2, _stop)
    logger.info("recording control: SIGUSR1 start / SIGUSR2 stop (pid %d)", os.getpid())
