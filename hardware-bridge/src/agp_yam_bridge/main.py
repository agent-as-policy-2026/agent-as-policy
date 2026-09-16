"""Command-line entry point for the P4 safety-owned YAM bridge."""

from __future__ import annotations

import argparse
import logging
import os
import time
from pathlib import Path

from agp_yam_bridge.config import BridgeConfig, load_config
from agp_yam_bridge.motion import MotionController
from agp_yam_bridge.preflight import DEFAULT_CONFIG, run_preflight
from agp_yam_bridge.server import YamBridgeServer
from agp_yam_bridge.source import (
    FakeYamSource,
    I2rtYamSource,
    StartupHoldError,
    YamSource,
)


def build_source(
    kind: str,
    config: BridgeConfig,
    *,
    acknowledge_i2rt_startup_motion: bool,
) -> YamSource:
    if kind == "fake":
        return FakeYamSource(config)
    if kind != "i2rt":
        raise ValueError(f"unknown observation source {kind!r}")
    if not acknowledge_i2rt_startup_motion:
        raise RuntimeError(
            "i2rt startup may enable motors and calibrate/move linear_4310; "
            "pass --acknowledge-i2rt-startup-motion only after following the hardware checklist"
        )
    return I2rtYamSource(config)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="P4 safety-owned AgP/YAM bridge")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--source", choices=("fake", "i2rt"), required=True)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--acknowledge-i2rt-startup-motion", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--acknowledge-first-motion-checklist", action="store_true")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument(
        "--record-dir",
        type=Path,
        default=None,
        help="enable run recording of the wrist + top cameras (SIGUSR1 start / SIGUSR2 stop); "
             "output <dir>/target.txt's path if present, else <dir>/<timestamp>/",
    )
    parser.add_argument(
        "--record-live-readable",
        action="store_true",
        help="2026-09-14: write the run recording as fragmented MP4 (keyframe every ~1 s) so the files "
             "can be decoded while the recording runs (throwing sessions: the agent aligns camera "
             "frames with a program's execution trace); default = plain MP4, readable only after stop",
    )
    args = parser.parse_args(argv)
    if (
        args.source == "i2rt"
        and args.enable_motion
        and not args.acknowledge_first_motion_checklist
    ):
        raise RuntimeError(
            "real motion requires the completed first-motion checklist and "
            "--acknowledge-first-motion-checklist"
        )
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    config = load_config(args.config)
    from agp_yam_bridge.record import install_recorder, install_signal_handlers

    recorder = install_recorder(args.record_dir, live_readable=args.record_live_readable)
    if args.record_dir is not None:
        args.record_dir.mkdir(parents=True, exist_ok=True)
    # ALWAYS install the SIGUSR1/SIGUSR2 handlers: without them the default action of
    # either signal is to TERMINATE the process, so a recording request sent to a bridge
    # started without --record-dir would kill the bridge mid-session.
    install_signal_handlers(recorder)
    if args.source == "i2rt":
        print(run_preflight(config), flush=True)
    server = YamBridgeServer(
        None,
        host=args.host or config.bridge.host,
        port=config.bridge.port if args.port is None else args.port,
    )
    try:
        source = build_source(
            args.source,
            config,
            acknowledge_i2rt_startup_motion=args.acknowledge_i2rt_startup_motion,
        )
    except StartupHoldError as exc:
        server.server_close()
        try:
            try:
                print(
                    "status: STARTUP_HOLD\n"
                    f"hold_confirmed: {str(exc.hold_active).lower()}\n"
                    "The motor chain remains owned by this process. Support the arm, then press Ctrl-C "
                    "to release it and exit.",
                    flush=True,
                )
            except OSError:
                logging.exception("Could not print YAM startup-hold instructions")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                return 2
        finally:
            exc.release()
    except BaseException:
        server.server_close()
        raise
    attached = False
    joint_logger = None
    try:
        server.attach_source(source)
        attached = True
        programs = ""
        if args.enable_motion:
            limits = config.acceptance.speed_limits
            if limits.programs_enabled:
                # 2026-09-14: a config that names program J4 limits (config/left_arm_throw.yaml)
                # gets the buffered-program controller; every other config keeps the plain one.
                from agp_yam_bridge.program_controller import ProgramController

                controller: MotionController = ProgramController(
                    source,
                    source.kinematics,
                    config,
                    log_dir=config.safety.action_log_path.parent / "programs",
                )
                programs = (
                    f"\njoint programs: enabled (J4 up to {limits.program_j4_velocity_deg_s:g} deg/s, "
                    f"{limits.program_j4_acceleration_deg_s2:g} deg/s^2; other joints and ordinary "
                    f"moves {limits.joint_velocity_deg_s:g}/{limits.joint_acceleration_deg_s2:g})"
                )
            else:
                controller = MotionController(source, source.kinematics, config)
            server.attach_motion_controller(controller)
        if recorder.enabled:
            # 50 Hz arm-state log (joints.csv) alongside the videos while a recording is
            # active — read-only sampling of the same feedback the controller uses.
            from agp_yam_bridge.record import JointLogger

            joint_logger = JointLogger(source, recorder, hz=50.0)
        host, port = server.server_address
        mode = "motion-enabled" if args.enable_motion else "read-only"
        live = ", live-readable fragmented MP4" if args.record_live_readable else ""
        rec = f"\nrecording: {'armed (SIGUSR1/SIGUSR2, pid ' + str(os.getpid()) + ', videos + joints.csv @50 Hz' + live + ')' if args.record_dir else 'off'}"
        print(f"status: SERVING\nendpoint: {host}:{port}\nmode: {mode}{rec}{programs}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    finally:
        if joint_logger is not None:
            try:
                joint_logger.close()
            except Exception:  # noqa: BLE001
                logging.exception("joint logger stop failed at shutdown")
        try:
            recorder.stop()
        except Exception:  # noqa: BLE001
            logging.exception("recorder stop failed at shutdown")
        if not attached:
            source.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
