#!/usr/bin/env python3
"""Clear a latched 'command heartbeat timed out' bridge fault (2026-09-11).

When a session is interrupted (Ctrl-C) while a motion command is in flight, the server's
release (cancel) is lost, the bridge watchdog times the hold out, puts the arm into
gravity-comp idle and LATCHES safety_state=fault. Every client then refuses to connect
("YAM bridge was not ready within 30.0s") because the connector waits for state=ok, and only
a new motion action clears the latch.

This tool connects WITHOUT the readiness wait, and (only with --yes) sends one zero-length
absolute-joint move to the arm's CURRENT joints. That clears the latch and leaves the arm held
where it already is. The tool then releases the hold with the normal cancel, which puts the arm
back into gravity-comp idle, and re-reads the health. It refuses any other fault, a moving arm
or an active client.

  clear_bridge_fault.py --port 9021 [--robot yam_left|yam_right] [--yes]
"""
import argparse
import sys
import time

import numpy as np

FAULT = "command heartbeat timed out"


def make_env(robot, host, port):
    from gap.envs.registry import resolve   # vendored connector; no wait_ready here
    factory, key = resolve("yam_real")
    kw = {"host": host, "port": port}
    if robot == "yam_right":
        kw["robot_spec"] = "yam_real_right"
    env, _config = factory(key, 0, camera_names=None, **kw)
    return env


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--robot", choices=("yam_left", "yam_right"), default="yam_left")
    ap.add_argument("--yes", action="store_true", help="actually send the zero-length move (default: report only)")
    a = ap.parse_args()
    np.set_printoptions(precision=4, suppress=True)

    env = make_env(a.robot, a.host, a.port)
    try:
        o0 = env.get_observation()
        time.sleep(0.5)   # the velocity field is noisy in gravity-comp idle: judge rest by position drift
        o = env.get_observation()
        h = dict(o["bridge_health"])
        q = np.asarray(o["robot_joint_pos_0"], dtype=np.float64)
        drift = np.abs(q[:6] - np.asarray(o0["robot_joint_pos_0"], dtype=np.float64)[:6]).max()
        print(f"bridge {a.host}:{a.port}: state={h.get('state')} safety={h.get('safety_state')} "
              f"detail={h.get('detail')!r}")
        print(f"  joints {q[:6]}  gripper {q[6]:.3f}  drift over 0.5 s {drift:.4f} rad")
        if h.get("state") == "ok":
            print("healthy: nothing to clear")
            return 0
        if h.get("safety_state") != "fault" or h.get("detail") != FAULT:
            print(f"REFUSED: only a latched {FAULT!r} fault is cleared here; restart the bridge for anything else")
            return 3
        if drift > 0.01:
            print("REFUSED: the arm is moving (someone handling it?); wait until it rests")
            return 4
        if not a.yes:
            print("report only. Re-run with --yes to hold the arm at these joints for a moment and release it "
                  "(clears the latch; e-stop in reach)")
            return 5
        r = env.move_to_joints_blocking(q[:6].astype(np.float32), timeout_s=10.0, tolerance=0.02)
        o2 = env.get_observation()
        dq = np.abs(np.asarray(o2["robot_joint_pos_0"], dtype=np.float64)[:6] - q[:6]).max()
        print(f"zero-length move: status={r.get('status')} detail={r.get('detail')!r} max |dq| {dq:.4f} rad; "
              f"health now {o2['bridge_health'].get('state')}/{o2['bridge_health'].get('safety_state')}")
    finally:
        env.close()   # cancel_trajectory on the held lease -> gravity-comp idle, no fault
    time.sleep(0.5)
    env = make_env(a.robot, a.host, a.port)
    try:
        h = dict(env.get_observation()["bridge_health"])
    finally:
        env.close()
    print(f"after release: state={h.get('state')} safety={h.get('safety_state')} detail={h.get('detail')!r}")
    return 0 if h.get("state") == "ok" else 6


if __name__ == "__main__":
    sys.exit(main())
