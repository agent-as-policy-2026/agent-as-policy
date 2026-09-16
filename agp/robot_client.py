#!/usr/bin/env python3
"""CLI client for the AgP robot server (file bridge).

Usage:
    python3 robot_client.py <session_dir> <command> ['<json-args>']

Examples:
    python3 robot_client.py . status
    python3 robot_client.py . frames '{"cams": ["wrist", "top"]}'
    python3 robot_client.py . deproject '{"capture": 1, "cam": "top", "u": 960, "v": 540, "plane_z": -0.045}'
    python3 robot_client.py . move_delta '{"dpos": [0, 0, -0.03]}'
    python3 robot_client.py . gripper '{"action": "close"}'
    python3 robot_client.py . help
    python3 robot_client.py . --arm right state      # two-arm sessions: --arm left|right right after <session_dir> (default left)

The robot server runs in a separate process and watches <session_dir>/bridge/.
This client writes bridge/req_<n>.json, waits for bridge/resp_<n>.json and
prints it to stdout (single JSON object). Commands are executed strictly one
at a time, in order. Uses only the Python standard library.
"""
import json
import os
import sys
import time

TIMEOUT_S = 240.0


def main() -> int:
    if len(sys.argv) < 3:
        print(json.dumps({"ok": False, "error": "usage: robot_client.py <session_dir> <command> ['<json-args>']"}))
        return 2
    argv = list(sys.argv[1:])
    # optional '--arm left|right' (or '--arm=right') immediately after <session_dir>; default left = today's bridge/
    arm = "left"
    if len(argv) >= 2 and argv[1] == "--arm":
        if len(argv) < 3:
            print(json.dumps({"ok": False, "error": "--arm needs a value: left or right"}))
            return 2
        arm = argv[2]
        del argv[1:3]
    elif len(argv) >= 2 and argv[1].startswith("--arm="):
        arm = argv[1][len("--arm="):]
        del argv[1]
    if arm not in ("left", "right"):
        print(json.dumps({"ok": False, "error": f"unknown arm {arm!r}: use --arm left or --arm right"}))
        return 2
    if len(argv) < 2:
        print(json.dumps({"ok": False, "error": "usage: robot_client.py <session_dir> [--arm left|right] <command> ['<json-args>']"}))
        return 2
    session = os.path.abspath(argv[0])
    cmd = argv[1]
    try:
        args = json.loads(argv[2]) if len(argv) > 2 else {}
    except json.JSONDecodeError as e:
        print(json.dumps({"ok": False, "error": f"args is not valid JSON: {e}"}))
        return 2
    bridge = os.path.join(session, "bridge" if arm == "left" else "bridge_" + arm)
    server_log = "server.log" if arm == "left" else f"server_{arm}.log"
    if not os.path.isdir(bridge):
        print(json.dumps({"ok": False, "error": f"no bridge dir at {bridge} (is the robot server running?)"}))
        return 2
    # next sequence number
    seqs = [0]
    for f in os.listdir(bridge):
        if f.startswith("req_") and f.endswith(".json"):
            try:
                seqs.append(int(f[4:-5]))
            except ValueError:
                pass
    n = max(seqs) + 1
    req = os.path.join(bridge, f"req_{n:06d}.json")
    resp = os.path.join(bridge, f"resp_{n:06d}.json")
    tmp = req + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"id": n, "cmd": cmd, "args": args, "t": time.time()}, f)
    os.replace(tmp, req)
    timeout_s = TIMEOUT_S
    if cmd == "run_program":   # a buffered program may legitimately outlast the fixed client timeout
        try:
            duration = float(args["program"]["times_s"][-1])
            timeout_s = max(TIMEOUT_S, max(10.0, 2.0 * duration + 5.0) + 30.0)
        except Exception:  # noqa: BLE001  (a malformed program is refused by the server)
            pass
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if os.path.exists(resp):
            time.sleep(0.05)  # let the writer finish the rename settle
            with open(resp) as f:
                print(f.read())
            return 0
        time.sleep(0.2)
    print(json.dumps({"ok": False, "error": f"timeout after {timeout_s}s waiting for {resp}; the server may be busy or down (check server liveness with: tail {server_log})"}))
    return 1


if __name__ == "__main__":
    sys.exit(main())
