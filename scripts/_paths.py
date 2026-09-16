#!/usr/bin/env python3
"""Where the export scripts read the rig's data from - one definition for all of them.

Resolution order, most specific first:

  1. --fa / --workspace on the command line
  2. $AGP_FREE_AGENT / $AGP_WORKSPACE
  3. this checkout: the repository root, and the harness tree inside it

WORKSPACE is the rig repository root (hardware-bridge/logs/ lives there); FA is
the harness tree inside it (sessions/, paper_runs/, goal_sets/), which is what
all but one of the exporters read.

The experiment data is not in git, so no default is right on every machine and
none is invented here: the last resort is this checkout, which carries a tree of
the same shape. Whatever a run resolves to is checked before the exporters use
it, and a directory that is missing, or that holds none of the data they read,
stops the script with a message naming the flag and the variable instead of
exporting from the wrong tree. The exporters driven by a session list go one
step further through require_sessions(): a tree that passes on paper_runs/
alone - a fresh checkout does - but lacks a listed session stops them too,
before any output is written.

A workspace's session tree is <workspace>/agp. The older name
<workspace>/free_agent is accepted too, so data recorded by earlier rigs can
be exported with the same command.
"""
import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# What the exporters actually read. A resolved directory holding none of these
# is how a wrong - or simply data-less - tree is caught before the export runs.
FA_MARKERS = ("sessions", "paper_runs", "goal_sets")
WORKSPACE_MARKER = "hardware-bridge"

_FA_HINT = ("pass --fa <dir>, or set $AGP_FREE_AGENT (or $AGP_WORKSPACE, whose "
            "free_agent/ or agp/ subtree is used).\nThe session data is not part "
            "of this repository, so it has to be pointed at.")
_WS_HINT = ("pass --workspace <dir>, or set $AGP_WORKSPACE.\nThe rig's logs are "
            "not part of this repository, so they have to be pointed at.")


def _fa_of(workspace):
    """<workspace>/free_agent if that older name exists, else <workspace>/agp."""
    legacy = os.path.join(workspace, "free_agent")
    return legacy if os.path.isdir(legacy) else os.path.join(workspace, "agp")


WORKSPACE = os.environ.get("AGP_WORKSPACE") or REPO_ROOT

FA = os.environ.get("AGP_FREE_AGENT") or _fa_of(WORKSPACE)


def _checked_fa(path):
    """Validate --fa, including when it comes from the default (argparse runs
    type= over a string default too, after --help has had its chance)."""
    if not os.path.isdir(path):
        raise argparse.ArgumentTypeError(f"no such session tree: {path}\n{_FA_HINT}")
    if not any(os.path.isdir(os.path.join(path, m)) for m in FA_MARKERS):
        raise argparse.ArgumentTypeError(
            f"{path} holds none of {', '.join(m + '/' for m in FA_MARKERS)}, so it "
            f"is not a session tree.\n{_FA_HINT}")
    return path


def _checked_workspace(path):
    if not os.path.isdir(path):
        raise argparse.ArgumentTypeError(f"no such workspace: {path}\n{_WS_HINT}")
    if not os.path.isdir(os.path.join(path, WORKSPACE_MARKER)):
        raise argparse.ArgumentTypeError(
            f"{path} has no {WORKSPACE_MARKER}/, so it is not a rig workspace.\n{_WS_HINT}")
    return path


def add_fa_arg(ap):
    """--fa: the live harness tree the exporters read sessions from."""
    ap.add_argument("--fa", default=FA, type=_checked_fa,
                    help="live session tree to read from ($AGP_FREE_AGENT; default %(default)s)")


def add_workspace_arg(ap):
    """--workspace: the rig repository root above the harness tree."""
    ap.add_argument("--workspace", default=WORKSPACE, type=_checked_workspace,
                    help="rig repository root ($AGP_WORKSPACE; default %(default)s)")


def require_sessions(fa, session_ids, what="--snapshot"):
    """Stop with exit 2 unless every listed session has a directory under <fa>/sessions/.

    _checked_fa accepts a tree on paper_runs/ alone, and a fresh checkout is such
    a tree: an exporter that merely skipped what it could not find would then
    write an empty release and exit 0. Call this once the list and the tree are
    known and before any output exists. It reads nothing and returns nothing, so
    a run whose sessions are all present is byte-for-byte what it was without it.
    """
    ids = list(session_ids)
    root = os.path.join(fa, "sessions")
    missing = list(dict.fromkeys(s for s in ids if not os.path.isdir(os.path.join(root, s))))
    if not missing:
        return
    shown = ", ".join(missing[:3]) + (", ..." if len(missing) > 3 else "")
    sys.stderr.write(
        f"error: {len(missing)} of {len(ids)} sessions listed in {what} have no directory "
        f"under {root}: {shown}\n"
        f"resolved session tree: {fa}\n"
        f"pass --fa <dir> or set $AGP_FREE_AGENT / $AGP_WORKSPACE\n")
    sys.exit(2)
