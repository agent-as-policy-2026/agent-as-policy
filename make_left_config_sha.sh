#!/usr/bin/env bash
# Recompute the i2rt tracked-diff fingerprint and patch it into
# hardware-bridge/config/left_arm.yaml.
#
# Thin wrapper (since 2026-09-08) around make_config_sha.sh, which does the
# same for any bridge config (left_arm.yaml, right_arm.yaml). Behaviour and
# output for the left config are unchanged; extra arguments (e.g. --check)
# are forwarded.
#
# The bridge preflight fingerprints the i2rt working tree as
#     sha256( git -C i2rt diff --binary HEAD -- i2rt pyproject.toml )
# (hardware-bridge/src/agp_yam_bridge/preflight.py:hardware_source_diff_sha256)
# and refuses to serve when the config's i2rt.tracked_diff_sha256 differs.
# Local i2rt modifications are intentional hardware facts; a changed sha means
# "re-review the diff, then bless it".

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${repo_root}/make_config_sha.sh" "$@" "${repo_root}/hardware-bridge/config/left_arm.yaml"
