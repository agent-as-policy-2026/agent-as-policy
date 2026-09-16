#!/usr/bin/env bash
# Recompute the i2rt tracked-diff fingerprint and patch it into ONE bridge
# config (hardware-bridge/config/<name>.yaml).
#
#   ./make_config_sha.sh hardware-bridge/config/left_arm.yaml
#   ./make_config_sha.sh right_arm.yaml            # bare name -> hardware-bridge/config/
#   ./make_config_sha.sh --check right_arm.yaml    # report only, exit 3 on mismatch
#
# The bridge preflight fingerprints the i2rt working tree as
#     sha256( git -C i2rt diff --binary HEAD -- i2rt pyproject.toml )
# (hardware-bridge/src/agp_yam_bridge/preflight.py:hardware_source_diff_sha256)
# and refuses to serve when the config's i2rt.tracked_diff_sha256 differs.
#
# Every runtime config (left_arm.yaml, right_arm.yaml) pins the SAME i2rt
# working tree, so after any reviewed i2rt change run this once per config.
# Local i2rt modifications are intentional hardware facts; a changed sha means
# "re-review the diff, then bless it". make_left_config_sha.sh is a thin
# wrapper around this script for the left config (unchanged behaviour).

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
i2rt_dir="${repo_root}/i2rt"
config_dir="${repo_root}/hardware-bridge/config"

usage() {
    echo "usage: $(basename "$0") [--check] <config.yaml | bare-name-in-hardware-bridge/config>" >&2
    exit 2
}

check_only=0
config_arg=""
while [ $# -gt 0 ]; do
    case "$1" in
        --check) check_only=1 ;;
        -h|--help) usage ;;
        -*) echo "ERROR: unknown option $1" >&2; usage ;;
        *) [ -z "${config_arg}" ] || usage; config_arg="$1" ;;
    esac
    shift
done
[ -n "${config_arg}" ] || usage

# Resolve: as given (absolute or cwd-relative); else a bare name inside
# hardware-bridge/config (with or without the .yaml suffix).
if [ -f "${config_arg}" ]; then
    config="${config_arg}"
elif [ -f "${config_dir}/${config_arg}" ]; then
    config="${config_dir}/${config_arg}"
elif [ -f "${config_dir}/${config_arg}.yaml" ]; then
    config="${config_dir}/${config_arg}.yaml"
else
    echo "ERROR: config ${config_arg} not found (tried as given and in ${config_dir})" >&2
    exit 1
fi
config="$(cd "$(dirname "${config}")" && pwd)/$(basename "${config}")"

[ -d "${i2rt_dir}/.git" ] || [ -f "${i2rt_dir}/.git" ] || {
    echo "ERROR: ${i2rt_dir} is not a git checkout" >&2; exit 1;
}

# Exactly the preflight's fingerprint: binary diff of tracked runtime source
# and dependency metadata, docs/examples excluded by pathspec.
sha="$(git -C "${i2rt_dir}" diff --binary HEAD -- i2rt pyproject.toml | sha256sum | cut -d' ' -f1)"

old="$(sed -n 's/^[[:space:]]*tracked_diff_sha256:[[:space:]]*"\([0-9a-f]\{64\}\)".*$/\1/p' "${config}" | head -n1)"
[ -n "${old}" ] || {
    echo "ERROR: no tracked_diff_sha256 line found in ${config}" >&2; exit 1;
}

if [ "${old}" = "${sha}" ]; then
    echo "tracked_diff_sha256 already current: ${sha}"
    exit 0
fi

if [ "${check_only}" = 1 ]; then
    echo "tracked_diff_sha256 STALE in ${config}"
    echo "  config:   ${old}"
    echo "  i2rt now: ${sha}"
    echo "run without --check to patch it (after reviewing 'git -C ${i2rt_dir} diff')."
    exit 3
fi

sed -i "s/^\([[:space:]]*tracked_diff_sha256:[[:space:]]*\)\"[0-9a-f]\{64\}\"/\1\"${sha}\"/" "${config}"
echo "tracked_diff_sha256: ${old} -> ${sha}"
echo "patched ${config}"
git -C "${i2rt_dir}" rev-parse HEAD | sed 's/^/i2rt HEAD: /'
echo "NOTE: review 'git -C ${i2rt_dir} diff' before serving — the sha blesses it."
