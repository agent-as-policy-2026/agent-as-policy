#!/usr/bin/env bash
# Recompute the per-file sha256 digests of this vendored directory and compare
# them with file_digest.txt / file_digest_aggregate.txt.
#
#   ./verify_gap.sh            # check (exit 1 on any mismatch)
#   ./verify_gap.sh --write    # regenerate the two digest files
#
# What a green run proves: the bytes in this directory are the bytes that were
# reviewed and published, and nothing was edited afterwards. It is content-only
# and does not depend on git, unlike third_party/i2rt's patch fingerprint.
#
# What it does NOT prove: that this tree can be re-derived from upstream. Two of
# the vendored modules do not exist at the pinned commit, and upstream's
# gap/connector/real.py is missing 1123 lines of the file here — a clean clone
# of graph-as-policy cannot run this harness. See UPSTREAM.md §8/§9.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIGEST="$HERE/file_digest.txt"
AGG="$HERE/file_digest_aggregate.txt"

# Every file we ship, minus the digest files themselves and anything Python or
# a build backend may have dropped in (__pycache__, *.egg-info, *.pyc).
list_files() {
    find "$HERE" \
        \( -name '__pycache__' -o -name '*.egg-info' -o -name '.venv' \) -prune -o \
        -type f \
        ! -name 'file_digest.txt' \
        ! -name 'file_digest_aggregate.txt' \
        ! -name '*.pyc' \
        -printf '%P\n' | LC_ALL=C sort
}

compute() {
    local f
    while IFS= read -r f; do
        printf '%s  %s\n' "$(sha256sum "$HERE/$f" | cut -d' ' -f1)" "$f"
    done < <(list_files)
}

if [ "${1:-}" = "--write" ]; then
    compute > "$DIGEST"
    sha256sum "$DIGEST" | cut -d' ' -f1 > "$AGG"
    echo "wrote $DIGEST ($(wc -l < "$DIGEST") files)"
    echo "wrote $AGG ($(cat "$AGG"))"
    exit 0
fi

[ -f "$DIGEST" ] || { echo "ERROR: $DIGEST missing" >&2; exit 1; }

rc=0
if ! diff -u "$DIGEST" <(compute); then
    echo "MISMATCH: the files above differ from file_digest.txt" >&2
    rc=1
fi

if [ -f "$AGG" ]; then
    want="$(cat "$AGG")"
    got="$(sha256sum "$DIGEST" | cut -d' ' -f1)"
    echo "aggregate expected $want"
    echo "aggregate actual   $got"
    [ "$want" = "$got" ] || { echo "MISMATCH: file_digest.txt itself was edited" >&2; rc=1; }
fi

if [ "$rc" = 0 ]; then
    echo "OK — $(wc -l < "$DIGEST") files match"
else
    echo "A mismatch here means the vendored bytes were changed after publication."
    echo "Review the change, then re-run './verify_gap.sh --write' to bless it."
fi
exit "$rc"
