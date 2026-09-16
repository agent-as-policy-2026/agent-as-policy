#!/usr/bin/env bash
# 用固定的 git 配置重算 tracked_diff 的 sha256，消除环境差异。
# 用法: ./verify_i2rt.sh /path/to/your/i2rt-clone
set -euo pipefail
I="${1:?用法: $0 <i2rt clone 目录>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
want="$(cat "$HERE/expected_sha256.txt")"
got="$(git -c core.autocrlf=false -c diff.algorithm=myers -c diff.noprefix=false \
        -C "$I" diff --binary --no-renames HEAD -- i2rt pyproject.toml | sha256sum | cut -d' ' -f1)"
echo "expected $want"
echo "actual   $got"
if [ "$want" = "$got" ]; then echo "OK"; exit 0; fi
echo "不一致。这通常不是数据损坏，而是 git 版本或配置差异（diff.algorithm / renames / autocrlf / .gitattributes）。"
echo "请改用 file_digest.txt 逐文件比对内容摘要；若内容一致，用 make_config_sha.sh 在本地重新落盘一次配置即可。"
exit 1
