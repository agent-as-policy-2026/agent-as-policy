#!/usr/bin/env python3
"""Release checks for the trials table (one row per session) and the files beside it.

Local part (no network): row identity against the release list, the Viewer's page-size
bound, the traces column against the flat command/capture records, every URL against the
files in the repository, images decodable, and the scrubbing patterns over every string
leaf of the table and over the raw traces.
Remote part: datasets-server sees exactly the declared (config, split) pairs and serves them.

38 check() sites: 33 local and 5 remote. A full local run prints 35 results -- the URL
check is a single site called once per URL family -- and fewer when a flag is absent:
--exclude enables 1 of them, --flat 9, and a crops/*.csv in the tree 5. The remote half
prints 3 results plus 2 per (config, split). The token comes from $HF_TOKEN, else from
--token-file (env HF_TOKEN_FILE), else the requests are anonymous; --local-only reads no
token and makes no request.
"""
import argparse, csv, glob, io, json, os, re, sys, urllib.request

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def api(url, token):
    req = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}"} if token else {})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.load(r)


def finish():
    bad = [n for n, ok in RESULTS if not ok]
    print("\n  " + ("全部通过" if not bad else f"{len(bad)} 项未通过: {bad}"))
    sys.exit(1 if bad else 0)


def string_leaves(table):
    """Yield (label, pa.Array of strings) for every string leaf, flattening lists and structs."""
    import pyarrow as pa, pyarrow.compute as pc
    def walk(arr, label):
        t = arr.type
        if pa.types.is_string(t) or pa.types.is_large_string(t):
            yield label, arr
        elif pa.types.is_list(t) or pa.types.is_large_list(t):
            yield from walk(pc.list_flatten(arr), label + "[]")
        elif pa.types.is_struct(t):
            if isinstance(arr, pa.ChunkedArray):
                arr = arr.combine_chunks()
            for i, f in enumerate(t):
                if f.name == "bytes":
                    continue
                yield from walk(arr.field(i), label + "." + f.name)
    for name in table.column_names:
        yield from walk(table[name], name)


def rel_of(u):
    return u.split("/resolve/main/", 1)[1] if "/resolve/main/" in u else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", help="e.g. Agent-as-Policy/agent-as-policy (required unless --local-only)")
    ap.add_argument("--local", required=True, help="dataset root: README.md, trials/, videos/, ...")
    ap.add_argument("--flat", action="append", help="flat intermediates root (repeatable: one per batch)")
    ap.add_argument("--snapshot", help="released session list; defaults to <local>/../RELEASE_<batch>.txt")
    ap.add_argument("--exclude", help="EXCLUDE_<batch>.txt: sessions that must appear nowhere in the release")
    ap.add_argument("--batch", default="b01")
    ap.add_argument("--token-file", default=os.environ.get("HF_TOKEN_FILE"),
                    help="file holding the Hub token (env HF_TOKEN_FILE); $HF_TOKEN, when set, is used "
                         "instead of any file. Neither = anonymous, which is enough for a public dataset")
    ap.add_argument("--local-only", action="store_true", help="run only the local invariants: no token read, no request to the Hub")
    a = ap.parse_args()
    if not a.local_only and not a.repo:
        ap.error("--repo is required unless --local-only")
    import pyarrow as pa, pyarrow.compute as pc, pyarrow.parquet as pq

    # ---- the table --------------------------------------------------------
    files = {sp: sorted(glob.glob(os.path.join(a.local, "trials", f"trials_{sp}-*.parquet")) or glob.glob(os.path.join(a.local, sp, f"trials_{sp}-*.parquet")))
             for sp in ("evaluated", "reset", "auxiliary")}
    files = {sp: fs for sp, fs in files.items() if fs}
    tables = {sp: pa.concat_tables([pq.read_table(p) for p in fs]) for sp, fs in files.items()}
    n = {sp: t.num_rows for sp, t in tables.items()}
    sids = [s for t in tables.values() for s in t["session_id"].to_pylist()]
    snap = a.snapshot or os.path.join(os.path.dirname(a.local.rstrip("/")), f"RELEASE_{a.batch}.txt")
    sessions = [l.strip() for l in open(snap) if l.strip()] if os.path.exists(snap) else None
    check("行数恒等式 各 split 之和 == 发布会话数", sessions is not None and sum(n.values()) == len(sessions) and set(sids) == set(sessions),
          f"{'+'.join(f'{k} {v}' for k, v in n.items())} = {sum(n.values())} vs {len(sessions) if sessions else '无发布清单'}")
    check("session_id 全表唯一", len(sids) == len(set(sids)), f"{len(sids)} 行")
    if a.exclude:
        excl = set(l.split("\t")[0].strip() for l in open(a.exclude) if l.strip() and not l.startswith("#"))
        where = []
        if excl & set(sids): where.append("table")
        if any(os.path.isdir(os.path.join(a.local, sub, s)) for s in excl for sub in ("videos", "videos_side")): where.append("videos")
        if any(os.path.exists(os.path.join(a.local, "traces", f"{s}.jsonl")) for s in excl): where.append("traces")
        # depth is easy to leak: it is a per-session directory that a whole-tree symlink would pull in
        if any(os.path.isdir(os.path.join(a.local, "depth", s)) for s in excl): where.append("depth")
        dep_dirs = {os.path.basename(q) for q in glob.glob(os.path.join(a.local, "depth", "*")) if os.path.isdir(q)}
        if dep_dirs - set(sids): where.append(f"depth+{len(dep_dirs - set(sids))}个非发布会话")
        check("被排除的会话在表、视频、traces、depth 里都不存在", not where,
              f"{len(excl)} 个排除会话" + (f"; 残留于 {where}" if where else ""))
    rg = [pq.read_metadata(p).num_row_groups == pq.read_metadata(p).num_rows for fs in files.values() for p in fs]
    check("每个行组恰好一行（Viewer 与按行读取的前提）", all(rg), f"{sum(rg)}/{len(rg)} 张分片")
    feats = json.loads(pq.read_schema(files["evaluated"][0]).metadata[b"huggingface"])["info"]["features"]
    kinds = {k: v.get("_type") for k, v in feats.items() if isinstance(v, dict) and v.get("_type") in ("Image", "Video")}
    check("特征元数据里 Image/Video 类型齐全", set(kinds) == {"evidence_before", "evidence_after", "video_preview"}, str(kinds))
    check("列数与列序", list(feats) == ["session_id", "batch", "trial", "task", "evidence_before", "evidence_after",
                                        "video_preview", "arms", "model", "backend", "effort",
                                        "outcome", "labeling", "timing", "bridge", "tokens", "reasoning_summary",
                                        "traces", "prompt_text", "interface_text", "reference", "videos"],
          f"{len(feats)} 列")
    # the Viewer reads 100-row pages under a 300 MB uncompressed scan limit
    worst = 0
    for sp, fs in files.items():
        if not fs[0].startswith(os.path.join(a.local, "trials")):
            continue
        sizes = [pq.read_metadata(p).row_group(i).total_byte_size for p in fs for i in range(pq.read_metadata(p).num_row_groups)]
        for i in range(0, len(sizes), 100):
            worst = max(worst, sum(sizes[i:i + 100]))
    check("任意 100 行的解压字节 < 300 MB（Viewer 分页上限）", worst < 300_000_000, f"最大 {worst / 2**20:.0f} MiB")

    # ---- traces against the flat records -----------------------------------
    steps = [(sid, e) for t in tables.values() for sid, row in zip(t["session_id"].to_pylist(), t["traces"].to_pylist()) for e in row]
    check("每个会话都有 traces 步骤", all(len(row) for t in tables.values() for row in t["traces"].to_pylist()),
          f"{len(steps)} 步, {sum(1 for t in tables.values() for row in t['traces'].to_pylist() if not row)} 个空会话")
    kinds_seen = {}
    for _, e in steps:
        kinds_seen[e["kind"]] = kinds_seen.get(e["kind"], 0) + 1
    check("traces.kind 只用已知取值", set(kinds_seen) <= {"reasoning", "message", "command", "file_change", "error"}, str(kinds_seen))
    check("step 在每个会话内从 0 连续递增",
          all([e["step"] for e in row] == list(range(len(row))) for t in tables.values() for row in t["traces"].to_pylist()), "")
    if a.flat:
        def flat_tables(stem, cols):
            ps = [q for root in a.flat for q in sorted(glob.glob(os.path.join(root, "metadata", f"{stem}-*.parquet")))]
            assert ps, f"没有找到任何 {stem}-*.parquet（--flat 指向了哪里？）"
            return pa.concat_tables([pq.read_table(q, columns=cols) for q in ps])
        cmd = flat_tables("commands", ["session_id", "cmd"])
        cap = flat_tables("captures", ["session_id", "capture_index"])
        rel = pa.array(sorted(set(sids)))
        n_cmd = int(pc.sum(pc.is_in(cmd["session_id"], value_set=rel)).as_py() or 0)
        n_frames = int(pc.sum(pc.and_(pc.is_in(cmd["session_id"], value_set=rel), pc.equal(cmd["cmd"], "frames"))).as_py() or 0)
        n_cap = int(pc.sum(pc.is_in(cap["session_id"], value_set=rel)).as_py() or 0)
        calls = [(sid, c) for sid, e in steps for c in e["calls"]]
        tbl_ids, tbl_caps, tbl_cmd = {}, {}, {}
        for r in flat_tables("commands", ["session_id", "cmd_id", "cmd"]).to_pylist():
            if r["session_id"] in set(sids):
                tbl_ids.setdefault(r["session_id"], {})[int(r["cmd_id"])] = r["cmd"]
                tbl_cmd[r["cmd"]] = tbl_cmd.get(r["cmd"], 0) + 1
        for r in cap.to_pylist():
            if r["session_id"] in set(sids):
                tbl_caps.setdefault(r["session_id"], set()).add(r["capture_index"])
        bad_id = [(sid, c["cmd_id"]) for sid, c in calls if c["cmd_id"] is not None and c["cmd_id"] not in tbl_ids.get(sid, {})]
        check("每个 cmd_id 都是该会话桥接记录里的 id", not bad_id, f"越界 {len(bad_id)}")
        seen = {}
        for sid, c in calls:
            if c["cmd_id"] is not None:
                seen[(sid, c["cmd_id"])] = seen.get((sid, c["cmd_id"]), 0) + 1
        check("同一条桥接请求不被两个调用认领", all(v == 1 for v in seen.values()), f"重复 {sum(1 for v in seen.values() if v > 1)}")
        cov = len(seen) / n_cmd if n_cmd else 0
        check("桥接请求覆盖率 >= 97%（其余是回复未进 trace 的孤儿）", cov >= 0.97, f"{len(seen)}/{n_cmd} = {cov:.1%}")
        wrong = [(sid, c["cmd_id"]) for sid, c in calls
                 if c["cmd_id"] is not None and c["cmd"] and c["cmd"] != tbl_ids.get(sid, {}).get(c["cmd_id"])]
        check("每个调用的命令名 == 该 cmd_id 在平表里的命令名", not wrong, f"不符 {len(wrong)}")
        got_caps = {}
        for sid, c in calls:
            if c["capture_index"] is not None:
                got_caps.setdefault(sid, set()).add(c["capture_index"])
        diff = [sid for sid in set(sids) if got_caps.get(sid, set()) != tbl_caps.get(sid, set())]
        check("每个会话的抓拍集合 == 平表的抓拍集合", not diff, f"{sum(len(v) for v in got_caps.values())} vs {n_cap}; 不符会话 {len(diff)} {diff[:2]}")
        dupc = {}
        for sid, c in calls:
            if c["capture_index"] is not None:
                dupc[(sid, c["capture_index"])] = dupc.get((sid, c["capture_index"]), 0) + 1
        check("同一抓拍不被两个调用认领", all(v == 1 for v in dupc.values()), f"重复 {sum(1 for v in dupc.values() if v > 1)}")
        depth_ok = all(bool(c["depth_url"]) == (c["capture_index"] is not None and os.path.exists(os.path.join(a.local, "depth", sid, f"{c['capture_index']:04d}_wrist_depth.png"))) for sid, c in calls)
        check("depth_url 当且仅当该抓拍有深度图时非空", depth_ok, "")
        pose_ok = all(bool(c["pose_json"]) == (c["capture_index"] is not None) for _, c in calls)
        check("pose_json 当且仅当该步骤链接了抓拍时非空", pose_ok, "")
        tokens_null = [r for t in tables.values() for r in zip(t["session_id"].to_pylist(), t["tokens"].to_pylist(), t["timing"].to_pylist()) if r[1]["input"] is None]
        unexplained = [s for s, _, tm in tokens_null if not (tm["timed_out"] or tm["aborted"] or tm["duration_s"] is None)]
        check("tokens 只在超时/中止/未完成的会话上为空", not unexplained, f"空 {len(tokens_null)}，无法解释 {unexplained}")

    # ---- every URL points at a file that ships ------------------------------
    def urls(getter, label):
        us = [u for t in tables.values() for u in getter(t) if u]
        rels = [rel_of(u) for u in us]
        miss = [r for r in rels if r is None or not os.path.exists(os.path.join(a.local, r))]
        check(f"{label} 指向仓内存在的文件", not miss, f"{len(us)} 条 URL" + (f", 缺 {len(miss)} 例 {miss[:2]}" if miss else ""))
        return us
    urls(lambda t: [v["url"] for row in t["videos"].to_pylist() for v in row], "videos[].url")
    urls(lambda t: [f["url"] for g in t["reference"].to_pylist() for f in g["files"]], "reference.files[].url")
    urls(lambda t: [c["depth_url"] for row in t["traces"].to_pylist() for e in row for c in e["calls"] if c["depth_url"]], "traces[].calls[].depth_url")
    vrows = [v for t in tables.values() for row in t["videos"].to_pylist() for v in row]
    on_disk = set(os.path.relpath(p, a.local) for p in glob.glob(os.path.join(a.local, "videos", "*", "*.mp4")) + glob.glob(os.path.join(a.local, "videos_side", "*", "*.mp4")))
    listed = set(rel_of(v["url"]) for v in vrows)
    on_manifest = set()
    for sub in ("videos", "videos_side"):
        m = os.path.join(a.local, sub, "metadata.csv")
        if os.path.exists(m):
            on_manifest |= {f"{sub}/{r['file_name']}" for r in csv.DictReader(open(m, newline=""))}
    check("videos/ 与 videos_side/ 的清单 == 磁盘上的 mp4 集合", on_manifest == on_disk,
          f"清单 {len(on_manifest)}, 磁盘 {len(on_disk)}, 差 {len(on_manifest ^ on_disk)}")
    check("表里的 videos[] 都在清单中", listed <= on_manifest, f"表 {len(listed)} 条, 清单外 {len(listed - on_manifest)}")
    bad_bytes = [v for v in vrows if rel_of(v["url"]) in on_disk and os.path.getsize(os.path.join(a.local, rel_of(v["url"]))) != v["bytes"]]
    check("videos[].bytes == 文件大小", not bad_bytes, f"不符 {len(bad_bytes)}")
    stray = [p for p in glob.glob(os.path.join(a.local, "videos*", "**", "*"), recursive=True)
             if os.path.isfile(p) and not (p.endswith(".mp4") or os.path.basename(p) == "metadata.csv")]
    check("videos/ 与 videos_side/ 下只有 mp4 与根部 metadata.csv", not stray, f"多余 {len(stray)}")
    with_v = {s.split("/")[1] for s in on_disk}
    pv = [(sid, x) for t in tables.values() for sid, x in zip(t["session_id"].to_pylist(), t["video_preview"].to_pylist())]
    ok_pv = all(bool(x is not None and x["bytes"]) == (sid in with_v) for sid, x in pv)
    check("有视频的会话都有内嵌预览", ok_pv, f"预览 {sum(1 for _, x in pv if x)} / 有视频会话 {len(with_v)}")

    # ---- the top-camera crop, recorded in files rather than columns -----------
    cps = sorted(glob.glob(os.path.join(a.local, "crops", "crops-*.csv")))
    if cps:
        crows = [r for cp in cps for r in csv.DictReader(open(cp, newline=""))]
        check("crops 表覆盖且只覆盖发布会话", {r["session_id"] for r in crows} == set(sids),
              f"{len(crows)} 行 / {len(cps)} 张表")
        bad = [r["session_id"] for r in crows if r["crop_w"] and not (
            int(r["crop_x"]) >= 0 and int(r["crop_y"]) >= 0
            and int(r["crop_x"]) + int(r["crop_w"]) <= 1920 and int(r["crop_y"]) + int(r["crop_h"]) <= 1080)]
        check("每个矩形都落在标定网格 1920x1080 内", not bad, f"越界 {len(bad)}")
        cropped = {r["session_id"] for r in crows if r["crop_w"]}
        import collections as _c
        why = _c.Counter(r["basis"] for r in crows if not r["crop_w"])
        stated = {r["session_id"] for r in crows if not r["crop_w"] and r["basis"].startswith("not_cropped_")}
        check("未裁剪的会话都注明了理由", len(cropped) + len(stated) == len(crows),
              f"裁剪 {len(cropped)}, 留原样 {len(stated)} {dict(why)}")
        vm = os.path.join(a.local, "videos", "metadata.csv")
        vrows = list(csv.DictReader(open(vm, newline=""))) if os.path.exists(vm) else []
        mism = [r["session_id"] for r in vrows if r["camera"] == "top" and bool(r.get("crop_w")) != (r["session_id"] in cropped)]
        check("videos/metadata.csv 的 top 行与 crops 表一致", not mism, f"不一致 {len(mism)}")
        import subprocess as _sp
        probed, wrong = 0, []
        for r in [x for x in vrows if x["camera"] == "top"][:8]:
            f_ = os.path.join(a.local, "videos", r["file_name"])
            if not os.path.exists(f_):
                continue
            o = _sp.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                         "stream=width,height", "-of", "csv=p=0", os.path.realpath(f_)],
                        capture_output=True, text=True).stdout.strip().split(",")
            probed += 1
            if r.get("crop_w") and (o[0], o[1]) != (r["crop_w"], r["crop_h"]):
                wrong.append(r["file_name"])
        check("抽样顶视视频的实际尺寸 == 清单里的矩形", not wrong and probed > 0, f"抽样 {probed}, 不符 {wrong[:2]}")

    # ---- documents ----------------------------------------------------------
    docs = sorted(glob.glob(os.path.join(a.local, "documents", "*", "*.md")))
    texts = {t_: set() for t_ in ("prompt_text", "interface_text")}
    for t in tables.values():
        for c in texts:
            texts[c] |= {x for x in t[c].to_pylist() if x}
    on_file = {open(p).read() for p in docs}
    check("documents/ 覆盖表里出现的每一份 prompt / interface 文本", (texts["prompt_text"] | texts["interface_text"]) <= on_file,
          f"{len(docs)} 个文件, 表中 {len(texts['prompt_text'])} 份 prompt + {len(texts['interface_text'])} 份 interface")

    # ---- images decodable ----------------------------------------------------
    from PIL import Image as PILImage
    bad_img = 0
    for t in tables.values():
        for c in ("evidence_before", "evidence_after"):
            for x in t[c].to_pylist()[:3]:
                if x and x["bytes"]:
                    try:
                        PILImage.open(io.BytesIO(x["bytes"])).verify()
                    except Exception:
                        bad_img += 1
    check("抽样 evidence 缩略图可解码", bad_img == 0, f"坏图 {bad_img}")

    # ---- scrubbing over every string leaf + the raw traces --------------------
    user = os.path.basename(os.path.expanduser("~"))
    pats = {"本机路径": re.compile(r"/home/[a-z][a-z0-9_-]*/"), "裸用户名": re.compile(r"\b" + re.escape(user) + r"\b"),
            "右臂桥工作区": re.compile(r"mz_workspace"),
            "相机序列号": re.compile(r"usb-[0-9A-Za-z_]+_[0-9A-Za-z]{6,}"),
            "token": re.compile(r"\b(sk-[A-Za-z0-9_-]{20,}|hf_[A-Za-z0-9]{20,}|ghp_[A-Za-z0-9]{20,})"),
            "运行时socket": re.compile(r"/run/user/"),
            "harness元数据": re.compile(r"permissionMode|messaging_socket_path|memory_paths|mcp_servers")}
    leaks = {}
    for t in tables.values():
        for label, arr in string_leaves(t):
            if label.endswith("url") or label.endswith(".url") or label.startswith("video_preview"):
                continue  # Hub URLs, no user text
            for name, rx in pats.items():
                k = int(pc.sum(pc.match_substring_regex(arr, rx.pattern)).as_py() or 0)
                if k:
                    leaks[f"{label}:{name}"] = leaks.get(f"{label}:{name}", 0) + k
    for q in glob.glob(os.path.join(a.local, "traces", "*.jsonl")):
        txt = open(q, errors="replace").read()
        for name, rx in pats.items():
            k = len(rx.findall(txt))
            if k:
                leaks[f"traces/*.jsonl:{name}"] = leaks.get(f"traces/*.jsonl:{name}", 0) + k
    check("脱敏七类模式在全部字符串列与原始 traces 上零命中", not leaks, f"泄漏 {leaks}" if leaks else "0")

    if a.local_only:
        finish(); return

    # ---- remote ---------------------------------------------------------------
    # $HF_TOKEN wins; else --token-file / $HF_TOKEN_FILE; else no token at all (a public dataset needs none).
    token = os.environ.get("HF_TOKEN")
    if not token and a.token_file:
        if not os.path.exists(a.token_file):
            sys.exit(f"--token-file: no such file: {a.token_file}  (or set $HF_TOKEN, or use --local-only)")
        token = open(a.token_file).read().strip()
    import yaml
    declared = yaml.safe_load(open(os.path.join(a.local, "README.md")).read().split("---", 2)[1]).get("configs", [])
    try:
        splits = api(f"https://datasets-server.huggingface.co/splits?dataset={a.repo}", token)
    except Exception as e:
        check("datasets-server 可达", False, str(e)[:70]); splits = {}
    failed = splits.get("failed", [])
    check("datasets-server 报告零失败 config", not failed, f"失败 {[f.get('config') for f in failed][:4]}")
    got = {(s["config"], s["split"]) for s in splits.get("splits", [])}
    want = {(c["config_name"], d["split"]) for c in declared for d in c["data_files"]}
    check("远端可见 (config, split) == 卡片声明", got == want, f"{sorted(got)}" + ("  （Viewer 尚在构建）" if not got else ""))
    for cfg, sp in sorted(want):
        try:
            fr = api(f"https://datasets-server.huggingface.co/first-rows?dataset={a.repo}&config={cfg}&split={sp}", token)
            ok, detail = bool(fr.get("rows")), f"{len(fr.get('rows', []))} 行, {len(fr.get('features', []))} 列"
        except Exception as e:
            ok, detail = False, str(e)[:70]
        check(f"first-rows {cfg}/{sp}", ok, detail)
        try:
            rw = api(f"https://datasets-server.huggingface.co/rows?dataset={a.repo}&config={cfg}&split={sp}&offset=0&length=100", token)
            ok, detail = bool(rw.get("rows")), f"{len(rw.get('rows', []))} 行"
        except Exception as e:
            ok, detail = False, str(e)[:70]
        check(f"整页 100 行 {cfg}/{sp}", ok, detail)
    finish()


if __name__ == "__main__":
    main()
