# Dataset contents — batch `b01`

The bytes live on the Hub, not in this repository:
<https://huggingface.co/datasets/Agent-as-Policy/agent-as-policy>

This file is the correspondence between the two. The dataset is **one table,
one row per session**, plus folders of per-session files the rows link to. Every path below is a
glob, so later batches drop new shards alongside the existing ones and the
dataset card's single config never needs editing.

Snapshot frozen 2026-09-13 17:31 local time, 196 sessions; 169 released (`snapshots/b01/released.txt`) — the 21 auxiliary sessions and 6 evaluated sessions without a batch id are left out (`snapshots/b01/excluded.txt`).

| what | path pattern | files | rows / count | size |
|---|---|---|---|---|
| `trials` — the table, 22 columns (config, one split) | `trials/trials_evaluated-*.parquet` | 21 | 162 | 293 MiB |
| scene-reset sessions, same columns, not a viewer split | `reset/trials_reset-*.parquet` | 4 | 27 | 29 MiB |
| videos — the overhead view cropped per trial, wrist views untouched | `videos/<session_id>/<camera>.mp4` + `videos/metadata.csv` | 388 | 387 | 18.25 GiB |
| side videos, re-encoded 720p | `videos_side/<session_id>/side.mp4` + `videos_side/metadata.csv` | 158 | 157 | 1.42 GiB |
| raw capture images, WebDataset, always full frame | `captures/captures-b0N-*.tar` | 11 | 8,760 samples | 14.90 GiB |
| wrist depth frames, PNG16 uint16 mm | `depth/<session_id>/<NNNN>_wrist_depth.png` | 8,755 | 8,755 | 0.22 GiB |
| evidence photos, original PNG, cropped with the video | `evidence/<batch>/trial_<NN>_<kind>.png` + `evidence/metadata.csv` | 523 | 522 | 0.39 GiB |
| raw agent transcripts | `traces/<session_id>.jsonl` | 189 | 30,130 steps | 292 MiB |
| overhead crop rectangles | `crops/crops-b0N.csv` | 2 | 189 | 26 KiB |
| reference photographs | `reference/<id>/…` | 67 | 12 sets | 53 MiB |
| prompts and interfaces | `documents/{prompts,interfaces}/<stem>-<sha8>.md` | 37 | 37 | 330 KiB |
| **total** | | **10,122** | | **35.8 GiB** |

Two batches: `b01` frozen 2026-09-13 (169 released of 196 frozen) and `b02` frozen
2026-09-14 (20 released, nothing excluded). 189 sessions = 162 scored trials + 27
scene-reset sessions. The batch id is in every shard and tar name, so b02's files sit
beside b01's under the same glob and the card's single config never changed.

What the row holds:

| in the row | links to files |
|---|---|
| identity, condition, `outcome`, `labeling{}`, `timing{}`, `bridge{}`, `tokens{}` | — |
| `evidence_before` / `evidence_after` (960 px JPEG thumbnails), `video_preview` (480 px / 12 fps) | `evidence/`, `videos/` |
| `reasoning_summary` — the model's own words for the trial | — |
| `traces[]` — 30,130 steps: `kind`, `text`, `command`, `output`, and `calls[]` with `cmd_id`, `cmd`, `args_json`, `ok`, `capture_index`, `depth_url`, `pose_json` | `depth/`, `traces/*.jsonl` |
| `prompt_text`, `interface_text`, `reference{id, files[…url]}` | `documents/`, `reference/` |
| `videos[camera, url, bytes, rec_start_s, rec_stop_s]` | `videos/`, `videos_side/` |

A robot-bridge call cannot be recognised from the command string — 3,306 of the
14,960 commands that drive the robot (22%) never mention the client, because the
agents write their own wrappers. It is recognised from the output: every bridge
answer is a top-level JSON object starting `{"ok"` and carrying an `id` that is the
recorded request's `cmd_id`. Answers printed inside a larger JSON are rejected (they
are other sessions' logs the agent read back), and in the eleven dual-arm trials the
two bridges number their requests independently, so each answer is attributed to an
arm before it is joined. 23,305 of 23,657 requests and all 8,760 captures are
attached this way; the rest are requests whose answer never reached the transcript.

**The overhead view is cropped per trial.** Two overhead cameras watch the shared
table, one per arm station (BRIO `178B0DAE` left, `B8C7F203` right), and each sees
both halves, so a left-arm trial's video also showed the right-arm side. A rectangle
per `(task, arms)` is derived from the union of the pixels the agent pointed at and
the gripper positions projected through that session's calibration, trimmed at the
0.2/99.8 percentile with a 200 px margin, snapped to 16 px; b02 reuses b01's
rectangles verbatim so the two batches frame identically. The dual-arm towel trials
and the whole-table throwing trials are left uncropped with the reason recorded.
Videos are re-derived from the untouched originals at `-preset medium -crf 23`
(50.5 dB PSNR, 0.994 SSIM against the decoded original crop), and the previews and
thumbnails are regenerated from the cropped sources rather than cropped twice.
**Every recorded pixel coordinate stays in the original 1920x1080 grid** — the
`deproject` requests in `traces[]`, the raw transcripts and the `calib.json` in the
capture shards are untouched, and the full-frame `top.png` in those shards is the
reference they point at. The rectangles ship as `crops/crops-b0N.csv` and as six
extra columns on the two media manifests; no crop information is in the table.

The per-session joint, alignment and action-log files are **not published**.

Why the row is light: the Viewer serves 100-row pages and scans at most 300 MB of
uncompressed parquet per page. A row that embedded everything was ~18 MB, so the
page could not be served; at ~1.5 MB per row every 100-row page stays under the
limit (verify_dataset.py checks it), the Viewer renders the thumbnails and plays the
preview, and the heavy data are one URL away.

Published in derived form, or not at all:

- `side` camera video is published re-encoded (720p, CRF 30, audio dropped,
  faststart), 23.19 GiB down to 1.53 GiB. It carries no per-frame alignment
  key, so it cannot be joined to the joint trajectories the way the other three
  cameras can; `alignment_granularity` in `videos[]` is `second` (from
  rec_start_s) or `none`. Originals stay local and on Google Drive.
- Original float32 `.npy` depth. The Hub does not read `.npy`. Depth ships as
  uint16 millimetre PNG instead, 1 mm quantisation, round-trip error bounded by
  1 mm, inside `traces[].calls[].depth_url` and inside the `captures/` tar shards.

Regenerate from a snapshot with the scripts in `scripts/`, in this order: the flat exporters `export_hf_dataset.py`, `export_lookup_tables.py`,
`export_traces.py`, `export_evidence.py`, then `export_videos_metadata.py` (it
needs a `trials_*.parquet`), `copy_videos.py`, `transcode_side.py` →
`export_videos_side_metadata.py`, and `convert_depth.py` → `pack_captures.py`;
`derive_top_crops.py` then `crop_top_media.py` apply the overhead crop;
`make_video_previews.py` encodes the 480 px previews; write `RELEASE_bNN.txt`
(snapshot minus `EXCLUDE_bNN.txt`) before `pack_captures.py --snapshot RELEASE`;
`build_trials_table.py --dataset <root> --batch bNN --out <release> --sessions RELEASE_bNN.txt`
writes `trials/`, `reset/`, `reference/` and `documents/`; `verify_dataset.py --local <release> --flat <root>
--snapshot RELEASE --exclude EXCLUDE` checks the result, including the 300 MB / 100-row bound.
