#!/usr/bin/env python3
"""Table 1 metrics (SR / time / tokens / cost) for the paper batches, from the codex rollout
traces (per-request token usage) + paper_runs/<batch>/results.csv (durations, verdicts).

  python3 tools/paper_table1_metrics.py [--out paper_runs/table1_metrics.csv]

Accounting (paper Sec. 4.1 + Appendix metrics):
  SR      = operator-confirmed complete successes / evaluated trials
  time    = median minutes over successful trials, task delivery -> final physical verification
            (= codex turn total minus terminal reporting when the reference trace timing exists,
            else the session wall clock)
  tokens  = mean (input incl. cached + output incl. reasoning) per evaluated trial, thousands
  cost    = sum over requests of  uncached_in*P_in + cached_in*P_cached + cache_write*P_cw + out*P_out,
            with the long-context rule (>272K input tokens in ONE request: 2x input/cache, 1.5x output)
            applied per request; Standard rates, and the actual tier (fast = 2x) reported alongside.
Rates: OpenAI GPT-6 Astra API list price (developers.openai.com, fetched 2026-09-09):
  input $10 / cached $1 / cache write $12.5 / output $50 per 1M tokens; Fast tier 2x.
"""
import argparse, csv, glob, json, os, statistics, sys
from pathlib import Path
FA = str(Path(__file__).resolve().parents[1])              # this repo's agp/
SESS = os.path.expanduser('~/.codex/sessions')
# Optional cross-check against the paper-analysis tree, which lives in a separate workspace
# outside this repository: --detailed-runs, or the env var AGP_DETAILED_RUNS_CSV. Unset (the
# default) or a missing file = session wall clock.
DETAILED = os.environ.get('AGP_DETAILED_RUNS_CSV', '')
P = dict(inp=10.0, cached=1.0, cw=12.5, out=50.0)          # USD per 1M, Standard
LONG_CTX = 272_000
# operator verdicts (final after-photos reviewed 2026-09-09; agent RESULT.md agrees)
BATCHES = [  # (batch dir, table label, {trial: success})
    ('pyramid_tools1',    'pyramid',         {f'{i:02d}': True for i in range(1, 11)}),
    ('twopiles_tools1',   'two towers',      {f'{i:02d}': True for i in range(1, 11)}),
    ('onebigpile_tools1', 'six block tower', {'01': True, '02': False, '03': True, '04': True, '05': True}),
    ('dice_tools1',       'die flipping',    {'01': True, '02': True}),
]

def thread_id(session_dir):
    with open(os.path.join(session_dir, 'agent_events.jsonl')) as f:
        for line in f:
            try: ev = json.loads(line)
            except Exception: continue
            if ev.get('type') == 'thread.started':
                return ev['thread_id']
    return None

def rollout_for(tid):
    hits = glob.glob(f'{SESS}/*/*/*/rollout-*-{tid}.jsonl')
    return hits[0] if hits else None

def per_request_usage(rollout):
    reqs = []; last_key = None
    with open(rollout) as f:
        for line in f:
            try: ev = json.loads(line)
            except Exception: continue
            if ev.get('type') != 'event_msg' or ev.get('payload', {}).get('type') != 'token_count': continue
            info = ev['payload'].get('info') or {}
            u = info.get('last_token_usage'); tot = info.get('total_token_usage')
            if not u: continue
            # codex re-emits a token_count for a retried/duplicated stream without advancing the
            # cumulative total; keep only events that advance it (matches codex's own reported usage)
            key = (tot['input_tokens'], tot['output_tokens']) if tot else None
            if key is not None and key == last_key: continue
            last_key = key
            reqs.append((u['input_tokens'], u.get('cached_input_tokens', 0), u.get('cache_write_input_tokens', 0) or 0,
                         u['output_tokens'], u.get('reasoning_output_tokens', 0)))
    return reqs

def cost_usd(reqs, mult=1.0):
    c = 0.0
    for inp, cached, cw, out, _ in reqs:
        lc_in, lc_out = (2.0, 1.5) if inp > LONG_CTX else (1.0, 1.0)
        uncached = max(inp - cached - cw, 0)
        c += (uncached * P['inp'] + cached * P['cached'] + cw * P['cw']) * lc_in + out * P['out'] * lc_out
    return c / 1e6 * mult

def trace_timing(path=DETAILED):
    t = {}
    if os.path.exists(path):
        with open(path) as f:
            for r in csv.DictReader(f):
                t[os.path.basename(r['session'])] = (float(r['total_s']), float(r['report_s']))
    return t

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', default=f'{FA}/paper_runs/table1_metrics.csv')
    ap.add_argument('--detailed-runs', default=DETAILED,
                    help='trace-timing CSV from the paper-analysis workspace (optional; env AGP_DETAILED_RUNS_CSV)')
    a = ap.parse_args()
    timing = trace_timing(a.detailed_runs); rows = []
    for batch, label, verd in BATCHES:
        with open(f'{FA}/paper_runs/{batch}/results.csv') as f:
            for r in csv.DictReader(f):
                if r['trial'] not in verd: continue
                sd = f"{FA}/sessions/{r['session']}"; tid = thread_id(sd); ro = rollout_for(tid) if tid else None
                if not ro: print(f'!! no rollout for {r["session"]} (thread {tid})', file=sys.stderr); continue
                reqs = per_request_usage(ro)
                inp = sum(x[0] for x in reqs); cached = sum(x[1] for x in reqs); cw = sum(x[2] for x in reqs)
                out = sum(x[3] for x in reqs); rea = sum(x[4] for x in reqs)
                if r.get('tokens_input') and int(r['tokens_input']) != inp:
                    print(f'!! {r["session"]}: results.csv input {r["tokens_input"]} != trace {inp}', file=sys.stderr)
                mult = 2.0 if r['service_tier'] == 'fast' else 1.0
                tot, rep = timing.get(r['session'], (None, None))
                task_min = (tot - rep) / 60 if tot is not None else float(r['duration_s']) / 60
                rows.append(dict(batch=batch, config=label, trial=r['trial'], session=r['session'], model=r['model'], effort=r['effort'],
                    tier=r['service_tier'], success=int(verd[r['trial']]), agent_verdict=r['agent_verdict'].split()[0],
                    session_min=round(float(r['duration_s']) / 60, 2), task_min=round(task_min, 2),
                    timing_src='trace' if tot is not None else 'session', requests=len(reqs), max_req_input=max(x[0] for x in reqs),
                    tokens_input=inp, tokens_cached=cached, tokens_cache_write=cw, tokens_output=out, tokens_reasoning=rea,
                    tokens_total_k=round((inp + out) / 1e3, 1), cost_std_usd=round(cost_usd(reqs), 3),
                    cost_tier_usd=round(cost_usd(reqs, mult), 3), rollout=ro))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator='\n'); w.writeheader(); w.writerows(rows)
    print(f'wrote {a.out} ({len(rows)} trials)\n')
    print(f"{'config':16s} {'n':>2s} {'SR%':>5s} {'time_med_min':>12s} {'sess_med_min':>12s} {'tokens_k_mean':>13s} {'cost_std_mean':>13s} {'cost_tier_mean':>14s} tier")
    for batch, label, _ in BATCHES:
        R = [r for r in rows if r['batch'] == batch]; S = [r for r in R if r['success']]
        print(f"{label:16s} {len(R):2d} {100*len(S)/len(R):5.0f} {statistics.median(r['task_min'] for r in S):12.1f} "
              f"{statistics.median(r['session_min'] for r in S):12.1f} {statistics.mean(r['tokens_total_k'] for r in R):13.0f} "
              f"{statistics.mean(r['cost_std_usd'] for r in R):13.2f} {statistics.mean(r['cost_tier_usd'] for r in R):14.2f} {set(r['tier'] for r in R)}")
    print('\nper trial:')
    for r in rows:
        print(f"{r['config']:16s} t{r['trial']} ok={r['success']} agent={r['agent_verdict']:8s} task={r['task_min']:5.1f}min sess={r['session_min']:5.1f} "
              f"req={r['requests']:3d} maxin={r['max_req_input']:6d} in={r['tokens_input']/1e6:5.2f}M cached={100*r['tokens_cached']/r['tokens_input']:4.1f}% "
              f"cw={r['tokens_cache_write']} out={r['tokens_output']/1e3:5.1f}k reas={r['tokens_reasoning']/1e3:4.1f}k tot={r['tokens_total_k']:7.0f}k "
              f"cost_std=${r['cost_std_usd']:6.2f} tier=${r['cost_tier_usd']:6.2f} [{r['timing_src']}]")
if __name__ == '__main__': main()
