#!/usr/bin/env python3
"""Table 2 metrics (two pair assembly, model comparison): SR, time, tokens, cost per model batch.

  python3 tools/paper_table2_metrics.py [--out paper_runs/table2_metrics.csv]

Conventions = the paper's Table 1/2 accounting (analysis/two_pair_assembly/README.md, analysis/efficiency/detailed.py):
  SR      operator review of the final overhead photo: BOTH demonstrated pairs assembled after release (left set only)
  time    (report boundary - task_started)/60 over successful trials; the boundary is the server-side create_time of the
          LAST tool call whose write target is scratch/RESULT.md (patch Add/Update File, '> RESULT.md', write_text), i.e. the final
          report update when the agent verified further after a first report
  tokens  input (incl. cached) + output (incl. reasoning) per trial, from per-request token_count events of the codex
          rollout (duplicate events that do not advance the cumulative total are skipped), thousands
  cost    per request: uncached*P_in + cached*P_cached + cache_write*P_cw + output*P_out (long context >272K/request:
          2x input & cache, 1.5x output), Standard tier. Rates = OpenAI model pages fetched 2026-09-09/10.
Claude Code backend (Opus 5): agent_events.jsonl is the stream-json log; usage = result.modelUsage (cumulative, every model
  incl. the Haiku helper calls), cost at Anthropic list rates (1h cache writes 2x, reads 0.1x, output incl. thinking),
  which reproduces Claude Code's own total_cost_usd; the report boundary = timestamp of the tool_result/user event that
  starts the turn writing RESULT.md (the analogue of codex's create_time); task start = AGENT_START_S (CLI launch).
Aggregates (mean [min, max]) are over successful trials, as in Table 1.
"""
import argparse, csv, datetime, glob, json, os, re, statistics, sys
from pathlib import Path
FA = str(Path(__file__).resolve().parents[1])              # this repo's agp/
SESS = os.path.expanduser('~/.codex/sessions')
# Optional cross-check against the paper-analysis tree, which lives in a separate workspace
# outside this repository: --per-run, or the env var AGP_PER_RUN_CSV. Unset (the default) or
# a missing file = the cross-check is skipped.
PER_RUN = os.environ.get('AGP_PER_RUN_CSV', '')
LONG_CTX = 272_000
RATES = {  # USD per 1M tokens: input, cached input, cache write, output
    'gpt-6-astra':   dict(inp=10.0, cached=1.0, cw=12.5, out=50.0),
    'gpt-5.6-sol':   dict(inp=4.0,  cached=0.4, cw=5.0,  out=20.0),
    'gpt-5.6-terra': dict(inp=2.0,  cached=0.2, cw=2.5,  out=12.0),
    # Anthropic list rates (platform.claude.com/docs/en/about-claude/pricing, 2026-09-10): cache read 0.1x, 5m write 1.25x, 1h write 2x
    'claude-opus-5':  dict(inp=5.0, cached=0.5, cw=6.25, cw1h=10.0, out=25.0),
    'claude-haiku-4-5': dict(inp=1.0, cached=0.1, cw=1.25, cw1h=2.0, out=5.0),
}
# operator verdicts from paper_runs/<batch>/trial_XX_after.png (reviewed 2026-09-10): both pairs of the LEFT set engaged
BATCHES = [  # (batch dir, table row label, {trial: success}); several batches may share one label (pooled, e.g. left + right arm)
    ('twopairs_full_high_56sol',       'GPT-5.6 Sol',        {f'{i:02d}': True for i in range(1, 6)}),
    ('twopairs_full_high_56terra',     'GPT-5.6 Terra',      {'01': False, '02': False, '03': False, '04': True, '05': False}),
    ('twopairs_full_low_6astra',       'GPT-6 Astra low',    {'01': True, '03': True, '04': True}),   # left arm trials
    ('twopairs_full_low_6astra_right', 'GPT-6 Astra low',    {'02': True, '05': True}),               # right arm trials (same task, right set)
    ('twopairs_full_medium_6astra',       'GPT-6 Astra medium', {'01': True, '03': True, '04': True, '05': True}),  # left arm
    ('twopairs_full_medium_6astra_right', 'GPT-6 Astra medium', {'02': True}),                                      # right arm
    ('twopairs_full_high_6astra',         'GPT-6 Astra high',   {'03': True, '05': True}),                          # left arm
    ('twopairs_full_high_6astra_right',   'GPT-6 Astra high',   {'01': True, '02': True, '04': True}),              # right arm
    ('twopairs_full_high_copus5',         'Claude Opus 5',      {'01': True, '03': True, '05': True}),              # left arm (Claude Code backend)
    ('twopairs_full_high_copus5_right',   'Claude Opus 5',      {'02': True, '04': True}),                          # right arm
]
def ts(s): return datetime.datetime.fromisoformat(s.replace('Z', '+00:00')).timestamp()
def report_write(src):  # a call whose WRITE TARGET is RESULT.md (patch add/update, shell redirect, python write)
    return bool(re.search(r'(?:(?:Add|Update) File: \S*RESULT\.md|>\s*\S*RESULT\.md|write_text\(|writeFile\(|open\([^)]*RESULT\.md[^)]*[\'"]w)', src)) and 'RESULT.md' in src
def thread_id(sd):
    for line in open(f'{sd}/agent_events.jsonl'):
        try: e = json.loads(line)
        except Exception: continue
        if e.get('type') == 'thread.started': return e['thread_id']
def parse_rollout(ro):
    reqs, last_key = [], None; start = end = None; calls = []
    for line in open(ro):
        try: e = json.loads(line)
        except Exception: continue
        p = e.get('payload') or {}; k = e.get('type')
        if k == 'event_msg' and p.get('type') == 'token_count':
            u, tot = p.get('info', {}).get('last_token_usage'), p.get('info', {}).get('total_token_usage')
            if not u: continue
            key = (tot['input_tokens'], tot['output_tokens']) if tot else None
            if key is not None and key == last_key: continue
            last_key = key
            reqs.append((u['input_tokens'], u.get('cached_input_tokens', 0), u.get('cache_write_input_tokens', 0) or 0, u['output_tokens'], u.get('reasoning_output_tokens', 0)))
        elif k == 'event_msg' and p.get('type') == 'task_started': start = ts(e['timestamp'])
        elif k == 'event_msg' and p.get('type') == 'task_complete': end = ts(e['timestamp'])
        elif k == 'response_item' and p.get('type') in ('custom_tool_call', 'function_call'):
            src = p.get('input') or p.get('arguments') or ''
            ct = float((p.get('internal_chat_message_metadata_passthrough') or {}).get('create_time', ts(e['timestamp'])))
            calls.append(dict(t=ts(e['timestamp']), create=ct, src=src, robot='robot_client' in src, write=report_write(src),
                              motion=bool(re.search(r'robot_client\.py \.(?: --arm \w+)? (move_ee|move_delta|move_joints|home|gripper|run_program)', src))))
    return reqs, start, end, calls
def parse_claude(sd):
    """Claude Code print-mode stream-json (agent_events.jsonl). Returns (usage_by_model, start, end, calls) where
    calls carry t (tool_use emission), create (start of the model turn = timestamp of the preceding tool_result/user
    event, the analogue of codex's response create_time), src, robot, write, motion."""
    env = dict(l.strip().split('=', 1) for l in open(f'{sd}/session.env') if '=' in l)
    start, end = float(env['AGENT_START_S']), float(env['AGENT_END_S'])
    evs = [json.loads(l) for l in open(f'{sd}/agent_events.jsonl') if l.strip()]
    calls, prev_user, res = [], start, None
    for e in evs:
        if e.get('type') == 'user' and 'timestamp' in e: prev_user = ts(e['timestamp'])
        elif e.get('type') == 'result': res = e
        elif e.get('type') == 'assistant':
            for c in e['message'].get('content', []):
                if c.get('type') != 'tool_use': continue
                inp = c.get('input') or {}; src = json.dumps(inp); fp = inp.get('file_path', '')
                write = fp.endswith('RESULT.md') if c['name'] in ('Write', 'Edit') else report_write(inp.get('command', '') if c['name'] == 'Bash' else '')
                calls.append(dict(t=ts(e['timestamp']), create=prev_user, src=src, robot='robot_client' in src, write=write,
                                  motion=bool(re.search(r'robot_client\.py \.(?: --arm \w+)? (move_ee|move_delta|move_joints|home|gripper|run_program)', src))))
    usage = {}
    for model, u in (res.get('modelUsage') or {}).items():
        cc = res['usage'].get('cache_creation') or {}
        canon = u.get('canonicalModel', model)
        # the 1h/5m split is only reported for the whole session; all writes here were 1h (checked), attribute by share
        usage[canon] = dict(inp=u['inputTokens'], cached=u['cacheReadInputTokens'], cw=u['cacheCreationInputTokens'], out=u['outputTokens'],
                            cw1h_share=(cc.get('ephemeral_1h_input_tokens', 0) / max(res['usage'].get('cache_creation_input_tokens', 1), 1)),
                            reported_cost=u.get('costUSD'))
    return usage, start, end, calls, res

def cost_claude(usage):
    c = 0.0
    for model, u in usage.items():
        P = RATES[model]; cw1h = u['cw'] * u['cw1h_share']; cw5m = u['cw'] - cw1h
        c += u['inp'] * P['inp'] + u['cached'] * P['cached'] + cw1h * P['cw1h'] + cw5m * P['cw'] + u['out'] * P['out']
    return c / 1e6

def cost_usd(reqs, P):
    c = 0.0
    for inp, cached, cw, out, _ in reqs:
        li, lo = (2.0, 1.5) if inp > LONG_CTX else (1.0, 1.0)
        c += (max(inp - cached - cw, 0) * P['inp'] + cached * P['cached'] + cw * P['cw']) * li + out * P['out'] * lo
    return c / 1e6
def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', default=f'{FA}/paper_runs/table2_metrics.csv')
    ap.add_argument('--per-run', default=PER_RUN,
                    help='per-run CSV from the paper-analysis workspace for the Sol cross-check (optional; env AGP_PER_RUN_CSV)')
    a = ap.parse_args()
    rows = []
    for batch, label, verd in BATCHES:
        for r in csv.DictReader(open(f'{FA}/paper_runs/{batch}/results.csv')):
            if r['trial'] not in verd: continue
            sd = f"{FA}/sessions/{r['session']}"
            if r.get('backend') == 'claude':
                usage, start, end, calls, res = parse_claude(sd); hits = [f'{sd}/agent_events.jsonl']; reqs = []
                inp = sum(u['inp'] + u['cw'] + u['cached'] for u in usage.values()); cached = sum(u['cached'] for u in usage.values())
                cw = sum(u['cw'] for u in usage.values()); out = sum(u['out'] for u in usage.values()); rea = int(r.get('tokens_reasoning') or 0)
                cost = cost_claude(usage)
                if abs(cost - float(res.get('total_cost_usd') or 0)) > 0.005:
                    print(f'!! {r["session"]}: list-rate cost {cost:.4f} != Claude Code total_cost_usd {res.get("total_cost_usd")}', file=sys.stderr)
                max_in = max(int(r['tokens_input']), 1)  # per-request input not exposed in stream-json; long-context surcharge does not exist for Claude 4.6+
            else:
                tid = thread_id(sd); hits = glob.glob(f'{SESS}/*/*/*/rollout-*-{tid}.jsonl')
                if not hits: print(f'!! no rollout for {r["session"]}', file=sys.stderr); continue
                reqs, start, end, calls = parse_rollout(hits[0])
                inp = sum(x[0] for x in reqs); cached = sum(x[1] for x in reqs); cw = sum(x[2] for x in reqs); out = sum(x[3] for x in reqs); rea = sum(x[4] for x in reqs)
                if int(r['tokens_input']) != inp or int(r['tokens_output']) != out:
                    print(f'!! {r["session"]}: results.csv in/out {r["tokens_input"]}/{r["tokens_output"]} != trace {inp}/{out}', file=sys.stderr)
                cost = cost_usd(reqs, RATES[r['model']]); max_in = max(x[0] for x in reqs)
            writes = [c for c in calls if c['write']]
            last_motion = max([c['t'] for c in calls if c['motion']], default=start)
            post = [c for c in writes if c['create'] >= last_motion]
            boundary = post[-1]['create'] if post else end          # final report update (see docstring)
            first_boundary = post[0]['create'] if post else end
            rows.append(dict(batch=batch, model=label, model_id=r['model'], backend=r.get('backend', 'codex'), effort=r['effort'], tier=r['service_tier'], arm=r.get('arms', 'left'), trial=r['trial'], session=r['session'],
                n_motion_calls=sum(1 for c in calls if c['motion']),
                success=int(verd[r['trial']]), agent_verdict=r['agent_verdict'].split()[0], task_started_s=round(start, 3), report_start_s=round(boundary, 6),
                first_report_s=round(first_boundary, 6), task_complete_s=round(end, 3), time_min=round((boundary - start) / 60, 3),
                time_first_report_min=round((first_boundary - start) / 60, 3), session_min=round(float(r['duration_s']) / 60, 2), n_report_writes=len(writes),
                robot_calls_after_first_report=sum(1 for c in calls if c['robot'] and post and c['t'] > post[0]['t']), requests=len(reqs),
                max_req_input=max_in, tokens_input=inp, tokens_cached=cached, tokens_cache_write=cw, tokens_output=out, tokens_reasoning=rea,
                tokens_k=round((inp + out) / 1e3, 3), cost_usd=round(cost, 4), rollout=hits[0]))
    with open(a.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator='\n'); w.writeheader(); w.writerows(rows)
    print(f'wrote {a.out} ({len(rows)} trials)\n')
    def agg(v): return f"{statistics.mean(v):.2f} [{min(v):.2f}, {max(v):.2f}]"
    for label in dict.fromkeys(b[1] for b in BATCHES):
        R = [r for r in rows if r['model'] == label]; S = [r for r in R if r['success']]
        print(f"{label:16s} SR {len(S)}/{len(R)}  time_min {agg([r['time_min'] for r in S])}  tokens_k {agg([r['tokens_k'] for r in S])}  cost_usd {agg([r['cost_usd'] for r in S])}")
    print('\nper trial:')
    for r in rows:
        print(f"{r['model']:16s} {r['arm']:5s} t{r['trial']} ok={r['success']} agent={r['agent_verdict']:8s} motions={r['n_motion_calls']:3d} time={r['time_min']:6.2f} (first report {r['time_first_report_min']:6.2f}, session {r['session_min']:5.1f}) "
              f"writes={r['n_report_writes']} robot_after_first={r['robot_calls_after_first_report']} req={r['requests']:3d} maxin={r['max_req_input']:6d} "
              f"tok={r['tokens_k']:8.1f}k cost=${r['cost_usd']:.3f}")
    # cross-check with the reference Sol accounting (per_run.csv) when present
    pr = a.per_run
    if os.path.exists(pr):
        print('\nreference per_run.csv vs this script (Sol):')
        col = {os.path.basename(x['session']): x for x in csv.DictReader(open(pr))}
        for r in rows:
            c = col.get(r['session'])
            if c: print(f"  t{r['trial']} start {float(c['start_s'])-r['task_started_s']:+.3f}s report_start {float(c['report_start_s'])-r['report_start_s']:+.6f}s "
                        f"time {float(c['time_min'])-r['time_min']:+.4f}min tokens_k {float(c['tokens_k'])-r['tokens_k']:+.3f} cost {float(c['cost_usd'])-r['cost_usd']:+.4f}")
if __name__ == '__main__': main()
