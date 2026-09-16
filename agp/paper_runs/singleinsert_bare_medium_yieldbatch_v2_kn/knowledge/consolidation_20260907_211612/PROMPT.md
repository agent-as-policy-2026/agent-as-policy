# Consolidate task knowledge from previous autonomous sessions (offline — NO robot)

You are preparing notes and helper scripts for future autonomous sessions of a
REAL 6-DOF arm doing ONE specific task: "singleinsert". There is no robot attached to
this workspace: do not try to run robot commands. Work only inside this directory.

- `inputs/<session>/` — one folder per previous session on this task: the
  agent's final report `RESULT.md`, its scripts and notes from `scratch/`, and
  the server command log `server.log` (every robot command with its outcome).
- `inputs/TASK_PROMPT.md` and `inputs/README_interface.md` — the task and the
  robot interface those sessions worked with (the next session gets the same).
- `knowledge/` — what the task's knowledge store already holds (may be empty).

Write `scratch/knowledge_delta.md` for the NEXT session on this same task:

1. Facts, as table rows `| fact | applies when | evidence | confidence |`:
   object sizes and heights that were measured, grasp widths and tool
   orientations that worked, insertion offsets, table height, pitfalls and
   what failed. Task-specific numbers are wanted. Give evidence as the input
   session folder plus the numbers, not frame paths. Only what the inputs
   actually show.
2. The procedure that worked, as ONE playbook block `## playbook: <name>`:
   step by step with the commands and parameters used, marking which numbers
   depend on where the parts lie (positions) and which do not (sizes, heights,
   widths, orientations).
3. Helper scripts worth reusing: copy the best version of each into
   `scratch/knowledge_delta_tools/<name>.py`, deduplicated across sessions,
   parametrised so that scene positions are arguments rather than constants,
   with this header at the top (plain comment lines, all seven keys, one line each):
       # tool: <name>.py
       # category: process | geometry | task
       # purpose: <what it does>
       # usage: python3 knowledge/tools/<name>.py <args>    (run from the session directory)
       # inputs/outputs: <what it reads; what it prints or writes>
       # assumptions: <hard-coded values, formats, sizes, tolerances — or "none">
       # verified: used successfully in the session that wrote it
   No session names, dates, frame numbers or absolute paths inside a tool. Then
   add to `scratch/knowledge_delta.md` one section `## tool: <name>.py` per
   script with one sentence on what it is for; if you keep none, add the line
   `## tools: none`.

Do not invent anything the inputs do not support. Do not repeat what
`knowledge/` already contains. Keep the whole delta under about 120 lines.
