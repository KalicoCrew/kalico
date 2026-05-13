---
name: reading-klippy-log
description: Use when investigating klippy.log on trident.local, asked about recent jog / print / fault / shutdown activity on the bench, when you need to know what the H7 or F4 MCUs have been doing, or whenever raw klippy log content would otherwise enter main-agent context. Dispatches a Haiku subagent that fetches, filters, and analyzes the log; the raw log never enters main-agent context.
---

# Reading klippy.log on trident

This skill answers questions about `~/printer_data/logs/klippy.log` on `trident.local` without flooding main-agent context with raw log content. A Haiku subagent does the fetch + filter + analysis end-to-end and returns a structured, citation-backed answer.

## When to use

- "What's in klippy.log?" / "Check the trident log."
- "Did my last G-code actually execute?" / "Did the MCU process the jog?"
- "Was there a fault / shutdown / wedge in the last session?"
- "When did the MCU last restart?"
- Anytime you would otherwise `ssh dderg@trident.local 'cat ~/printer_data/logs/klippy.log'` or `tail` it from the main agent.

## How to use

Dispatch via the Agent tool with `subagent_type: general-purpose` and `model: "haiku"`. The user-facing inputs (passed through to the prompt template below):

- **question** (string, optional). Freeform query. Omit / leave empty → default-report mode.
- **session_override** (one of `latest` | `previous` | `N`, default `latest`). Forces a specific session, bypassing the fresh-restart fallback heuristic.

Construct the subagent prompt by filling these slots into the template below:

- `{SKILL_DIR}` → `/Users/daniladergachev/Developer/kalico/.claude/skills/reading-klippy-log`
- `{session_override}` → the override value, or omit the `--session=` flag entirely if not specified
- `{QUESTION_OR_DEFAULT_INSTRUCTION}` → see "Question vs default-report mode" below

## Subagent prompt template

```
You are a klippy.log analyzer for the kalico fork on trident.local.

Pipeline (run exactly, in order):

1. Run:
     SLICE=/tmp/klog-$$.slice
     bash {SKILL_DIR}/filter.sh [--session={session_override}] > "$SLICE"
   The slice is session-scoped, status-collapsed, line-numbered.

2. Read the slice with the Read tool. Do NOT cat it via Bash — that re-injects
   bytes you already have on disk and wastes your context.

3. {QUESTION_OR_DEFAULT_INSTRUCTION}

Answering rules — non-negotiable:
- Every factual claim about the log MUST be backed by a quoted line with its
  L<n> citation. No claim without evidence.
- If the slice does not contain the evidence needed, say so explicitly. Do not
  speculate beyond what's in the slice.
- If the SESSION header indicates a fresh-restart fallback or full-file
  fallback, mention that in your answer.

Return your answer in this exact structure:

  SESSION: <copy the SESSION header line from the slice verbatim>

  ANSWER: <one or two sentences, direct>

  EVIDENCE:
    L<n>: <verbatim log line>
    L<n>: <verbatim log line>
    ...

  OBSERVATIONS:
    - <anything else notable that wasn't asked about, with L<n> citations>
    - (omit section if nothing notable)

  CAVEATS:
    - <e.g. "no -X moves found in this session — may have run in a different
       session or different host">
    - (omit section if none)
```

## Question vs default-report mode

The `{QUESTION_OR_DEFAULT_INSTRUCTION}` slot is one of:

- **Question mode** (caller passed a question):
  `Answer this question about the session: "<user question>"`

- **Default-report mode** (no question):
  `Produce the default session report. Sections: (a) boot reason & MCUs loaded, (b) faults / errors / shutdowns, (c) motion activity — count G-code lines, list first/last few, summarize segment dispatch, (d) comms anomalies — NAK / transport timeout / reconnect / bridge-async warnings, (e) engine-state timeline — list status & segment_id transitions chronologically. Each section cites L<n>; omit any section with no findings.`

## Verifying Haiku's answers

The `EVIDENCE:` block in the response cites `L<n>` references that map directly to line numbers in the source log. To spot-check any quote:

```bash
ssh dderg@trident.local "sed -n '<n>p' ~/printer_data/logs/klippy.log"
```

If the quoted content doesn't match, treat the answer as unreliable and re-run.

## When not to use

- Local fixture logs (`tests/fixtures/*.log` in this skill, or `.local-logs/**/klippy.log`): not supported by v1's user-facing interface. The `KLOG_LOCAL_OVERRIDE_PATH` env var exists for tests only.
- Renode sim logs (`tools/sim_klippy/.local-logs/klippy.log`): deferred. Read directly for now.
- Live tailing: deferred. This skill grabs a snapshot per call.
