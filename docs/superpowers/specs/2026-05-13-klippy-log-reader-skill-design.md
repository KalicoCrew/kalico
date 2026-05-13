# klippy-log reader skill — design

**Status:** Draft for review. v1 scope; build-order item is tooling-only, not a planner step.

**Problem.** Reading `klippy.log` on `trident.local` directly from the main agent floods context with status-frame noise. Today's instance: 48 MB / 163k status-frame lines, 0 G-code lines, 0 motion events. Investigating "did my -X jog get issued?" against that consumed the main agent's window for zero diagnostic yield.

**Goal.** A repo-local skill that lets the main agent ask "what happened on trident?" without ever ingesting the raw log. A Haiku subagent does the fetch + filter + analysis end-to-end and returns a structured, citation-backed answer.

**Non-goals (v1).** Local-file sources. Sim/Renode logs. Live tailing. Cross-session correlation. On-disk cache. Multi-host. UI/companion-mode rendering.

---

## Architecture

End-to-end Haiku subagent. The main agent dispatches via the Agent tool with `subagent_type: general-purpose`, `model: "haiku"`, and a prompt built from a template embedded in `SKILL.md`. The subagent ssh's to trident, runs the preprocessor against the live log, reads the filtered slice from a temp file, reasons over it, and returns a structured response. The raw log bytes and the filtered slice both stay inside the subagent's context; only the final structured answer crosses back to the main agent.

```
main agent
  ├─ Agent tool (general-purpose, model=haiku)
  │    prompt = SKILL.md template with {question}, {session_override} filled
  ↓
Haiku subagent
  ├─ Bash: bash $SKILL_DIR/filter.sh [--session=...] > /tmp/<rand>.slice
  ├─ Read: /tmp/<rand>.slice
  ├─ reason
  ↓
  return structured answer (string)
main agent
  └─ relay to user
```

## File layout

```
.claude/skills/reading-klippy-log/
  SKILL.md       # frontmatter + dispatch instructions + Haiku prompt template
  filter.sh      # deterministic preprocessor (bash + awk)
```

Mirrors the existing `.claude/skills/diagnosing-h7-mcu-wedge/` convention.

## Invocation

Main agent invokes the skill via the standard Skill tool. The SKILL.md body instructs me (the main agent) to call the Agent tool with the constructed prompt. The skill takes two optional inputs:

- **question** (string, optional). Freeform query. Empty → default-report mode.
- **session_override** (one of `latest` | `previous` | `N`, default `latest`). Forces a specific session choice, bypassing the fresh-restart fallback heuristic.

Frontmatter `description` auto-triggers when the main agent is about to read klippy.log on trident, is asked about recent jog / print / fault activity on the bench, or when raw klippy bytes would otherwise enter main context. Auto-trigger mirrors `diagnosing-h7-mcu-wedge`'s symptom-keyword approach.

## Preprocessor (`filter.sh`)

Pure-function shell script. Inputs: optional `--session=<spec>` flag; optional `KLOG_LOCAL_OVERRIDE_PATH` env var. Output: filtered slice on stdout. Six stages, in order:

**1. Fetch.** Default: `ssh dderg@trident.local 'cat ~/printer_data/logs/klippy.log'`. If `KLOG_LOCAL_OVERRIDE_PATH` is set, `cat "$KLOG_LOCAL_OVERRIDE_PATH"` instead. The env-var override exists for fixture-based tests and for future local-source expansion; it is not a user-facing feature in v1.

**2. Index boot banners.** Walk once, record line number + wall-clock timestamp of every `Start printer at <date>` line. These are session boundaries.

**3. Pick the session.** Default = last banner.

  Fresh-restart fallback: if `--session=latest` (default) **and** the chosen session either started < 60 s ago by host wall clock OR has fewer than 100 *non-status lines* between banner and EOF, fall back to the previous banner. ("Non-status line" = any line not matching `kalico_status_v6`, per stage 5's definition.) Emit the decision and reason in the SESSION header. Explicit `--session=previous|N` skips this heuristic.

  If the fallback would target a previous banner that doesn't exist (only one session in the file), skip the fallback, keep the latest, and note "fallback skipped: no previous session available" in the SESSION header.

  No banner found anywhere → use entire file with warning in SESSION header.

**4. Slice.** From chosen banner line → EOF. Preserve **absolute line numbers** from the source file. Every output line is prefixed `L<n>\t<content>` so any quote downstream is verifiable against the raw log on trident.

**5. Collapse status-frame runs.** A status frame is any line matching `kalico_status_v6`. Track state **per MCU** (the log multiplexes `mcu` and `mcu 'bottom'`) on four fields: `engine_status`, `current_segment_id`, `last_fault`, `fault_detail`.

  - First frame of a run → emit verbatim.
  - Subsequent frames where all four fields are unchanged from the same MCU's previous frame → suppress, increment a counter.
  - Any field change → close the run with one summary line `[L<start>–L<end>] mcu='<name>' status unchanged: engine_status=<s> segment_id=<id> last_fault=<f> fault_detail=<d> (<frames> frames, <duration>s)`, then emit the changed frame verbatim and start a new run.

  Result: every transition is preserved verbatim with its line number; only dead-air between transitions is compressed.

**6. Pass-through verbatim.** Anything that is not a `kalico_status_v6` frame is emitted as-is (with line-number prefix): G-code lines, `Shutdown`, `Error`, `Warning`, `prior_*` (H7 wedge-diag self-postmortem), `boot_diag emit`, `bridge-async`, lines containing `fault` / `nack` / `transport` / `reconnect`, `Loaded MCU`, banner lines themselves, blank lines.

**Header.** First line of stdout is the SESSION metadata: chosen banner timestamp, banner line number, slice line range, non-status line count, fresh-restart-fallback decision and reason, full-file-fallback warning if applicable.

## Subagent prompt template

Embedded in `SKILL.md` as a heredoc. The main agent fills three slots before dispatching:

- `{SKILL_DIR}` → absolute path to the skill directory (`/Users/daniladergachev/Developer/kalico/.claude/skills/reading-klippy-log`). Hardcoded in SKILL.md so the main agent does not have to derive it.
- `{session_override}` → `latest` | `previous` | `N`, or omitted entirely (drops the `--session=` flag).
- `{QUESTION_OR_DEFAULT_INSTRUCTION}` → see below.

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

`{QUESTION_OR_DEFAULT_INSTRUCTION}` is one of:

- **Question mode** (caller passed a question): `Answer this question about the session: "<user question>"`
- **Default-report mode** (no question): `Produce the default session report. Sections: (a) boot reason & MCUs loaded, (b) faults / errors / shutdowns, (c) motion activity — count G-code lines, list first/last few, summarize segment dispatch, (d) comms anomalies — NAK / transport timeout / reconnect / bridge-async warnings, (e) engine-state timeline — list status & segment_id transitions chronologically. Each section cites L<n>; omit any section with no findings.`

## Output contract (subagent → main agent)

Single string in the structured format above. Main agent quotes it back to the user verbatim or paraphrases as appropriate. The `EVIDENCE:` block makes Haiku's claims spot-checkable: any line `L<n>: <content>` can be verified by re-reading the source log on trident with `sed -n '<n>p' ~/printer_data/logs/klippy.log`.

## Error handling

- **ssh failure** (host unreachable, auth failure, etc.): filter.sh exits non-zero with the ssh error on stderr. Subagent's Bash step fails; subagent returns a clean error: `ssh dderg@trident.local failed: <reason>`. Main agent surfaces verbatim.
- **No `Start printer at` banner anywhere in the log:** filter.sh emits a SESSION header with `WARNING: no boot banner found, using entire file` and proceeds to slice from line 1.
- **Empty / missing log file:** filter.sh exits non-zero with `klippy.log empty or unreadable at <path>`. Subagent surfaces verbatim.
- **`--session=N` out of range:** filter.sh exits non-zero with `requested session N not found (found M sessions)`.
- **filter.sh missing or non-executable:** subagent's Bash fails with a path/permission error; surfaces verbatim.

No retry logic. Failures are user-visible and immediate.

## Testing

v1 ships with one manual smoke test, no automated harness:

**Smoke test.** Against the current trident log (as of 2026-05-13), run the skill with no question. Expected output highlights:
- SESSION header showing the latest banner.
- Default-report sections all populated.
- Motion section: 0 G-code lines, 0 segment dispatches.
- Engine-state timeline: MCU pinned to `engine_status=2 segment_id=26` for ~155s (today's exact pattern).
- No faults, no shutdowns.

This is a known-good oracle because I hand-investigated the current state earlier today and have ground truth on what the answer should be.

**Future fixture-based tests.** The `KLOG_LOCAL_OVERRIDE_PATH` env var lets filter.sh read a local file. Future tests can build synthetic klippy.log fixtures (one with motion, one with faults, one fresh-restart, etc.) and assert filter.sh output. Out of scope for v1.

## Open questions / deferred

- **Per-call ssh cost** (~0.5–1s). Acceptable for v1. Add `tail -c <N>` upper bound if logs grow unwieldy.
- **Local-source expansion** (`.local-logs/**/klippy.log`, `tools/sim_klippy/.local-logs/klippy.log`). Deferred. `KLOG_LOCAL_OVERRIDE_PATH` is the seam.
- **Live tailing** (skill-driven `tail -f` for in-flight tests). Deferred.
- **Cross-session correlation** (e.g. "find the most recent session with a Shutdown"). Deferred; today's `--session=N` is the manual workaround.

## Acceptance criteria (v1 done)

1. `.claude/skills/reading-klippy-log/SKILL.md` and `filter.sh` exist and are executable.
2. Smoke test against trident matches expectations above.
3. Default-report mode and question mode both produce the structured output format.
4. Status-frame collapse: <500 emitted lines for a 100k-status-frame idle session.
5. Every `EVIDENCE:` line is verifiable by `sed -n '<n>p' ~/printer_data/logs/klippy.log` on trident.
