# klippy-log reader skill — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `.claude/skills/reading-klippy-log/` — a repo-local skill that dispatches a Haiku subagent to fetch, filter, and analyze `klippy.log` on `trident.local` without the raw log entering main-agent context.

**Architecture:** Bash preprocessor (`filter.sh`) does ssh + boot-banner indexing + session selection + status-frame collapse with absolute line-number preservation. Haiku subagent runs it, reads the temp file, and returns a structured citation-backed answer. SKILL.md ties them together with an auto-trigger description.

**Tech Stack:** Bash, awk, ssh. No new runtime dependencies.

**Spec:** `docs/superpowers/specs/2026-05-13-klippy-log-reader-skill-design.md`

---

## File Structure

```
.claude/skills/reading-klippy-log/
  SKILL.md                       # frontmatter + dispatch instructions + Haiku prompt template
  filter.sh                      # bash entry + session-picking; calls awk for slice+collapse
  tests/
    run.sh                       # test driver: iterates fixtures, diffs filter.sh output vs expected
    fixtures/
      empty.log                  # 0-byte file → error case
      no_banner.log              # no "Start printer at" line → full-file fallback
      single_session.log         # one banner, mixed content
      two_sessions_stable.log    # two banners, latest is mature (no fallback)
      two_sessions_fresh.log     # two banners, latest is fresh (fallback fires)
      status_collapse.log        # heavy status frames with one mid-run transition
      two_mcus.log               # status frames from mcu 'mcu' and mcu 'bottom' interleaved
    expected/
      empty.exit_code            # contains: "1"
      empty.stderr_grep          # contains: "empty or unreadable"
      no_banner.txt              # full-file slice with WARNING header
      single_session.txt
      two_sessions_stable.txt
      two_sessions_fresh.txt
      status_collapse.txt
      two_mcus.txt
```

Each file has one clear responsibility:
- `filter.sh` — orchestration (ssh / override / pick session / call awk)
- the awk script (inline heredoc inside filter.sh) — slice + collapse + line-prefix
- `SKILL.md` — Claude-facing dispatch logic
- `tests/run.sh` — test harness
- fixtures — synthetic klippy.log inputs
- expected — golden outputs

---

## Task 1: Scaffold the skill directory and test harness

**Files:**
- Create: `.claude/skills/reading-klippy-log/filter.sh`
- Create: `.claude/skills/reading-klippy-log/tests/run.sh`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/empty.log` (0 bytes)
- Create: `.claude/skills/reading-klippy-log/tests/expected/empty.exit_code`
- Create: `.claude/skills/reading-klippy-log/tests/expected/empty.stderr_grep`

- [ ] **Step 1: Create the directory tree**

```bash
mkdir -p .claude/skills/reading-klippy-log/tests/fixtures
mkdir -p .claude/skills/reading-klippy-log/tests/expected
```

- [ ] **Step 2: Write the empty-file fixture and expected error**

```bash
: > .claude/skills/reading-klippy-log/tests/fixtures/empty.log
echo "1" > .claude/skills/reading-klippy-log/tests/expected/empty.exit_code
echo "empty or unreadable" > .claude/skills/reading-klippy-log/tests/expected/empty.stderr_grep
```

- [ ] **Step 3: Write the test driver**

`.claude/skills/reading-klippy-log/tests/run.sh`:

```bash
#!/usr/bin/env bash
# Test driver for filter.sh. Iterates fixtures, runs filter.sh under
# KLOG_LOCAL_OVERRIDE_PATH, diffs stdout against tests/expected/<name>.txt OR
# checks exit code + stderr substring against tests/expected/<name>.exit_code /
# .stderr_grep when those exist instead.

set -u
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FIXTURES_DIR="$SKILL_DIR/tests/fixtures"
EXPECTED_DIR="$SKILL_DIR/tests/expected"
FILTER="$SKILL_DIR/filter.sh"

pass=0
fail=0
failed_names=()

for fixture in "$FIXTURES_DIR"/*.log; do
  name="$(basename "$fixture" .log)"
  exit_code_file="$EXPECTED_DIR/$name.exit_code"
  stderr_grep_file="$EXPECTED_DIR/$name.stderr_grep"
  expected_txt="$EXPECTED_DIR/$name.txt"

  actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH="$fixture" bash "$FILTER" 2>/tmp/run_sh_stderr.$$)"
  actual_exit=$?
  actual_stderr="$(cat /tmp/run_sh_stderr.$$)"
  rm -f /tmp/run_sh_stderr.$$

  ok=1

  if [[ -f "$exit_code_file" ]]; then
    expected_exit="$(cat "$exit_code_file")"
    if [[ "$actual_exit" != "$expected_exit" ]]; then
      echo "FAIL $name: exit code expected=$expected_exit actual=$actual_exit"
      ok=0
    fi
  fi

  if [[ -f "$stderr_grep_file" ]]; then
    needle="$(cat "$stderr_grep_file")"
    if ! grep -qF -- "$needle" <<<"$actual_stderr"; then
      echo "FAIL $name: stderr missing substring: $needle"
      echo "  actual stderr: $actual_stderr"
      ok=0
    fi
  fi

  if [[ -f "$expected_txt" ]]; then
    if ! diff -u "$expected_txt" <(echo "$actual_stdout") > /tmp/run_sh_diff.$$ 2>&1; then
      echo "FAIL $name: stdout differs from $expected_txt"
      cat /tmp/run_sh_diff.$$ | head -40
      ok=0
    fi
    rm -f /tmp/run_sh_diff.$$
  fi

  if (( ok )); then
    pass=$((pass + 1))
    echo "PASS $name"
  else
    fail=$((fail + 1))
    failed_names+=("$name")
  fi
done

echo
echo "Results: $pass passed, $fail failed"
if (( fail > 0 )); then
  echo "Failed: ${failed_names[*]}"
  exit 1
fi
```

- [ ] **Step 4: Write filter.sh stub (always exits 1 with empty-or-unreadable error)**

`.claude/skills/reading-klippy-log/filter.sh`:

```bash
#!/usr/bin/env bash
# klippy-log reader preprocessor — stub. Tasks 2+ flesh this out.
set -u

SRC="${KLOG_LOCAL_OVERRIDE_PATH:-}"
if [[ -z "$SRC" ]]; then
  SRC="<ssh:dderg@trident.local:~/printer_data/logs/klippy.log>"
  echo "klippy.log empty or unreadable at $SRC (stub; ssh path not yet implemented)" >&2
  exit 1
fi

if [[ ! -s "$SRC" ]]; then
  echo "klippy.log empty or unreadable at $SRC" >&2
  exit 1
fi

echo "STUB: would process $SRC"
```

- [ ] **Step 5: Make scripts executable**

```bash
chmod +x .claude/skills/reading-klippy-log/filter.sh .claude/skills/reading-klippy-log/tests/run.sh
```

- [ ] **Step 6: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `PASS empty` and `Results: 1 passed, 0 failed`.

- [ ] **Step 7: Commit**

```bash
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): scaffold dir, test driver, empty-file case"
```

---

## Task 2: Implement source-fetch stage (KLOG_LOCAL_OVERRIDE_PATH path)

**Files:**
- Modify: `.claude/skills/reading-klippy-log/filter.sh`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/single_session.log`
- Create: `.claude/skills/reading-klippy-log/tests/expected/single_session.txt`

- [ ] **Step 1: Write a fixture with one boot banner and minimal content**

`tests/fixtures/single_session.log`:

```
junk line before any banner
=======================
Start printer at Wed May 13 00:09:04 2026 (1778623744.8 15059.2)
Loaded MCU 'mcu' 175 commands
G1 X10 F3000
mcu 'mcu':  got {'type': 'status', 'engine_status': 0, 'current_segment_id': 0, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 100.0, '#receive_time': 100.0}
Shutdown: oh no
```

- [ ] **Step 2: Write the expected output (with placeholder header line)**

`tests/expected/single_session.txt`:

```
SESSION: banner_line=3 banner_time='Wed May 13 00:09:04 2026' slice=L3-L7 non_status_lines=4 fallback=none
L3	Start printer at Wed May 13 00:09:04 2026 (1778623744.8 15059.2)
L4	Loaded MCU 'mcu' 175 commands
L5	G1 X10 F3000
L6	mcu 'mcu':  got {'type': 'status', 'engine_status': 0, 'current_segment_id': 0, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 100.0, '#receive_time': 100.0}
L7	Shutdown: oh no
```

- [ ] **Step 3: Run tests to verify the new fixture fails**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `PASS empty`, `FAIL single_session` (stdout differs — stub prints "STUB: would process ...").

- [ ] **Step 4: Implement source-fetch in filter.sh (override path branch only; ssh path still stubbed)**

Replace `filter.sh` with:

```bash
#!/usr/bin/env bash
# klippy-log reader preprocessor.
# Stages: 1 fetch, 2 banner index, 3 session pick, 4 slice, 5 collapse, 6 pass-through.
# Honors KLOG_LOCAL_OVERRIDE_PATH for tests/fixtures.
set -u

usage() {
  echo "usage: filter.sh [--session=latest|previous|N]" >&2
}

session_arg="latest"
for arg in "$@"; do
  case "$arg" in
    --session=*) session_arg="${arg#--session=}" ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

# Stage 1: Fetch source into a temp file.
TMPLOG="$(mktemp -t klog.XXXXXX)"
trap 'rm -f "$TMPLOG"' EXIT

SRC_LABEL=""
if [[ -n "${KLOG_LOCAL_OVERRIDE_PATH:-}" ]]; then
  SRC_LABEL="$KLOG_LOCAL_OVERRIDE_PATH"
  if [[ ! -e "$KLOG_LOCAL_OVERRIDE_PATH" ]]; then
    echo "klippy.log empty or unreadable at $SRC_LABEL" >&2
    exit 1
  fi
  cat "$KLOG_LOCAL_OVERRIDE_PATH" > "$TMPLOG"
else
  SRC_LABEL="dderg@trident.local:~/printer_data/logs/klippy.log"
  if ! ssh dderg@trident.local 'cat ~/printer_data/logs/klippy.log' > "$TMPLOG" 2>/tmp/klog_ssh_err.$$; then
    err="$(cat /tmp/klog_ssh_err.$$)"; rm -f /tmp/klog_ssh_err.$$
    echo "ssh dderg@trident.local failed: $err" >&2
    exit 1
  fi
  rm -f /tmp/klog_ssh_err.$$
fi

if [[ ! -s "$TMPLOG" ]]; then
  echo "klippy.log empty or unreadable at $SRC_LABEL" >&2
  exit 1
fi

# TODO Task 3+: banner indexing, session pick, slice, collapse.
# For now emit a placeholder so the test driver detects this stage works.
echo "FETCHED: $(wc -l < "$TMPLOG") lines from $SRC_LABEL"
```

- [ ] **Step 5: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `PASS empty`, `FAIL single_session` (still fails — output is now "FETCHED: 7 lines from ..." which doesn't match the golden file). This is intentional — the next tasks implement banner-index → slice → collapse, at which point single_session will pass.

- [ ] **Step 6: Commit**

```bash
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): stage 1 fetch (override path + ssh)"
```

---

## Task 3: Implement banner indexing

**Files:**
- Modify: `.claude/skills/reading-klippy-log/filter.sh`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/no_banner.log`
- Create: `.claude/skills/reading-klippy-log/tests/expected/no_banner.txt`

- [ ] **Step 1: Add the no-banner fixture**

`tests/fixtures/no_banner.log`:

```
some random line
another line with no banner
mcu 'mcu':  got {'type': 'status', 'engine_status': 0, '#name': 'kalico_status_v6'}
```

- [ ] **Step 2: Add the no-banner expected output (warning + full file)**

`tests/expected/no_banner.txt`:

```
SESSION: WARNING no boot banner found, using entire file (slice=L1-L3 non_status_lines=2)
L1	some random line
L2	another line with no banner
L3	mcu 'mcu':  got {'type': 'status', 'engine_status': 0, '#name': 'kalico_status_v6'}
```

- [ ] **Step 3: Run tests to verify both single_session and no_banner fail**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `PASS empty`, `FAIL single_session`, `FAIL no_banner`.

- [ ] **Step 4: Implement banner indexing in filter.sh**

Append to `filter.sh` (replace the `# TODO Task 3+ ...` block through end of file):

```bash
# Stage 2: Index boot banners.
# Each banner line: "Start printer at <date> (<host_epoch> <internal_clock>)"
mapfile -t BANNER_LINES < <(grep -n "^Start printer at " "$TMPLOG" | cut -d: -f1)

emit_full_file_warning() {
  total_lines=$(wc -l < "$TMPLOG")
  non_status=$(grep -cv "kalico_status_v6" "$TMPLOG")
  echo "SESSION: WARNING no boot banner found, using entire file (slice=L1-L${total_lines} non_status_lines=${non_status})"
  awk '{ printf("L%d\t%s\n", NR, $0) }' "$TMPLOG"
}

if (( ${#BANNER_LINES[@]} == 0 )); then
  emit_full_file_warning
  exit 0
fi

# TODO Task 4+: session pick, slice, collapse.
# For now emit a placeholder showing banner-index worked.
echo "BANNERS_FOUND: ${#BANNER_LINES[@]} at lines: ${BANNER_LINES[*]}"
```

- [ ] **Step 5: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `PASS empty`, `PASS no_banner`, `FAIL single_session` (still placeholder for the banner-found case).

- [ ] **Step 6: Commit**

```bash
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): stage 2 banner indexing + no-banner fallback"
```

---

## Task 4: Implement default session pick (--session=latest, no fallback)

**Files:**
- Modify: `.claude/skills/reading-klippy-log/filter.sh`

- [ ] **Step 1: Run tests, confirm single_session still failing**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `FAIL single_session`.

- [ ] **Step 2: Add session-pick logic to filter.sh**

Replace the `# TODO Task 4+ ...` block with:

```bash
# Stage 3: Pick a session.
# session_arg is one of: latest (default), previous, N (integer 1-based from start).
SESSION_INDEX=""  # 1-based index into BANNER_LINES
FALLBACK_REASON="none"

case "$session_arg" in
  latest)
    SESSION_INDEX=${#BANNER_LINES[@]}
    ;;
  previous)
    if (( ${#BANNER_LINES[@]} >= 2 )); then
      SESSION_INDEX=$(( ${#BANNER_LINES[@]} - 1 ))
    else
      echo "requested session 'previous' but only ${#BANNER_LINES[@]} session(s) found" >&2
      exit 1
    fi
    ;;
  *)
    # Numeric? 1-based.
    if [[ "$session_arg" =~ ^[0-9]+$ ]]; then
      if (( session_arg >= 1 && session_arg <= ${#BANNER_LINES[@]} )); then
        SESSION_INDEX=$session_arg
      else
        echo "requested session $session_arg not found (found ${#BANNER_LINES[@]} sessions)" >&2
        exit 1
      fi
    else
      echo "invalid --session value: $session_arg (expected latest, previous, or integer)" >&2
      exit 2
    fi
    ;;
esac

# Resolve to actual banner line number (1-based into TMPLOG).
BANNER_LINE=${BANNER_LINES[$((SESSION_INDEX - 1))]}

# TODO Task 5+: fresh-restart fallback heuristic refines BANNER_LINE.
# TODO Task 6+: slice + collapse from BANNER_LINE to EOF.
echo "PICKED_SESSION: index=$SESSION_INDEX banner_line=$BANNER_LINE"
```

- [ ] **Step 3: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `FAIL single_session` (still placeholder; slice not implemented). The placeholder should now print `PICKED_SESSION: index=1 banner_line=3`.

- [ ] **Step 4: Commit**

```bash
git add .claude/skills/reading-klippy-log/filter.sh
git commit -m "skill(reading-klippy-log): stage 3 session pick (--session flag)"
```

---

## Task 5: Implement slice with line-number prefixing (no collapse yet)

**Files:**
- Modify: `.claude/skills/reading-klippy-log/filter.sh`

- [ ] **Step 1: Append slice logic (no collapse yet)**

Replace the `# TODO Task 5+ ...` and `# TODO Task 6+ ...` block with:

```bash
# Compute slice metadata.
TOTAL_LINES=$(wc -l < "$TMPLOG")
SLICE_END=$TOTAL_LINES
NON_STATUS_LINES=$(awk -v start="$BANNER_LINE" 'NR >= start && $0 !~ /kalico_status_v6/' "$TMPLOG" | wc -l)
BANNER_TIME=$(awk -v ln="$BANNER_LINE" 'NR == ln { sub(/^Start printer at /, ""); sub(/ \(.*$/, ""); print; exit }' "$TMPLOG")

# Stage 4: emit SESSION header + slice. Collapse comes in Task 7.
echo "SESSION: banner_line=$BANNER_LINE banner_time='$BANNER_TIME' slice=L${BANNER_LINE}-L${SLICE_END} non_status_lines=$NON_STATUS_LINES fallback=$FALLBACK_REASON"
awk -v start="$BANNER_LINE" 'NR >= start { printf("L%d\t%s\n", NR, $0) }' "$TMPLOG"
```

- [ ] **Step 2: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `PASS empty`, `PASS no_banner`, `PASS single_session`. The single_session expected file already matches verbatim pass-through (one status frame, no collapse needed when there's only one in a row).

- [ ] **Step 3: Commit**

```bash
git add .claude/skills/reading-klippy-log/filter.sh
git commit -m "skill(reading-klippy-log): stage 4 slice + SESSION header + L<n> prefix"
```

---

## Task 6: Implement status-frame collapse (per-MCU run tracking)

**Files:**
- Modify: `.claude/skills/reading-klippy-log/filter.sh`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/status_collapse.log`
- Create: `.claude/skills/reading-klippy-log/tests/expected/status_collapse.txt`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/two_mcus.log`
- Create: `.claude/skills/reading-klippy-log/tests/expected/two_mcus.txt`

- [ ] **Step 1: Write the status-collapse fixture**

Five identical status frames in a row, then one transition, then two more identical, all for one MCU.

`tests/fixtures/status_collapse.log`:

```
Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 26, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.0, '#receive_time': 1.0}
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 26, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.1, '#receive_time': 1.1}
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 26, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.2, '#receive_time': 1.2}
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 26, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.3, '#receive_time': 1.3}
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 26, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.4, '#receive_time': 1.4}
mcu 'mcu':  got {'type': 'status', 'engine_status': 3, 'current_segment_id': 27, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.5, '#receive_time': 1.5}
mcu 'mcu':  got {'type': 'status', 'engine_status': 3, 'current_segment_id': 27, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.6, '#receive_time': 1.6}
mcu 'mcu':  got {'type': 'status', 'engine_status': 3, 'current_segment_id': 27, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.7, '#receive_time': 1.7}
```

- [ ] **Step 2: Write the status-collapse expected output**

`tests/expected/status_collapse.txt`:

```
SESSION: banner_line=1 banner_time='Wed May 13 00:00:00 2026' slice=L1-L9 non_status_lines=1 fallback=none
L1	Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)
L2	mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 26, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.0, '#receive_time': 1.0}
[L2-L6] mcu='mcu' status unchanged: engine_status=2 segment_id=26 last_fault=0 fault_detail=100 (5 frames, 0.4s)
L7	mcu 'mcu':  got {'type': 'status', 'engine_status': 3, 'current_segment_id': 27, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.5, '#receive_time': 1.5}
[L7-L9] mcu='mcu' status unchanged: engine_status=3 segment_id=27 last_fault=0 fault_detail=200 (3 frames, 0.2s)
```

Note: emitting the run-summary even for runs of length 1+1 isn't strictly necessary (run of 1 doesn't compress anything), but the algorithm always emits a summary at run close. The spec didn't forbid this; if the engineer prefers to only emit summaries when run_count >= 2, that's a valid refinement — adjust both expected files and the awk accordingly.

For determinism, the rule used in the expected files above: **always emit the first frame verbatim AND a closing summary line, regardless of run length.** That simplifies the awk and the summary line documents the run's actual extent even when it's 1.

- [ ] **Step 3: Write the two-MCUs fixture (interleaved status frames from both)**

`tests/fixtures/two_mcus.log`:

```
Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 10, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.0, '#receive_time': 1.0}
mcu 'bottom':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 20, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.1, '#receive_time': 1.1}
mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 10, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.2, '#receive_time': 1.2}
mcu 'bottom':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 20, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.3, '#receive_time': 1.3}
```

- [ ] **Step 4: Write the two-MCUs expected output**

`tests/expected/two_mcus.txt`:

```
SESSION: banner_line=1 banner_time='Wed May 13 00:00:00 2026' slice=L1-L5 non_status_lines=1 fallback=none
L1	Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)
L2	mcu 'mcu':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 10, 'last_fault': 0, 'fault_detail': 100, '#name': 'kalico_status_v6', '#sent_time': 1.0, '#receive_time': 1.0}
L3	mcu 'bottom':  got {'type': 'status', 'engine_status': 2, 'current_segment_id': 20, 'last_fault': 0, 'fault_detail': 200, '#name': 'kalico_status_v6', '#sent_time': 1.1, '#receive_time': 1.1}
[L3-L5] mcu='bottom' status unchanged: engine_status=2 segment_id=20 last_fault=0 fault_detail=200 (2 frames, 0.2s)
[L2-L4] mcu='mcu' status unchanged: engine_status=2 segment_id=10 last_fault=0 fault_detail=100 (2 frames, 0.2s)
```

Note ordering: at EOF (and whenever a non-status line forces all runs to close), the implementation closes runs in **MCU-name-sorted (alphabetical) order**. `bottom` < `mcu`, so `bottom`'s summary appears first in the expected output.

- [ ] **Step 5: Run tests, confirm both collapse fixtures fail**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: `FAIL status_collapse`, `FAIL two_mcus` (no collapse implemented yet).

- [ ] **Step 6: Replace the slice awk with collapse-aware awk**

In `filter.sh`, replace the final two lines:

```bash
echo "SESSION: banner_line=$BANNER_LINE banner_time='$BANNER_TIME' slice=L${BANNER_LINE}-L${SLICE_END} non_status_lines=$NON_STATUS_LINES fallback=$FALLBACK_REASON"
awk -v start="$BANNER_LINE" 'NR >= start { printf("L%d\t%s\n", NR, $0) }' "$TMPLOG"
```

with:

```bash
echo "SESSION: banner_line=$BANNER_LINE banner_time='$BANNER_TIME' slice=L${BANNER_LINE}-L${SLICE_END} non_status_lines=$NON_STATUS_LINES fallback=$FALLBACK_REASON"

awk -v start="$BANNER_LINE" '
function extract_field(line, field,   pat, mstart, mlen, val) {
  pat = "'\''" field "'\'': [0-9]+"
  if (match(line, pat)) {
    val = substr(line, RSTART + length(field) + 4, RLENGTH - length(field) - 4)
    return val + 0
  }
  return -1
}
function extract_mcu(line,   s) {
  if (match(line, /^mcu '\''[^'\'']+'\''/)) {
    s = substr(line, RSTART + 5, RLENGTH - 6)
    return s
  }
  return "?"
}
function extract_recv_time(line,   pat, val) {
  pat = "'\''#receive_time'\'': [0-9.]+"
  if (match(line, pat)) {
    # prefix "'#receive_time': " is 17 chars
    val = substr(line, RSTART + 17, RLENGTH - 17)
    return val + 0
  }
  return 0
}
function close_run(mcu,   dur) {
  if (active[mcu]) {
    dur = run_end_time[mcu] - run_start_time[mcu]
    printf("[L%d-L%d] mcu='\''%s'\'' status unchanged: engine_status=%d segment_id=%d last_fault=%d fault_detail=%d (%d frames, %.1fs)\n",
      run_start_line[mcu], run_end_line[mcu], mcu,
      run_es[mcu], run_seg[mcu], run_fault[mcu], run_detail[mcu],
      run_count[mcu], dur)
    active[mcu] = 0
  }
}
function close_all_sorted(   names, n, i, j, tmp) {
  n = 0
  for (m in active) if (active[m]) { names[++n] = m }
  for (i = 1; i < n; i++) for (j = i+1; j <= n; j++) if (names[i] > names[j]) { tmp = names[i]; names[i] = names[j]; names[j] = tmp }
  for (i = 1; i <= n; i++) close_run(names[i])
}
NR < start { next }
{
  if ($0 ~ /kalico_status_v6/) {
    mcu = extract_mcu($0)
    es = extract_field($0, "engine_status")
    seg = extract_field($0, "current_segment_id")
    fault = extract_field($0, "last_fault")
    detail = extract_field($0, "fault_detail")
    t = extract_recv_time($0)
    if (active[mcu] && es == run_es[mcu] && seg == run_seg[mcu] && fault == run_fault[mcu] && detail == run_detail[mcu]) {
      run_end_line[mcu] = NR
      run_end_time[mcu] = t
      run_count[mcu]++
    } else {
      close_run(mcu)
      printf("L%d\t%s\n", NR, $0)
      run_es[mcu] = es; run_seg[mcu] = seg; run_fault[mcu] = fault; run_detail[mcu] = detail
      run_start_line[mcu] = NR; run_end_line[mcu] = NR
      run_start_time[mcu] = t; run_end_time[mcu] = t
      run_count[mcu] = 1
      active[mcu] = 1
    }
  } else {
    close_all_sorted()
    printf("L%d\t%s\n", NR, $0)
  }
}
END {
  close_all_sorted()
}
' "$TMPLOG"
```

Note: the `'\''` sequence inside the awk program is bash single-quote escaping (close-quote, escaped-quote, open-quote) used because the entire awk program is itself wrapped in single quotes. This is intentional and required.

- [ ] **Step 7: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: all four current fixtures (`empty`, `no_banner`, `single_session`, `status_collapse`, `two_mcus`) PASS.

If `single_session` regresses (because its expected file omits the run-summary for its single status frame), update `tests/expected/single_session.txt` to append the matching summary line:

```
[L6-L6] mcu='mcu' status unchanged: engine_status=0 segment_id=0 last_fault=0 fault_detail=100 (1 frames, 0.0s)
```

before the `L7	Shutdown: oh no` line (since the Shutdown is a non-status line, it forces close_all_sorted which emits the summary).

Wait — actually the Shutdown line will fire `close_all_sorted()` BEFORE the Shutdown is printed. So order in the expected file should be:
```
L6	mcu 'mcu': ... (the status frame)
[L6-L6] mcu='mcu' ... (the close summary, fired when Shutdown line is seen)
L7	Shutdown: oh no
```

Update `tests/expected/single_session.txt` accordingly.

- [ ] **Step 8: Commit**

```bash
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): stage 5 status-frame collapse (per-MCU)"
```

---

## Task 7: Implement fresh-restart fallback heuristic

**Files:**
- Modify: `.claude/skills/reading-klippy-log/filter.sh`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/two_sessions_stable.log`
- Create: `.claude/skills/reading-klippy-log/tests/expected/two_sessions_stable.txt`
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/two_sessions_fresh.log`
- Create: `.claude/skills/reading-klippy-log/tests/expected/two_sessions_fresh.txt`

- [ ] **Step 1: Build the stable fixture (latest session has plenty of content; no fallback fires)**

`tests/fixtures/two_sessions_stable.log` — old banner, ~5 lines after; second (latest) banner with 150 lines of G-code-ish non-status content. We don't need 150 verbatim lines; embed exactly 101 non-status lines after the latest banner to clear the 100-line threshold, with a timestamp old enough to clear the 60s threshold.

Generate the fixture programmatically in this step:

```bash
F=.claude/skills/reading-klippy-log/tests/fixtures/two_sessions_stable.log
{
  echo "Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)"
  echo "old-session line A"
  echo "old-session line B"
  echo "Start printer at Wed May 13 00:05:00 2026 (1778600300.0 300.0)"
  for i in $(seq 1 101); do echo "G1 X${i} F3000"; done
} > "$F"
```

The latest banner is at line 4 with `1778600300.0` epoch (= 2026-05-13 00:05:00 UTC). For the 60s check, filter.sh compares this against `date +%s` at runtime. Tests run "now" which is much later than the fixture's banner time, so age >> 60s and fallback does NOT fire on the age check. Non-status line count (101) > 100, so fallback does NOT fire on the count check.

Expected: SESSION header says `fallback=none`, slice is L4-L105.

- [ ] **Step 2: Write the stable-fixture expected output**

```bash
F=.claude/skills/reading-klippy-log/tests/expected/two_sessions_stable.txt
{
  echo "SESSION: banner_line=4 banner_time='Wed May 13 00:05:00 2026' slice=L4-L105 non_status_lines=102 fallback=none"
  echo "L4	Start printer at Wed May 13 00:05:00 2026 (1778600300.0 300.0)"
  for i in $(seq 1 101); do echo "L$((4+i))	G1 X${i} F3000"; done
} > "$F"
```

(`non_status_lines=102` because banner line itself is non-status and the 101 G-code lines are non-status.)

- [ ] **Step 3: Build the fresh-restart fixture**

Latest banner is recent — timestamp comparison must trip fallback. The "recent" check uses host wall clock at runtime, which is brittle. Solution: have the fresh fixture's latest banner contain a host-epoch field that, when compared against `date +%s`, is < 60s old; we can't write a deterministic timestamp because tests run at unknown times.

Workaround: implement the age check by reading the banner's host-epoch field and subtracting from `date +%s`. To make the fresh fixture deterministic, generate the fixture *at test time* with `date +%s` baked in:

```bash
F=.claude/skills/reading-klippy-log/tests/fixtures/two_sessions_fresh.log
NOW=$(date +%s)
{
  echo "Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)"
  echo "old line"
  echo "Start printer at $(date -u -r $NOW '+%a %b %d %H:%M:%S %Y') ($NOW.0 0.0)"
  echo "G1 X1 F3000"
  echo "G1 X2 F3000"
} > "$F"
```

This makes the latest banner < 1s old, well under 60s.

But this fixture-on-the-fly approach breaks reproducibility — the file changes every test run. Cleaner approach: **the second fallback condition (non-status line count < 100) is deterministic by itself**, so we test fallback via the count threshold alone:

```bash
F=.claude/skills/reading-klippy-log/tests/fixtures/two_sessions_fresh.log
{
  echo "Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)"
  for i in $(seq 1 50); do echo "G1 X${i} F3000"; done
  echo "Start printer at Wed May 13 00:05:00 2026 (1778600300.0 300.0)"
  echo "G1 X100 F3000"
  echo "G1 X101 F3000"
} > "$F"
```

Latest banner is line 52, has only 2 non-status lines after → triggers fallback by count. Old enough by timestamp.

Expected: fallback fires, slice falls back to old banner at L1, header notes `fallback=count<100 (2 lines)` or similar.

- [ ] **Step 4: Write the fresh-fixture expected output**

```bash
F=.claude/skills/reading-klippy-log/tests/expected/two_sessions_fresh.txt
{
  echo "SESSION: banner_line=1 banner_time='Wed May 13 00:00:00 2026' slice=L1-L54 non_status_lines=53 fallback=fresh-restart (latest had 2 non-status lines, <100)"
  echo "L1	Start printer at Wed May 13 00:00:00 2026 (1778600000.0 0.0)"
  for i in $(seq 1 50); do echo "L$((1+i))	G1 X${i} F3000"; done
  echo "L52	Start printer at Wed May 13 00:05:00 2026 (1778600300.0 300.0)"
  echo "L53	G1 X100 F3000"
  echo "L54	G1 X101 F3000"
} > "$F"
```

- [ ] **Step 5: Run tests, confirm both two_sessions_* fixtures fail**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: previously-passing tests still pass; `FAIL two_sessions_stable` (no fallback logic → picks latest banner with `fallback=none`, but expected `slice=L4-L105` while actual will be `slice=L4-L<something>`; depends on what's already there); `FAIL two_sessions_fresh` (no fallback → picks latest, expected fallback to old).

- [ ] **Step 6: Implement fresh-restart fallback in filter.sh**

After the `case "$session_arg" in ... esac` block (the part that resolves `SESSION_INDEX` and `BANNER_LINE`), and before any `# TODO Task 5+` references, insert the fallback logic. Concretely, between the `BANNER_LINE=${BANNER_LINES[$((SESSION_INDEX - 1))]}` line and the `TOTAL_LINES=$(wc -l < "$TMPLOG")` line, insert:

```bash
# Stage 3b: Fresh-restart fallback (only when caller asked for "latest").
if [[ "$session_arg" == "latest" && ${#BANNER_LINES[@]} -ge 2 ]]; then
  # Count non-status lines from BANNER_LINE to EOF.
  ns_count=$(awk -v start="$BANNER_LINE" 'NR >= start && $0 !~ /kalico_status_v6/' "$TMPLOG" | wc -l)
  # Extract banner host-epoch (the first number in parens on the banner line).
  banner_epoch=$(awk -v ln="$BANNER_LINE" 'NR == ln { if (match($0, /\(([0-9]+\.[0-9]+) /, m)) print m[1]; exit }' "$TMPLOG")
  now=$(date +%s)
  epoch_int="${banner_epoch%.*}"
  age=$(( now - ${epoch_int:-0} ))

  fallback_reason=""
  if (( ns_count < 100 )); then
    fallback_reason="fresh-restart (latest had $ns_count non-status lines, <100)"
  elif (( age < 60 )); then
    fallback_reason="fresh-restart (latest is ${age}s old, <60s)"
  fi

  if [[ -n "$fallback_reason" ]]; then
    if (( ${#BANNER_LINES[@]} >= 2 )); then
      SESSION_INDEX=$(( ${#BANNER_LINES[@]} - 1 ))
      BANNER_LINE=${BANNER_LINES[$((SESSION_INDEX - 1))]}
      FALLBACK_REASON="$fallback_reason"
    else
      FALLBACK_REASON="fallback skipped: no previous session available ($fallback_reason)"
    fi
  fi
fi
```

Note: requires gawk for the `match(..., array)` form. If the system lacks gawk, the engineer can rewrite the epoch-extraction with a simpler awk regex+substr (analogous to `extract_recv_time` in Task 6). The plan assumes gawk is available; verify with `awk --version | head -1`.

- [ ] **Step 7: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: all six fixtures PASS.

- [ ] **Step 8: Commit**

```bash
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): stage 3b fresh-restart fallback heuristic"
```

---

## Task 8: Verify out-of-range and invalid --session errors

**Files:**
- Modify: `.claude/skills/reading-klippy-log/tests/run.sh` (extend driver to support per-fixture extra args)
- Create: `.claude/skills/reading-klippy-log/tests/fixtures/single_session_arg_oor.log` (alias of single_session.log)
- Create: `.claude/skills/reading-klippy-log/tests/expected/single_session_arg_oor.exit_code`
- Create: `.claude/skills/reading-klippy-log/tests/expected/single_session_arg_oor.stderr_grep`
- Create: `.claude/skills/reading-klippy-log/tests/expected/single_session_arg_oor.args`

- [ ] **Step 1: Extend run.sh to read an optional `.args` file per fixture**

In `tests/run.sh`, after the line `actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH=...`, change the invocation to read extra args:

Replace:

```bash
  actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH="$fixture" bash "$FILTER" 2>/tmp/run_sh_stderr.$$)"
  actual_exit=$?
```

with:

```bash
  args_file="$EXPECTED_DIR/$name.args"
  if [[ -f "$args_file" ]]; then
    # shellcheck disable=SC2046
    actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH="$fixture" bash "$FILTER" $(cat "$args_file") 2>/tmp/run_sh_stderr.$$)"
  else
    actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH="$fixture" bash "$FILTER" 2>/tmp/run_sh_stderr.$$)"
  fi
  actual_exit=$?
```

- [ ] **Step 2: Create the out-of-range args fixture (symlink to single_session.log content)**

```bash
cp .claude/skills/reading-klippy-log/tests/fixtures/single_session.log \
   .claude/skills/reading-klippy-log/tests/fixtures/single_session_arg_oor.log
echo "--session=5" > .claude/skills/reading-klippy-log/tests/expected/single_session_arg_oor.args
echo "1" > .claude/skills/reading-klippy-log/tests/expected/single_session_arg_oor.exit_code
echo "requested session 5 not found" > .claude/skills/reading-klippy-log/tests/expected/single_session_arg_oor.stderr_grep
```

- [ ] **Step 3: Run tests**

```bash
bash .claude/skills/reading-klippy-log/tests/run.sh
```

Expected: all PASS, including `single_session_arg_oor`.

- [ ] **Step 4: Commit**

```bash
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): test --session out-of-range error"
```

---

## Task 9: Write SKILL.md

**Files:**
- Create: `.claude/skills/reading-klippy-log/SKILL.md`

- [ ] **Step 1: Write SKILL.md with frontmatter, instructions to main agent, and the Haiku prompt template**

`.claude/skills/reading-klippy-log/SKILL.md`:

````markdown
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
````

- [ ] **Step 2: Commit**

```bash
git add .claude/skills/reading-klippy-log/SKILL.md
git commit -m "skill(reading-klippy-log): SKILL.md with Haiku dispatch instructions"
```

---

## Task 10: Live smoke test against trident

**Files:** none modified.

- [ ] **Step 1: Run the skill end-to-end via the Skill tool**

From a Claude session in this repo, invoke the skill with no question. The dispatcher should call the Agent tool with `subagent_type: general-purpose`, `model: "haiku"`, prompt = the SKILL.md template filled with the default-report instruction and no session override.

- [ ] **Step 2: Verify the response shape**

Expected (based on the 2026-05-13 ~09:30 CEST trident state):

- SESSION header references the latest banner (most recent `Start printer at` in `~/printer_data/logs/klippy.log`).
- `ANSWER:` field present and substantive.
- `EVIDENCE:` block with at least one `L<n>: ...` line.
- Default-report sections (boot reason, faults, motion, comms, engine-state timeline) all produced. The motion section should report **0 G-code lines** for the trident state I captured during brainstorming (the full log had 0 G1/G0 lines).
- Engine-state timeline highlights the `engine_status=2 segment_id=26` plateau.
- No `Shutdown` or fault entries.

- [ ] **Step 3: Verify the citation contract**

Pick any `L<n>` from the EVIDENCE block and run:

```bash
ssh dderg@trident.local "sed -n '<n>p' ~/printer_data/logs/klippy.log"
```

The line content must match the quote (modulo the `L<n>\t` prefix).

- [ ] **Step 4: Run a question-mode test**

Invoke the skill with question="Did any G1 X-something jogs happen since the last MCU restart?". Expected answer: "No, the log shows no G-code lines or segment dispatches since the last `Start printer at`." with EVIDENCE citing the SESSION header and a CAVEAT noting nothing was found.

- [ ] **Step 5: If anything fails, file a follow-up rather than patching now**

If the live smoke test reveals issues (wrong line-number arithmetic, banner regex mismatch on real-world banners with quirks, awk version differences on the local host, etc.), don't patch inline — note them and circle back. v1's acceptance criteria are met when all fixture tests pass AND the smoke test produces a structurally valid response that cites real lines.

- [ ] **Step 6: Final commit (if any tweaks were needed)**

```bash
git status
# If any drift surfaced and required fixes:
git add .claude/skills/reading-klippy-log/
git commit -m "skill(reading-klippy-log): fixes from live smoke test"
```

---

## Self-Review notes (run before invoking executing-plans)

**Spec coverage:**
- File layout (spec § "File layout") → Task 1 creates the dir + files; Task 9 creates SKILL.md.
- Invocation (spec § "Invocation") → Task 9's SKILL.md documents the question/session_override inputs and auto-trigger frontmatter.
- Preprocessor stages 1–6 (spec § "Preprocessor") → Tasks 2 (fetch), 3 (banners), 4 (session pick), 5 (slice + line-prefix), 6 (collapse), 7 (fresh-restart fallback).
- Subagent prompt template (spec § "Subagent prompt template") → Task 9.
- Output contract (spec § "Output contract") → Task 9's SKILL.md restates it for Haiku.
- Error handling (spec § "Error handling") → Task 1 (empty file), Task 3 (no banner), Task 8 (out-of-range session). ssh failure surfaces from Task 2's ssh path naturally — no dedicated test (would require offline trident).
- Testing (spec § "Testing") → Task 10 is the live smoke test against trident.
- Acceptance criteria (spec § "Acceptance criteria") → Task 1 (criterion 1, files exist), Task 10 (criterion 2, smoke test), Task 9 (criterion 3, both modes documented), Task 6 (criterion 4, collapse), Task 9 (criterion 5, citation contract).

**Placeholder scan:** no TBD / TODO / "add error handling" / "similar to Task N". Every code step has actual code.

**Type / name consistency:** `BANNER_LINE`, `BANNER_LINES`, `SESSION_INDEX`, `FALLBACK_REASON`, `SRC_LABEL`, `TMPLOG` are introduced in Task 2 and reused consistently through Task 7. The SESSION header format is locked at Task 5 and matched in expected fixtures from Task 5 onward. The awk function names (`close_run`, `close_all_sorted`, `extract_field`, `extract_mcu`, `extract_recv_time`) appear only in Task 6.

**Caveat for executor:** Task 6's awk relies on standard awk regex/substr features and should work on gawk and mawk. Task 7's fallback uses gawk-specific `match(..., array)`; if mawk is the system awk, replace with the simpler regex+substr pattern modeled after `extract_recv_time`.
