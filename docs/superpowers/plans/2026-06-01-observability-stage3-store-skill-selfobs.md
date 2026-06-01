# Observability Stage 3 — VictoriaLogs + Vector + `query-logs` skill + self-observability

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax. Dispatch implementers **strictly serially** (shared worktree). Python host tasks → a general/claude subagent or the controller; any Rust touch → the `rust-engineer` subagent.

**Goal:** Stand up the queryable store (VictoriaLogs) fed by a checkpointed shipper (Vector) tailing the Stage 1/2 `events/*.jsonl`, ship a repo-committed `query-logs` skill (LogsQL over curl) as the primary agent interface, and add host-side self-observability (synthetic heartbeat + Vector liveness/lag check + the Stage-1 §16-item-11 last-gasp on sink failure) so a silent pipeline stall is itself a reportable fault.

**Architecture:** External, opt-in components fed by the durable JSONL the host already writes — klippy needs no code change to "enable VL." Vector (single Rust binary) tails `~/printer_data/logs/events/*.jsonl` with an on-disk checkpoint and pushes NDJSON to VictoriaLogs `/insert/jsonline`; VL (single Go binary) stores + indexes and answers LogsQL at `/select/logsql/query`. The `query-logs` skill carries time-bounded LogsQL recipes keyed to the exact schema. A small always-on klippy component emits a periodic heartbeat record and checks that Vector is alive and not lagging, surfacing staleness loudly. **Everything here is built and verified on the dev host (macOS) by running VL + Vector locally**; only the final on-printer service install and the §7.1 target-hardware profiling need Trident.

**Tech Stack:** VictoriaLogs, Vector (`file` source → `http`/`vector` sink), LogsQL over HTTP/curl, a repo `.claude/skills/query-logs/SKILL.md`, Python (klippy reactor timer) for self-observability, systemd units + install docs.

---

## Schema the store/skill must match (from Stage 1/2, already shipping)

One NDJSON object per line in `~/printer_data/logs/events/host-py.jsonl` and `host-rust.jsonl`. VictoriaLogs reserves `_time`, `_msg`, `_stream`, `_stream_id`; everything else is a free, fully-indexed field.

Core: `_time` (RFC3339 ms `Z`), `_msg`, `level` (`trace|debug|info|warn|error`), `source` (`host-py|host-rust|mcu-*|sim`), `subsystem`, `session_id` (`k-<unix>-<pid>`), `target`. Optional: `print_id` (always present, `""` when idle), `event`, `code`/`code_name`, typed payload (numbers stay numbers).

**Stream mapping (spec §5):** `_stream = {source, subsystem}`. One physical file per `source`; `subsystem` varies per line within a file. `session_id`/`print_id`/`event`/`code` are normal indexed fields.

---

## Conventions for all tasks

- The local test rig (Task 1) lives under a **gitignored** `.tools/observability/` in the worktree (binaries + a scratch data dir). Never commit binaries or VL data.
- Verify the host self-observability code with `PYTHONPATH=.:klippy python3 -m pytest test/<file> -q` and ruff (`/opt/homebrew/bin/ruff`, line-length 80).
- The end-to-end gate (Task 7) runs real VL + Vector locally and asserts records round-trip by `session_id`/`subsystem`/`event`.
- Commit after each task. **No `Co-Authored-By` trailer.** Fail-loudly: surface errors with clear codes; no silent recovery.
- Pin the VictoriaLogs + Vector versions actually fetched into the install docs (Task 4/6).

---

### Task 1: Local VL + Vector test rig (gitignored)

**Files:**
- Modify: `.gitignore` (add `.tools/`)
- Create: `.tools/observability/` (binaries + scratch; not committed)
- Create: `docs/kalico-rewrite/observability/README-local-testrig.md` (how to reproduce the local rig)

- [ ] **Step 1: Ignore the tools dir**

Add to `.gitignore` (repo root):
```
# Local observability test rig (VictoriaLogs + Vector binaries + scratch data)
.tools/
```
Run: `git check-ignore .tools/x` → expect `.tools/x`.

- [ ] **Step 2: Fetch the binaries (darwin-arm64)**

Fetch the single static binaries into `.tools/observability/bin/`. VictoriaLogs release asset is `victoria-logs-darwin-arm64-v*.tar.gz` (binary `victoria-logs-prod`); Vector ships a `vector-aarch64-apple-darwin.tar.gz`. Use `curl -L` from the official GitHub releases / vector.dev. Record the exact versions fetched. Verify each runs:
```bash
.tools/observability/bin/victoria-logs-prod --version
.tools/observability/bin/vector --version
```
Expected: version strings print. If a download fails (network), STOP and report — the rest of the rig depends on it. (Fallback: `brew install vectordotdev/brew/vector` for Vector; VictoriaLogs has no brew formula, must be the release binary.)

- [ ] **Step 3: Launch VictoriaLogs locally**

```bash
.tools/observability/bin/victoria-logs-prod \
  -storageDataPath=.tools/observability/vl-data \
  -httpListenAddr=127.0.0.1:9428 \
  -retentionPeriod=30d &
```
Verify it answers:
```bash
curl -s http://127.0.0.1:9428/health    # expect: OK-ish / 200
```
Document the start command in the README. Leave VL running for Tasks 3/7 (or restart as needed).

- [ ] **Step 4: Smoke-ingest one line directly (sanity, before Vector)**

```bash
printf '%s\n' '{"_time":"2026-06-01T12:00:00.000Z","_msg":"rig smoke","level":"info","source":"sim","subsystem":"observability","session_id":"k-rig-1","print_id":"","event":"rig.smoke","target":"testrig"}' \
| curl -s -X POST -H 'Content-Type: application/stream+json' --data-binary @- \
  'http://127.0.0.1:9428/insert/jsonline?_stream_fields=source,subsystem&_time_field=_time&_msg_field=_msg'
sleep 1
curl -s 'http://127.0.0.1:9428/select/logsql/query' --data-urlencode 'query=event:=rig.smoke _time:1h' --data-urlencode 'limit=10'
```
Expected: the query returns the smoke record (NDJSON, one JSON object). This confirms the VL ingest/query endpoints + the `_stream_fields`/`_time_field`/`_msg_field` parameters (which Vector will replicate). **Confirm the exact param names against the fetched VL version** (the ingest URL params are version-stable but verify).

- [ ] **Step 5: Write the README + commit**

Write `docs/kalico-rewrite/observability/README-local-testrig.md` documenting the fetch URLs, versions, and the VL/Vector start commands so the rig is reproducible. Commit:
```bash
git add .gitignore docs/kalico-rewrite/observability/README-local-testrig.md
git commit -m "docs(observability): local VL+Vector test-rig setup (gitignored binaries)"
```

---

### Task 2: Vector config (tail events/*.jsonl → VL)

**Files:**
- Create: `config/observability/vector.toml` (committed; the deployable config)
- Test: validated via `vector validate` + used live in Task 7

- [ ] **Step 1: Write the Vector config**

Create `config/observability/vector.toml`. It tails the events dir, parses each line as JSON (the line already IS the schema — no transform needed beyond JSON decode), and ships to VL `/insert/jsonline`. Keep rotated files uncompressed (spec §8). Use a real on-disk checkpoint dir.

```toml
# Vector config for the kalico observability pipeline.
# Tails the host's durable JSONL (the source of truth) and ships to a local
# VictoriaLogs. The JSONL files survive plug-pull; Vector resumes from its
# on-disk checkpoint so restarts neither lose nor duplicate lines.
#
# Rotated JSONL files MUST stay uncompressed (spec §8): Vector cannot seek into
# a gzipped file and would lose any un-shipped tail.
data_dir = "/var/lib/vector"   # checkpoint dir; override per-host in the unit

[sources.kalico_events]
type = "file"
include = ["/home/pi/printer_data/logs/events/*.jsonl"]   # override per-host
read_from = "beginning"
# one JSON object per physical line (Stage 1/2 guarantee)
[sources.kalico_events.decoding]
codec = "json"

[sinks.victorialogs]
type = "http"
inputs = ["kalico_events"]
uri = "http://127.0.0.1:9428/insert/jsonline?_stream_fields=source,subsystem&_time_field=_time&_msg_field=_msg"
method = "post"
encoding.codec = "json"
framing.method = "newline_delimited"
request.headers.Content-Type = "application/stream+json"
# Backpressure, not drop: never silently lose lines (fail-loud posture).
buffer.type = "disk"
buffer.max_size = 268435488   # 256 MiB on-disk buffer
buffer.when_full = "block"
```

Notes for the implementer: confirm the `http` sink + `application/stream+json` framing produces exactly the NDJSON VL expects (one object per line, `_time`/`_msg`/`_stream_fields` honored). If the `http` sink's JSON encoding double-wraps or reorders in a way VL rejects, the alternative is Vector's native VL sink if the fetched Vector version ships one; otherwise tune `encoding`/`framing`. The Task 7 e2e test is the arbiter.

- [ ] **Step 2: Validate**

```bash
.tools/observability/bin/vector validate config/observability/vector.toml
```
Expected: config valid (it may warn about the absolute include path not existing on the dev box — that's fine; Task 7 points it at a real local events dir via an override). If `validate` hard-fails on the path, use a dev override file in `.tools/` for Task 7 and keep the committed config with the deploy paths.

- [ ] **Step 3: Commit**

```bash
git add config/observability/vector.toml
git commit -m "feat(observability): Vector config tailing events/*.jsonl into VictoriaLogs"
```

---

### Task 3: The `query-logs` skill

**Files:**
- Create: `.claude/skills/query-logs/SKILL.md`

Model the frontmatter/format on the existing `.claude/skills/kalico-sim/SKILL.md`. The skill is the primary agent interface (spec §10): VL endpoint + conventions, **every recipe time-bounded**, a LogsQL cookbook keyed to the schema, NDJSON output parsing, and a pointer to `mcp-victorialogs` as an optional drop-in.

- [ ] **Step 1: Read the existing skill for format**

Read `.claude/skills/kalico-sim/SKILL.md` to match the frontmatter (`name`, `description`) and house style.

- [ ] **Step 2: Write the skill**

Create `.claude/skills/query-logs/SKILL.md` with:
- Frontmatter: `name: query-logs`, `description:` (when to use — "query the structured klippy/Rust host logs in VictoriaLogs by session/subsystem/level/event/print").
- **Endpoint + conventions:** `VL=${KALICO_VL:-http://127.0.0.1:9428}`; query via `curl -s "$VL/select/logsql/query" --data-urlencode 'query=...' --data-urlencode 'limit=N'`; output is NDJSON (one JSON object per line) — pipe through `jq -c` per line, do not `jq` the whole body.
- **Mandatory time bound on every recipe** (`_time:1h`, `_time:24h`, …) so the agent never full-scans.
- **Recipe cookbook** keyed to the schema:
  - errors in a session: `session_id:=k-… level:in(warn,error) _time:6h | sort by (_time)`
  - one print's homing: `print_id:=print-… subsystem:=homing _time:24h`
  - an event type over time: `event:=step_queue_overflow _time:7d | stats by (code_name) count()`
  - latest of a subsystem: `subsystem:=clocksync _time:1h | sort by (_time) desc | limit 50`
  - host-rust vs host-py split: `source:=host-rust _time:1h` / `source:=host-py _time:1h`
  - resolve a numeric code: `code:65228 _time:30d | fields code_name, _msg | limit 5`
  - free-text fallback: `"needs rehome" _time:1h`
  - **observability self-check** (Task 5): `subsystem:=observability event:=heartbeat _time:10m | sort by (_time) desc | limit 1` — if this returns nothing, the pipeline (Vector→VL) is stalled even though the printer is logging.
- **The session/print discovery recipes:** "find the current session" = `* _time:1h | stats by (session_id) count() | sort by (count) desc`; "list prints today" = `print_id:!="" _time:24h | stats by (print_id) min(_time), max(_time)`.
- **Output parsing note** + a one-paragraph **`mcp-victorialogs`** pointer (optional drop-in, same VL endpoint, no code from us).
- A short "how the records get here" note pointing at the Vector config + the schema, so a future reader understands the durability model.

- [ ] **Step 3: Validate the recipes against the live rig**

With VL running (Task 1) and a few records ingested (Task 1 Step 4 / Task 7), run 3-4 of the cookbook queries verbatim and confirm they return sensible NDJSON. Fix any LogsQL syntax that the fetched VL version rejects (LogsQL `:=` exact-match, `:in(...)`, `| stats by`, `| sort by` are version-stable, but verify).

- [ ] **Step 4: Commit**

```bash
git add .claude/skills/query-logs/SKILL.md
git commit -m "feat(observability): query-logs skill (time-bounded LogsQL cookbook)"
```

---

### Task 4: VictoriaLogs systemd unit + retention

**Files:**
- Create: `config/observability/victorialogs.service` (systemd unit)
- Create: `config/observability/vector.service` (systemd unit)

These are the on-printer deploy units (not run on macOS; authored + reviewed here, exercised on Trident later).

- [ ] **Step 1: Write the VictoriaLogs unit**

Create `config/observability/victorialogs.service`:
```ini
[Unit]
Description=VictoriaLogs (kalico observability store)
After=network.target

[Service]
Type=simple
User=pi
ExecStart=/usr/local/bin/victoria-logs-prod \
  -storageDataPath=/var/lib/victorialogs \
  -httpListenAddr=127.0.0.1:9428 \
  -retentionPeriod=30d \
  -retention.maxDiskSpaceUsageBytes=2GB
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
Retention: 30d OR ~2 GB, whichever hits first (spec §8/§16). Bound to localhost. Note in a comment that to run VL on a separate box, change `-httpListenAddr` to `:9428` and point Vector's sink URI at that host (spec §4 / the user's remote-logs requirement).

- [ ] **Step 2: Write the Vector unit**

Create `config/observability/vector.service`:
```ini
[Unit]
Description=Vector (kalico observability shipper)
After=network.target victorialogs.service

[Service]
Type=simple
User=pi
ExecStart=/usr/local/bin/vector --config /etc/kalico/vector.toml
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 3: Commit**

```bash
git add config/observability/victorialogs.service config/observability/vector.service
git commit -m "feat(observability): systemd units for VictoriaLogs + Vector"
```

---

### Task 5: Host self-observability — heartbeat + Vector liveness/lag

**Files:**
- Create: `klippy/extras/log_observability.py`
- Modify: `klippy/printer.py` (add `log_observability` to the on-by-default extras list ~line 331-336)
- Test: `test/test_log_observability.py` (Create)

A small always-on component (spec §12): (a) emits a periodic synthetic **heartbeat** structured record (`subsystem=observability event=heartbeat`) the agent/operator can query to confirm end-to-end health; (b) checks the **Vector checkpoint lag** (bytes behind file EOF) and that Vector is alive, surfacing staleness loudly (a `warn` in the text log + a queryable `subsystem=observability` event). A silent pipeline stall is itself a reportable fault.

- [ ] **Step 1: Write the failing tests**

Create `test/test_log_observability.py`:
```python
# Tests for the log_observability component: heartbeat emission and Vector
# checkpoint-lag detection. No live VL/Vector — the lag check reads checkpoint
# state from a function injected for the test.
import logging

import pytest

from klippy import structured_log
from klippy.extras import log_observability as lo


class CaptureHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture(autouse=True)
def _reset():
    structured_log.clear_print()
    structured_log.bind_session("k-test-1")
    yield
    structured_log.clear_session()


def test_heartbeat_emits_observability_event():
    cap = CaptureHandler()
    logging.getLogger("kalico.event").addHandler(cap)
    try:
        lo.emit_heartbeat()
    finally:
        logging.getLogger("kalico.event").removeHandler(cap)
    rec = next(r for r in cap.records if getattr(r, "event", None) == "heartbeat")
    assert rec.subsystem == "observability"


def test_lag_within_threshold_is_ok():
    # bytes_behind below threshold -> no warning, returns False (not stale)
    assert lo.check_lag(bytes_behind=1024, threshold=1_048_576) is False


def test_lag_over_threshold_is_flagged():
    # bytes_behind over threshold -> stale=True
    assert lo.check_lag(bytes_behind=5_000_000, threshold=1_048_576) is True
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTHONPATH=.:klippy python3 -m pytest test/test_log_observability.py -q`
Expected: FAIL (module missing).

- [ ] **Step 3: Implement the component**

Create `klippy/extras/log_observability.py`:
```python
# Host self-observability for the structured-logging pipeline (spec §12).
#
# Because "the store is down" is exactly the case that cannot self-report, the
# host emits a periodic heartbeat record (queryable to confirm end-to-end
# health) and checks that Vector is keeping up (checkpoint lag bounded). A
# silent pipeline stall is itself a reportable fault — surfaced loudly.
import logging
import os

from .. import structured_log

HEARTBEAT_INTERVAL = 30.0          # seconds between heartbeats (tunable)
LAG_CHECK_INTERVAL = 60.0          # seconds between lag checks
LAG_THRESHOLD_BYTES = 8 * 1024 * 1024   # bytes-behind-EOF before "stale"


def emit_heartbeat():
    # One structured heartbeat record. Querying for it confirms the whole
    # path (emit -> jsonl -> Vector -> VL) is live.
    structured_log.event(
        "observability", "heartbeat", level=logging.INFO, msg="pipeline heartbeat"
    )


def check_lag(bytes_behind, threshold=LAG_THRESHOLD_BYTES):
    # Pure predicate: True == stale (lag exceeds threshold). Kept side-effect
    # free so it is unit-testable; the component wraps it with logging.
    return bytes_behind > threshold


class LogObservability:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.events_dir = self.printer.get_start_args().get("log_events_dir")
        self._last_stale = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        now = self.reactor.monotonic()
        self.reactor.register_timer(self._heartbeat_timer, now + HEARTBEAT_INTERVAL)
        self.reactor.register_timer(self._lag_timer, now + LAG_CHECK_INTERVAL)

    def _heartbeat_timer(self, eventtime):
        emit_heartbeat()
        return eventtime + HEARTBEAT_INTERVAL

    def _vector_bytes_behind(self):
        # Best-effort: compare the events files' total size against Vector's
        # checkpoint offsets. Returns None if checkpoint state is unavailable
        # (Vector not installed / not running) -> treated as "unknown", not
        # stale. Concrete checkpoint parsing is refined against the deployed
        # Vector version in Task 7; the contract is "bytes behind EOF or None".
        if not self.events_dir or not os.path.isdir(self.events_dir):
            return None
        # Placeholder for the checkpoint diff; Task 7 wires the real Vector
        # checkpoint path. Until then, return None (unknown).
        return None

    def _lag_timer(self, eventtime):
        behind = self._vector_bytes_behind()
        if behind is not None:
            stale = check_lag(behind)
            if stale and not self._last_stale:
                logging.warning(
                    "observability: Vector shipper lagging %d bytes behind "
                    "events files — logs may not be reaching VictoriaLogs",
                    behind,
                )
                structured_log.event(
                    "observability", "shipper_lag", level=logging.WARNING,
                    msg="vector shipper lagging", bytes_behind=behind,
                )
            self._last_stale = stale
        return eventtime + LAG_CHECK_INTERVAL


def load_config(config):
    return LogObservability(config)
```

In `klippy/printer.py`, add `"log_observability"` to the on-by-default extras list (the `for section_config in ["force_move", "respond", "exclude_object", "telemetry"]:` block ~line 331):
```python
        for section_config in [
            "force_move",
            "respond",
            "exclude_object",
            "telemetry",
            "log_observability",
        ]:
```

- [ ] **Step 4: Run tests to pass + ruff**

Run: `PYTHONPATH=.:klippy python3 -m pytest test/test_log_observability.py -q` → PASS.
Run: `/opt/homebrew/bin/ruff check klippy/extras/log_observability.py test/test_log_observability.py && /opt/homebrew/bin/ruff format --check klippy/extras/log_observability.py test/test_log_observability.py` → clean.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/log_observability.py klippy/printer.py test/test_log_observability.py
git commit -m "feat(observability): host heartbeat + Vector lag self-observability"
```

---

### Task 6: Stage-1 §16-item-11 remainder — last-gasp on bg-thread sink failure

**Files:**
- Modify: `klippy/queuelogger.py` (the `QueueListener` bg-thread failure path)
- Test: `test/test_queuelogger_pipeline.py` (extend)

Stage 1 made `stop()` re-raise the captured `_bg_exc` (so shutdown fails loudly). The remaining piece (spec §16 item 11): when a sink write fails on the bg thread mid-run, write a **last-gasp operator message** through the surviving channel (the original `OSError` to stderr / the text log) so a mid-run sink failure is reported proactively, not only at the next log call or shutdown.

- [ ] **Step 1: Write the failing test**

Extend `test/test_queuelogger_pipeline.py` with a test that injects a sink whose `emit_record` raises `OSError`, drives one record through the bg thread, and asserts a last-gasp message reaches stderr (capture via `capsys`) AND `_bg_exc` is set. Use the existing test harness patterns in that file.

- [ ] **Step 2: Run to verify failure → implement → pass**

Implement in `queuelogger.py`: in the bg-thread drain loop's exception handler (where `_bg_exc` is captured), additionally emit a single last-gasp line to `sys.stderr` (and, if the text sink is still healthy, through it) with the original exception — guarded so it fires once (don't spam every record). Keep the existing re-raise-from-`stop()` behavior. Run the test to PASS; run the full Stage 1 queue tests to confirm no regression.

- [ ] **Step 3: ruff + commit**

```bash
/opt/homebrew/bin/ruff check klippy/queuelogger.py test/test_queuelogger_pipeline.py
git add klippy/queuelogger.py test/test_queuelogger_pipeline.py
git commit -m "feat(observability): last-gasp operator message on bg-thread sink failure"
```

Then update spec §16 item 11: mark the "Remaining for Stage 3" last-gasp piece DONE (heartbeat/lag liveness done in Task 5; last-gasp done here). Commit the spec edit.

---

### Task 7: End-to-end local round-trip (the gate)

**Files:**
- Create: `test/observability/e2e_local.sh` (a scripted local round-trip; not a pytest, a documented manual/CI script)

This is the strongest hardware-free verification: real VL + real Vector + real host JSONL, queried via the skill's recipes.

- [ ] **Step 1: Drive real host logs into a local events dir**

Use a scratch events dir under `.tools/observability/events/`. Generate records two ways:
1. **Python host:** a short `PYTHONPATH=.:klippy python3` snippet that calls `structured_log.bind_session(...)`, sets up the `jsonl` sink pointed at the scratch dir (reuse `log_sinks.JsonlSink`), and emits a handful of `structured_log.event(...)` records across subsystems/levels + one `emit_heartbeat()`.
2. **Rust host:** reuse the Stage 2 integration path — point `motion_bridge_native.logging.init_logging` at the scratch events dir and emit a couple events (a tiny Rust harness or the existing integration test pointed at the scratch dir).

- [ ] **Step 2: Run Vector against the scratch dir → local VL**

Start Vector with a dev-override config (the committed `vector.toml` with `include` pointed at `.tools/observability/events/*.jsonl` and `data_dir` under `.tools/`). Confirm Vector tails and ships.

- [ ] **Step 3: Query via the skill recipes and assert**

Run several `query-logs` cookbook queries against the local VL and assert:
- records come back filtered by `session_id`, `subsystem`, `event`;
- both `source:=host-py` and `source:=host-rust` records are present;
- numeric fields are queryable as numbers (e.g. `trigger_mm:>10`);
- the heartbeat is findable (`subsystem:=observability event:=heartbeat`);
- a record with an embedded newline/quote in `_msg` came through as exactly one record (sanitization end-to-end).

- [ ] **Step 4: Durability check**

Kill VL mid-run, emit more records, restart VL, confirm Vector backfills from its checkpoint (no loss, no dup) — spec §14 durability test, run locally.

- [ ] **Step 5: Commit the script + a results note**

```bash
git add test/observability/e2e_local.sh
git commit -m "test(observability): local end-to-end VL+Vector round-trip script"
```

---

## Trident-only follow-ups (documented, not done here)

- **On-printer install:** copy binaries to `/usr/local/bin`, install the two systemd units + `vector.toml` (paths adjusted to `~/printer_data/logs/events`), enable + start. Add an install doc step. Needs Trident free.
- **§7.1 target-hardware profiling:** the default text+jsonl fan-out cost on the Pi under a representative high-speed print, before declaring the default config safe. Needs Trident + a real print.
- **Wire the real Vector checkpoint path** into `_vector_bytes_behind` (Task 5) once the deployed Vector `data_dir` layout is confirmed on the Pi.

## Self-Review (completed during planning)

- **Spec coverage:** §8 (Vector tail→VL, uncompressed rotation, retention) → Tasks 1,2,4; §10 (query-logs skill, time bounds, NDJSON, mcp pointer) → Task 3; §12 (heartbeat + Vector liveness/lag, last-gasp) → Tasks 5,6; §14 (integration + durability) → Task 7; §16 item 11 remainder → Task 6.
- **Trident-free:** Tasks 1-7 all run/verify on macOS via the local rig. Only the on-printer install + §7.1 profiling are deferred (explicitly listed).
- **No placeholders that hide work:** the one acknowledged placeholder (`_vector_bytes_behind` returning None until the real checkpoint path is wired) is called out as a Trident-only follow-up, and the heartbeat (the primary liveness signal) is fully functional without it.
- **Type/name consistency:** `emit_heartbeat()`, `check_lag(bytes_behind, threshold)`, `LogObservability`, `log_events_dir` start-arg (matches Stage 2), `subsystem=observability`/`event=heartbeat`/`event=shipper_lag` used consistently across Tasks 3/5.
