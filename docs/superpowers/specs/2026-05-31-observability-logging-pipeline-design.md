# Observability Logging Pipeline — Foundation Design

- **Date:** 2026-05-31
- **Branch / worktree:** `observability` (`.worktrees/observability`, off `sota-motion`)
- **Status:** Design — pending user review
- **Scope:** Host-side foundation only. MCU log endpoint and UI are explicit follow-on specs (see §15).

---

## 1. Problem

Logging today is a flat, unstructured text pile that is hard to search and impossible to slice. Confirmed by a codebase recon (Python host, Rust host, MCU C/Rust, comms bridge, sim):

- **Python host (klippy):** stdlib `logging` → async queue → rotating **plain-text** file. ~480 call sites. Records carry **no timestamp, no level, no module** in the emitted line. "Finding the log" means grepping ad-hoc string prefixes (`[bridge-trace]`, `[probe-homing]`, `[diag]`, `[config-send]`). No correlation id ties a print, command, or homing op together.
- **Rust host:** `log` crate + `env_logger`, but the klippy/systemd wrapper **swallows stderr**, so the code works around it with hardcoded `/tmp/*.log` append-writes (`cax-trace.log`, `interceptor_trace.log`, `kalico-firewire.log`). Scattered, unrotated, **silent on write failure** (`let _ = writeln!`), and **wiped on reboot** — violating the project rule that diagnostics must survive a plug-pull.
- **MCU (out of scope here, but constrains the schema):** bare integer fault codes (e.g. `65228`), three overlapping channels (`output()` strings, `StatusHeartbeat`, `FaultEvent`), persistent BKPSRAM/`.persistent_diag` fault capture.
- **Bridge:** a structured event pipeline already exists — `RuntimeEvent` (`host-rt`) carries `Status / Fault / Trace / EndstopTripped / Heartbeat / UnknownOutput` through a 1 ms poller, and a clock-sync already maps MCU ticks → host wall-time. **`RuntimeEvent::Trace` is decoded and then silently dropped at the bridge** — a ready-made hook for the MCU follow-on.

**Goal:** one structured, durable, queryable log pipeline whose primary consumer is an AI agent (querying per session / subsystem / level / event), with a human-readable text view retained for back-compat, and a path to a dashboard UI later.

## 2. Goals / Non-goals

**Goals (this spec):**
1. A single structured logging pipeline for the **Python host** and **Rust host**. No parallel legacy logging code path: emitters produce **one** structured record, consumed by a set of pluggable sinks.
2. **Pluggable sink architecture:** a registerable sink interface (klippy `extras` style) with built-in `text` and `jsonl` sinks; future/third-party sinks (a direct Loki sink, remote syslog, a Moonraker bridge) plug in without core changes.
3. **Plug-pull durability:** the on-disk JSONL record is the source of truth, written under `~/printer_data/logs/`.
4. A **queryable store** (VictoriaLogs) the agent can drive via a repo-committed **skill** (LogsQL over HTTP), with an optional first-party MCP. VL is an *external opt-in* fed by the `jsonl` sink.
5. **Stock-format text view:** the `text` sink emits the stock Klipper `klippy.log` format, so existing log analytics (`logextract.py`, `graphstats.py`, Klippain, Moonraker's log view) keep working in every configuration.
6. **Noise control:** per-subsystem default levels.
7. A schema that the MCU follow-on can feed into unchanged.

**Non-goals (deferred):**
- **User-facing sink/tier selection config** (e.g. `[logging] backend = klipper|structured|victorialogs`, disabling the `jsonl` sink to save SSD writes, runtime level changes). The sink interface is *built for* this, but the config surface is a deliberate follow-on — this spec hard-wires the default active set (`text` + `jsonl`).
- MCU-side structured log emission / transport (spec #2).
- Any Grafana / Mainsail dashboard work (spec #3).
- Migrating historical log files into VictoriaLogs (we start fresh; "no old logs").
- Replacing the realtime motion/telemetry frames (`StatusHeartbeat`, `FaultEvent`) — those stay; this is about *logs*, not control telemetry.

## 3. Decisions (locked with the user)

| Decision | Choice | Rationale |
|---|---|---|
| Store | **VictoriaLogs** | Light on the Pi, indexes **every** field (per-session/per-event queries are first-class — Loki's label model is not), HTTP+LogsQL the agent can curl, built-in UI, Grafana plugin later, Apache-2.0. |
| Agent interface | **Repo-committed `query-logs` skill** (LogsQL + curl), `mcp-victorialogs` optional | User prefers a skill over MCP; VL has both. `mcp-victorialogs` verified first-party + lightweight (needs only the VL HTTP endpoint). |
| Ingestion | **File-first + shipper** | JSONL on disk = durable source of truth (survives plug-pull); VL is a rebuildable index; the realtime path never blocks on the store. |
| Session model | **`session_id` + `print_id`**, `op_id` reserved | Scope to a whole boot or one print now; per-command/op tracing later with no schema change. |
| Migration | **Facade backend-swap + structured helper** | Instant blanket coverage of all ~480 sites with zero call-site edits, plus a clean forward API for new/hot-path code. No big-bang, no throwaway. |
| Legacy text log | **Kept as a derived rendering** of the structured stream (the `text` sink) | One source of records, two views. Preserves Mainsail/sim/fetch-to-tmp without a second logging path. |
| Sink architecture | **Pluggable sink registry** (built now); built-in `text` + `jsonl` | Generalizes "two views" into N sinks; this is the "make the logger a plugin" requirement. |
| Default active sinks | **`text` + `jsonl`** | Agent-queryable out of the box with no external processes; VL is an opt-in upgrade; SSD-light hosts drop to text-only later. |
| Sink/tier selection config | **Deferred** (interface designed for it) | User-facing toggle to disable sinks / drop to text-only for SSD savings comes later; not needed to ship value now. |

## 4. Architecture

Emitters produce one structured record; a **sink registry** fans it out to the active sinks (§4.1). This spec ships `text` + `jsonl` active by default. VictoriaLogs is an *external opt-in* fed by the `jsonl` sink — klippy needs no config to enable it, and a host that never installs it pays nothing.

```
  Python host (klippy)                         Rust host (motion-bridge / host-rt)
  stdlib logging ─ facade swap ─┐              log:: + tracing ─ subscriber ─┐
  structured_log.event(...) ────┤              klog!(...) ──────────────────┤
                                ▼                                            ▼
        ContextFilter: session_id, print_id, source, target, subsystem, level
                                │                                            │
                                ▼                                            ▼
                       ┌──────────────────── SINK REGISTRY ─────────────────────┐
                       │  active set selected by config (default: text + jsonl)  │
                       │   • text  sink  → klippy.log  (stock Klipper format)     │
                       │   • jsonl sink  → printer_data/logs/events/*.jsonl       │
                       │   • <custom>    → registerable (Loki, syslog, Moonraker) │
                       └─────────────────────────────────────────────────────────┘
                                                  │  jsonl files (durable truth)
                                                  ▼
                                shipper (Vector) — external, opt-in
                                  tails events/*.jsonl, checkpointed
                                                  │
                                                  ▼
                                  VictoriaLogs (127.0.0.1:9428)
                              /insert/jsonline · /select/logsql/query
                                                  │
            ┌──────────────────────────────────────┼──────────────────────────────┐
            ▼                                        ▼                              ▼
   query-logs skill (curl)              mcp-victorialogs (optional)         VL Web UI / Grafana
   primary agent interface               drop-in, same endpoint                  (spec #3)
```

### 4.1 Pluggable sinks

A **sink** is a small interface that consumes structured records and decides what to do with them. The registry holds the active set; emitters never reference sinks directly.

- **Built-in now:** `text` (stock `klippy.log` format — keeps existing analytics working) and `jsonl` (per-source files under `printer_data/logs/events/`, the durable source of truth).
- **Registerable:** new sinks register as klippy `extras` modules (klippy's existing extension model), so a direct Loki sink, remote syslog, or a Moonraker bridge can be added **without touching core**. This is the "logger as a plugin" requirement.
- **Cross-language:** the active-sink set is pushed across the FFI, so Rust `tracing` events (and decoded MCU frames, spec #2) land in exactly the active sinks. In a text-only configuration the JSON serialization + `jsonl` file I/O are never paid — that is the "lightweight on a Pi 3B" mode.
- **Selection is deferred config:** this spec hard-wires the default active set (`text` + `jsonl`). The user-facing switch to change/disable sinks (e.g. text-only to save SSD writes) is a documented follow-on the interface is built for (§2 non-goals, §16).

**Data-flow invariants:**
- The `jsonl` sink writes a record to its per-source file **before** anything downstream. That write is the durability point: if VL or the shipper is down, nothing is lost.
- Vector tails the JSONL files with an on-disk checkpoint — a restart neither re-sends nor loses lines.
- VictoriaLogs holds an **index**, not the source of truth; it can be wiped and rebuilt by re-tailing the JSONL.
- The `text` and `jsonl` sinks render the *same* record objects — never separate logging calls.

## 5. Log record schema

One JSONL line = one event. VictoriaLogs reserves `_time`, `_msg`, `_stream`, `_stream_id`; all other fields are free and fully indexed.

**Core fields (every record):**

| Field | Type | Meaning |
|---|---|---|
| `_time` | RFC3339 (host wall clock) | event time |
| `_msg` | string | human message (default LogsQL full-text target) |
| `level` | enum: `trace`/`debug`/`info`/`warn`/`error` | severity |
| `source` | enum: `host-py`/`host-rust`/`mcu-h7`/`mcu-f4`/`sim` | emitter |
| `subsystem` | string: `motion`/`homing`/`bridge`/`clocksync`/`mcu-comms`/`probe`/`temp`/`config`/… | logical area (replaces the `[bridge-trace]`-style prefixes) |
| `session_id` | string `k-<unix>-<pid>` | one per klippy lifecycle |
| `target` | string | Python logger name (`module.Class`) or Rust module path (`motion_bridge::probe_homing`) |

**Optional fields:**

| Field | Type | Meaning |
|---|---|---|
| `print_id` | string (empty when idle) | set for the duration of a print |
| `op_id` | string (reserved) | finer correlation (one gcode command / homing move); populated opportunistically later |
| `event` | string stable key (`homing.endstop_trip`, `step_queue_overflow`) | the "log type" — filterable regardless of message wording |
| `code` / `code_name` | int / string | numeric fault/event code + resolved symbol (kills "what is 65228?") |
| *payload* | typed | any structured fields: `axis`, `velocity`, `gcode_line`, `oid`, … |

**Stream selection:** `_stream = {source, subsystem}` (low-cardinality grouping for compression/fast narrowing). `session_id`, `print_id`, `event`, `code` stay normal fields — VL indexes them efficiently, which is the entire reason we are not on Loki.

**Examples**

```json
{"_time":"2026-05-31T14:02:11.482Z","_msg":"endstop trip on Z at 12.40mm",
 "source":"host-rust","subsystem":"homing","level":"info",
 "session_id":"k-1748700131-4412","print_id":"",
 "event":"homing.endstop_trip","axis":"z","trigger_mm":12.40,
 "target":"motion_bridge::probe_homing"}
```
```json
{"_time":"2026-05-31T14:18:55.701Z","_msg":"G1 move queued",
 "source":"host-py","subsystem":"motion","level":"debug",
 "session_id":"k-1748700131-4412","print_id":"print-1748700500",
 "event":"motion.move_queued","gcode_line":1843,"target":"gcode.GCodeDispatch"}
```

## 6. Session / print correlation

- **`session_id`** is generated once at klippy startup: `k-<unix_seconds>-<pid>`. It is the backbone correlation id stamped on every record from every source.
- **Python:** bound via `contextvars` at startup (`session_id`) and at print start/end (`print_id`); a `ContextFilter` injects them into every `LogRecord`. `contextvars` is async-safe and correct under klippy's reactor.
- **Rust host:** `session_id` and current `print_id` are passed across the PyO3 FFI at bridge init / on print-state change and held in a process-global the `tracing` layer reads, so every Rust record carries the same ids without threading them through call signatures.
- **MCU (follow-on):** MCU frames do **not** carry `session_id`. When the host decoder re-emits a decoded MCU log into the JSONL stream, it **stamps** the current `session_id`/`print_id`. (Prior-boot persistent-diag records emitted at startup get the current `session_id` plus a `mcu_prior_boot:true` marker and the MCU boot id — detail for spec #2.)

## 7. Emission design

### 7.1 Python host

Keep the existing async design (`QueueHandler` → background `QueueListener`); it is non-blocking and good. Change only what is behind the facade:

1. **`ContextFilter`** — injects `session_id`, `print_id`, `source="host-py"`, and `target` (logger name) into every record from contextvars. Added at the root logger (`printer.py` setup).
2. **`jsonl` sink** — renders the record to a schema-conformant JSON line; promotes `extra=` keys to top-level fields; maps `levelno`→`level`; uses record creation time for `_time`. Writes to `printer_data/logs/events/host-py.jsonl` (rotating). The durable source of truth.
3. **`text` sink** — renders the *same* record to the **stock Klipper** `klippy.log` line format (back-compat with existing analytics). Same records, never a separate logging call.
4. Both sinks are entries in the **`SinkRegistry`**; the formatted output of the async `QueueListener` is fanned out to the active set (default `{text, jsonl}`).
5. **`structured_log.py`** — thin forward helper: `event(subsystem, event, *, level="info", msg=None, **fields)` calls stdlib logging with `extra=`. New/hot-path code uses this; it can **require** `subsystem`+`event` (fail-loudly). Existing `logging.*` calls keep working untouched and still get `_time`/`level`/`source`/`target`/`session_id`/`print_id` for free.
6. **Incremental enrichment:** the ad-hoc `[bridge-trace]`/`[probe-homing]`/`[diag]` prefixes get migrated to real `subsystem`/`event` fields in the high-traffic paths (homing, bridge, clocksync) over time. No big-bang.

No new hard dependency: stdlib `logging` + `contextvars` + a JSON serializer. (structlog considered; rejected for the backbone because the facade-swap needs zero call-site changes and fewer deps. It remains an option if ergonomics demand it.)

### 7.2 Rust host

Replace `env_logger` with a `tracing` stack (this is real Rust work — implement via the `rust-engineer` subagent):

- `tracing` + `tracing-subscriber` (json) + `tracing-appender` (non-blocking rolling file) + `tracing-log` (capture existing `log::*` macros so no call-site edits).
- A custom `Layer` injects `source="host-rust"`, `session_id`, `print_id`, `target` (module path), `subsystem` (from a span field or target mapping).
- Output: `printer_data/logs/events/host-rust.jsonl` (rotating, non-blocking).
- **Retire** the hardcoded `/tmp/*.log` writes and `eprintln!` diagnostics → `tracing::event!` with explicit `subsystem`/`event` fields. These are the only meaningful Rust edits and they are exactly the diagnostics that should be structured.
- **Fail-loudly:** silent `let _ = writeln!(...)` is removed; write/flush failures are surfaced (counter + a warn-level event, or a hard error per the project's fail-loudly default — to be settled in the plan).
- `klog!` macro = the Rust twin of `structured_log.event` for new/hot-path code.

## 8. Ingestion / shipper / store

VictoriaLogs and Vector are **external, opt-in** components fed by the `jsonl` sink's files. klippy needs no configuration to enable them; a host that doesn't install them is unaffected and pays nothing. "Turning on VL" = installing/running Vector + VL pointed at the JSONL files — not a klippy code path.

- **Shipper:** **Vector** (single Rust binary, robust checkpointed `file` source, backpressure). Config tails `printer_data/logs/events/*.jsonl` and pushes NDJSON to VL `/insert/jsonline`. *Alternatives considered:* Fluent Bit (lighter C footprint) and a VL-native agent — to be confirmed at plan time; Vector is the default for checkpoint durability and Rust-stack fit.
- **VictoriaLogs:** runs as a systemd service, bound to `127.0.0.1:9428`. Ingest `/insert/jsonline`; query `/select/logsql/query` (NDJSON out).
- **VL data placement (SD-wear option):** default on-disk data dir under `printer_data/`. Because the JSONL files are the durable source of truth, an SD-wear-sensitive option is to put the **VL index on tmpfs (RAM)** and let Vector re-ingest from the JSONL on boot — zero index writes to SD, durability preserved. Default off (on-disk) for simplicity; documented as a toggle.
- **Retention:** VL `-retentionPeriod=30d` **and** a disk-usage cap (e.g. `-retention.maxDiskSpaceUsageBytes≈2GB`), whichever hits first. JSONL files: size-based rotation (e.g. 32 MB × 5, gzip old). Defaults; tunable.

## 9. Noise control

- Global default level `info`. `trace`/`debug` retained on disk only when enabled (so default ingest volume stays modest → less SD/RAM pressure).
- **Per-subsystem default levels** (config), mirrored to both Python and Rust (the Rust level is pushed across FFI). A runtime `SET_LOG_LEVEL SUBSYSTEM=… LEVEL=…` command and live sink toggling are part of the **deferred configuration follow-on** (§2 non-goals, §16) — the static defaults ship now.
- Because filtering is now a *field query*, "noise" is also handled at read time: the agent narrows by `subsystem`/`level`/`event` instead of grepping.

## 10. The `query-logs` skill

Repo-committed skill (primary agent interface). Contents:
- VL endpoint + auth conventions.
- LogsQL recipe cookbook keyed to the schema, e.g.:
  - errors in a session: `session_id:=k-… level:in(warn,error) | sort by (_time)`
  - one print's homing: `print_id:=print-… subsystem:=homing`
  - an event type across time: `event:=step_queue_overflow | stats by (code_name) count()`
  - free-text fallback: `"needs rehome"`
- Output parsing note (NDJSON, one JSON object per line).
- A pointer to `mcp-victorialogs` as an optional drop-in (same VL endpoint, no code from us).

## 11. Component boundaries

Each unit has one purpose, a defined interface, and explicit deps:

| Unit | Purpose | Interface | Depends on |
|---|---|---|---|
| `structured_log.py` | Python forward API + context binding | `event(...)`, `bind_session/print(...)` | stdlib logging, contextvars |
| `SinkRegistry` | hold the active sink set, fan out records | `register(sink)`, `emit(record)` | — |
| `text` sink | render stock `klippy.log` | sink interface | — |
| `jsonl` sink | render durable per-source JSONL | sink interface | schema |
| `ContextFilter` | enrich records (session/print/source/target) | logging filter API | contextvars |
| Rust `tracing` layer + `klog!` | Rust emission + enrichment | `tracing` macros, FFI ctx setter | tracing crates |
| FFI session bridge | pass session/print ids Python→Rust | `extern "C"` setter at bridge init | PyO3 boundary |
| Vector config | tail JSONL → VL | files in, HTTP out | Vector |
| VL service | store + query | HTTP | systemd |
| `query-logs` skill | agent query recipes | LogsQL over curl | VL HTTP |

## 12. Failure modes (fail-loudly)

- **VL down / shipper down:** emitters keep writing JSONL; nothing lost; Vector catches up from its checkpoint. No emitter ever blocks on VL.
- **JSONL write failure (disk full / perms):** surfaced, not swallowed (replaces today's silent `let _ = writeln!`). Policy (warn+counter vs hard error) settled in the plan per fail-loudly default.
- **Shipper backlog:** Vector backpressure; monitored via VL ingest metrics.
- **Schema drift:** the structured helper enforces required fields; malformed `extra=` is caught at format time.

## 13. kalico-sim integration

`kalico-sim` currently greps the text `klippy.log` and injects `[sim-trace]`/`[sim-diag]` markers. Because the text view is preserved, existing grep assertions keep working. New, more precise assertions can query the JSONL directly (or a sim-local VL), and `source="sim"` separates simulator runs. Sim-trace markers become `subsystem=sim` / `event=…` fields over time.

## 14. Testing strategy

- **Unit:** `jsonl` sink schema conformance; `text` sink stock-format conformance; `ContextFilter` injection; level gating; Rust layer field injection; code→code_name resolution.
- **Integration:** emit (Python + Rust) → JSONL on disk → Vector → VL → `query-logs` round-trip returns the expected records by `session_id`/`subsystem`/`event`.
- **Durability:** kill VL mid-run, confirm JSONL intact and VL backfills on restart (and the tmpfs-rebuild path if enabled).
- **Sim:** existing grep-based `kalico-sim` assertions still pass against the derived text log.

## 15. Out of scope → follow-on specs

- **Spec #2 — MCU log endpoint:** a C-owned structured log frame in the kalico protocol, written by C *and* the Rust staticlib (respecting the C-owns-shared-memory boundary), decoded host-side into this same schema — reusing the already-present-but-dropped `RuntimeEvent::Trace` channel and the existing tick→walltime clock-sync. This foundation's schema and host re-emit path are designed to receive it unchanged.
- **Spec #3 — UI:** VL built-in Web UI → Grafana (VL datasource plugin) → optional Mainsail panel.

## 16. Open items (defaults chosen; flag to steer)

1. **Shipper:** Vector (default) vs Fluent Bit vs VL-native agent — confirm at plan time.
2. **Retention numbers:** 30d / ~2 GB cap, 32 MB×5 JSONL rotation — tunable defaults.
3. **VL index on tmpfs vs disk:** default disk; tmpfs toggle for SD-wear.
4. **Write-failure policy:** warn+counter vs hard error (fail-loudly leaning).
5. **session_id format:** `k-<unix>-<pid>` (sortable, greppable).
6. **Sink-selection config surface (deferred follow-on):** when added, a `[logging]` section selects the active sink set (e.g. `backend = klipper|structured|victorialogs` or an explicit sink list), enables runtime `SET_LOG_LEVEL`, and lets SSD-constrained hosts drop to text-only. This spec builds the registry/interface so this is a small, additive change — no rework.
