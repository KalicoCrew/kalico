# Observability Logging Pipeline — Foundation Design

- **Date:** 2026-05-31
- **Branch / worktree:** `observability` (`.worktrees/observability`, off `sota-motion`)
- **Base commit:** `84a28f5b9` (sota-motion HEAD at worktree creation)
- **Status:** Design — adversarially reviewed (multi-agent), review fixes applied; pending user approval to proceed to an implementation plan.
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
3. **Plug-pull durability:** the on-disk JSONL record is the source of truth, written under `~/printer_data/logs/`, with an explicit flush/durability and write-failure contract (§7, §12).
4. A **queryable store** (VictoriaLogs) the agent can drive via a repo-committed **skill** (LogsQL over HTTP), with an optional first-party MCP. VL is an *external opt-in* fed by the `jsonl` sink.
5. **Stock-format text view:** the `text` sink emits the stock Klipper `klippy.log` format, so existing log analytics (`logextract.py`, `graphstats.py`, Klippain, Moonraker's log view) keep working in every configuration.
6. **Noise control:** per-subsystem default levels, with known-noisy subsystems pre-gated (§9).
7. A schema that the MCU follow-on can feed into unchanged.

**Non-goals (deferred):**
- **User-facing sink/tier selection config** (e.g. `[logging] backend = klipper|structured|victorialogs`, disabling the `jsonl` sink to save SSD writes, runtime level changes). The sink interface is *built for* this, but the config surface is a deliberate follow-on — this spec hard-wires the default active set (`text` + `jsonl`).
- **tmpfs/RAM VL index** (SD-wear optimization) — moved to the deferred config follow-on (§16) to avoid the checkpoint/rebuild contradiction (was a review blocker).
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
| Session model | **`session_id` + `print_id`**, `op_id` reserved | Scope to a whole boot or one print now; per-command/op tracing later with no schema change (forward-compat contract in §5/§16). |
| Migration | **Facade backend-swap + structured helper** | Instant blanket coverage of all ~480 sites with zero call-site edits, plus a clean forward API for new/hot-path code. No big-bang, no throwaway. |
| Legacy text log | **Kept as a derived rendering** of the structured stream (the `text` sink) | One source of records, two views. Preserves Mainsail/sim/fetch-to-tmp without a second logging path. |
| Sink architecture | **Pluggable sink registry** (built now); built-in `text` + `jsonl` | Generalizes "two views" into N sinks; this is the "make the logger a plugin" requirement. |
| Default active sinks | **`text` + `jsonl`** | Agent-queryable out of the box with no external processes; VL is an opt-in upgrade; SSD-light hosts drop to text-only later. |
| Sink/tier selection config | **Deferred** (interface designed for it) | User-facing toggle to disable sinks / drop to text-only for SSD savings comes later; not needed to ship value now. |
| **Write-failure policy** | **Fail loudly — hard error on the first JSONL/Rust write or flush failure**, plus a pre-print disk-space preflight (§12) | CLAUDE.md fail-loudly is binding unless explicitly overridden; a silently-degraded durable store defeats the whole design. **Softening to warn-and-continue requires explicit user sign-off (not given).** |

> **Contract items are specified, not deferred.** The engineering contracts these decisions imply — JSONL flush/durability and write-failure handling (§7, §12), the queue-overflow policy (§7.1), the FFI session-context synchronization and binding-timing (§6), and pipeline self-liveness (§12) — are pinned in this spec. The only items deliberately deferred to a later config spec are user-facing sink/tier *selection*, runtime level changes, and the tmpfs index (§2 non-goals, §16). A few numeric defaults (retention sizes, the exact bounded-queue depth) are tunable and called out in §16.

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
- **Cross-language:** the active-sink set is pushed across the FFI, so Rust `tracing` events (and decoded MCU frames, spec #2) land in exactly the active sinks. In a text-only configuration the JSON serialization + `jsonl` file I/O are never paid — that is the "lightweight on a Pi 3B" mode. (The default text+jsonl cost is not assumed free; see the profiling requirement in §7.1.)
- **Selection is deferred config:** this spec hard-wires the default active set (`text` + `jsonl`). The user-facing switch to change/disable sinks (e.g. text-only to save SSD writes) is a documented follow-on the interface is built for (§2 non-goals, §16).

**Data-flow invariants:**
- The `jsonl` sink writes a record to its per-source file **before** anything downstream, and the durability contract (§7, §12) defines the flush behavior that makes that write the plug-pull-safe point. If VL or the shipper is down, nothing is lost.
- Vector tails the JSONL files with an on-disk checkpoint (byte offset). It resumes from the checkpoint, so on restart it neither re-sends nor loses lines — **subject to** the rotation/compression constraint in §8 (Vector cannot seek into a gzipped file, so rotated files are kept uncompressed by default).
- VictoriaLogs holds an **index**, not the source of truth. It can be rebuilt by re-tailing the JSONL — **bounded by the JSONL retention window**: records whose JSONL files have been rotated out and deleted (§8) are no longer in the index after a rebuild, but remain queryable only as long as their files exist. The unbounded "wipe and rebuild everything" framing is therefore scoped to the retained window.
- The `text` and `jsonl` sinks render the *same* record objects — never separate logging calls.

## 5. Log record schema

One JSONL line = one event, **always a single physical line** (the `jsonl` sink emits exactly one complete JSON object per line; embedded newlines/control chars in any value are JSON-escaped — see §7.1 sanitization). VictoriaLogs reserves `_time`, `_msg`, `_stream`, `_stream_id`; all other fields are free and fully indexed.

**Core fields (every record):**

| Field | Type | Meaning |
|---|---|---|
| `_time` | RFC3339 (host wall clock) | event time |
| `_msg` | string | human message (default LogsQL full-text target); never written raw — sanitized |
| `level` | enum: `trace`/`debug`/`info`/`warn`/`error` | severity |
| `source` | enum: `host-py`/`host-rust`/`mcu-h7`/`mcu-f4`/`sim` | emitter |
| `subsystem` | string: `motion`/`homing`/`bridge`/`clocksync`/`mcu-comms`/`probe`/`temp`/`config`/… | logical area (replaces the `[bridge-trace]`-style prefixes) |
| `session_id` | string `k-<unix>-<pid>` | one per klippy lifecycle |
| `target` | string | Python logger name (`module.Class`) or Rust module path (`motion_bridge::probe_homing`) |

**Optional fields:**

| Field | Type | Meaning |
|---|---|---|
| `print_id` | string (empty when idle) | set for the duration of a print |
| `op_id` | string (reserved) | finer correlation (one gcode command / homing move); populated opportunistically later (§16 forward-compat) |
| `event` | string stable key (`homing.endstop_trip`, `step_queue_overflow`) | the "log type" — filterable regardless of message wording |
| `code` / `code_name` | int / string | numeric fault/event code + resolved symbol (kills "what is 65228?") |
| *payload* | typed | any structured fields: `axis`, `velocity`, `gcode_line`, `oid`, … |

**Numeric fields stay JSON numbers** (e.g. `trigger_mm: 12.40`, not `"12.40"`) so LogsQL range/comparison queries (`trigger_mm:>10`) work.

**Stream / file mapping:** `source` maps to **both** the JSONL filename (`events/<source>.jsonl`) **and** the first `_stream` field; `subsystem` is the second `_stream` field. So `_stream = {source, subsystem}` (low-cardinality grouping for compression/fast narrowing), while one physical file per source carries many subsystems (the Rust case: a single `host-rust.jsonl` with `subsystem` varying per line). `session_id`, `print_id`, `event`, `code` stay normal fields — VL indexes them efficiently, which is the entire reason we are not on Loki.

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

- **`session_id`** is generated once at klippy startup: `k-<unix_seconds>-<pid>`. It is the backbone correlation id stamped on every record from every source. (pid reuse across reboots is possible but the unix-seconds prefix makes collisions negligible — acceptable.)
- **Binding-timing invariant (fail-loudly):** `session_id` is generated and bound — Python `contextvars` set, Rust process-global initialized — **before the first logging call on that side**. The Python root logger + `ContextFilter` is installed at the earliest startup point (before the first "Starting Klippy" line, which today fires before `Printer.__init__`); the Rust context is set at module/bridge init before any Rust log can fire. A record emitted before its side's `session_id` is bound is a schema violation and **fails loudly** (it must not silently emit an empty `session_id`).
- **Python:** `session_id` bound at startup, `print_id` bound at print start and cleared at print end, via `contextvars`; a `ContextFilter` injects them into every `LogRecord`. `contextvars` is async-safe and correct under klippy's reactor.
- **Rust host — synchronization contract:** `session_id` and current `print_id` are passed across the PyO3 seam (a `#[pymethods]` setter on the bridge — see §11; this is the host Python↔Rust seam, **not** the MCU `extern "C"` ABI) at bridge init and on print-state change. They are stored behind an **atomically-swapped `Arc<SessionContext>`** (an `arc_swap::ArcSwap` or equivalent). The `tracing` layer, which runs on multiple Rust threads (planner, pump, per-MCU clock-sync), does a single atomic load per record. Ordering guarantee: a record emitted concurrently with a `print_id` update deterministically observes **either the old or the new context, never a torn read**. Carrying the *old* `print_id` for records in flight during the swap is **acceptable and expected** (a print boundary is not instantaneous); this is stated so it is not mistaken for a bug. `session_id` never changes within a process lifetime, so it is race-free by construction.
- **MCU (follow-on):** MCU frames do **not** carry `session_id`. When the host decoder re-emits a decoded MCU log into the JSONL stream, it **stamps** the current `session_id`/`print_id`. (Prior-boot persistent-diag records emitted at startup get the current `session_id` plus a `mcu_prior_boot:true` marker and the MCU boot id — detail for spec #2.)

## 7. Emission design

### 7.1 Python host

Keep the existing async design (`QueueHandler` → background `QueueListener`); it is non-blocking and good. Change only what is behind the facade:

1. **`ContextFilter`** — injects `session_id`, `print_id`, `source="host-py"`, and `target` (logger name) into every record from contextvars. Added at the root logger (`printer.py` setup), honoring the binding-timing invariant (§6).
2. **`jsonl` sink** — renders the record to a schema-conformant JSON line; promotes `extra=` keys to top-level fields; maps `levelno`→`level`; uses record creation time for `_time`. Writes to `printer_data/logs/events/host-py.jsonl` (rotating). The durable source of truth.
3. **Sanitization (fail-loudly + injection-safe):** the `jsonl` sink serializes via a standard JSON encoder so **all** field values — including user-controlled `_msg` content from gcode comments, filenames, `M117`, macro output — are JSON-escaped. This guarantees one physical line per record and prevents a crafted gcode line (embedded newline) from forging a second JSONL record or breaking NDJSON parsing. `_msg` is never written raw.
4. **`text` sink** — renders the *same* record to the **stock Klipper** `klippy.log` line format (back-compat with existing analytics). Same records, never a separate logging call.
5. **Durability contract:** the `jsonl` sink flushes each record to the OS (`flush()`); the rotation handler **closes and flushes a file before it is rotated** so Vector never reads a partial last line at a boundary. Whether each line is `fsync`'d (strict plug-pull guarantee, higher SD cost) vs flushed-only (relies on OS writeback, cheaper) is a tunable in §16 — default flush-per-record + periodic fsync; the write-failure policy (§12) applies to both `write` and `flush`/`fsync` failures.
6. **Queue-overflow policy (fail-loudly):** the existing `QueueHandler` queue is made **bounded**; on overflow the pipeline does **not** silently drop (today's `put_nowait` + `handleError` swallow path is removed). It either blocks the producer briefly or fails loudly with a dropped-record counter that is itself surfaced — never silent loss. Exact depth + block-vs-fail is a tunable (§16).
7. **`structured_log.py`** — thin forward helper: `event(subsystem, event, *, level="info", msg=None, **fields)` calls stdlib logging with `extra=`. New/hot-path code uses this; it can **require** `subsystem`+`event` (fail-loudly). Existing `logging.*` calls keep working untouched and still get `_time`/`level`/`source`/`target`/`session_id`/`print_id` for free.
8. **Both sinks are entries in the `SinkRegistry`;** the formatted output of the async `QueueListener` is fanned out to the active set (default `{text, jsonl}`).
9. **Incremental enrichment:** the ad-hoc `[bridge-trace]`/`[probe-homing]`/`[diag]` prefixes get migrated to real `subsystem`/`event` fields in the high-traffic paths (homing, bridge, clocksync) over time. No big-bang.

**Default-config cost is not assumed free.** The fan-out of text+jsonl on the single listener thread must be **profiled on target hardware (Pi 3B/4) under a representative high-speed print** during implementation before the default config is declared safe. If profiling shows the listener falling behind or unacceptable SD wear, revisit the level defaults (§9) or the default active set (the user chose text+jsonl; do not silently reverse — flag).

No new hard dependency: stdlib `logging` + `contextvars` + a JSON serializer. (structlog considered; rejected for the backbone because the facade-swap needs zero call-site changes and fewer deps. It remains an option if ergonomics demand it.)

### 7.2 Rust host

Replace `env_logger` with a `tracing` stack (this is real Rust work — implement via the `rust-engineer` subagent):

- `tracing` + `tracing-subscriber` (json) + `tracing-appender` (non-blocking rolling file) + `tracing-log` (capture existing `log::*` macros so no call-site edits).
- A custom `Layer` injects `source="host-rust"`, `target` (module path), `subsystem` (from a span field or target mapping), and `session_id`/`print_id` via a single `ArcSwap<SessionContext>` load per event (the synchronization contract in §6).
- Output: `printer_data/logs/events/host-rust.jsonl` (rotating, non-blocking, rotated files uncompressed per §8).
- **Retire** the hardcoded `/tmp/*.log` writes and `eprintln!` diagnostics → `tracing::event!` with explicit `subsystem`/`event` fields. These are the only meaningful Rust edits and they are exactly the diagnostics that should be structured. (Cutover note: the `/tmp/*.log` writers are removed in the same change that introduces the `jsonl` output, so there is no window of double-logging or lost diagnostics.)
- **Fail-loudly:** silent `let _ = writeln!(...)` is removed; write/flush failures follow the §3/§12 hard-fail policy (surface a clear error; do not silently degrade the durable store).
- `klog!` macro = the Rust twin of `structured_log.event` for new/hot-path code.

## 8. Ingestion / shipper / store

VictoriaLogs and Vector are **external, opt-in** components fed by the `jsonl` sink's files. klippy needs no configuration to enable them; a host that doesn't install them is unaffected and pays nothing. "Turning on VL" = installing/running Vector + VL pointed at the JSONL files — not a klippy code path. *(Endpoints/flags below verified against VictoriaLogs docs; pin to the deployed VictoriaLogs version in the install docs at plan time.)*

- **Shipper:** **Vector** (single Rust binary, robust checkpointed `file` source, backpressure). Config tails `printer_data/logs/events/*.jsonl` and pushes NDJSON to VL `/insert/jsonline`. *Alternatives considered:* Fluent Bit (lighter C footprint) and a VL-native agent — to be confirmed at plan time; Vector is the default for checkpoint durability and Rust-stack fit.
- **Rotation/compression constraint (review fix):** rotated JSONL files are kept **uncompressed by default**, because Vector will not resume reading a file once it is gzipped (it cannot seek into gzip), which would lose any un-shipped tail. If compression is later wanted to save disk, it must use a delaycompress-style grace period that guarantees Vector has tailed the file to EOF before compression — documented as a constraint, not enabled now.
- **VictoriaLogs:** runs as a systemd service, bound to `127.0.0.1:9428`. Ingest `/insert/jsonline`; query `/select/logsql/query` (NDJSON out).
- **Retention:** VL `-retentionPeriod=30d` **and** a disk-usage cap (`-retention.maxDiskSpaceUsageBytes≈2GB`), whichever hits first. JSONL files: size-based rotation (e.g. 32 MB × 5, uncompressed). On a 32 GB SD card the disk-usage cap is the bound expected to dominate; confirm the desired dominant bound at install time. Defaults; tunable (§16).
- **tmpfs/RAM index:** *deferred to §16* (removed from this spec to avoid the checkpoint-vs-rebuild contradiction).

## 9. Noise control

- Global default level `info`. `trace`/`debug` are **dropped at emit** (cheapest — gated before record construction where possible, both Python `isEnabledFor` and Rust `tracing` level filters), not merely at the sink, so disabled levels cost nearly nothing.
- **Per-subsystem default levels** (config), mirrored to both Python and Rust (the Rust level is pushed across the FFI alongside the session context). **Known-noisy subsystems are pre-gated below `info`** to protect the default volume budget — initially `clocksync` and the heartbeat/status drain path pinned to `warn`. The explicit list lives here in §9 so it is reviewable and adjustable.
- A runtime `SET_LOG_LEVEL SUBSYSTEM=… LEVEL=…` command and live sink toggling are part of the **deferred configuration follow-on** (§2 non-goals, §16) — the static defaults ship now.
- Because filtering is now a *field query*, "noise" is also handled at read time: the agent narrows by `subsystem`/`level`/`event` instead of grepping.

## 10. The `query-logs` skill

Repo-committed skill (primary agent interface). Contents:
- VL endpoint + auth conventions.
- **Every recipe carries an explicit time bound** (e.g. `_time:1h`) so the agent never accidentally full-scans the store.
- LogsQL recipe cookbook keyed to the schema, e.g.:
  - errors in a session: `session_id:=k-… level:in(warn,error) _time:6h | sort by (_time)`
  - one print's homing: `print_id:=print-… subsystem:=homing _time:24h`
  - an event type across time: `event:=step_queue_overflow _time:7d | stats by (code_name) count()`
  - free-text fallback: `"needs rehome" _time:1h`
- Output parsing note (NDJSON, one JSON object per line).
- A pointer to `mcp-victorialogs` as an optional drop-in (same VL endpoint, no code from us).

## 11. Component boundaries

Each unit has one purpose, a defined interface, and explicit deps:

| Unit | Purpose | Interface | Depends on |
|---|---|---|---|
| `structured_log.py` | Python forward API + context binding | `event(...)`, `bind_session/print(...)` | stdlib logging, contextvars |
| `SinkRegistry` | hold the active sink set, fan out records | `register(sink)`, `emit(record)` | — |
| `text` sink | render stock `klippy.log` | sink interface | — |
| `jsonl` sink | render durable per-source JSONL (sanitized, flushed) | sink interface | schema |
| `ContextFilter` | enrich records (session/print/source/target) | logging filter API | contextvars |
| Rust `tracing` layer + `klog!` | Rust emission + enrichment | `tracing` macros, `ArcSwap<SessionContext>` load | tracing crates |
| Host session bridge | pass session/print ids Python→Rust-host | **PyO3 `#[pymethods]` setter** (host Python↔Rust seam; *not* the MCU `extern "C"` ABI) → `ArcSwap<SessionContext>` | PyO3 boundary |
| Vector config | tail JSONL → VL | files in, HTTP out | Vector |
| VL service | store + query | HTTP | systemd |
| `query-logs` skill | agent query recipes | LogsQL over curl | VL HTTP |

## 12. Failure modes (fail-loudly)

- **Write-failure policy (locked, §3):** the first JSONL write / flush / fsync failure (disk full, perms, I/O error) is a **hard error with a clear code** — it is not swallowed and the durable store is not silently degraded. This replaces today's silent `let _ = writeln!`.
- **Disk-space preflight:** a pre-print check refuses to start a print when free space under `printer_data/logs/` is below a reserve threshold, so a disk-full logging failure is caught *before* mid-print rather than interrupting a running job. Threshold is a tunable (§16).
- **VL down / shipper down:** emitters keep writing JSONL; nothing lost; Vector catches up from its checkpoint (subject to the uncompressed-rotation constraint, §8). No emitter ever blocks on VL.
- **Queue overflow:** bounded queue, no silent drop (§7.1) — block or fail-loud with a surfaced counter.
- **Pipeline self-observability (fail-loudly):** because "VL is down" is exactly the case that cannot self-report, the host runs a lightweight liveness check: (a) a periodic **synthetic heartbeat record** the agent/operator can query to confirm end-to-end health, and (b) a host-side check that the Vector process is alive and its checkpoint lag (bytes behind file EOF) is bounded; staleness/lag beyond threshold is surfaced loudly (warning in the text log + a queryable `subsystem=observability` event). A silent pipeline stall is itself a reportable fault.
- **Schema drift:** the structured helper enforces required fields; malformed `extra=` is caught at format time.

## 13. kalico-sim integration

`kalico-sim` currently greps the text `klippy.log` and injects `[sim-trace]`/`[sim-diag]` markers. Because the text view is preserved, existing grep assertions keep working. New, more precise assertions can query the JSONL directly (or a sim-local VL), and `source="sim"` separates simulator runs. Sim records also carry the **sim git SHA** (as a field) so runs are separable. Sim-trace markers become `subsystem=sim` / `event=…` fields over time.

## 14. Testing strategy

- **Unit:** `jsonl` sink schema conformance; **sanitization** (a record whose `_msg`/fields contain embedded newlines, quotes, and control chars yields exactly one valid JSON line, no forged second record); `text` sink stock-format conformance; `ContextFilter` injection; level drop-at-emit gating; Rust layer field injection; code→code_name resolution; queue-overflow fail-loud (no silent drop); write-failure hard-error path.
- **Concurrency:** Rust `ArcSwap<SessionContext>` — a log emitted concurrently with a `print_id` swap always carries a coherent (old-or-new, never torn/empty) context; binding-timing invariant (no record before `session_id` bound).
- **Integration:** emit (Python + Rust) → JSONL on disk → Vector → VL → `query-logs` round-trip returns the expected records by `session_id`/`subsystem`/`event`.
- **Durability:** kill VL mid-run, confirm JSONL intact and VL backfills on restart from the checkpoint; confirm rotated (uncompressed) files are still picked up; disk-full triggers the hard-error + preflight, not silent loss.
- **Sim:** existing grep-based `kalico-sim` assertions still pass against the derived text log.

## 15. Out of scope → follow-on specs

*(Filenames reserved so forward-references don't dangle: `2026-…-mcu-log-endpoint-design.md`, `2026-…-observability-ui-design.md`.)*

- **Spec #2 — MCU log endpoint:** a C-owned structured log frame in the kalico protocol, written by C *and* the Rust staticlib (respecting the C-owns-shared-memory boundary in `docs/kalico-rewrite/mcu-c-rust-boundary.md`), decoded host-side into this same schema — reusing the already-present-but-dropped `RuntimeEvent::Trace` channel and the existing tick→walltime clock-sync. This foundation's schema and host re-emit path are designed to receive it unchanged.
- **Spec #3 — UI:** VL built-in Web UI → Grafana (VL datasource plugin) → optional Mainsail panel.

## 16. Open items (defaults chosen; flag to steer)

1. **Shipper:** Vector (default) vs Fluent Bit vs VL-native agent — confirm at plan time.
2. **Retention numbers:** 30d / ~2 GB cap, 32 MB×5 JSONL rotation (uncompressed) — tunable defaults; confirm dominant bound for the target media.
3. **Durability strength:** flush-per-record + periodic fsync (default) vs fsync-per-record (strict, higher SD cost) — tunable.
4. **Queue bound:** depth + block-vs-fail-loud on overflow — tunable.
5. **Disk-space preflight reserve:** the free-space threshold below which a print is refused — tunable.
6. **session_id format:** `k-<unix>-<pid>` (sortable, greppable).
7. **op_id forward-compat (reserved):** the JSONL schema validator MUST treat an absent/unknown `op_id` as valid so adding it later is non-breaking; when populated, `op_id` is generated host-side at the gcode-command / homing-move boundary and (for spec #2) threaded to the MCU — the threading cost is acknowledged now, not hidden.
8. **tmpfs/RAM VL index (deferred from §8):** when added in the config follow-on, it requires a boot path that resets Vector's checkpoint to byte 0 when the index is empty so the index is fully re-shipped from JSONL (otherwise a wiped tmpfs index rebuilds only post-checkpoint). Until then, on-disk index only.
9. **Sink-selection config surface (deferred follow-on):** a `[logging]` section selects the active sink set (e.g. `backend = klipper|structured|victorialogs` or an explicit sink list), enables runtime `SET_LOG_LEVEL`, and lets SSD-constrained hosts drop to text-only. This spec builds the registry/interface so this is a small, additive change — no rework.
10. **Per-subsystem default level map (§9):** initial noisy-subsystem pins (`clocksync`, heartbeat → `warn`) — reviewable/adjustable.

## 17. Implementation staging (recommended decomposition)

Per the review, this is **decomposed into three staged implementation plans** (each independently testable, matching the project's incremental, test-first norm), rather than one monolithic plan:

1. **Stage 1 — Python host structured logging.** `SinkRegistry`, `ContextFilter`, `jsonl` + `text` sinks, `structured_log.py`, session/print binding, sanitization, bounded queue, durability/write-failure policy. Delivers standalone value: queryable JSONL + stock text log with zero external services. Independently testable.
2. **Stage 2 — Rust host `tracing` swap.** Replace `env_logger`, capture `log::*`, `ArcSwap<SessionContext>` via PyO3, retire `/tmp` + `eprintln!`, `klog!`. Carries distinct tooling/risk (FFI, async threads).
3. **Stage 3 — VL/Vector deployment + `query-logs` skill + self-observability.** External-process install, Vector config, retention, heartbeat/liveness, the skill. Distinct ops/tooling surface.

MCU log endpoint (spec #2) and UI (spec #3) follow as their own brainstorm→spec→plan cycles.
