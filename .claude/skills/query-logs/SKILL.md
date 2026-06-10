---
name: query-logs
description: Use when investigating the kalico host logs — finding why homing/a print/an MCU op failed, filtering by session or print or subsystem or level or event type, resolving a numeric fault code, checking whether the logging pipeline itself is healthy, or comparing host-py vs host-rust behavior. Queries the structured logs in VictoriaLogs via LogsQL over curl, instead of grepping klippy.log.
---

# Querying kalico structured logs (VictoriaLogs / LogsQL)

The kalico host writes structured NDJSON to `~/printer_data/logs/events/*.jsonl`
(durable, plug-pull-safe source of truth). Vector ships it into **VictoriaLogs**,
which indexes **every** field — so instead of grepping a flat log you query by
`session_id` / `print_id` / `subsystem` / `level` / `event` / numeric payload.

## Endpoint + how to query

```bash
VL="${KALICO_VL:-http://127.0.0.1:9428}"   # set KALICO_VL to a remote box if VL runs off-printer

curl -s "$VL/select/logsql/query" \
  --data-urlencode 'query=<LogsQL here>' \
  --data-urlencode 'limit=50'
```

- **Output is NDJSON** — one JSON object per line. Parse per line (`... | jq -c .`),
  do NOT pipe the whole body to one `jq` filter.
- **Always include a time bound** on every query so you never full-scan the store.
  Relative bounds are anchored to *now*: `_time:1h`, `_time:6h`, `_time:24h`, `_time:7d`.
  Absolute range (rarely needed): `_time:[2026-06-01T00:00:00Z, 2026-06-02T00:00:00Z]`.
- VL renders all field values as **strings** in the output (it's a logs store), but
  numeric range filters still work at query time (see the numeric recipe below).
- **Check VL is up BEFORE interpreting any result** (hard rule): `curl -s "$VL/health"`
  must print `OK`. With `curl -s`, a dead VL produces **empty output that is
  indistinguishable from "no matching records"** — on 2026-06-10 this made an
  uninstalled VL read as "clean boot" for weeks. An empty query result is only
  meaningful after a passing health check; a non-zero curl exit or empty `/health`
  means *pipeline down*, and the answer lives in the raw JSONL instead.

## The schema (fields you filter on)

Core (every record): `_time`, `_msg`, `level` (`trace|debug|info|warn|error`),
`source` (`host-py|host-rust|mcu-h7|mcu-f4|sim`), `subsystem`, `session_id`
(`k-<unix>-<pid>`), `target`. Optional: `print_id` (empty when idle), `event`
(stable key like `homing.endstop_trip`), `code`/`code_name`, plus typed payload
(`axis`, `trigger_mm`, `gcode_line`, `oid`, …). `_stream = {source, subsystem}`.

LogsQL operators you'll use: `field:=value` (exact), `field:in(a,b)` (set),
`field:>N` / `field:<N` (numeric range), `field:!=""` (present), `"text"`
(full-text on `_msg`), `| sort by (_time)` / `desc`, `| fields a, b`, `| limit N`,
and `| stats by (f) count() as n` — **alias aggregates** (`as n`) so you can then
`| sort by (n)`; `sort by (count(*))` does not parse.

## Recipe cookbook

```bash
VL="${KALICO_VL:-http://127.0.0.1:9428}"
q(){ curl -s "$VL/select/logsql/query" --data-urlencode "query=$1" --data-urlencode "limit=${2:-50}"; }

# --- orient yourself ---
# current/recent sessions (most active first). NOTE: alias aggregates (`as hits`)
# — LogsQL cannot `sort by (count(*))` directly:
q '* _time:1h | stats by (session_id) count() as hits | sort by (hits) desc'
# prints in the last day with their start/end:
q 'print_id:!="" _time:24h | stats by (print_id) min(_time) as first, max(_time) as last'

# --- investigate a failure ---
# all warnings+errors in a session, oldest first:
q 'session_id:=k-1748700131-4412 level:in(warn,error) _time:6h | sort by (_time)'
# one print's homing activity:
q 'print_id:=print-1748700500 subsystem:=homing _time:24h | sort by (_time)'
# latest of a noisy subsystem:
q 'subsystem:=clocksync _time:1h | sort by (_time) desc' 50

# --- by event type / code ---
# how often an event fired, grouped by resolved code name:
q 'event:=step_queue_overflow _time:7d | stats by (code_name) count() as n | sort by (n) desc'
# resolve "what is 65228?":
q 'code:65228 _time:30d | fields code_name, _msg' 5
# numeric range (works despite string rendering):
q 'subsystem:=homing trigger_mm:>10 _time:24h'

# --- host-py vs host-rust ---
q 'source:=host-rust level:in(warn,error) _time:1h | sort by (_time)'
q 'source:=host-py  subsystem:=motion       _time:1h | sort by (_time) desc' 100

# --- free-text fallback (searches _msg) ---
q '"needs rehome" _time:1h'
```

## Is the pipeline itself healthy?

"VL is down / Vector stalled" is exactly the case that can't self-report through
the logs. The host emits a heartbeat every ~30 s (`subsystem=observability`,
`event=heartbeat`). If this returns **nothing**, the printer may still be logging
to disk but the records aren't reaching VL — check Vector + VL, not the printer:

```bash
q 'subsystem:=observability event:=heartbeat _time:10m | sort by (_time) desc' 1
# a logged shipper-lag warning, if any:
q 'subsystem:=observability event:=shipper_lag _time:6h | sort by (_time) desc' 5
```

Service-level check (run on the printer; both must be `active`):

```bash
systemctl is-active victorialogs vector
journalctl -u victorialogs -u vector --no-pager | tail -20   # why, if not
```

On the printers the stack lives in `~/observability/` (binaries
`victoria-logs-prod` v1.50.0 + `vector` 0.55.0, unit files, `vector.toml`,
data dirs) with the units symlinked into systemd via
`sudo systemctl enable --now /home/<user>/observability/*.service` —
chosen because the printer sudoers only passwordless-allows `systemctl`.
If the units don't exist at all, the stack was never installed on that
host: deploy per `config/observability/*.service` headers (adapt `User=`
and the `/home/pi/printer_data` path in `vector.toml`).

The on-disk JSONL is the source of truth regardless — if VL is empty but a print
just failed, the records are still in `~/printer_data/logs/events/*.jsonl` and VL
will backfill from Vector's checkpoint once it's healthy.

## How records get here (context)

`structured_log.event(...)` / Rust `tracing` → `events/{host-py,host-rust}.jsonl`
(durable) → Vector tails + checkpoints → VL `/insert/jsonline` → you query
`/select/logsql/query`. See `config/observability/vector.toml` and the design
spec `docs/superpowers/specs/2026-05-31-observability-logging-pipeline-design.md`.

## Optional: mcp-victorialogs

If you prefer tool-calls over curl, `mcp-victorialogs` is a first-party MCP that
talks to the **same** VL endpoint (`$KALICO_VL`) — drop it in, no code from us.
The LogsQL recipes above are identical through the MCP.
