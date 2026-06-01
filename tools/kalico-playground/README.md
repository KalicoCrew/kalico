# Kalico Playground

A MainsailOS-style sandbox: our klippy + simulated H7/F4 MCUs driven through the
**Mainsail** web UI, with the structured logs flowing into **VictoriaLogs** so
you can watch them light up as you click. No printer hardware required — it runs
the real MCU firmware in the kalico-sim, so it exercises the actual host +
firmware code paths and the full observability pipeline (Stages 1–3).

```
┌─ printer (one container, supervisord) ─────────────┐   ┌─ vector ─┐   ┌─ victorialogs ─┐
│  klippy (our branch) + sim H7/F4 MCUs  ── api.sock ─┼──▶│  tail    │──▶│  store + LogsQL │
│  Moonraker ── /printer_data/klippy.sock             │   │ events/  │   │  :9428 (vmui)   │
│  nginx + Mainsail  :80                              │   └──────────┘   └─────────────────┘
└─────────────────────────────────────────────────────┘
   events/*.jsonl (host-py + host-rust, shared volume) ──┘
```

## Build

Two images: the kalico-sim firmware/klippy image for the branch, then this
playground image (Moonraker + Mainsail) layered on top.

```bash
# 1. Build the sim image for the branch under test (compiles MCU firmware +
#    Rust staticlib + klippy). Tags it kalico-sim-<branch>:
bash tools/kalico-sim/run.sh --branch observability

# 2. Build the playground image on top of it:
docker build -t kalico-playground \
  --build-arg BASE=kalico-sim-observability:latest \
  -f tools/kalico-playground/Dockerfile tools/kalico-playground
```

## Run

```bash
docker compose -f tools/kalico-playground/docker-compose.yml up -d
```

| What | URL |
|---|---|
| **Mainsail** (click around) | http://localhost:8080 |
| **VictoriaLogs UI** (explore logs) | http://localhost:9428/select/vmui |
| Moonraker API (direct) | http://localhost:7125 |

Reset (wipe logs + state): `docker compose -f tools/kalico-playground/docker-compose.yml down -v`

## Exploring the logs (VictoriaLogs)

In the VL UI, set the time range to **Last 15 minutes**, then paste a LogsQL
filter into the query box:

| Goal | Query |
|---|---|
| Everything, newest first | `* \| sort by (_time) desc` |
| Just the Python host | `source:=host-py` |
| Just the Rust host | `source:=host-rust` |
| Only problems | `level:in(warn,error)` |
| The fault chain (why klippy dropped) | `source:=host-rust level:in(warn,error)` |
| One boot, start→fault | `session_id:=k-…` (copy an id from any expanded line) |

The repo's `query-logs` skill has the full cookbook. Click any log line to
expand it and see all fields (`source`, `level`, `session_id`, `subsystem`, …).

## Known reality

- **klippy auto-cycles ~every 30s.** Our motion code can't sustain operation
  yet, so the MCU transport faults under a sustained idle hold (`EXIT_ON_FAULT`).
  supervisor restarts klippy, so you get repeated ~30s interactive windows and
  the logs continuously capture the fault chain — which is exactly the
  observability story this tooling exists to show. This ceiling is the motion
  code's maturity, not the playground.
- **`RUST_LOG` pin** in the compose mutes a debug-grade pump/bridge heartbeat
  (`PIECEDIAG`) that otherwise floods at `info`. The clean fix is lowering those
  to `debug` in the Rust source; the pin is the no-rebuild stopgap.
- **Python logs have no `subsystem` yet** (they log via the root logger) —
  filter them by `source:=host-py` / `level` / free-text. Promoting the
  high-traffic paths to `subsystem`/`event` fields is a follow-up.

## How it's wired

- `tools/kalico-sim/runner.py --serve --data-dir /printer_data` keeps klippy +
  the sim MCUs alive long-lived (the `--serve` mode), exposing klippy's API
  socket for Moonraker. `--serve` also appends Mainsail-friendly config sections
  (input_shaper/pause_resume/macros) — interactive only; full/batch sim modes
  keep the plain minimal config.
- `moonraker.conf` points `klippy_uds_address` at `/printer_data/klippy.sock`
  (`[machine] provider: none` — no systemd in the container).
- `nginx-mainsail.conf` serves Mainsail + proxies the Moonraker API/websocket.
- `vector.toml` tails `/printer_data/logs/events/*.jsonl` → VictoriaLogs (same
  shape as the deployable `config/observability/vector.toml`).
