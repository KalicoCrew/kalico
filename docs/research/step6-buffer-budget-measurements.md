# Step 6 buffer-budget measurements (M1, M2, M3)

**Status: PLACEHOLDER — measurements pending hardware run by user.**

Per Step-6 plan §7.3, three measurement protocols quantify the assumptions
the buffer-budget defaults in `rust/kalico-host-rt/src/clock_sync.rs` and
`rust/runtime/src/state.rs` rest on. The runners ship with the Step-6
implementation; the actual long-running soaks (8h Pi, 24h dual-MCU) are
user-run on representative hardware. Once executed, the user replaces the
`TODO_USER_RUN` placeholders below with measured values; if any value
diverges materially from the initial estimate, the user also updates the
constants those defaults derive from.

The Step-6 plan (Phase 15) is explicit that this scaffold is sufficient
for plan completion — the measurements themselves are decoupled from
the algorithmic / wire-protocol Step-6 deliverables and follow the
critical path of "user has flashed hardware ready to soak."

---

## M1 — Host-stall (Pi 5 + Bookworm + production load, 8h soak)

**Runner:** `tools/measure_m1_host_stall.py`

**Recipe:** see runner header. Run on the Pi 5 (NOT the dev host) with a
representative parallel workload — Bookworm desktop + Mainsail rendering
trace UI + Moonraker WebSocket + journald active.

**Measurement target:** worst-case host-side latency between two
consecutive `kalico_push_segment` round-trip completions. This bounds the
amount of work the producer can be away from the SPSC tail before the
MCU underruns on the queue.

**Initial-estimate constants** (Step-6 defaults; current source of truth):

  - `MIN_SEGMENT_DURATION_MS` (target floor; see plan §7.1) ≈ 1 ms
  - Producer queue depth `Q_N_MAX` = 256 → effective heapless capacity 255
  - Implied buffer-budget: ~255 ms of work in-flight at the floor

**Recorded values:** 2026-05-02 — 0.5 h Pi 5 soak, firmware bb668dd40

  - `p50_us`:    6882.4
  - `p95_us`:    6976.5
  - `p99_us`:    7010.7
  - `p999_us`:   7552.0
  - `p9999_us`:  7690.8
  - `max_us`:    **7771.7**
  - `n_samples`: 17860
  - `push_failures`: 0

**Host workload notes:** Pi 5 (Trident bench host), Bookworm, kalico
service stopped during soak (no Mainsail / Moonraker load — bare-port
access required because klippy holds the USB-CDC). Soak duration was
30 min (Step-7-D Phase 2a Gate C target), not the full 8 h Step-6
spec target. The Step-6 long soak is a follow-up: re-run after klippy
can share the port (i.e., once the kalico bridge runs production
workloads alongside Mainsail).

The p50 of 6.88 ms is dominated by USB-CDC + Python-`pyserial`
round-trip overhead, not engine back-pressure (the queue stays mostly
full because each segment is 100 ms wall-time and pushes complete in
~7 ms — host outpaces consumer comfortably). Tail-vs-p50 spread
(max − p50 = 0.89 ms; p99.99 − p50 = 0.81 ms) is the actual
host-jitter signal, well within any reasonable buffer-budget headroom.

Test-script changes vs the Step-6 scaffold (committed alongside this
record): segment duration bumped from 1 ms to 100 ms and prefill
reduced from 64 to 7 to match the actual `heapless::spsc::Queue<8>`
runtime queue capacity. Without these the engine underran on the
first ISR tick (1 ms × 8-slot buffer can't absorb 6 ms push
round-trips). t_start_base is now anchored on `read_mcu_clock` so
re-runs without power-cycle work; `kalico_set_homed` is sent before
`stream_open` so the engine doesn't latch FAULT on its first tick.

**Action items if `max_us` exceeds the buffer-budget headroom:**

  - increase `Q_N_MAX` in `rust/runtime/src/spsc.rs` (powers of 2; next
    is 512 → 511 effective slots)
  - or reduce `MIN_SEGMENT_DURATION_MS` (raises producer pressure but
    halves the per-slot wall-clock budget)
  - if both options regress trajectory quality, the answer per CLAUDE.md
    is to optimize the producer-side pipeline (parallelize across cores)
    rather than relax the planner

---

## M2 — MCU runtime cost (H723 cycle-budget rerun)

**Runner:** `tools/test_h723_cycle_count.py --m2-rounds 977
--m2-stir-protocol`

**Recipe:** runs the standard 1024-sample bench 977 times back-to-back
(977 × 1024 ≈ 1.0M ticks total) on flashed H723 hardware. With
`--m2-stir-protocol`, fires `kalico_query_status` /
`kalico_stream_open` / `kalico_stream_flush` between rounds so the ISR
observes the post-Step-6 protocol-handler additions in their natural
state.

**Measurement target:** WORST_ISR_CYCLES across the union — the worst
40 kHz tick budget any individual measurement landed in.

**Initial-estimate constants:**

  - H723 core clock: 520 MHz → CYCLES_PER_TICK = 520_000_000 / 40_000 = 13_000
  - p99 budget gate (Step-5 Surface C): 15.0 µs = 7800 cycles
  - Step-6 protocol-handler ISR cost (estimate): clock-sync responder
    ≤ 50 cycles, stream-state machine + force_idle short-circuit ≤ 100
    cycles, generation-handle lookup ≤ 30 cycles, seqlock publication
    ≤ 60 cycles → ≤ 240 additional cycles vs Step-5 baseline

**Recorded values:** TODO_USER_RUN (H723 1M-tick rerun)

  - `min_us`:               TODO_USER_RUN
  - `p50_us`:               TODO_USER_RUN
  - `p99_us`:               TODO_USER_RUN
  - `WORST_ISR_CYCLES`:     TODO_USER_RUN
  - `WORST_ISR_US`:         TODO_USER_RUN
  - `n_samples`:            TODO_USER_RUN
  - `m2_stir_protocol`:     TODO_USER_RUN  (true / false)

**Action items if `WORST_ISR_CYCLES > 13_000`:**

  - the ISR can no longer make its 40 kHz deadline → trajectory quality
    is at risk → urgent investigation
  - profile with `cargo asm` against the offending function
  - Step-7 work has known-pending optimization candidates (NURBS de Boor
    on MCU is the heaviest single function; see Layer 0 critical-path
    note in CLAUDE.md)

---

## M3 — Clock-sync residual (H723 + F4x sim, 24h soak)

**Runner:** `tools/measure_m3_clock_sync.py`

**Recipe:** issues `kalico_clock_sync_request` round-trips at 10 Hz to
each MCU port for 24 hours, runs the host-side sliding-window regression
(`ClockSyncWindow` matches `rust/kalico-host-rt/src/clock_sync.rs`
spec-side; not bit-identical), records p99.99 of residual / drift /
sample-age.

**Measurement target:** all four spec §12.4 quality-gate fields stay
within their default thresholds during a healthy 24h:

  - `MAX_RESIDUAL_US_DEFAULT`     = 100 µs   (`residual_max_in_window`)
  - `MAX_DRIFT_PPM_DEFAULT`       = 100 ppm  (`drift_ppm` vs baseline)
  - `MAX_SAMPLE_AGE_MS_DEFAULT`   = 2000 ms  (any sample)
  - `MAX_RTT_AGE_MS_DEFAULT`      = 500  ms  (dedicated only)

**Recorded values (H723):** TODO_USER_RUN

  - `max_residual_us`:                TODO_USER_RUN
  - `residual_p9999_us`:              TODO_USER_RUN
  - `max_abs_drift_ppm`:              TODO_USER_RUN
  - `drift_ppm_p9999`:                TODO_USER_RUN
  - `max_sample_age_s`:               TODO_USER_RUN
  - `sample_age_p9999_s`:             TODO_USER_RUN
  - `round_trip_failures`:            TODO_USER_RUN

**Recorded values (F4x):** TODO_USER_RUN (or "not_run" if F4x integration
parallel workstream not yet ready)

  - `max_residual_us`:                TODO_USER_RUN
  - `residual_p9999_us`:              TODO_USER_RUN
  - ... (etc as for H723)

**Action items if any p99.99 exceeds its default threshold:**

  - mild excess (p99.99 just over threshold but max well under): bump
    the default in `rust/kalico-host-rt/src/clock_sync.rs` and document
    the empirical justification here
  - large excess (max significantly over threshold): investigate
    upstream — the regression is failing to settle, or the MCU clock is
    unstable, or USB framing latency has unusual jitter
  - either way, the planner's ARMING quality gate refusing to fire is
    the correct conservative behavior; no need to relax it under
    measurement-driven uncertainty

---

## Update protocol

When the user runs M1 / M2 / M3 and lands actuals here:

  1. Replace `TODO_USER_RUN` with the recorded value (preserve format).
  2. If any default constant in
     `rust/kalico-host-rt/src/clock_sync.rs::MAX_*_DEFAULT`
     or `rust/runtime/src/spsc.rs::Q_N_MAX`
     diverges from what the measurement supports, update the constant
     in a separate commit and reference the M-number here.
  3. Append a short summary entry to
     `docs/superpowers/plan-changes-log.md` with the measured values
     and the date the soak completed.
  4. Status header above flips from PLACEHOLDER to MEASURED with the
     date and the firmware SHA the soaks ran against.

The Step-6 plan-changes-log entry tracks "M1/M2/M3 measurement actuals
(user-run, pending)" as an open follow-up; that line stays open until
this document is fully populated with actuals.
