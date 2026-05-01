# Step 7-C-io tail — sim-soak coverage + cutover

**Layer:** 5 (host↔MCU communication hardening — closes Step 7-C-io's Phase F).

**Scope:** This spec is the tail of [Step 7-C-io](2026-04-30-step-7c-io-design.md). It rewrites the original spec's §9 Phase F ("Soak + capture corpus") into two harnesses — a deterministic MockTransport battery for arithmetic / GC / monotonicity invariants, and a Renode-based sim-soak for memory-, livelock-, and reconnect-class signals. It also defines the interim cutover from `python-diff-test` to corpus-replay regression.

The hardware-only items the original Phase F implicitly bundled (Surface-C cycle actuals, USB-CDC byte-sequence fidelity, real unplug semantics, IWDG real-world pacing, 24h wall-clock soak, canonical CI capture corpus) explicitly **do not** belong to 7-C-io tail. They are deferred to [Step 7-D](../../kalico-rewrite/dependency-graph.md). Once Harness A and Harness B land, 7-C-io is declared done.

## 1. Goals & non-goals

### 1.1 In scope

1. **Harness A — MockTransport deterministic battery.** Seven new test files in `rust/kalico-host-rt/tests/` covering seq-arithmetic edge cases, NAK/RTO branches, AwaitingResponse three-layer GC, NAK+submit race, partial-frame TCP-style reads, ArcSwap snapshot monotonicity, and host-side clock-sync drift over synthetic virtual hours.
2. **Harness B — Renode H723 sim-soak.** A new `bin/soak.rs` driver inside the existing `kalico-host-rt` crate, plus `tools/sim/run_soak.sh`. Default scenario: 1h wall-clock soak with trajectory loop + disconnect/reconnect injection + backpressure flood + capture tap + metrics watcher. Multi-MCU scenario: 15min, 2 Renode instances.
3. **Capture corpus from sim.** Tee framed bytes into `tests/captures/sim-default-<timestamp>.bin`. Commit a baseline capture, un-ignore `corpus_covers_required_decode_surfaces`, retire the `python-diff-test` Cargo feature.
4. **Driver reuse contract.** `bin/soak.rs` accepts both `socket://...` (sim) and real serial paths so 7-D can reuse the same binary against H723 hardware with no code changes.

### 1.2 Out of scope (deferred to 7-D)

- 24h wall-clock soak on real H723 hardware.
- Canonical CI capture corpus (H723-firmware-emitted bytes).
- Surface-C cycle-budget actuals.
- USB-CDC byte-sequence fidelity / physical unplug semantics.
- IWDG real-world pacing.
- Trace/status pacing realism (sim runs slower than wall-clock; pacing observations have correctness signal but not timing-realism signal).

### 1.3 Non-goals (deliberate punts)

- **Unified scenario DSL spanning both harnesses.** Harness A items are heterogeneous (arithmetic, GC, ordering, monotonicity, drift); each test wires the MockTransport hooks it needs. No scenario-description language.
- **Soak in CI.** The sim-soak is a manual `tools/sim/run_soak.sh` invocation, run at PR-finalize and before declaring 7-C-io done. CI continues to run cargo test (Harness A + corpus-replay against committed captures) but does not boot Renode.
- **Renaming sim captures once H723 captures exist.** 7-D decides whether to keep sim captures as USART2-path coverage or drop them. Both naming conventions (`sim-*.bin`, `h723-*.bin`) coexist by design.

## 2. Architecture

### 2.1 File layout

All work lands inside the existing crate; no new crates.

```
rust/kalico-host-rt/
  src/bin/soak.rs                          # NEW — sim-soak driver binary
  tests/
    mock_transport.rs                      # additive hooks
    reactor_seq_wrap.rs                    # NEW — A1
    nak_rto_branches.rs                    # NEW — A2
    awaiting_response_gc.rs                # NEW — A3
    nak_submit_race.rs                     # NEW — A4
    partial_frame_assembly.rs              # NEW — A5
    status_arcswap_monotonic.rs            # NEW — A6
    clock_sync_drift.rs                    # NEW — A7
    captures_replay.rs                     # un-ignore corpus_covers_required_decode_surfaces
    captures/sim-default-baseline.bin      # NEW — committed sim capture
    fixtures/soak-trajectory.bin           # NEW — canned G5 trajectory for soak workload

tools/sim/
  run_soak.sh                              # NEW — boots Renode, runs soak driver
```

`Cargo.toml`: drop `python-diff-test` feature.

### 2.2 MockTransport hooks (additive)

`tests/mock_transport.rs` gains a small hook surface for deterministic injection. Each hook is single-purpose; no DSL.

| Hook | Purpose | Used by |
|---|---|---|
| `inject_duplicate_ack(seq)` | Inject a duplicate-ack frame on the rx path. | A2 |
| `inject_corrupt_frame()` | Inject a frame with bad CRC on the rx path. | A2 |
| `force_rto_now()` | Trigger RTO on next reactor tick (clock shim). | A2 |
| `set_clock_offset(Duration)` | Replaceable clock for RTO clamp tests. | A2, A7 |
| `split_next_read_at(offset)` | Split the next outbound rx delivery into two reads at byte `offset`. | A5 |
| `interleave_at(submission_id, event)` | Order-control: deliver `event` between submission ingestion and reactor response. | A4 |
| `disconnect()` / `reconnect()` | Simulate transport-level disconnect for GC test. | A3 |

The hooks are gated behind a `#[cfg(any(test, feature = "test-hooks"))]` so they never compile into production.

## 3. Harness A — MockTransport deterministic battery

Each item is one test file. Pass criteria are concrete; no statistical thresholds. All run in default `cargo test` and in CI.

### 3.1 A1 — `reactor_seq_wrap.rs`

Drive the reactor with synthetic ack-bearing frames whose 4-bit wire seq forces `decode_absolute` to roll across the 16-frame boundary 0→15→0 at three absolute-counter values: low (counter=15→16), mid (near 2³¹), high (near 2⁶³−16).

For each boundary, assert:
- `last_ack_seq` advances monotonically in absolute space.
- UnackedWindow pops use strict `<` not `≤`: just-acked frame removed, pending frames remain.
- `ignore_nak_seq` damper never matches a frame from a previous wrap epoch.
- No panics; UnackedWindow depth returns to expected value after each boundary cross.

### 3.2 A2 — `nak_rto_branches.rs`

Six sub-tests, one per branch:

1. Duplicate-ack triggers retransmit.
2. `ignore_nak_seq` suppresses the second NAK in a paired duplicate-ack.
3. RTO fires at SRTT + 4·RTTVAR (RFC 6298).
4. RTO clamped to floor `25 ms` when estimator collapses (e.g., zero RTTVAR with tiny SRTT).
5. RTO clamped to ceiling `5 s` when estimator inflates.
6. `MAX_RETRY_COUNT = 8` closure → `KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED`, UnackedWindow cleared.

Each sub-test forces exactly one branch via `inject_duplicate_ack` / `force_rto_now` / `set_clock_offset`. No real timing.

### 3.3 A3 — `awaiting_response_gc.rs`

Three sub-tests, one per GC layer:

1. **Abandon-on-drop.** Caller drops `CallHandle` mid-flight → AwaitingResponse entry GC'd on next reactor tick → late response silently discarded → queue size 0.
2. **Per-entry dispatcher timeout.** Entry exceeds deadline → `KALICO_ERR_HOST_DISPATCHER_TIMEOUT` to caller → entry removed → queue size 0.
3. **Disconnect-clears-all.** `disconnect()` → every pending entry resolves with `TransportError::Disconnected` → queue size 0 post-recovery.

### 3.4 A4 — `nak_submit_race.rs`

Single ordering test: `interleave_at` lets the test deliver a NAK for frame N−2 *between* submission N entering the reactor's mpsc and the reactor processing the retransmit.

Assert:
- Retransmit of N−2 goes on the wire before frame N.
- Post-tick `send_seq` and UnackedWindow are consistent.
- No frame dropped or duplicated.

### 3.5 A5 — `partial_frame_assembly.rs`

Proptest generator: a random sequence of 1–32 well-formed frames, split into chunk boundaries chosen uniformly from `[0, total_len]`. Feed chunks through `rx_buf` one at a time; assert `extract_packet` recovers the original frame sequence exactly.

Generated cases must exercise: split mid-header, split mid-CRC, split mid-payload, multiple frames per chunk, single byte at a time. 1024 cases per run; pass = 100% recovery.

### 3.6 A6 — `status_arcswap_monotonic.rs`

8 reader threads loop reading the status `ArcSwap` and recording observed generation values. 1 writer thread updates with monotonically-increasing generations. 1M reads per reader.

Assert: each reader's observed sequence is monotonically non-decreasing (never sees gen K then gen K−1). No torn snapshots — every observation is a complete, internally-consistent status snapshot.

### 3.7 A7 — `clock_sync_drift.rs`

Drive `clock_sync.rs` estimator with synthetic samples spanning 24 hours of host-`Instant`-equivalent virtual time (clock shim from MockTransport hook), with a fixed firmware drift rate of 100 ppm.

Assert:
- Residual stays within the documented ε (TBD per current `clock_sync.rs` constants — read at implementation time).
- Per-MCU `request_id` strictly monotonic across all arm attempts.
- Sample-freshness logic ages samples out at the documented threshold.
- Estimator does not accumulate unbounded error — final residual is bounded by the same ε.

**Out of scope:** firmware-side CYCCNT realism — that's 7-D.

## 4. Harness B — Renode sim-soak

### 4.1 Driver binary — `src/bin/soak.rs`

CLI:

```
soak [OPTIONS]
  --port <socket-or-tty>           Default: socket://localhost:3334
  --duration <secs>                 Default: 3600 (1 hour)
  --scenario <name>                 Default: default | multi-mcu
  --record-to <dir>                 Default: rust/kalico-host-rt/tests/captures/
  --multi-mcu <ports...>            For multi-mcu scenario only
  --metrics-out <file>              Default: target/soak-metrics-<timestamp>.csv
```

The same binary, with `--port /dev/cu.usbmodem...`, is what 7-D runs against H723 hardware. **No sim-specific code paths.**

### 4.2 Default scenario — five concurrent activities

Run for `--duration` seconds:

1. **Trajectory loop.** Replay `tests/fixtures/soak-trajectory.bin` (canned G5 trajectory; generated once from a representative file via the existing planner). Submit `kalico_push_segment` frames at a rate matched to credit replenishment. Loop indefinitely.
2. **Disconnect/reconnect injector.** Every 60 s, drop the TCP socket for 2 s, reopen. Confirm identify-during-reconnect race recovery (§6.4 of the parent spec) lands clean. Assert no AwaitingResponse leaks across the bounce.
3. **Backpressure flood.** Every 5 min, subscribe to all async-event channels and stop draining them for 30 s, then resume. Verify per-channel drop policy (snap for credit, latched for fault, ring for trace, bounded-warn for runtime) and host-event diagnostics emission. Also confirms reactor doesn't starve: with channels full, the 4-submission-per-loop drain still services serial / pending / RTO / GC.
4. **Capture tap.** Tee framed bytes (both directions) into `<record-to>/sim-default-<timestamp>.bin`. Per-frame header: `u8 dir`, `u64 ns_offset`, `u16 len`. Cap 100 MB per file; rotate if exceeded.
5. **Metrics watcher.** Every 10 s, snapshot RSS, AwaitingResponse depth, UnackedWindow depth, trace-ring fill, channel queue depths. Append to CSV.

### 4.3 Multi-MCU scenario — 15 min

Two Renode instances on TCP ports 3334, 3335. Driver invocation:

```
soak --scenario multi-mcu \
     --multi-mcu socket://localhost:3334 socket://localhost:3335 \
     --duration 900
```

Run `arm_all_mcus` repeatedly across both. Assert:
- Per-MCU `request_id` strictly monotonic.
- Zero cross-MCU response misroutes (driver records every dispatch+response pair, asserts MCU-index match on every response).

### 4.4 Pass criteria

A clean soak run requires **all** of:

| # | Criterion |
|---|---|
| 1 | Zero panics, zero reactor wedges (driver heartbeat completes every 5 s). |
| 2 | RSS slope over the soak ≤ 1 MB/hour (linear-fit on the metrics CSV). |
| 3 | AwaitingResponse and UnackedWindow depths return to 0 between trajectory loop iterations. |
| 4 | After every disconnect/reconnect cycle: AwaitingResponse=0, UnackedWindow=0, status snapshot reflects post-recovery state within 1 s. |
| 5 | After every backpressure cycle: all channels recover to "current" within 5 s of resuming drain; host-event diagnostics for any drops are present. |
| 6 | Multi-MCU: zero `request_id` collisions, zero cross-MCU response misroutes. |
| 7 | Capture corpus from `default` scenario covers `REQUIRED_SURFACES`. |

### 4.5 Explicit non-claims

A passing sim-soak does **not** claim:

- Surface-C cycle-budget actuals (Renode CYCCNT is virtual-time).
- USB-CDC byte-sequence fidelity (sim uses USART2 backend).
- Physical USB unplug semantics (sim uses TCP socket drop).
- IWDG real-world pacing.
- Trace/status absolute timing realism (sim slower than wall-clock).
- Canonical CI capture corpus (interim only; H723-emitted captures land in 7-D).

## 5. Cutover

Once Harness A is green and one full Harness B `default` run satisfies §4.4:

1. **Commit `tests/captures/sim-default-baseline.bin`.** Decision on git-lfs deferred until actual size is known from a real run; if > 50 MB after gzip, switch to git-lfs at commit time.
2. **Un-ignore `corpus_covers_required_decode_surfaces`** in `tests/captures_replay.rs:64`. With baseline present, asserts on every CI run.
3. **Retire `python-diff-test`.** Drop the Cargo feature, the dedicated CI lane, and the python-side bootstrap differential code. Corpus-replay is now the canonical regression oracle for host-side decode.
4. **Update `CLAUDE.md` build-order entry for 7-C-io:** flip `[~]` → `[x]`. Replace the "Phase F pending" parenthetical with a one-line note that sim-soak coverage is in place; canonical-corpus + hardware-only items deferred to 7-D.
5. **Update parent spec's §9 Phase F.** Replace with a back-pointer to this spec; preserve the "deferred to 7-D" enumeration.
6. **Plan-changes log entry** in `docs/superpowers/plan-changes-log.md`: 7-C-io tail / 7-D scope shift.

## 6. Handoff to 7-D

7-D inherits these items unchanged in shape — they reuse the `bin/soak.rs` driver:

1. **Canonical capture corpus.** `soak --port /dev/cu.usbmodem...` against H723; commit captures to `tests/captures/h723-<scenario>-<timestamp>.bin`. Both interim sim and canonical hardware captures coexist; both feed `corpus_covers_required_decode_surfaces`. The spec's Phase-1 "canonical CI reference" cutover lands here.
2. **24h wall-clock soak on bench.** Same driver, `--duration 86400`, same pass criteria, plus: USB-CDC byte-sequence fidelity (compare hardware vs sim capture bytes for protocol-correctness drift), real unplug semantics (unplug USB during soak; verify BrokenPipe/EIO matches sim's TCP-drop behavior at the host-state level), IWDG real-world pacing.
3. **Surface-C cycle-budget actuals.** Real H723 DWT->CYCCNT measurements.
4. **Trace/status pacing realism.** Re-run backpressure scenario at hardware wall-clock rates; tighten the 5-s recovery threshold if real timing reveals it should be faster.

The driver reuse contract (§2.1, §4.1) is the design constraint that makes (1) and (2) cheap for 7-D.

## 7. References

- [Step 7-C-io spec](2026-04-30-step-7c-io-design.md) — parent spec; this spec rewrites its §9 Phase F.
- `rust/kalico-host-rt/tests/captures_replay.rs` — corpus-replay scaffold; `REQUIRED_SURFACES` defined there.
- `rust/kalico-host-rt/src/host_io/reactor.rs` — reactor loop, drain budget, GC layers.
- `rust/kalico-host-rt/src/host_io/window.rs` — UnackedWindow, AwaitingResponse.
- `rust/kalico-host-rt/src/clock_sync.rs` — host-side clock estimator.
- `tools/sim/h723_sim.resc` — Renode platform definition.
- `tools/sim/README.md` — sim limitations (CYCCNT virtual-time, USART2 instead of USB-CDC, slower-than-wall pacing).
