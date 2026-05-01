# Step 7-C-io tail — sim-soak coverage + cutover

**Layer:** 5 (host↔MCU communication hardening — closes Step 7-C-io's Phase F).

**Scope:** This spec is the tail of [Step 7-C-io](2026-04-30-step-7c-io-design.md). It rewrites the original spec's §9 Phase F ("Soak + capture corpus") into two harnesses — a deterministic MockTransport battery for arithmetic / GC / monotonicity invariants, and a Renode-based sim-soak for memory-, livelock-, and reconnect-class signals. It also defines the interim cutover from `python-diff-test` to corpus-replay regression.

The hardware-only items the original Phase F implicitly bundled (Surface-C cycle actuals, USB-CDC byte-sequence fidelity, real unplug semantics, IWDG real-world pacing, 24h wall-clock soak, canonical CI capture corpus) explicitly **do not** belong to 7-C-io tail. They are deferred to [Step 7-D](../../kalico-rewrite/dependency-graph.md). Once Harness A and Harness B land, 7-C-io is declared done.

## 1. Goals & non-goals

### 1.1 In scope

1. **Harness A — MockTransport deterministic battery.** Seven new test files in `rust/kalico-host-rt/tests/` covering seq-arithmetic edge cases, NAK/RTO branches, AwaitingResponse three-layer GC, NAK+submit race, partial-frame TCP-style reads, ArcSwap snapshot monotonicity, and host-side clock-sync drift over synthetic virtual hours.
2. **Harness B — Renode H723 sim-soak.** A new `bin/soak.rs` driver inside the existing `kalico-host-rt` crate, plus `tools/sim/run_soak.sh`. Default scenario: 1h wall-clock soak with trajectory loop + disconnect/reconnect injection + backpressure flood + capture tap + metrics watcher. Multi-MCU scenario: 15min, 2 Renode instances.
3. **Capture corpus from sim.** Tee framed bytes into `tests/captures/sim-default-<timestamp>.bin`. Commit a baseline capture; un-ignore `corpus_covers_required_decode_surfaces`. **`python-diff-test` is NOT retired in this step** — that retirement is gated on canonical H723 captures and lives in 7-D (see §5 and parent spec §4.13).
4. **Driver reuse contract.** `bin/soak.rs` accepts both `socket://...` (sim) and real serial paths so 7-D can reuse the same binary against H723 hardware. The `socket://` transport is implemented as a new `Transport` impl in this step (see §2.3); the disconnect injector for sim runs at the TCP layer inside that impl. Real-serial unplug semantics are not validated here — that's 7-D.

5. **Reactor test seam + clock seam (Phase-0 prereqs).** Add a lower-level reactor test harness that owns `rx_buf`, a clock shim, and reactor state directly — `MockTransport` is the wrong layer for arithmetic/RTO/partial-frame tests (it's a high-level RPC mock). Add a monotonic-clock seam to `clock_sync.rs` so A7 can age samples deterministically.

### 1.2 Out of scope (deferred to 7-D)

- 24h wall-clock soak on real H723 hardware.
- Canonical CI capture corpus (H723-firmware-emitted bytes).
- **Retirement of the `python-diff-test` Cargo feature** — gated on canonical H723 captures per parent spec §4.13.
- Surface-C cycle-budget actuals.
- USB-CDC byte-sequence fidelity / physical unplug semantics.
- IWDG real-world pacing.
- Trace/status pacing realism (sim runs slower than wall-clock; pacing observations have correctness signal but not timing-realism signal).
- Tight real-time response bounds (e.g. < 1 s status snapshot recovery, < 5 s channel recovery at wall-clock). Sim runs at 0.05–0.5x wall (`tools/sim/README.md:118`); the soak validates these in *sim time scaled by the observed pacing factor* only.

### 1.3 Non-goals (deliberate punts)

- **Unified scenario DSL spanning both harnesses.** Harness A items are heterogeneous (arithmetic, GC, ordering, monotonicity, drift); each test wires the MockTransport hooks it needs. No scenario-description language.
- **Soak in CI.** The sim-soak is a manual `tools/sim/run_soak.sh` invocation, run at PR-finalize and before declaring 7-C-io done. CI continues to run cargo test (Harness A + corpus-replay against committed captures) but does not boot Renode.
- **Renaming sim captures once H723 captures exist.** 7-D decides whether to keep sim captures as USART2-path coverage or drop them. Both naming conventions (`sim-*.bin`, `h723-*.bin`) coexist by design.

## 2. Architecture

### 2.1 File layout

All work lands inside the existing crate; no new crates.

```
rust/kalico-host-rt/
  src/
    clock_sync.rs                          # ADD monotonic-clock seam (Phase-0 prereq for A7)
    host_io/
      reactor.rs                           # ADD #[cfg(test)] state-injection seam (Phase-0 prereq for A1/A2/A4)
      socket_transport.rs                  # NEW — Transport impl for `socket://host:port` paths
    bin/soak.rs                            # NEW — sim-soak driver binary
  tests/
    reactor_harness.rs                     # NEW — low-level reactor test harness (rx_buf + clock shim + reactor state)
    reactor_seq_wrap.rs                    # NEW — A1
    nak_rto_branches.rs                    # NEW — A2
    awaiting_response_gc.rs                # NEW — A3 (uses MockTransport at RPC layer)
    nak_submit_race.rs                     # NEW — A4
    partial_frame_assembly.rs              # NEW — A5 (pure parser test, no transport)
    status_arcswap_monotonic.rs            # NEW — A6
    clock_sync_drift.rs                    # NEW — A7 (uses clock seam from clock_sync.rs)
    captures_replay.rs                     # un-ignore corpus_covers_required_decode_surfaces
    captures/sim-default-baseline.bin      # NEW — committed sim capture (raw concatenated framed bytes)
    fixtures/soak-trajectory.bin           # NEW — canned G5 trajectory for soak workload

tools/sim/
  run_soak.sh                              # NEW — boots Renode, runs soak driver
```

`Cargo.toml`: keep `python-diff-test` feature alive — it retires in 7-D, not here.

### 2.2 Test seams

The existing `tests/mock_transport.rs` is a high-level `Transport` RPC mock — wrong layer for arithmetic / RTO / partial-frame tests. This step introduces three test seams, each at the right layer for its consumers:

**Seam 1 — Reactor test harness (`tests/reactor_harness.rs`).**
A test-only constructor that builds a `Reactor` outside the production `KalicoHostIo::open` path with direct access to `rx_buf`, the clock shim (Seam 2), `UnackedWindow`, `AwaitingResponse`, and an injectable rx-byte queue (no real serial/socket). Methods:

| Method | Purpose | Used by |
|---|---|---|
| `feed_rx_bytes(&[u8])` | Append bytes to `rx_buf` (split however the caller wants). | A1, A2 (corrupt frames), A4 |
| `tick_once()` | Run one reactor iteration deterministically. | A1, A2, A4 |
| `force_rto_now()` | Mark all UnackedWindow entries as RTO-expired before next `tick_once()`. | A2 |
| `submit(cmd) -> CallHandle` | Submit through the same submission queue the reactor reads from. | A2, A4 |
| `interleave(events)` | Atomically queue an rx event + a submission so they arrive in defined order. | A4 |
| `unacked_window_depth()` / `awaiting_response_depth()` | Inspect state. | A1, A2, A3, A4 |

Lives behind `#[cfg(test)]`; never compiled in release.

**Seam 2 — Monotonic-clock seam in `clock_sync.rs` and `RttEstimator`.**
Replace direct `Instant::now()` calls in `clock_sync.rs:100,183` and `host_io/rtt.rs` with a `Clock` trait (default impl: real `Instant`; test impl: hand-driven `MockClock`). Required because `last_sample_age()` and `last_dedicated_sample_age()` use `.elapsed()` on real instants today, which a transport-level `set_clock_offset` cannot affect. Phase-0 prereq for A7 and the RTO-clamp branches of A2.

**Seam 3 — `MockTransport` stays as-is.**
The existing high-level RPC mock is the right layer for A3 (caller-drop / dispatcher-timeout / disconnect-clears-all). No changes needed.

**Disconnect injection.** Lives at the `socket://` `Transport` impl level (§2.3), not in MockTransport. The reactor harness exercises §6.4's identify-during-reconnect race via Seam 1's rx-byte injection plus a `simulate_close()` method on the `socket_transport` impl.

### 2.3 `socket://` transport

`src/host_io/socket_transport.rs` is a new module providing TCP-backed I/O so the soak driver can attach to Renode's USART2 bridge using the same `bin/soak.rs` binary that 7-D will use against real serial.

The Step-6 transport seam already opens the port via a `path` string in `KalicoHostIo::open_with_config` (`rust/kalico-host-rt/src/host_io/mod.rs:139`). Extend that opener: if `path` starts with `socket://`, route through `socket_transport::connect(host, port)` and produce a wire-level `Read + Write` handle wrapping the `TcpStream`. Otherwise the existing `serialport::new(...)` path runs unchanged.

Public surface beyond the transport plumbing:

- `simulate_close()` — drops the underlying TCP stream; subsequent reactor reads see EOF / `ErrorKind::BrokenPipe`. Used by the soak driver's disconnect injector.
- Reconnect happens via the existing reactor reconnect path; the transport's `connect()` is re-invoked on reopen.

Gated behind `#[cfg(any(test, feature = "sim-transport"))]`. The soak binary enables `sim-transport`; production `KalicoHostIo` users don't.

**This is the load-bearing design decision for the driver-reuse contract** — the soak binary opens transports through `KalicoHostIo::open` regardless of `socket://` or `/dev/cu...`, so its workload, injectors, and capture tap are transport-agnostic. Only `simulate_close()` is sim-specific, and it's only called when the disconnect-injector scenario runs against `socket://` paths.

## 3. Harness A — MockTransport deterministic battery

Each item is one test file. Pass criteria are concrete; no statistical thresholds. All run in default `cargo test` and in CI.

### 3.1 A1 — `reactor_seq_wrap.rs`

Use Seam 1 (reactor harness). Drive the reactor with synthetic ack-bearing frames whose 4-bit wire seq forces `decode_absolute` to roll across the 16-frame boundary 0→15→0 at three absolute-counter values: low (counter=15→16), mid (near 2³¹), high (near 2⁶³−16).

For each boundary, assert:
- `last_ack_seq` advances monotonically in absolute space.
- `UnackedWindow::pop_acked` uses strict `<`: ack with `rseq = R` removes entries with `entry.seq < R` and leaves entries with `entry.seq >= R` (i.e. the entry whose seq equals the new `receive_seq` boundary stays — see `host_io/window.rs`).
- `ignore_nak_seq` damper never matches a frame from a previous wrap epoch (its absolute-seq comparison must dominate the 4-bit wire equality).
- No panics; UnackedWindow depth returns to expected value after each boundary cross.

### 3.2 A2 — `nak_rto_branches.rs`

Six sub-tests, one per branch:

1. Duplicate-ack triggers retransmit.
2. `ignore_nak_seq` suppresses the second NAK in a paired duplicate-ack.
3. RTO fires at SRTT + 4·RTTVAR (RFC 6298).
4. RTO clamped to floor `25 ms` when estimator collapses (e.g., zero RTTVAR with tiny SRTT).
5. RTO clamped to ceiling `5 s` when estimator inflates.
6. `MAX_RETRY_COUNT = 8` closure → `KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED`, UnackedWindow cleared.

Each sub-test uses Seam 1 (reactor harness) for state injection plus Seam 2 (clock seam) for time advancement. Duplicate acks and corrupt frames go in via `feed_rx_bytes`; RTO fires by advancing the `MockClock`. No real timing dependency.

### 3.3 A3 — `awaiting_response_gc.rs`

Three sub-tests, one per GC layer. Uses Seam 3 (`MockTransport`) at the RPC layer plus Seam 1 (`reactor_harness`) for the disconnect path.

1. **Abandon-on-drop.** Caller drops `CallHandle` mid-flight → AwaitingResponse entry GC'd on next reactor tick → late response silently discarded → queue size 0.
2. **Per-entry dispatcher timeout.** Entry exceeds deadline → `KALICO_ERR_HOST_DISPATCHER_TIMEOUT` to caller → entry removed → queue size 0.
3. **Disconnect-clears-all.** Reactor disconnect path → every pending entry resolves with `TransportError::Closed` (the variant emitted by `host_io/reactor.rs:485`) → queue size 0 post-recovery.

### 3.4 A4 — `nak_submit_race.rs`

Uses Seam 1's `interleave(events)` to atomically queue an rx-side NAK for frame N−2 alongside a fresh submission of frame N, so the reactor processes them in a defined order within a single `tick_once()`.

Assert:
- Retransmit of N−2 goes on the wire before frame N.
- Post-tick `send_seq` and UnackedWindow are consistent.
- No frame dropped or duplicated.

### 3.5 A5 — `partial_frame_assembly.rs`

Pure parser test against `host_io/wire.rs:extract_packet` — no reactor, no transport. Five proptest strategies, each with its own dedicated generator (uniform random splits do not reliably hit the failure paths):

1. **Mid-length-prefix splits** — every byte boundary inside the leading length byte gets exercised explicitly.
2. **Mid-CRC splits** — splits land inside the trailing CRC field.
3. **Mid-payload splits** — at least one split lands inside the payload region (parameterized over payload size).
4. **Multi-frame chunks** — 2–8 frames packed into a single read buffer.
5. **Resync-after-corruption** — proptest #5: insert a random invalid byte (bad SYNC / bad length / bad CRC) before a valid frame; assert `extract_packet` resyncs by dropping bytes one at a time and recovers the trailing valid frame. This is the path at `wire.rs:33` that uniform-random splits would miss.

Each strategy: 256 cases. Pass = 100% recovery / correct resync.

### 3.6 A6 — `status_arcswap_monotonic.rs`

8 reader threads loop reading the status `ArcSwap` and recording observed generation values. 1 writer thread updates with monotonically-increasing generations. 1M reads per reader.

Assert: each reader's observed sequence is monotonically non-decreasing (never sees gen K then gen K−1). No torn snapshots — every observation is a complete, internally-consistent status snapshot.

### 3.7 A7 — `clock_sync_drift.rs`

**Phase-0 prereq:** Seam 2 (monotonic-clock seam in `clock_sync.rs`) lands first; without it `last_sample_age()` and `last_dedicated_sample_age()` cannot be aged deterministically.

Drive the estimator with synthetic samples spanning 24 virtual hours via the `MockClock`, with a fixed firmware drift rate of **50 ppm** (well below `MAX_DRIFT_PPM_DEFAULT = 100.0`, so we're testing in-band behavior, not exact-cap brittleness).

Assert:
- Residual stays within the documented ε (read from `clock_sync.rs` constants at implementation time and pinned in the test).
- Per-MCU `request_id` strictly monotonic across all simulated arm attempts.
- Sample-freshness logic ages samples out at the documented threshold (now exercisable because `last_sample_age()` consults `MockClock`).
- Estimator does not accumulate unbounded error — final residual ≤ same ε.

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
4. **Capture tap.** Tee **MCU→host** framed bytes — raw, concatenated, no per-frame header — into `<record-to>/sim-default-<timestamp>.bin`. The format matches what `tests/captures_replay.rs:82` already feeds to `extract_packet`. Host→MCU direction and timestamps are not captured here (out of scope for the current corpus-replay test). Cap 100 MB per file; rotate if exceeded.
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
| 1 | Zero panics, zero reactor wedges. The driver heartbeats every 5·*p* sim-seconds where *p* is the observed pacing factor (Renode runs at 0.05–0.5x wall per `tools/sim/README.md:118`); the heartbeat must complete within that window. |
| 2 | RSS slope over the soak ≤ 1 MB/hour (linear-fit on the metrics CSV). **This is a smoke-only signal** — one-hour runs cannot reliably distinguish slow leaks from allocator/cache noise. Canonical leak coverage is 7-D's 24h soak. |
| 3 | UnackedWindow depth does not grow without bound across iterations: across the soak, `max(depth) ≤ MAX_PENDING_BLOCKS = 12`, and at scenario boundaries (backpressure-flood end, disconnect/reconnect cycle end) depth returns to 0 within a bounded drain barrier the driver explicitly inserts. AwaitingResponse depth is monitored but not bounded between iterations under sustained submission. |
| 4 | After every disconnect/reconnect cycle: AwaitingResponse and UnackedWindow drain to 0 within the cycle window; status snapshot reflects post-recovery state. **No wall-clock timing requirement** — sim pacing makes that meaningless; 7-D adds the wall-clock bound. |
| 5 | After every backpressure cycle: every channel that was flooded shows the documented drop policy in effect (snap/latched/ring/bounded-warn) AND eventually catches up to "current" once draining resumes. **No wall-clock recovery bound** — sim pacing makes that meaningless; 7-D adds the wall-clock bound. |
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

1. **Commit `tests/captures/sim-default-baseline.bin`.** File is named `sim-*` to mark it as USART2-path / Renode-firmware — explicitly NOT the canonical H723 corpus. Decision on git-lfs deferred until actual size is known from a real run; if > 50 MB after gzip, switch to git-lfs at commit time.
2. **Un-ignore `corpus_covers_required_decode_surfaces`** in `tests/captures_replay.rs:64`. With baseline present, asserts on every CI run.
3. **`python-diff-test` stays.** Per parent spec §4.13, retirement is gated on canonical H723 captures — it lives in 7-D. Sim captures (USART2 backend, Renode firmware path, Renode timing) do not close the regression-coverage gap that the python parser oracle covers. Keeping the feature alive costs us a CI lane; the alternative is an unverified coverage gap, which is worse.
4. **Update `CLAUDE.md` build-order entry for 7-C-io:** flip `[~]` → `[x]`. Replace the "Phase F pending" parenthetical with a one-line note that sim-soak coverage is in place; canonical-corpus, `python-diff-test` retirement, and hardware-only items all deferred to 7-D.
5. **Update parent spec's §9 Phase F.** Replace with a back-pointer to this spec; preserve the "deferred to 7-D" enumeration.
6. **Plan-changes log entry** in `docs/superpowers/plan-changes-log.md`: 7-C-io tail / 7-D scope shift, with explicit note that `python-diff-test` retirement moved from this step to 7-D.

## 6. Handoff to 7-D

7-D inherits these items unchanged in shape — they reuse the `bin/soak.rs` driver:

1. **Canonical capture corpus + `python-diff-test` retirement.** `soak --port /dev/cu.usbmodem...` against H723; commit captures to `tests/captures/h723-<scenario>-<timestamp>.bin`. Both interim sim and canonical hardware captures coexist; both feed `corpus_covers_required_decode_surfaces`. **At this point** the parent spec's Phase-1 "canonical CI reference" cutover lands AND `python-diff-test` is retired (Cargo feature dropped, CI lane removed, python-side bootstrap differential code deleted).
2. **24h wall-clock soak on bench.** Same driver, `--duration 86400`, same pass criteria, plus: real-time bounds (status snapshot recovery, channel-recovery threshold) at wall-clock rates; USB-CDC byte-sequence fidelity (compare hardware vs sim capture bytes for protocol-correctness drift); real unplug semantics (unplug USB during soak; verify BrokenPipe/EIO matches sim's TCP-drop behavior at the host-state level); IWDG real-world pacing.
3. **Canonical leak coverage.** 24h wall-clock RSS curve; replaces the smoke-only 1-hour slope from §4.4 #2.
4. **Surface-C cycle-budget actuals.** Real H723 DWT->CYCCNT measurements.
5. **Trace/status pacing realism.** Re-run backpressure scenario at hardware wall-clock rates; pin the real-time recovery thresholds that §4.4 #4–5 deferred.

The driver reuse contract (§2.1, §4.1) is the design constraint that makes (1) and (2) cheap for 7-D.

## 7. References

- [Step 7-C-io spec](2026-04-30-step-7c-io-design.md) — parent spec; this spec rewrites its §9 Phase F.
- `rust/kalico-host-rt/tests/captures_replay.rs` — corpus-replay scaffold; `REQUIRED_SURFACES` defined there.
- `rust/kalico-host-rt/src/host_io/reactor.rs` — reactor loop, drain budget, GC layers.
- `rust/kalico-host-rt/src/host_io/window.rs` — UnackedWindow, AwaitingResponse.
- `rust/kalico-host-rt/src/clock_sync.rs` — host-side clock estimator.
- `tools/sim/h723_sim.resc` — Renode platform definition.
- `tools/sim/README.md` — sim limitations (CYCCNT virtual-time, USART2 instead of USB-CDC, slower-than-wall pacing).
