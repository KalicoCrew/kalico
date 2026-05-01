# Step 7-C-io tail — deterministic test battery + Clock seam

**Layer:** 5 (host↔MCU communication hardening — closes Step 7-C-io's Phase F).

**Scope:** This spec is the tail of [Step 7-C-io](2026-04-30-step-7c-io-design.md). It rewrites the original spec's §9 Phase F into a single deliverable: a deterministic test battery that catches arithmetic, GC, ordering, and edge-case bugs in the host I/O reactor — bugs which hardware testing cannot reliably surface. The supporting refactor is the load-bearing piece: a `Clock` trait + `tick_once()` test entrypoint threaded through `host_io/reactor.rs`, `host_io/window.rs`, `host_io/mod.rs`, and `clock_sync.rs`. Once the battery is green, 7-C-io is declared done and remaining items move to 7-D where they belong.

A Renode-based sim-soak was considered and dropped — see §8 for the rationale. The short version: with wall-clock bounds undefined under sim pacing, USART2 instead of USB-CDC, and one-hour leak detection too short to trust, the sim-soak's catchable failure modes are dominated by what this deterministic battery already covers; remaining items (24h soak, canonical capture corpus, USB-CDC fidelity) are inherently hardware-only and belong to 7-D.

## 1. Goals & non-goals

### 1.1 In scope

1. **Production-code refactor: `Clock` trait** threaded through every `Instant::now()` and `Instant`-typed field that participates in deterministic-testable behavior. Concrete sites in §3.
2. **Production-code refactor: `tick_once()` reactor entrypoint** extracted from `Reactor::run()`'s loop body, exported under `pub(crate)`, callable from a test harness without spawning a thread or owning a real serial port.
3. **Test harness — `tests/reactor_harness.rs`** that constructs a `Reactor` outside `KalicoHostIo::open`, owning `rx_buf` injection, a hand-driven `MockClock`, and direct access to `UnackedWindow` / `AwaitingResponse` depths.
4. **Seven deterministic test files** covering: seq-arithmetic edge cases (A1), NAK/RTO branches (A2), AwaitingResponse three-layer GC (A3), NAK+submit race (A4), partial-frame TCP-style read assembly (A5), ArcSwap snapshot monotonicity (A6), clock-sync drift over virtual hours (A7).
5. **Cutover trim:** un-ignore `corpus_covers_required_decode_surfaces` only if a baseline capture is committed alongside (otherwise leave it ignored). `python-diff-test` stays alive.

### 1.2 Out of scope (deferred to 7-D)

- **Renode sim-soak** — driver binary, capture tap, multi-MCU scenario, RSS leak watcher, all of it. Rationale in §8.
- **Canonical CI capture corpus** (H723-firmware-emitted bytes).
- **`python-diff-test` retirement** — gated on canonical H723 captures per parent spec §4.13.
- **24h wall-clock soak.**
- **Surface-C cycle-budget actuals.**
- **USB-CDC byte-sequence fidelity / physical unplug semantics.**
- **IWDG real-world pacing.**
- **Real-time response bounds** for status-snapshot recovery, channel drain, etc.

### 1.3 Non-goals (deliberate punts)

- **A `socket://` `Transport` impl.** Not needed without the sim-soak.
- **`tick_once()` as a public API.** Stays `pub(crate)`; only the harness in `tests/reactor_harness.rs` consumes it. Production code keeps using `Reactor::run()`.
- **Replacing `MockTransport`.** The existing high-level RPC mock stays for A3's caller-drop / dispatcher-timeout sub-tests. The new reactor harness is a separate, lower-layer seam.

## 2. Architecture

### 2.1 File layout

```
rust/kalico-host-rt/
  src/
    clock.rs                                 # NEW — Clock trait + RealClock + MockClock
    clock_sync.rs                            # CHANGED — thread Clock through new_with_clock + freshness paths
    host_io/
      reactor.rs                             # CHANGED — extract tick_once(); thread Clock; A1/A2/A4 as #[cfg(test)] mod
      mod.rs                                 # CHANGED — thread Clock for deadline computation in call/call_typed
      test_harness.rs                        # NEW — gated #[cfg(any(test, feature = "test-harness"))]
  tests/
    awaiting_response_gc.rs                  # NEW — A3 (integration; uses MockTransport)
    partial_frame_assembly.rs                # NEW — A5 (integration; pure parser proptest)
    status_arcswap_monotonic.rs              # NEW — A6 (integration; ArcSwap)
    clock_sync_drift.rs                      # NEW — A7 (integration; uses ClockSyncEstimator pub API + MockClock)
```

A1, A2, A4 live as `#[cfg(test)] mod` blocks inside `src/host_io/reactor.rs` (or as `#[cfg(test)] mod tests_a1` etc., split if file gets too large). They need `pub(crate)` field access on `Reactor` and `pub(crate)` methods on the harness.

`window.rs` is unchanged — `UnackedEntry.sent_at` is constructed by the reactor with `clock.now()` and passed in.

No new crate dependencies. `Cargo.toml` adds an opt-in `test-harness` feature for the harness module's gating; `python-diff-test` stays.

### 2.2 The `Clock` trait

```rust
// src/clock.rs
pub trait Clock: Send + Sync {
    fn now(&self) -> Instant;
}

pub struct RealClock;
impl Clock for RealClock {
    fn now(&self) -> Instant { Instant::now() }
}

pub struct MockClock { /* interior-mutable Instant, advance_by(Duration) */ }
impl Clock for MockClock { /* now() returns the held Instant */ }
```

**Holding strategy.** Every type that today calls `Instant::now()` and is reachable from production gains an `Arc<dyn Clock>` field (or `&dyn Clock` parameter) at construction. Production wires `Arc::new(RealClock)`; tests wire `Arc::new(MockClock::new())`. A single `Arc<dyn Clock>` per `KalicoHostIo` instance is shared across `Reactor`, `UnackedWindow`, `AwaitingResponse`, `ClockSyncEstimator`.

Why `Arc<dyn Clock>` and not a generic parameter: the reactor crosses thread boundaries (one per port) and is held inside `KalicoHostIo` whose public surface is concrete; a generic `Clock` parameter would virally infect every consumer with a type parameter, including downstream callers who never see the clock. Cost: one virtual call per `now()`. `now()` is not on a hot path (called O(N) per reactor tick where N = window depth ≤ 12); the cost is negligible.

### 2.3 Production-code call sites that must thread `Clock`

**Scope:** in scope = reactor loop + host-call deadlines + clock-sync freshness. Out of scope = `identify.rs`, `stream.rs`, `events.rs` `Instant` calls (their behavior is tested against real time elsewhere; not load-bearing for the deterministic battery).

| File | Line | Context | Treatment |
|---|---|---|---|
| `host_io/reactor.rs` | 153 | `let now = Instant::now()` for `sent_at` on submission | `clock.now()` |
| `host_io/reactor.rs` | 231 | `Instant::now() - entry.sent_at` for RTT sample | `clock.now() - entry.sent_at` |
| `host_io/reactor.rs` | 390 | `let now = Instant::now()` (close-deadline path) | `clock.now()` |
| `host_io/reactor.rs` | 524 | `let now = Instant::now()` for RTO check | `clock.now()` |
| `host_io/reactor.rs` | 540 | `let now = Instant::now()` for AwaitingResponse layer-2 GC | `clock.now()` |
| `host_io/mod.rs` | 190 | `let deadline = Instant::now() + timeout` in `call()` | `clock.now() + timeout` |
| `host_io/mod.rs` | 229 | same in `call_typed()` | `clock.now() + timeout` |
| `clock_sync.rs` | 102 | `epoch: Instant::now()` in `ClockSyncEstimator::new` | new path: `epoch = clock.now()` (see §4 Phase 0 for the constructor pattern that keeps existing call sites unchanged) |
| `clock_sync.rs` | 183 | `let now = Instant::now()` in `add_dedicated_sample` | `self.clock.now()` |
| `clock_sync.rs` | 206 | `recorded_at: Instant::now()` in `add_piggyback_sample` | `self.clock.now()` |
| `clock_sync.rs` | 278 | `s.recorded_at.elapsed()` in `last_sample_age` | `self.clock.now() - s.recorded_at` |
| `clock_sync.rs` | 285 | `t.elapsed()` in `last_dedicated_sample_age` | `self.clock.now() - t` |

`window.rs:136` is `#[cfg(test)]` test scaffolding (not production); it stays as-is. `reactor.rs:651` / `reactor.rs:918` are also `#[cfg(test)]` and stay as-is.

`UnackedEntry.sent_at` is constructed inside the reactor at line 153 and threaded into `UnackedWindow::push` — `window.rs` itself doesn't need the clock; the reactor's clock supplies the value.

### 2.4 `tick_once()` extraction

`Reactor::run()` is currently a single loop body (see `host_io/reactor.rs:503`–`551`). The body is: drain commands (≤4) → poll serial → drain pending submissions → RTO step → host-fault drain → host-event drain → AwaitingResponse GC → closed-state exit. Extract this body into:

```rust
pub(crate) fn tick_once(&mut self) -> TickOutcome { /* ... */ }

pub(crate) enum TickOutcome { Continue, Closed }
```

`run()` becomes:

```rust
loop {
    if matches!(self.tick_once(), TickOutcome::Closed) { break; }
}
```

The closed-state cleanup (`flush_all_completions` at `reactor.rs:548`) happens inside `tick_once()` when `self.state == Closed`, before returning `TickOutcome::Closed`. This preserves the existing semantics exactly: a tick that observes `Closed` flushes once, then the loop exits.

The harness drives the reactor by calling `tick_once()` between rx-byte injections and clock advances. No thread spawn; no real serial port.

### 2.5 Reactor harness — `src/host_io/test_harness.rs`

**Visibility.** The `Reactor` struct is `pub` but every interesting field (`unacked_window`, `awaiting_response`, `send_seq`, `receive_seq`, `last_ack_seq`, `ignore_nak_seq`, …) is `pub(crate)` (see `host_io/reactor.rs:20`–`50`). `CallHandle` is `pub(crate)` too (`host_io/call_handle.rs:9`). An integration test in `tests/reactor_harness.rs` cannot read those fields. The harness must therefore live **inside** the crate, where `pub(crate)` is visible.

Choice: put the harness at `src/host_io/test_harness.rs`, gated `#[cfg(any(test, feature = "test-harness"))]`. A1, A2, and A4 — the tests that need direct field access — also live as `#[cfg(test)] mod` blocks alongside the harness inside `src/host_io/` rather than as integration tests. A3 (uses `MockTransport`), A5 (pure parser proptest), A6 (ArcSwap monotonicity), and A7 (uses `ClockSyncEstimator`'s public API + `MockClock`) stay as integration tests in `tests/` because they only depend on `pub` API.

```rust
// src/host_io/test_harness.rs (cfg-gated)
pub(crate) struct ReactorHarness {
    reactor: Reactor,
    clock:   Arc<MockClock>,
    rx_pipe: Arc<Mutex<VecDeque<u8>>>,
    tx_log:  Arc<Mutex<Vec<u8>>>,
}

impl ReactorHarness {
    pub(crate) fn new() -> Self { /* construct Reactor with fake SerialPort + MockClock via Reactor::new_with_clock */ }
    pub(crate) fn feed_rx(&mut self, bytes: &[u8]);
    pub(crate) fn advance_clock(&mut self, by: Duration);
    pub(crate) fn tick(&mut self) -> TickOutcome;
    pub(crate) fn submit(&self, payload: Vec<u8>, name: &str, deadline: Instant) -> Receiver<Result<MessageParams, TransportError>>;
    pub(crate) fn unacked_depth(&self) -> usize;
    pub(crate) fn awaiting_depth(&self) -> usize;
    pub(crate) fn tx_log(&self) -> Vec<u8>;
    pub(crate) fn force_rto(&mut self);
}
```

`submit()` returns the `Receiver` half of a sync_channel directly rather than a `CallHandle` — keeps the harness independent of `CallHandle`'s `pub(crate)` API surface. Tests check completion by polling the receiver.

**Fake `SerialPort` surface.** The reactor calls `write_all`, `flush`, `set_timeout`, `read` on its `Box<dyn serialport::SerialPort>` (`reactor.rs:105` for write/flush, `reactor.rs:372` for read). The fake must impl the **full** `serialport::SerialPort` trait — Rust trait coherence requires every method — but only the four behaviorally-relevant methods do anything; the rest stub out (e.g., return `Err(io::ErrorKind::Unsupported.into())` for control-line ops). The existing `reactor.rs:586`–`621` test scaffolding is the reference for how to do this.

## 3. Test specifications

Each test file targets one concern. Pass criteria are concrete; no statistical thresholds.

### 3.1 A1 — `#[cfg(test)] mod` inside `src/host_io/reactor.rs`

`decode_absolute` (`reactor.rs:212`–`215`) computes `(wire_seq - receive_seq) & 0x0F` and adds `delta` to `receive_seq`. The structurally meaningful boundaries are: every mod-16 wire roll, the empty-window snap path (`reactor.rs:222`–`227`), and overflow risk near `u64::MAX`.

Three boundary cases via the harness:

1. **Empty-window snap.** First MCU response with `unacked_window` empty: `update_receive_seq` snaps both `send_seq` and `receive_seq` to `rseq`. Submit no frames; inject an ack with non-zero rseq → both counters jump.
2. **Mid-range mod-16 wrap.** Submit 12 frames (window cap = `MAX_PENDING_BLOCKS`); advance `MockClock` so RTOs don't fire; inject acks whose 4-bit wire seq advances `receive_seq` across multiple mod-16 boundaries (e.g., counter 14 → 18 spans 16). Verify each ack pops the correct entries.
3. **Near-`u64::MAX` overflow guard.** Force `receive_seq` to `u64::MAX - 15` (test-only setter behind cfg-gate, or repeated submission/ack cycles); inject an ack that would push `receive_seq + delta` to wrap. Assert: no debug-mode panic; the wrapping_sub at `reactor.rs:213` is correct under the boundary.

For each boundary, assert:
- `last_ack_seq` advances monotonically in absolute space.
- `UnackedWindow::pop_acked` removes only entries with `entry.seq < rseq` (entries with `seq == rseq` stay — `host_io/window.rs:35`).
- `ignore_nak_seq` damper's absolute-seq comparison dominates the 4-bit wire equality so it never matches a frame from a previous wrap epoch.
- No panics; depth returns to expected value after each boundary cross.

### 3.2 A2 — `nak_rto_branches.rs`

Six sub-tests, one per branch. All use harness + `MockClock`.

1. **Duplicate-ack triggers retransmit.** `feed_rx` an ack-with-data, then `feed_rx` an immediate duplicate ack on the same seq → `tx_log` shows the original frame retransmitted.
2. **`ignore_nak_seq` suppresses paired second NAK.** Two duplicate acks in quick succession on the same seq → exactly one retransmit in `tx_log`.
3. **RTO fires at SRTT + 4·RTTVAR.** Submit frame; let estimator reach a known SRTT/RTTVAR via injected acks; advance `MockClock` to exactly the RTO threshold → retransmit fires on next tick.
4. **RTO clamped to floor 25 ms.** Drive estimator to near-zero RTTVAR + tiny SRTT; assert `current_rto()` returns ≥ 25 ms; verify retransmit timing matches floor.
5. **RTO clamped to ceiling 5 s.** Inflate estimator (large RTT samples); assert `current_rto()` returns ≤ 5 s.
6. **`MAX_RETRY_COUNT = 8` closure.** Each call to `write_retransmit()` increments `retry_count` for **every** unacked entry (`reactor.rs:293`–`305`). Drive 8 successive `TimeoutDriven` retransmits via clock-advance + `tick()`; on the 8th iteration, `retry_count >= MAX_RETRY_COUNT` triggers state→`Closed`, stages `KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED` in `pending_host_fault`, and returns `Err(TransportError::Closed)` from `write_retransmit`. UnackedWindow itself is **not** cleared inside `write_retransmit`; the closed-state cleanup runs on the next `tick_once()` and flushes pending completions. The test asserts: fault staged with the right code; subsequent `tick_once()` returns `TickOutcome::Closed`; pending submissions complete with `TransportError::Closed`.

### 3.3 A3 — `awaiting_response_gc.rs`

Three sub-tests using `MockTransport` (Step 1's high-level RPC mock):

1. **Abandon-on-drop.** Caller drops `CallHandle` mid-flight → AwaitingResponse entry GC'd on next reactor tick → late response silently discarded → depth returns to 0.
2. **Per-entry dispatcher timeout.** Entry exceeds deadline → `KALICO_ERR_HOST_DISPATCHER_TIMEOUT` to caller → entry removed → depth 0.
3. **Disconnect-clears-all.** Reactor disconnect path → every pending entry resolves with `TransportError::Closed` (the variant emitted at `host_io/reactor.rs:485`) → depth 0 post-recovery.

### 3.4 A4 — `#[cfg(test)] mod` inside `src/host_io/reactor.rs`

The `Reactor::run()` loop body (`reactor.rs:503`–`550`) processes phases in fixed order: command drain → serial poll → pending drain → RTO → fault drain → event drain → AwaitingResponse GC → closed exit. Within a single `tick_once()`, a queued submission is processed in step 1 before a queued NAK byte is read in step 2. So same-tick interleaving has a deterministic, *known* shape: new frame writes first, retransmit follows.

The test is a consistency check, not a "what order does the wire see" check. Queue submission of frame N alongside a NAK byte for frame N−2 (via `feed_rx`); call `tick()`; assert:

- `tx_log` contains both the new frame N and the retransmit buffer for N−2 — both go on the wire within the single tick.
- Post-tick `send_seq`, `last_ack_seq`, and UnackedWindow depth are consistent: window contains exactly `{N−2, N−1, N}` if N−2 was the only outstanding entry pre-tick (retransmit doesn't pop it; the in-flight ack does).
- No frame dropped, no duplicates beyond the intended retransmit.

The order-on-wire (new frame first, retransmit second) is documented as observed behavior, not as the invariant under test. The invariant is consistency of internal state.

### 3.5 A5 — `partial_frame_assembly.rs`

Pure proptest against `host_io/wire.rs:extract_packet` — no reactor, no harness. Five strategies:

1. **Mid-length-prefix splits** — every byte boundary inside the leading length byte exercised.
2. **Mid-CRC splits** — splits inside the trailing CRC.
3. **Mid-payload splits.**
4. **Multi-frame chunks** — 2–8 frames packed into one read buffer.
5. **Resync-after-corruption** — insert a random invalid byte (bad SYNC / bad length / bad CRC) before a valid frame; assert `extract_packet` resyncs by dropping bytes one at a time and recovers (the path at `wire.rs:33`).

256 cases per strategy. Pass = 100% recovery.

### 3.6 A6 — `status_arcswap_monotonic.rs`

8 reader threads, 1 writer thread, 1M reads per reader. Writer publishes monotonically-increasing generation values into the status `ArcSwap`. Each reader records its observed sequence; assert: each reader's sequence is monotonically non-decreasing (never gen K then gen K−1). No torn snapshots.

### 3.7 A7 — `clock_sync_drift.rs`

Direct test of `ClockSyncEstimator` with a `MockClock` (no reactor needed). 24 virtual hours of synthetic samples at **50 ppm** firmware drift (well below `MAX_DRIFT_PPM_DEFAULT = 100.0`).

Assert:
- Residual stays within the documented ε (read from `clock_sync.rs` constants at implementation time and pinned in the test).
- Per-MCU `request_id` strictly monotonic across simulated arm attempts.
- `last_sample_age()` and `last_dedicated_sample_age()` age out at the documented threshold (works because the seam routes them through `MockClock`).
- Final residual ≤ same ε; no unbounded accumulation.

**Out of scope:** firmware-side CYCCNT realism — that's 7-D.

## 4. Implementation order

Six phases, each producing a green build with the previous still passing.

1. **Phase 0 — `Clock` trait + production threading (backward-compatible).** Add `src/clock.rs`. For each affected type (`ClockSyncEstimator`, `Reactor`), keep the existing constructor signature (`new(...)`) and have it default to `Arc::new(RealClock)`; add a sibling `new_with_clock(..., clock: Arc<dyn Clock>)` that the harness uses. Internally store the clock and route all §2.3 call sites through it. Existing call sites in `mod.rs:164`, the `#[cfg(test)]` blocks at `reactor.rs:640`/`:904`/`:946`, and existing `ClockSyncEstimator` tests do not change. Commit.
2. **Phase 1 — `tick_once()` extraction.** Refactor `Reactor::run()` per §2.4. Existing tests pass unchanged. Commit.
3. **Phase 2 — Reactor harness.** Add `tests/reactor_harness.rs` and the `#[cfg(test)]` `Reactor` constructor. Smoke test in the harness file: build, submit, tick, assert empty tx_log when no rx fed. Commit.
4. **Phase 3 — A1, A2, A4 (harness consumers).** All three together — they share scaffolding. Commit per file or batched.
5. **Phase 4 — A3, A5, A6, A7 (independent of harness).** Can land in any order; A5 is pure parser, A6 is pure ArcSwap, A3 reuses MockTransport, A7 uses MockClock directly.
6. **Phase 5 — Optional capture-corpus surface assertion.** If a baseline `tests/captures/sim-default-baseline.bin` happens to land separately (e.g. someone runs the legacy diff-test path), un-ignore `corpus_covers_required_decode_surfaces`. If not, leave it ignored — its un-ignoring is not gating 7-C-io completion.

After Phase 4: 7-C-io is declared done. CLAUDE.md flips `[~]` → `[x]`.

## 5. Cutover

1. **`CLAUDE.md` build-order entry for 7-C-io:** flip `[~]` → `[x]`. Replace the "Phase F pending" parenthetical with a one-line note that the deterministic battery is in place; sim-soak / canonical corpus / `python-diff-test` retirement / hardware-only items moved to 7-D.
2. **Update parent spec's §9 Phase F.** Replace with a back-pointer to this spec; preserve the "deferred to 7-D" enumeration with new items added (sim-soak, etc.).
3. **Plan-changes log entry** in `docs/superpowers/plan-changes-log.md`: 7-C-io tail / 7-D scope shift, with explicit notes that (a) the Renode sim-soak moved entirely to 7-D, and (b) `python-diff-test` retirement moved from 7-C-io tail to 7-D.
4. **`python-diff-test` stays alive.** No Cargo.toml changes.

## 6. Handoff to 7-D

Items inherited unchanged in shape:

1. **Renode sim-soak driver + workload.** When 7-D builds the bench setup, a sim-soak with the originally-considered scenarios (trajectory loop, disconnect/reconnect injection, backpressure flood, multi-MCU correlation, capture tap) is straightforward to add against either Renode-on-bench or hardware directly. The deterministic battery from this spec gives 7-D a green-baseline reference for the bugs already eliminated.
2. **Canonical capture corpus + `python-diff-test` retirement.** `soak --port /dev/cu.usbmodem...` against H723; commit captures to `tests/captures/h723-<scenario>-<timestamp>.bin`. **At this point** the parent spec's Phase-1 "canonical CI reference" cutover lands AND `python-diff-test` retires (Cargo feature dropped, CI lane removed, python-side bootstrap differential code deleted). This is the parent spec §4.13 cutover, finally honored.
3. **24h wall-clock soak.** Real-time bounds (status snapshot recovery, channel-recovery thresholds) at hardware wall-clock rates; canonical leak coverage; USB-CDC byte-sequence fidelity; physical unplug semantics; IWDG real-world pacing.
4. **Surface-C cycle-budget actuals.**

The `Clock` trait from this spec is the right structure for any future timing-dependent test 7-D adds.

## 7. Risks & mitigations

| Risk | Mitigation |
|---|---|
| `Clock` threading is more invasive than estimated | Phase 0 lands in isolation; if call sites multiply, scope-creep contained to Phase 0. The §2.3 table is the bound. |
| `tick_once()` extraction breaks `run()` semantics under concurrent submissions | Existing integration tests (`producer_unit.rs`, `subscriber_api.rs`, etc.) regress-check this. Phase 1 ships only when they're green. |
| Fake `SerialPort` impl in the harness drifts from real `serialport::SerialPort` semantics | Keep the fake minimal — only the methods the reactor actually calls. Audit the calls before writing the fake. |
| `MAX_DRIFT_PPM_DEFAULT` or other clock_sync constants change without updating A7 | A7 reads constants from `clock_sync.rs` directly rather than hardcoding them. |
| A1's wrap-boundary tests are brittle if `decode_absolute` signature changes | Tests target observable behavior (counters and depths) not internal computation. |

## 8. Why no Renode sim-soak in this step

A Renode-based sim-soak was specified through two prior revisions of this document. It is dropped here. The reasoning, recorded for future reference:

**What sim-soak could catch in principle:** memory leaks over long runs; reactor livelock under sustained backpressure; disconnect/reconnect race recovery; async-event subscriber drop policy; multi-MCU `request_id` correlation; frame-routing under simultaneous NAK + new submission.

**Why each fails the cost/benefit gate in sim:**

- *Memory leaks* — one-hour RSS curve cannot reliably distinguish slow leaks from allocator/cache noise. Canonical leak coverage is inherently 7-D's longer hardware soak.
- *Reactor livelock under backpressure* — partially catchable in the harness. The harness can flood the submission channel beyond the per-tick drain budget (`MAX_SUBMITS_PER_ITER = 4` at `reactor.rs:505`) and observe drain-budget consumption deterministically; it can also exercise the AwaitingResponse-GC path. What it does NOT replicate is realistic `Reactor::run()`-loop pacing under sustained real-time scheduling pressure — that's a sim-or-hardware concern. The harness gives us a deterministic "did the loop body progress correctly under contention" signal, not a "would the production loop wedge under load" signal. The latter is 7-D's bench soak.
- *Disconnect/reconnect race* — sim disconnects are TCP socket drops, not USB-CDC `BrokenPipe`/`EIO`; the host-state-machine logic is exercisable through the reactor harness's `feed_rx(eof_sentinel)` + reconnect path. Real unplug semantics are 7-D.
- *Subscriber drop policy* — pure logic; deterministic unit tests against `EventDispatcher` are sharper than soak observation.
- *Multi-MCU `request_id` correlation* — the harness can construct two `Reactor` instances and exercise the same `arm_all_mcus` path that production hits; no Renode needed.
- *Simultaneous NAK + new submission* — A4 already covers this deterministically in the reactor harness.

**Plus the structural costs the sim-soak would have imposed:**

- A `socket://` `Transport` adapter in production code (concrete `SerialPort` impl over `TcpStream`).
- A capture-tap design that accommodates bidirectional flow (one-way capture cannot satisfy `REQUIRED_SURFACES` because host commands like `kalico_push_segment` only appear on the host→MCU direction).
- A Renode pacing-factor calibration step every time soak thresholds are evaluated.
- A soak-driver binary, multi-MCU launch script, metrics CSV emitter, etc.

**What we lose by dropping it:** mostly nothing concrete. The sim-soak's strongest signal was "did anything wedge over many hours" which only the 7-D bench soak actually delivers; sim was always interim.

**What we gain:** scope locked to a focused, real-bug-finding battery. The `Clock` trait + `tick_once()` refactor is the right structural change regardless of whether soak ever ships.

## 9. References

- [Step 7-C-io spec](2026-04-30-step-7c-io-design.md) — parent spec; this spec rewrites its §9 Phase F. Parent §4.13 defines the canonical-corpus cutover that gates `python-diff-test` retirement.
- `rust/kalico-host-rt/src/host_io/reactor.rs` — reactor loop, drain budget, GC layers, `Instant::now()` call sites at lines 153/231/390/524/540.
- `rust/kalico-host-rt/src/host_io/window.rs` — `UnackedWindow::pop_acked` strict-`<` semantics at line 35; `sent_at: Instant::now()` at line 136.
- `rust/kalico-host-rt/src/host_io/mod.rs` — `KalicoHostIo::open_with_config` at line 139; `call`/`call_typed` deadline computation at lines 190/229.
- `rust/kalico-host-rt/src/host_io/wire.rs` — `extract_packet` resync path at line 33.
- `rust/kalico-host-rt/src/clock_sync.rs` — `last_sample_age` / `last_dedicated_sample_age` use real `Instant::elapsed()` at lines 100/183; `MAX_DRIFT_PPM_DEFAULT = 100.0`.
- `rust/kalico-host-rt/src/transport.rs` — `TransportError::Closed` (no `Disconnected` variant).
- `rust/kalico-host-rt/tests/captures_replay.rs` — corpus-replay scaffold; `REQUIRED_SURFACES`.
- `tools/sim/README.md` — Renode pacing limits (0.05–0.5x wall-clock) that informed §8's drop-rationale.
