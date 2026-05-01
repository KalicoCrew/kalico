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
    clock_sync.rs                            # CHANGED — every Instant::now() routes through &impl Clock
    host_io/
      reactor.rs                             # CHANGED — extract tick_once(); thread Clock
      window.rs                              # CHANGED — thread Clock for sent_at/submitted_at
      mod.rs                                 # CHANGED — thread Clock for deadline computation in call/call_typed
  tests/
    reactor_harness.rs                       # NEW — harness module imported by A1/A2/A4
    reactor_seq_wrap.rs                      # NEW — A1
    nak_rto_branches.rs                      # NEW — A2
    awaiting_response_gc.rs                  # NEW — A3 (uses MockTransport)
    nak_submit_race.rs                       # NEW — A4
    partial_frame_assembly.rs                # NEW — A5 (pure parser test, no harness)
    status_arcswap_monotonic.rs              # NEW — A6
    clock_sync_drift.rs                      # NEW — A7 (uses MockClock directly, no reactor)
```

No new crate dependencies. `Cargo.toml` unchanged. `python-diff-test` feature stays.

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

These are the exact `Instant::now()` and `Instant`-typed-construction sites in production code that need the `Clock`. (Test-only `Instant::now()` calls in existing test files stay as-is — they test against `RealClock` semantics.)

| File | Line | Context | Treatment |
|---|---|---|---|
| `host_io/reactor.rs` | 153 | `let now = Instant::now()` for `sent_at` on submission | `clock.now()` |
| `host_io/reactor.rs` | 231 | `Instant::now() - entry.sent_at` for RTT sample | `clock.now() - entry.sent_at` |
| `host_io/reactor.rs` | 390 | `let now = Instant::now()` (close-deadline path) | `clock.now()` |
| `host_io/reactor.rs` | 524 | `let now = Instant::now()` for RTO check | `clock.now()` |
| `host_io/reactor.rs` | 540 | `let now = Instant::now()` for AwaitingResponse layer-2 GC | `clock.now()` |
| `host_io/window.rs` | 136 | `sent_at: Instant::now()` in `UnackedWindow::push` | take `now: Instant` param; reactor passes `clock.now()` |
| `host_io/mod.rs` | 190 | `let deadline = Instant::now() + timeout` in `call()` | `clock.now() + timeout` |
| `host_io/mod.rs` | 229 | same in `call_typed()` | `clock.now() + timeout` |
| `clock_sync.rs` | 102 | `epoch: Instant::now()` in `ClockSyncEstimator::new` | take `clock: Arc<dyn Clock>`; store `clock.now()` |
| `clock_sync.rs` | 183 | `let now = Instant::now()` in `add_dedicated_sample` | `self.clock.now()` |
| `clock_sync.rs` | 206 | `recorded_at: Instant::now()` in `add_piggyback_sample` | `self.clock.now()` |

`reactor.rs:651` and `reactor.rs:918` are inside test code — left as `Instant::now()` (those tests are not part of this work).

### 2.4 `tick_once()` extraction

`Reactor::run()` is currently a loop containing: drain submission queue → poll serial → service unacked window (RTO) → AwaitingResponse layer-2 GC → handle close requests. Extract one iteration's body into `pub(crate) fn tick_once(&mut self)`. `run()` becomes `while !self.closed { self.tick_once() }`. No production behavior changes.

The harness drives the reactor by calling `tick_once()` directly between rx-byte injections and clock advances. No thread spawn; no real serial port.

### 2.5 Reactor harness — `tests/reactor_harness.rs`

```rust
pub struct ReactorHarness {
    reactor: Reactor,             // built without spawning a thread
    clock:   Arc<MockClock>,
    rx_pipe: Arc<Mutex<VecDeque<u8>>>,  // injected via a fake SerialPort impl
    tx_log:  Arc<Mutex<Vec<u8>>>,       // captured frames the reactor would have written
}

impl ReactorHarness {
    pub fn new() -> Self { /* construct Reactor with fake SerialPort + MockClock */ }
    pub fn feed_rx(&mut self, bytes: &[u8]);
    pub fn advance_clock(&mut self, by: Duration);
    pub fn tick(&mut self);                                      // wraps tick_once
    pub fn submit(&self, cmd: &str, deadline: Instant) -> CallHandle;
    pub fn unacked_depth(&self) -> usize;
    pub fn awaiting_depth(&self) -> usize;
    pub fn tx_log(&self) -> Vec<u8>;
    pub fn force_rto(&mut self);   // advance clock past current_rto for all entries
}
```

The fake `SerialPort` impl reads from `rx_pipe` (returning 0 bytes when empty, like a non-blocking read with no data), writes to `tx_log`, and supports the trait methods the reactor calls. Lives in `reactor_harness.rs`, not in production code.

`#[cfg(test)]` guards the test-only `Reactor` constructor that takes a fake port and a `MockClock`. The production `Reactor::new` path is unchanged.

## 3. Test specifications

Each test file targets one concern. Pass criteria are concrete; no statistical thresholds.

### 3.1 A1 — `reactor_seq_wrap.rs`

Three boundary cases via the harness: low (counter 15→16), mid (near 2³¹), high (near 2⁶³−16). For each:

- Submit frames until just below the boundary; advance `MockClock` so RTOs don't fire.
- Inject ack frames whose 4-bit wire seq forces `decode_absolute` across the boundary.
- Assert: `last_ack_seq` advances monotonically in absolute space; `UnackedWindow::pop_acked` removes only entries with `entry.seq < rseq` (entries with `seq == rseq` stay — see `host_io/window.rs:35`); `ignore_nak_seq` damper's absolute-seq comparison dominates the 4-bit wire equality so it never matches a frame from a previous wrap epoch.
- No panics; depth returns to expected value after each boundary cross.

### 3.2 A2 — `nak_rto_branches.rs`

Six sub-tests, one per branch. All use harness + `MockClock`.

1. **Duplicate-ack triggers retransmit.** `feed_rx` an ack-with-data, then `feed_rx` an immediate duplicate ack on the same seq → `tx_log` shows the original frame retransmitted.
2. **`ignore_nak_seq` suppresses paired second NAK.** Two duplicate acks in quick succession on the same seq → exactly one retransmit in `tx_log`.
3. **RTO fires at SRTT + 4·RTTVAR.** Submit frame; let estimator reach a known SRTT/RTTVAR via injected acks; advance `MockClock` to exactly the RTO threshold → retransmit fires on next tick.
4. **RTO clamped to floor 25 ms.** Drive estimator to near-zero RTTVAR + tiny SRTT; assert `current_rto()` returns ≥ 25 ms; verify retransmit timing matches floor.
5. **RTO clamped to ceiling 5 s.** Inflate estimator (large RTT samples); assert `current_rto()` returns ≤ 5 s.
6. **`MAX_RETRY_COUNT = 8` closure.** Force 8 successive RTOs on the same seq → `KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED` emitted; UnackedWindow cleared.

### 3.3 A3 — `awaiting_response_gc.rs`

Three sub-tests using `MockTransport` (Step 1's high-level RPC mock):

1. **Abandon-on-drop.** Caller drops `CallHandle` mid-flight → AwaitingResponse entry GC'd on next reactor tick → late response silently discarded → depth returns to 0.
2. **Per-entry dispatcher timeout.** Entry exceeds deadline → `KALICO_ERR_HOST_DISPATCHER_TIMEOUT` to caller → entry removed → depth 0.
3. **Disconnect-clears-all.** Reactor disconnect path → every pending entry resolves with `TransportError::Closed` (the variant emitted at `host_io/reactor.rs:485`) → depth 0 post-recovery.

### 3.4 A4 — `nak_submit_race.rs`

Single ordering test via harness. The harness's `feed_rx` and `submit` queue events; one `tick()` processes them in defined order. The test queues a NAK for frame N−2 alongside a fresh submission of frame N, ticks, and asserts:

- Retransmit of N−2 appears in `tx_log` before frame N.
- Post-tick `send_seq` and UnackedWindow are consistent.
- No frame dropped or duplicated.

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

1. **Phase 0 — `Clock` trait + production threading.** Add `src/clock.rs`. Thread `Arc<dyn Clock>` through every site in §2.3. Wire `RealClock` everywhere production constructs the affected types. Existing tests pass unchanged. Commit.
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
- *Reactor livelock under backpressure* — catchable, but the reactor harness's `tick()` loop with sustained submission + zero ack drain catches the same starvation deterministically and faster.
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
