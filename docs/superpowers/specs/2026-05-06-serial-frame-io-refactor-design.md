# SerialFrameIo refactor — design spec

**Date:** 2026-05-06
**Status:** Brainstormed; ready for implementation plan
**Related:** Step 7-D (Hardware bring-up + first print), reverted commits `df07d5a03` / `9c5dedc33`, follow-on to `599e92c5b`

## 1. Problem

Real H7 bring-up (2026-05-06) reproduces a deterministic failure: the first `bridge_call` after `identify_handshake` (concretely, clocksync `get_uptime`) times out, blocking homing. Two prior point-fixes were attempted and reverted:

- **`df07d5a03`** — demuxed reactor's `rx_buf_initial` at construction, forwarded only `KlipperFrame` outputs into `rx_buf`. Reverted: re-injected the already-consumed `identify_response` and desynced reactor seq state.
- **`9c5dedc33`** — dropped `rx_buf_initial` entirely. Reverted: lost partial in-flight Klipper bytes (which lived in identify's local demuxer state, not in `rx_buf`).

Both attempts patched symptoms. The structural fault is a seam: the host runs **two independent `Demuxer` instances** (one in identify, one in reactor) with a raw-byte handoff via `rx_buf_initial`. Three coupled bugs ride that seam:

1. **Seed-bytes-bypass-demuxer.** Identify forwards raw bytes to the reactor; reactor assigns them straight into `self.rx_buf`; the legacy `wire::extract_packet` parser sees kalico-native `0x55` chatter (firmware emits ~10 Hz from boot), enters O(n²) byte-by-byte resync, and exceeds the per-attempt deadline.
2. **Demuxer-state-loss-on-handoff.** A partial Klipper frame mid-stream in identify's last read lives in identify's `Demuxer::InsideKlipper` buffer. Dropping or transplanting raw bytes either loses or duplicates this state.
3. **Latent seq-state hardcoding.** `Reactor::new` sets `send_seq: 1, receive_seq: 1` regardless of identify's actual seq progress. Identify burns sequences `0..K` (`identify.rs:72` increments after each request), then returns `seq` to `mod.rs:240` which discards it as `_seq`. The first sequence-tracked `bridge_call` issues `wire_seq=1` to a firmware that's already ACK'd up to identify's last request — H7 firmware doesn't tolerate this, F4-class firmware sometimes does.

The H7 timeout is the visible symptom of (1) + (3) compounding. Codex's 10-line "filter rx_buf to validated klipper leftovers only" variant addresses (1) but not (3); it would have failed identically to `df07d5a03` on H7. The architecturally honest fix closes the seam.

## 2. Goal

Replace the two-Demuxer-with-raw-byte-handoff arrangement with a single owner of the wire (port + demuxer + scratch + pending stream errors) that survives the identify→reactor handoff by value. The seam — and with it the bug class — becomes structurally unrepresentable.

### 2.1 Non-goals

- Async/threaded I/O. Reactor stays single-threaded synchronous.
- Identify-as-reactor-coroutine (subsumed handshake). Out of scope; future cleanup.
- Touching kalico-frame semantics (already correctly CRC-validated by the demuxer).
- MCU-side, planner, or write-path changes.

## 3. Architecture

### 3.1 New types

In `kalico-native-transport`:

```rust
pub enum Frame {
    Klipper(KlipperFrame),
    Kalico { channel: u8, payload: Vec<u8> },
}

pub struct KlipperFrame { bytes: Vec<u8> }   // length + CRC + 0x7E trailer all validated
impl KlipperFrame {
    pub fn seq_byte(&self) -> u8;            // bytes[1]
    pub fn body(&self) -> &[u8];             // bytes[2 .. len-3]
    pub fn bytes(&self) -> &[u8];            // tests/replay
    pub fn into_bytes(self) -> Vec<u8>;      // retransmit/await-response stash
}

pub enum StreamError {
    KlipperCrcMismatch    { seq: u8, expected: u16, actual: u16 },
    KlipperBadTrailer     { got: u8 },
    KlipperLenOutOfRange  { len: u8 },
    KalicoCrcMismatch     { channel: u8, expected: u16, actual: u16 },
    KalicoLenBelowMin     { len: u16 },
    KalicoFrameTooShort   { got: usize },
}

pub enum PollOutcome {
    Frames { frames: Vec<Frame>, errors: Vec<StreamError> },
    Timeout,
    PhantomZero,
}

// Test-only generic. No write side; corpus replay reads only.
pub struct FrameSource<R: Read> {
    reader: R,
    set_timeout: Box<dyn FnMut(&mut R, Duration) -> io::Result<()>>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
    pending_errors: Vec<StreamError>,
}
impl<R: Read> FrameSource<R> {
    pub fn from_read_no_timeout(reader: R) -> Self;
    pub fn poll_frames_until(&mut self, deadline: Instant)
        -> Result<PollOutcome, TransportError>;
}
```

In `kalico-host-rt::host_io`:

```rust
pub struct SerialFrameIo {
    port: Box<dyn SerialPort>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
    pending_errors: Vec<StreamError>,
}
impl SerialFrameIo {
    pub fn new(port: Box<dyn SerialPort>) -> Self;
    pub fn poll_frames_until(&mut self, deadline: Instant)
        -> Result<PollOutcome, TransportError>;

    // Raw byte passthrough. Does not validate, frame, or re-shape outbound bytes.
    // Both Klipper-shaped frames (build_frame) and Kalico-native frames
    // (KalicoIdentify::build_*) are pre-built by their encoders and written verbatim.
    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError>;
    pub fn flush(&mut self) -> Result<(), TransportError>;
}

pub struct IdentifySeqState {
    pub next_send_seq_abs: u64,
    pub mcu_receive_seq_abs: u64,
}
```

### 3.2 Crate-boundary rationale

`SerialFrameIo` lives in `kalico-host-rt` because it owns `Box<dyn SerialPort>` (a hardware-I/O concern). `FrameSource<R: Read>` and the `Frame` / `KlipperFrame` / `StreamError` / `PollOutcome` types live in `kalico-native-transport` (no `serialport` dep, reusable by EtherCAT backend, corpus replay, fuzz harness). The demuxer logic itself is centralized in `kalico-native-transport::demux`; both `SerialFrameIo` and `FrameSource` embed it.

`SerialFrameIo` calls `self.port.set_timeout(...)` directly because `SerialPort` is a known trait. `FrameSource` over `R: Read` needs the closure indirection because `std::io::Read` has no timeout concept; the closure adapter is the seam between the abstract reader and the concrete deadline. The closure returns `io::Result<()>` so `set_timeout` failures propagate rather than being silently erased.

`KlipperFrame { bytes: Vec<u8> }`'s field is **private**. Public access goes through `seq_byte()`, `body()`, `bytes()`, `into_bytes()`. This keeps the validation invariant ("the bytes inside passed length+CRC+trailer checks") unforgeable.

### 3.3 Lifetime contract

Identify and reactor share **one** `SerialFrameIo` instance:

1. `mod.rs::open_with_port` constructs `SerialFrameIo::new(port_box)`.
2. `identify_handshake(&mut io, timeout)` runs, captures the seq nibble of every accepted `identify_response`, walks an absolute-seq counter via `decode_absolute`, and returns `(parser, raw_identify_blob, IdentifySeqState)`.
3. The reactor thread is spawned with the live `SerialFrameIo` moved by value into `Reactor::new_with_clock(io, parser, ..., seq, ...)`.
4. The demuxer state, the port, the pending stream errors — all carry over with no transplant. There is no `rx_buf_initial`. There is no second demuxer.

### 3.4 Demuxer changes

`Demuxer::feed` gains symmetric Klipper-frame validation. When the trailing byte arrives in `InsideKlipper`:

- Validate CRC16-CCITT over `bytes[0..len-3]` (length byte + seq byte + payload) against `bytes[len-3..len-1]` interpreted big-endian (`hi << 8 | lo`). Per `wire::build_frame` / `extract_packet`.
- Validate `bytes[len-1] == 0x7E` (MESSAGE_SYNC).
- On any failure, emit the appropriate `StreamError` variant **and** trigger 1-byte-shift resync.
- On success, emit `Frame::Klipper(KlipperFrame { bytes })`.

**1-byte-shift resync algorithm (precise):** `Demuxer` gains an internal field `replay: VecDeque<u8>`. On Klipper-completion failure: reset `state = WaitingForFrame`, push `bytes[1..]` (i.e. drop only the false-latch length byte) into `replay`. `Demuxer::feed` and `Demuxer::feed_slice` always drain `replay` first before consuming any new live byte — each replayed byte goes through `feed` exactly as a freshly-arrived byte would. This preserves the `demux.rs:13` invariant ("byte-oriented and interruptible at any boundary") and makes recovery deterministic. Worst-case work per failure is O(64²) (max-Klipper-frame window squared via successive failed re-latches), bounded; today's `extract_packet` resync is unbounded in `rx_buf` length.

Kalico-frame validation already exists at `parse_kalico_frame`; that path is unchanged except for emission shape (`Frame::Kalico` instead of `DemuxOutput::KalicoFrame`). Existing kalico-side demux tests are migrated to assert against the new `Frame` shape.

### 3.5 PollOutcome semantics

`poll_frames_until(deadline)`:

- `port.set_timeout(deadline.saturating_duration_since(now))?` — propagates a real `Err` (timeout-setting can fail; do not silently erase).
- `port.read(&mut scratch)`:
  - `Ok(n) if n > 0` → feed demuxer; collect frames + drained pending_errors into `PollOutcome::Frames { frames, errors }`.
  - `Ok(0)` → `PollOutcome::PhantomZero` (caller debounces; reactor's `ZERO_BYTE_DEBOUNCE` policy lives in `poll_serial`, not in `SerialFrameIo`).
  - `Err(TimedOut | Interrupted | WouldBlock)` → `PollOutcome::Timeout`.
  - `Err(other)` → `Err(TransportError::Io(e))`.

The reactor's existing three-way fork (timeout no-op / Ok(0) debounce → HostDisconnect / Err immediate-disconnect) is preserved exactly; only its byte-level dispatch becomes frame-level.

`FrameSource` callers (corpus replay, fuzz) get the same `PollOutcome` shape. For them, `Ok(0)` from `Read` typically means EOF, so callers should treat `PollOutcome::PhantomZero` as terminal — there's no debounce-into-disconnect equivalent at that layer.

## 4. Migration plan

### 4.1 `kalico-native-transport`

- **`demux.rs`** — replace `DemuxOutput` with `Frame { Klipper(KlipperFrame), Kalico { channel, payload } }`. Add Klipper CRC + trailer validation in `InsideKlipper` completion path with 1-byte-shift resync on failure. Add `pending_errors: Vec<StreamError>` queue (drained by `FrameSource` / `SerialFrameIo`).
- **`frame_source.rs` (new)** — `FrameSource<R: Read>` per §3.1.
- **`frame.rs`** — single source of `crc16_ccitt`. (`kalico-host-rt::wire::crc16_ccitt` was a duplicate; that one becomes a `pub use` re-export.)
- **`lib.rs`** — re-export `Frame`, `KlipperFrame`, `StreamError`, `PollOutcome`, `FrameSource`.

### 4.2 `kalico-host-rt`

- **`host_io/serial_frame_io.rs` (new)** — `SerialFrameIo` per §3.1. ~80 LOC: constructor, `poll_frames_until`, `write_all`, `flush`.
- **`host_io/identify.rs`** — signature changes:
  ```rust
  pub fn identify_handshake(
      io: &mut SerialFrameIo,
      timeout: Duration,
  ) -> Result<(MsgProtoParser, Vec<u8>, IdentifySeqState), TransportError>;
  ```
  Internal state: `next_send_seq_abs: u64` initialized to 1 (matches reactor's pre-refactor default), incremented after each frame written; `mcu_recv_abs: u64` initialized to 0. Loop calls `io.poll_frames_until(attempt_deadline)`. On `PollOutcome::Frames { frames, errors }`: log errors, iterate frames; for **every** `Frame::Klipper(f)` emitted by the demuxer (which is already CRC- and trailer-validated per §3.4), update `mcu_recv_abs = wire::decode_absolute(mcu_recv_abs, f.seq_byte() & MESSAGE_SEQ_MASK)` *before* attempting `decode_identify_response(f.body())`. Drop `Frame::Kalico` silently. Wire-seq written into outbound frames is `(next_send_seq_abs as u8) & MESSAGE_SEQ_MASK`. Drain phase implemented as `poll_frames_until` loop with discard. On success returns `IdentifySeqState { next_send_seq_abs, mcu_receive_seq_abs: mcu_recv_abs }`. Walking on every Klipper frame (not just identify_response matches) keeps `mcu_recv_abs` consistent with the wire even if firmware sends a stray non-identify Klipper frame during handshake.
- **`host_io/reactor.rs`**:
  - Replace fields `port: Box<dyn SerialPort>`, `rx_buf: Vec<u8>`, `kalico_demuxer: Demuxer` with single `io: SerialFrameIo`.
  - `Reactor::new_with_clock` signature:
    ```rust
    pub fn new_with_clock(
        io: SerialFrameIo,
        parser: Arc<MsgProtoParser>,
        submissions: mpsc::Receiver<Submission>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        seq: IdentifySeqState,
        config: KalicoHostIoConfig,
        clock: Arc<dyn Clock>,
    ) -> Self;
    ```
    Initialize `send_seq = seq.next_send_seq_abs`, `receive_seq = seq.mcu_receive_seq_abs`, `last_ack_seq = seq.mcu_receive_seq_abs.saturating_sub(1)`. Drop hardcoded `send_seq: 1, receive_seq: 1`. After identify the `unacked_window` is empty, so the first inbound ack hits the first-connection-sentinel branch in `update_receive_seq` (`reactor.rs:431`) and snaps state — `last_ack_seq.saturating_sub(1)` is therefore advisory for the empty-window case and any underflow is harmless.
  - Add `Reactor::new_for_tests(port: Box<dyn SerialPort>, ...)` behind `#[cfg(any(test, feature = "test-harness"))]` that wraps the port internally. The function body literally constructs `IdentifySeqState { next_send_seq_abs: 1, mcu_receive_seq_abs: 1 }` — **no `Default` impl** on the public type, since `{ 1, 1 }` is not a meaningful default outside the test path. Preserves existing ~62 tests unchanged.
  - `write_frame` rewrites to `self.io.write_all(bytes)?; self.io.flush()`.
  - `poll_serial` becomes:
    ```rust
    match self.io.poll_frames_until(self.clock.now() + READ_TIMEOUT) {
        Ok(PollOutcome::Frames { frames, errors }) => {
            self.zero_byte_first_seen = None;
            for e in errors { log::warn!("stream error: {e}"); }
            for f in frames {
                match f {
                    Frame::Klipper(kf) => if self.handle_inbound_frame(kf).is_err() { return; },
                    Frame::Kalico { channel, payload } => self.handle_kalico_frame(channel, &payload),
                }
            }
        }
        Ok(PollOutcome::Timeout) => { self.zero_byte_first_seen = None; }
        Ok(PollOutcome::PhantomZero) => { /* existing ZERO_BYTE_DEBOUNCE policy */ }
        Err(_) => { /* existing immediate-disconnect policy */ }
    }
    ```
    `READ_TIMEOUT` deadline lives at the `poll_serial` callsite, not in `SerialFrameIo` — identify's long deadlines do not leak into the reactor's tick budget.
  - `handle_inbound_frame(&mut self, frame: KlipperFrame) -> Result<(), Closed>` (was `Vec<u8>`). Body access via `frame.body()`. Stash for retransmit (if any inbound stash exists; none today) via `frame.into_bytes()`.
  - Hoist `decode_absolute` from method on `Reactor` to free function `wire::decode_absolute(prev_abs: u64, wire_seq: u8) -> u64`. Identify calls it directly. Existing reactor callsites (`reactor.rs:472,553`) update to `wire::decode_absolute(self.receive_seq, wire_seq)` — the `&self.receive_seq` read becomes an explicit parameter, no behavior change.
- **`host_io/wire.rs`** — `extract_packet` retained for offline use (three integration tests + pin tests depend on it); marked `#[doc(hidden)]` and removed from reactor's hot path. Reactor's own test module (`reactor.rs:1918, 1961, 2048, 2080`) continues to call `extract_packet` to decode frames the reactor wrote through `write_all`; this is correct since `write_all` is raw passthrough. `crc16_ccitt` becomes `pub use kalico_native_transport::frame::crc16_ccitt`.
- **`host_io/mod.rs::open_with_port`** — constructs `SerialFrameIo::new(port_box)`, hands `&mut` to identify, moves by value into the reactor thread. The `_seq` discard becomes a real `IdentifySeqState` plumbed into `Reactor::new_with_clock`.

### 4.3 Deletions / closeouts

- `Reactor::rx_buf` field
- `Reactor::kalico_demuxer` field
- `rx_buf_initial` parameter on `Reactor::new` / `Reactor::new_with_clock`
- `kalico_native_transport::demux::DemuxOutput` enum (replaced by `Frame` + `StreamError`)
- `identify_handshake` returning `rx_buf: Vec<u8>` (replaced by `IdentifySeqState`)
- `Reactor::new` hardcoded `send_seq: 1, receive_seq: 1` (relocated into `Reactor::new_for_tests`'s test-only `IdentifySeqState { 1, 1 }` literal; production path adopts real values from identify)

### 4.4 Suggested commit split

Two commits on the same branch, both required to land together. Merge gated on §5.4 (A1–A7 green), §5.5 (Renode soak passes), and §5.6 (H7 `clocksync get_uptime` returns within deadline).

1. **`SerialFrameIo + Frame enum + drain-side B + CRC consolidation`** — adds `SerialFrameIo`, `FrameSource`, `Frame`, `KlipperFrame`, `StreamError`, `PollOutcome`. Demuxer gains Klipper validation + 1-byte-shift resync. CRC consolidated. `extract_packet` scoped to offline. Reactor's `poll_serial` + `write_frame` wired to `SerialFrameIo`. `rx_buf` and `kalico_demuxer` fields deleted; `rx_buf_initial` parameter removed from `Reactor::new`/`new_with_clock`. **Identify migrates to `&mut SerialFrameIo`** and returns `(MsgProtoParser, Vec<u8>)` (no seq state yet — return type drops the trailing `rx_buf` element entirely). Reactor still constructs with hardcoded `send_seq: 1, receive_seq: 1` on its side; the `_seq` site at `mod.rs:240` no longer exists because identify's signature no longer carries it. After this commit the H7 timeout symptom is unresolved (latent seq hardcode remains), but the seam is closed.
2. **`IdentifySeqState + reactor seq plumbing`** — identify captures seq nibble per Klipper frame, walks absolute counter, returns `IdentifySeqState`. `Reactor::new_with_clock` adopts seq from state. `decode_absolute` hoisted to free function. Reactor's hardcoded `send_seq: 1, receive_seq: 1` deleted (relocated to `Reactor::new_for_tests` literal). After this commit the H7 timeout regression test (§5.2) passes.

## 5. Tests

### 5.1 `kalico-native-transport`

**`demux.rs` (extending 7 existing tests):**
- `klipper_bad_crc_emits_stream_error_then_resyncs`
- `klipper_bad_trailer_emits_stream_error`
- `klipper_bad_crc_followed_immediately_by_valid_frame` — pins 1-byte-shift resync.
- `klipper_false_length_latch_recovers`
- `partial_klipper_frame_state_survives_across_feed_calls`

**`frame_source.rs`:**
- `poll_frames_until_returns_timeout_when_reader_yields_nothing`
- `poll_frames_until_returns_phantom_zero_on_ok_zero_read`
- `poll_frames_until_returns_io_error_on_other_errors`
- `poll_frames_until_propagates_set_timeout_error` — closure returns `Err`; assert it surfaces as `Err(TransportError::Io)`.
- `poll_frames_until_returns_frames_in_arrival_order`
- `poll_frames_until_carries_stream_errors_alongside_frames`

### 5.2 `kalico-host-rt`

**`serial_frame_io_tests.rs`:**
- `write_all_passes_klipper_bytes_through_unmodified`
- `write_all_passes_kalico_bytes_through_unmodified`
- `flush_called_after_write_all_in_reactor_path`

**Identify:**
- `identify_returns_seq_state_with_correct_absolute_decode`
- `identify_seq_state_walks_across_multiple_responses` — pins multi-response walk including a wrap.
- `identify_drops_kalico_frames_silently`
- `identify_logs_stream_errors_but_continues`
- `identify_drain_phase_handles_dirty_reconnect`

**Reactor seq adoption:**
- `reactor_adopts_send_seq_from_identify_state`
- `reactor_adopts_receive_seq_from_identify_state`
- **`reactor_first_bridge_call_after_identify_succeeds_with_nonzero_initial_seq`** — H7 regression test. Mandatory.

**Handoff (architect-mandated):**
- **`partial_klipper_frame_survives_identify_to_reactor_handoff`** — single shared `SerialFrameIo` over `FakeSerialPort`. Scripted port delivers complete `identify_response` followed by partial bytes of a next Klipper frame in identify's final read; reactor's first poll completes the frame. Mandatory — would have caught both `df07d5a03` and `9c5dedc33`.
- `no_pending_stream_errors_leak_from_identify_to_reactor` — pin the contract that identify drains and logs all stream errors before returning, so reactor's first `drain_stream_errors` is empty. Defends against silent error carryover.

### 5.3 Integration tests

- `tests/captures_replay.rs`, `tests/partial_frame_assembly.rs`, `tests/passthrough_integration.rs` — continue using `wire::extract_packet` (offline), no changes.
- `tests/serial_frame_io_handoff.rs` (new) — full-fidelity `KalicoHostIo::open_pipe_with_config` over a PTY with scripted bytes including identify response + partial follow-on + interleaved kalico_status.

### 5.4 A1–A7 deterministic battery

All 7 tests pass unchanged via `Reactor::new_for_tests`. `feed_rx`/`drain_tx`/`tx_log` remain transparent — they operate on `FakeSerialPort`'s shared inner buffers. A5 (partial-frame TCP-style assembly) is the most relevant regression check.

### 5.5 Renode soak

Re-run Phase-2 harness after both commits land. `G1 X10` / `G1 Z5` segment dispatch + clocksync `get_uptime` end-to-end on simulated hardware.

### 5.6 Hardware bring-up gate

H7 first-print path: `clocksync get_uptime` must return within deadline; homing must reach the endstop. This is the gate the refactor exists to clear.

## 6. Risks

- **`extract_packet` retention may surprise reviewers.** Justified by three integration-test users; flagged via `#[doc(hidden)]` and a comment cross-linking this spec.
- **Demuxer 1-byte-shift resync adds one new code path in `feed`.** Mitigated by explicit `klipper_bad_crc_followed_immediately_by_valid_frame` and `klipper_false_length_latch_recovers` tests covering the recovery state.
- **Default `IdentifySeqState { 1, 1 }` for test path could mask seq-state bugs in tests.** Mitigated by `reactor_adopts_send_seq_from_identify_state` / `reactor_adopts_receive_seq_from_identify_state` explicitly testing non-default values.
- **`SerialFrameIo::write_all` raw passthrough contract is convention, not type-enforced.** Mitigated by `write_all_passes_*_bytes_through_unmodified` pins in `serial_frame_io_tests.rs`.
- **Demuxer-side and `extract_packet`-side validation could diverge on edge-case malformed frames.** Mitigated by §4.1's CRC consolidation: both paths call the same `crc16_ccitt` (single source in `kalico-native-transport::frame`, re-exported by `wire`). `partial_frame_assembly.rs` proptests continue to exercise `extract_packet` against the same byte streams the demuxer sees in production.
- **First-connection-sentinel branch in `update_receive_seq` may interact non-obviously with non-1 initial `receive_seq`.** Mitigated by `reactor_first_bridge_call_after_identify_succeeds_with_nonzero_initial_seq` exercising exactly this path. F4-firmware's higher tolerance for stale wire-seq becomes irrelevant post-refactor since the host stops sending stale wire-seq spuriously; not a follow-up because the bug class is structurally eliminated.
- **`Demuxer::replay` queue grows unbounded under pathological streams.** Bounded in practice because every replay byte either advances a fresh `WaitingForFrame` → completion path or fails another length-latch (each failure shifts by 1, so at most O(replay_len) replay attempts before drainage). No mitigation beyond the bound; document expectation.

## 7. Out-of-scope follow-ups

- Identify-as-reactor-coroutine. Long-term cleanup; eliminates the handoff entirely.
- Killing identify's 300ms drain phase. With validating demuxer + 1-byte-shift resync, drain is likely obsolete. Leave for hardware-bring-up confirmation; delete in a follow-up commit.
- Fuzz coverage on `Demuxer::feed` resync paths. Worth landing once Step 7-D corpus-replay infrastructure exists.

## 8. Reference

- Bug history: commits `599e92c5b` (kept), `df07d5a03` (reverted), `9c5dedc33` (reverted), `c4307d9a2` / `c6b020ece` (reverts).
- Architect review rounds: design shape, API details, migration plan + final.
- Codex review: cross-checks on each round.
- CLAUDE.md anchors: Step 7-D pickup, "no throwaway code beyond 1-2 lines."
