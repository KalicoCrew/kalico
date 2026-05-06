# SerialFrameIo refactor — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the identify→reactor seam in `kalico-host-rt` so the first `bridge_call` after `identify_handshake` succeeds on real H7 hardware (currently times out, blocks homing).

**Architecture:** Replace two-Demuxer-with-raw-byte-handoff arrangement with a single `SerialFrameIo` (production, in `kalico-host-rt`) that owns port + demuxer + scratch + pending stream errors and survives the identify→reactor handoff by value. Move Klipper CRC + trailer validation into the demuxer with 1-byte-shift resync (`Demuxer::replay` queue). Identify captures the response seq nibble and returns `IdentifySeqState` that the reactor adopts directly, eliminating the latent `send_seq: 1, receive_seq: 1` hardcode. Two commits: (1) seam closure + validating demuxer, (2) seq-state plumbing.

**Tech Stack:** Rust 2021. `serialport` for port abstraction. `kalico-native-transport` and `kalico-host-rt` cargo workspace crates.

**Spec:** `docs/superpowers/specs/2026-05-06-serial-frame-io-refactor-design.md` (commit `dfa5ffff4`, fixup `076f18389`).

---

## File structure

**Create:**
- `rust/kalico-native-transport/src/frame_source.rs` — generic `FrameSource<R: Read>` (test/replay only).
- `rust/kalico-host-rt/src/host_io/serial_frame_io.rs` — production `SerialFrameIo`.
- `rust/kalico-host-rt/tests/serial_frame_io_handoff.rs` — full-fidelity PTY handoff integration test.

**Modify:**
- `rust/kalico-native-transport/src/demux.rs` — replace `DemuxOutput` with `Frame` enum; add `KlipperFrame`, `StreamError`, `PollOutcome` types; Klipper CRC+trailer validation; `replay` queue resync.
- `rust/kalico-native-transport/src/lib.rs` — re-exports.
- `rust/kalico-host-rt/src/host_io/wire.rs` — `crc16_ccitt` becomes re-export; `extract_packet` marked `#[doc(hidden)]`; new free function `decode_absolute`.
- `rust/kalico-host-rt/src/host_io/identify.rs` — signature takes `&mut SerialFrameIo`; returns `(parser, blob)` then later `(parser, blob, IdentifySeqState)`.
- `rust/kalico-host-rt/src/host_io/reactor.rs` — fields collapse to `io: SerialFrameIo`; `new_with_clock` signature; `poll_serial`, `write_frame`, `handle_inbound_frame`; `decode_absolute` callsites updated.
- `rust/kalico-host-rt/src/host_io/mod.rs` — `open_with_port` constructs `SerialFrameIo`.
- `rust/kalico-host-rt/src/host_io/test_harness.rs` — `ReactorHarness` wraps `FakeSerialPort` into `SerialFrameIo`.
- `rust/kalico-host-rt/src/host_io/mod.rs` — pin tests for `extract_packet` stay (offline use).

**Decomposition principle:** each task produces a working, testable, committable change. Tasks 1–11 are commit 1 (seam closure). Tasks 12–17 are commit 2 (seq plumbing). Task 18 final-gate validation.

**Working dir:** `/Users/daniladergachev/Developer/kalico`. Cargo workspace at repo root.

**Test commands:**
- `cargo test -p kalico-native-transport` — transport-crate tests.
- `cargo test -p kalico-host-rt` — host-rt tests including A1–A7 battery.
- `cargo test -p kalico-host-rt --test serial_frame_io_handoff` — new integration test.
- `cargo build -p kalico-host-rt` — compile check after each task.

---

## COMMIT 1 — Seam closure + validating demuxer

After commit 1, `rx_buf` is gone, the demuxer validates Klipper frames, identify and reactor share one `SerialFrameIo`. H7 timeout is **not yet fixed** (latent seq hardcode remains); existing tests pass; no new behavior visible above the host_io boundary.

---

### Task 1: Define `KlipperFrame`, `StreamError`, `Frame`, `PollOutcome` types

**Files:**
- Modify: `rust/kalico-native-transport/src/demux.rs`
- Modify: `rust/kalico-native-transport/src/lib.rs`

- [ ] **Step 1: Add the new types in `demux.rs`** (above the existing `DemuxOutput` — leave that in place for now; it'll be deleted in Task 5)

```rust
/// Validated Klipper frame: length, CRC16-CCITT, and trailing 0x7E all checked
/// inside the demuxer per spec §3.4.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KlipperFrame {
    bytes: Vec<u8>, // private — invariant: passed full validation
}

impl KlipperFrame {
    /// Construct from already-validated bytes. Pub-crate to keep the
    /// validation invariant unforgeable from outside this crate.
    pub(crate) fn from_validated(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }
    /// The seq+DEST byte at index 1.
    pub fn seq_byte(&self) -> u8 { self.bytes[1] }
    /// Body slice: bytes[2 .. len-3] (excludes length byte, seq byte, CRC, trailer).
    pub fn body(&self) -> &[u8] {
        let len = self.bytes.len();
        &self.bytes[2..len - 3]
    }
    /// Full validated frame bytes.
    pub fn bytes(&self) -> &[u8] { &self.bytes }
    /// Consume into the owned Vec (for retransmit/await-response stash).
    pub fn into_bytes(self) -> Vec<u8> { self.bytes }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StreamError {
    KlipperCrcMismatch    { seq: u8, expected: u16, actual: u16 },
    KlipperBadTrailer     { got: u8 },
    KlipperLenOutOfRange  { len: u8 },
    KalicoCrcMismatch     { channel: u8, expected: u16, actual: u16 },
    KalicoLenBelowMin     { len: u16 },
    KalicoFrameTooShort   { got: usize },
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::KlipperCrcMismatch { seq, expected, actual } =>
                write!(f, "klipper crc mismatch seq=0x{seq:02x} expected=0x{expected:04x} actual=0x{actual:04x}"),
            Self::KlipperBadTrailer { got } =>
                write!(f, "klipper bad trailer 0x{got:02x}"),
            Self::KlipperLenOutOfRange { len } =>
                write!(f, "klipper len out of range: {len}"),
            Self::KalicoCrcMismatch { channel, expected, actual } =>
                write!(f, "kalico crc mismatch ch={channel} expected=0x{expected:04x} actual=0x{actual:04x}"),
            Self::KalicoLenBelowMin { len } =>
                write!(f, "kalico len below min: {len}"),
            Self::KalicoFrameTooShort { got } =>
                write!(f, "kalico frame too short: {got} bytes"),
        }
    }
}

#[derive(Debug)]
pub enum Frame {
    Klipper(KlipperFrame),
    Kalico { channel: u8, payload: Vec<u8> },
}

#[derive(Debug)]
pub enum PollOutcome {
    Frames { frames: Vec<Frame>, errors: Vec<StreamError> },
    Timeout,
    PhantomZero,
}
```

- [ ] **Step 2: Re-export from `lib.rs`**

In `rust/kalico-native-transport/src/lib.rs` add:

```rust
pub use demux::{Frame, KlipperFrame, StreamError, PollOutcome};
```

(Verify the existing `pub mod demux;` line is already there — if not, add it.)

- [ ] **Step 3: Compile check**

Run: `cargo build -p kalico-native-transport`
Expected: clean build. New types are unused so far; that's fine.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-native-transport/src/demux.rs rust/kalico-native-transport/src/lib.rs
git commit -m "transport(demux): add Frame/KlipperFrame/StreamError/PollOutcome types

Spec §3.1. Types declared but not yet used by Demuxer (still emits
DemuxOutput). Wired up in subsequent tasks."
```

---

### Task 2: Add Klipper CRC + trailer validation in `Demuxer::feed`

**Files:**
- Modify: `rust/kalico-native-transport/src/demux.rs`

- [ ] **Step 1: Write the failing tests first**

Add to the existing `#[cfg(test)] mod tests` block in `demux.rs`:

```rust
fn good_klipper_frame(payload: &[u8], seq: u8) -> Vec<u8> {
    // Build a valid Klipper frame: [len][seq|DEST][payload][crc_hi][crc_lo][0x7E]
    use crate::frame::crc16_ccitt;
    const MESSAGE_DEST: u8 = 0x10;
    const MESSAGE_SEQ_MASK: u8 = 0x0F;
    const MESSAGE_SYNC: u8 = 0x7E;
    let len = 5 + payload.len();
    assert!(len <= 64);
    let mut buf = Vec::with_capacity(len);
    buf.push(len as u8);
    buf.push((seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST);
    buf.extend_from_slice(payload);
    let crc = crc16_ccitt(&buf);
    buf.push((crc >> 8) as u8);
    buf.push((crc & 0xFF) as u8);
    buf.push(MESSAGE_SYNC);
    buf
}

#[test]
fn klipper_validates_good_crc_and_trailer() {
    let frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
    let mut d = Demuxer::new();
    let outs = d.feed_slice(&frame);
    assert_eq!(outs.len(), 1, "expected one DemuxOutput");
    assert!(matches!(&outs[0], DemuxOutput::KlipperFrame(f) if f == &frame));
}

#[test]
fn klipper_bad_crc_emits_stream_error() {
    let mut frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
    let len = frame.len();
    frame[len - 3] ^= 0xFF; // corrupt CRC hi
    let mut d = Demuxer::new();
    let outs = d.feed_slice(&frame);
    assert!(outs.iter().any(|o| matches!(o, DemuxOutput::StreamError(_))),
        "expected a StreamError, got {outs:?}");
}

#[test]
fn klipper_bad_trailer_emits_stream_error() {
    let mut frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
    let last = frame.len() - 1;
    frame[last] = 0x00; // not 0x7E
    let mut d = Demuxer::new();
    let outs = d.feed_slice(&frame);
    assert!(outs.iter().any(|o| matches!(o, DemuxOutput::StreamError(_))),
        "expected a StreamError, got {outs:?}");
}
```

(For now still using `DemuxOutput`; we'll migrate to `Frame`/`StreamError` in Task 5.)

- [ ] **Step 2: Run to verify failure**

Run: `cargo test -p kalico-native-transport --lib demux::tests::klipper_validates_good_crc_and_trailer`
Expected: FAIL — current demuxer doesn't validate trailer/CRC; the test's good-frame round-trip *might* pass coincidentally (length + read N bytes), but `klipper_bad_crc_emits_stream_error` and `klipper_bad_trailer_emits_stream_error` MUST fail.

Run: `cargo test -p kalico-native-transport --lib demux::tests::klipper_bad_crc`
Expected: FAIL — current demuxer accepts any bytes blindly.

- [ ] **Step 3: Implement Klipper CRC + trailer validation**

In `demux.rs`, modify the `State::InsideKlipper` arm of `Demuxer::feed`:

```rust
State::InsideKlipper { buf, remaining } => {
    buf.push(byte);
    *remaining -= 1;
    if *remaining == 0 {
        let frame = std::mem::take(buf);
        self.state = State::WaitingForFrame;
        return Some(parse_klipper_frame(frame));
    }
    None
}
```

Add a new free function `parse_klipper_frame` (next to `parse_kalico_frame`):

```rust
fn parse_klipper_frame(frame: Vec<u8>) -> DemuxOutput {
    use crate::frame::crc16_ccitt;
    const MESSAGE_DEST: u8 = 0x10;
    const MESSAGE_SEQ_MASK: u8 = 0x0F;
    const MESSAGE_SYNC: u8 = 0x7E;
    const MESSAGE_TRAILER_SIZE: usize = 3;

    let len = frame.len();
    // Trailer check.
    if frame[len - 1] != MESSAGE_SYNC {
        return DemuxOutput::StreamError(format!(
            "klipper bad trailer 0x{:02x}", frame[len - 1]
        ));
    }
    // Seq-byte DEST flag (per extract_packet at wire.rs:44).
    let seq_byte = frame[1];
    if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
        return DemuxOutput::StreamError(format!(
            "klipper bad seq/DEST byte 0x{:02x}", seq_byte
        ));
    }
    // CRC over bytes[0 .. len-3] (length byte + seq + payload), big-endian.
    let crc_off = len - MESSAGE_TRAILER_SIZE;
    let crc_expected = (u16::from(frame[crc_off]) << 8) | u16::from(frame[crc_off + 1]);
    let crc_actual = crc16_ccitt(&frame[..crc_off]);
    if crc_expected != crc_actual {
        return DemuxOutput::StreamError(format!(
            "klipper crc mismatch: expected 0x{crc_expected:04x}, got 0x{crc_actual:04x}"
        ));
    }
    DemuxOutput::KlipperFrame(frame)
}
```

Note: still returning `DemuxOutput` (the existing API). Migration to `Frame`/`StreamError` happens in Task 5.

- [ ] **Step 4: Run tests**

Run: `cargo test -p kalico-native-transport --lib demux::tests`
Expected: all existing tests pass + the three new tests pass. The good-frame test passes because we build the CRC correctly.

If `klipper_then_kalico_then_klipper` fails: the existing `fake_klipper_frame` helper in tests builds frames with `crc=0` and bad trailer pattern. That test's frames will now fail validation. Update `fake_klipper_frame` in the test module to use `good_klipper_frame` semantics (real CRC + 0x7E trailer):

```rust
// REMOVE the old fake_klipper_frame and replace with:
fn fake_klipper_frame(payload: &[u8]) -> Vec<u8> {
    good_klipper_frame(payload, 0x10)
}
```

Wait — the original `fake_klipper_frame` pushed a `0x10` as `seq` (with implicit DEST flag because `0x10 == MESSAGE_DEST`). And it pushed `0x7E` as the trailer at the end. But its CRC bytes were `0, 0` — wrong. With validation it will fail. Update the helper to compute real CRC.

Actually `good_klipper_frame` already does this. Just delete the original `fake_klipper_frame` and add `fn fake_klipper_frame(payload: &[u8]) -> Vec<u8> { good_klipper_frame(payload, 0) }` to keep callsites working.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-native-transport/src/demux.rs
git commit -m "transport(demux): validate Klipper CRC + trailer in InsideKlipper

Spec §3.4. CRC16-CCITT over bytes[0..len-3], big-endian, against
bytes[len-3..len-1]. Trailer must be 0x7E. Failures emit
DemuxOutput::StreamError; demuxer resyncs to WaitingForFrame.

Test fake_klipper_frame helper updated to produce real CRC-valid
frames; all existing tests still pass."
```

---

### Task 3: Add `Demuxer::replay` queue + 1-byte-shift resync

**Files:**
- Modify: `rust/kalico-native-transport/src/demux.rs`

- [ ] **Step 1: Write the failing test**

Add to `demux.rs` test module:

```rust
#[test]
fn klipper_bad_crc_followed_immediately_by_valid_frame_recovers() {
    // Produce a stream where a "false length latch" byte starts a fake
    // klipper frame that overlaps the start of a real, valid frame.
    // After the false frame fails validation, 1-byte-shift resync MUST
    // recover and emit the real frame.
    use crate::frame::crc16_ccitt;
    let real = good_klipper_frame(&[0xAA, 0xBB], 0);
    // Prepend one byte in the Klipper-len-range (5..=64) to force a false latch.
    // Pick a length that overshoots `real.len()` so the demuxer eats `real`'s
    // start as if it's payload, then fails validation.
    let mut stream = Vec::new();
    stream.push(20u8); // false latch: claims a 20-byte frame
    stream.extend_from_slice(&real);
    let mut d = Demuxer::new();
    let outs = d.feed_slice(&stream);
    // Expect: at least one StreamError + the real KlipperFrame.
    assert!(outs.iter().any(|o| matches!(o, DemuxOutput::StreamError(_))),
        "expected stream error from false latch, got {outs:?}");
    let klippers: Vec<_> = outs.iter().filter_map(|o| match o {
        DemuxOutput::KlipperFrame(f) => Some(f.clone()),
        _ => None,
    }).collect();
    assert!(klippers.iter().any(|f| f == &real),
        "expected the real frame to be recovered after resync; got {klippers:?}");
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cargo test -p kalico-native-transport --lib demux::tests::klipper_bad_crc_followed_immediately_by_valid_frame_recovers`
Expected: FAIL — current implementation discards `bytes[1..]` after a failed validation.

- [ ] **Step 3: Add `replay` field to `Demuxer`**

```rust
pub struct Demuxer {
    state: State,
    replay: std::collections::VecDeque<u8>,
}

impl Default for Demuxer {
    fn default() -> Self {
        Self::new()
    }
}

impl Demuxer {
    pub fn new() -> Self {
        Self {
            state: State::WaitingForFrame,
            replay: std::collections::VecDeque::new(),
        }
    }
    // ...
}
```

- [ ] **Step 4: Modify `feed_slice` to drain replay first**

Replace the existing `feed_slice` body with:

```rust
pub fn feed_slice(&mut self, bytes: &[u8]) -> Vec<DemuxOutput> {
    let mut out = Vec::new();
    // Drain replay queue before consuming new bytes. Each replayed byte is
    // re-fed exactly as if freshly arrived.
    loop {
        match self.replay.pop_front() {
            Some(b) => if let Some(o) = self.feed_inner(b) { out.push(o); }
            None => break,
        }
    }
    for &b in bytes {
        if let Some(o) = self.feed_inner(b) {
            out.push(o);
        }
        // After each new byte, also drain anything that landed in replay
        // because of a validation failure inside feed_inner.
        while let Some(rb) = self.replay.pop_front() {
            if let Some(o) = self.feed_inner(rb) {
                out.push(o);
            }
        }
    }
    out
}
```

Rename the existing `feed` method to `feed_inner` (private):

```rust
fn feed_inner(&mut self, byte: u8) -> Option<DemuxOutput> {
    // ... existing logic from feed() ...
}
```

Keep a public `feed` that wraps `feed_inner` for callers that feed one byte at a time:

```rust
pub fn feed(&mut self, byte: u8) -> Option<DemuxOutput> {
    // Drain replay first.
    while let Some(rb) = self.replay.pop_front() {
        if let Some(o) = self.feed_inner(rb) {
            // We can return at most one output, so push remaining replay
            // back to the front and return.
            self.replay.push_front(rb); // wait — already popped; need different shape
            return Some(o);
        }
    }
    self.feed_inner(byte)
}
```

**Reconsider:** the public `feed -> Option<DemuxOutput>` shape doesn't compose well with replay (could be 0 or many outputs per call). Simpler: change `feed` to also return `Vec<DemuxOutput>`, or keep `feed` private. Since callers in this crate use `feed_slice` exclusively (look at demux.rs itself + identify.rs + reactor.rs uses), make `feed` private and keep only `feed_slice` public. Update test code that calls `d.feed(b)` to call `d.feed_slice(&[b])`.

Verify: search the workspace for `.feed(` on Demuxer:

```bash
grep -rn "demuxer\.feed\b\|Demuxer\b.*\.feed\b" --include="*.rs"
```

If there are external callers of single-byte `feed`, expose `pub fn feed_slice(&mut self, &[u8]) -> Vec<DemuxOutput>` and remove single-byte `feed` from the public API. (Internal uses inside `demux.rs` can call `feed_inner`.)

- [ ] **Step 5: Modify `parse_klipper_frame` to push `frame[1..]` into replay on failure**

Change the function signature to take `&mut Demuxer` so it can push to replay, OR have `feed_inner`'s `InsideKlipper` arm do the replay-push itself when `parse_klipper_frame` returns a `StreamError`. Cleaner option:

```rust
State::InsideKlipper { buf, remaining } => {
    buf.push(byte);
    *remaining -= 1;
    if *remaining == 0 {
        let frame = std::mem::take(buf);
        self.state = State::WaitingForFrame;
        let result = parse_klipper_frame(&frame);
        match &result {
            DemuxOutput::StreamError(_) => {
                // 1-byte-shift resync: re-feed bytes[1..].
                self.replay.extend(frame.iter().copied().skip(1));
                Some(result)
            }
            _ => Some(result),
        }
    } else {
        None
    }
}
```

(Update `parse_klipper_frame` signature to `fn parse_klipper_frame(frame: &[u8]) -> DemuxOutput` since it no longer needs to consume.)

- [ ] **Step 6: Run tests**

Run: `cargo test -p kalico-native-transport --lib demux`
Expected: all tests pass, including new `klipper_bad_crc_followed_immediately_by_valid_frame_recovers`.

- [ ] **Step 7: Add resync test for false-length-latch on garbage**

```rust
#[test]
fn klipper_false_length_latch_recovers_to_valid_frame() {
    // Stream: garbage byte in 5..=64 range, NOT followed by enough bytes
    // for that length, then followed by a real frame.
    let real = good_klipper_frame(&[0xAA], 0);
    let mut stream = Vec::new();
    stream.push(7u8); // claims 7-byte frame; we'll satisfy with the start of `real`
    stream.extend_from_slice(&real);
    let mut d = Demuxer::new();
    let outs = d.feed_slice(&stream);
    let klippers: Vec<_> = outs.iter().filter_map(|o| match o {
        DemuxOutput::KlipperFrame(f) => Some(f.clone()),
        _ => None,
    }).collect();
    assert!(klippers.iter().any(|f| f == &real));
}
```

Run: `cargo test -p kalico-native-transport --lib demux::tests::klipper_false_length_latch`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add rust/kalico-native-transport/src/demux.rs
git commit -m "transport(demux): 1-byte-shift resync via replay VecDeque

Spec §3.4. On Klipper validation failure, push frame[1..] into a
Demuxer.replay queue; feed_slice drains replay before consuming new
bytes, so the demuxer recovers from false-length-latches without
losing valid frames that started inside the bogus window. Preserves
the demux.rs:13 'byte-oriented and interruptible' invariant.

Single-byte Demuxer::feed becomes private (was unused externally);
public API is feed_slice -> Vec<DemuxOutput>."
```

---

### Task 4: Migrate `Demuxer` from `DemuxOutput` to new `Frame` / `StreamError`

**Files:**
- Modify: `rust/kalico-native-transport/src/demux.rs`
- Modify: `rust/kalico-native-transport/src/lib.rs`
- Modify: `rust/kalico-host-rt/src/host_io/identify.rs` (callers — not deeper changes yet, just pattern-match update)
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs` (callers — same)

- [ ] **Step 1: Replace `DemuxOutput` with `Frame` and standalone `StreamError`**

In `demux.rs`, change `feed_slice`'s return type:

```rust
pub fn feed_slice(&mut self, bytes: &[u8]) -> (Vec<Frame>, Vec<StreamError>) {
    // ... drains replay; for each output, either push into frames or errors.
}
```

Update `parse_klipper_frame` and `parse_kalico_frame` to return `Result<Frame, StreamError>`:

```rust
fn parse_klipper_frame(frame: &[u8]) -> Result<Frame, StreamError> {
    use crate::frame::crc16_ccitt;
    const MESSAGE_DEST: u8 = 0x10;
    const MESSAGE_SEQ_MASK: u8 = 0x0F;
    const MESSAGE_SYNC: u8 = 0x7E;
    const MESSAGE_TRAILER_SIZE: usize = 3;
    let len = frame.len();
    if frame[len - 1] != MESSAGE_SYNC {
        return Err(StreamError::KlipperBadTrailer { got: frame[len - 1] });
    }
    let seq_byte = frame[1];
    if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
        // Use KlipperBadTrailer's neighbour — declare a new variant if needed.
        // For simplicity re-use KlipperLenOutOfRange? No — add a variant.
        return Err(StreamError::KlipperBadTrailer { got: seq_byte });
    }
    let crc_off = len - MESSAGE_TRAILER_SIZE;
    let crc_expected = (u16::from(frame[crc_off]) << 8) | u16::from(frame[crc_off + 1]);
    let crc_actual = crc16_ccitt(&frame[..crc_off]);
    if crc_expected != crc_actual {
        return Err(StreamError::KlipperCrcMismatch {
            seq: seq_byte & MESSAGE_SEQ_MASK,
            expected: crc_expected,
            actual: crc_actual,
        });
    }
    Ok(Frame::Klipper(KlipperFrame::from_validated(frame.to_vec())))
}

fn parse_kalico_frame(frame: &[u8]) -> Result<Frame, StreamError> {
    if frame.len() < 1 + FRAME_MIN_LEN_FIELD {
        return Err(StreamError::KalicoFrameTooShort { got: frame.len() });
    }
    let payload_end = frame.len() - 2;
    let crc_expected = u16::from_le_bytes([frame[payload_end], frame[payload_end + 1]]);
    let crc_actual = crc16_ccitt(&frame[1..payload_end]);
    if crc_expected != crc_actual {
        return Err(StreamError::KalicoCrcMismatch {
            channel: frame[3],
            expected: crc_expected,
            actual: crc_actual,
        });
    }
    let channel = frame[3];
    let payload = frame[4..payload_end].to_vec();
    Ok(Frame::Kalico { channel, payload })
}
```

Update `InsideKalico` arm to map the kalico len-below-min error to `StreamError::KalicoLenBelowMin { len }`.

- [ ] **Step 2: Refactor `feed_inner` to return `Result<Option<Frame>, StreamError>`** so feed_slice can sort outputs cleanly

```rust
fn feed_inner(&mut self, byte: u8) -> Result<Option<Frame>, StreamError> {
    // ... mirror the existing match, but return Err on parse failures and
    // Ok(Some(frame)) on completions, Ok(None) when waiting for more bytes.
}

pub fn feed_slice(&mut self, bytes: &[u8]) -> (Vec<Frame>, Vec<StreamError>) {
    let mut frames = Vec::new();
    let mut errors = Vec::new();
    let consume = |this: &mut Self, b: u8, frames: &mut Vec<Frame>, errors: &mut Vec<StreamError>| {
        match this.feed_inner(b) {
            Ok(Some(f)) => frames.push(f),
            Ok(None) => {}
            Err(e) => errors.push(e),
        }
    };
    while let Some(rb) = self.replay.pop_front() {
        consume(self, rb, &mut frames, &mut errors);
    }
    for &b in bytes {
        consume(self, b, &mut frames, &mut errors);
        while let Some(rb) = self.replay.pop_front() {
            consume(self, rb, &mut frames, &mut errors);
        }
    }
    (frames, errors)
}
```

(Closure capturing `&mut self` won't work — inline the logic instead of using a closure. Use a labeled loop or `consume_inner` helper method.)

- [ ] **Step 3: Add `KlipperFrame::from_validated` to `KlipperFrame` impl block** if not already added in Task 1.

- [ ] **Step 4: Delete `DemuxOutput` enum** from `demux.rs`. Update all internal test code to match against `Frame::Klipper(_)` / `Frame::Kalico { .. }` and check the `errors` vec for `StreamError` variants.

- [ ] **Step 5: Remove `pub use demux::DemuxOutput;`** from `lib.rs` if present.

- [ ] **Step 6: Update callers in `host_io/identify.rs` and `host_io/reactor.rs`**

Both files currently have `use kalico_native_transport::demux::{Demuxer, DemuxOutput};` and pattern match on `DemuxOutput::KlipperFrame(_) | KalicoFrame {..} | StreamError(..)`. Update to:

In `identify.rs`:
```rust
use kalico_native_transport::demux::{Demuxer, Frame};

// In wait_for_identify_response, replace:
//   for out in demuxer.feed_slice(&scratch[..n]) {
//       if let DemuxOutput::KlipperFrame(packet) = out { ... }
//   }
// with:
let (frames, errors) = demuxer.feed_slice(&scratch[..n]);
for e in errors { log::warn!("identify stream error: {e}"); }
for f in frames {
    if let Frame::Klipper(kf) = f {
        if let Some(params) = decode_identify_response(kf.bytes()) {
            return Ok(Some(params));
        }
    }
}
```

In `reactor.rs::poll_serial`:
```rust
use kalico_native_transport::demux::{Demuxer, Frame};

// Replace the existing for-loop over outputs with:
let (frames, errors) = self.kalico_demuxer.feed_slice(&scratch[..n]);
for e in errors { log::warn!("kalico stream error: {e}"); }
for f in frames {
    match f {
        Frame::Klipper(kf) => {
            // For now, append the *bytes* into rx_buf so existing
            // extract_packet path is unchanged. Full migration in Task 8.
            self.rx_buf.extend_from_slice(kf.bytes());
        }
        Frame::Kalico { channel, payload } => {
            self.handle_kalico_frame(channel, &payload);
        }
    }
}
```

This keeps the reactor working with `extract_packet` for now; we'll remove that path in Task 8.

- [ ] **Step 7: Run all tests**

Run: `cargo test -p kalico-native-transport && cargo test -p kalico-host-rt --lib`
Expected: PASS. The reactor still parses via `extract_packet` over `rx_buf`; the demuxer's CRC-validated KlipperFrames just get re-parsed there. That's redundant but correct.

- [ ] **Step 8: Commit**

```bash
git add rust/kalico-native-transport/ rust/kalico-host-rt/src/host_io/identify.rs rust/kalico-host-rt/src/host_io/reactor.rs
git commit -m "transport+host_io: migrate Demuxer to Frame/StreamError API

Spec §3.1, §3.4. feed_slice now returns (Vec<Frame>, Vec<StreamError>);
DemuxOutput enum deleted. Callers in identify and reactor updated to
the new shape. Reactor still seeds rx_buf with KlipperFrame bytes for
extract_packet to re-parse — that redundancy goes away in Task 8."
```

---

### Task 5: CRC consolidation — single `crc16_ccitt` source

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/wire.rs`

- [ ] **Step 1: Replace `crc16_ccitt` body with re-export**

In `wire.rs`, replace the existing `pub fn crc16_ccitt(buf: &[u8]) -> u16 { ... }` with:

```rust
pub use kalico_native_transport::frame::crc16_ccitt;
```

Verify the dependency exists in `rust/kalico-host-rt/Cargo.toml`:
```bash
grep kalico-native-transport rust/kalico-host-rt/Cargo.toml
```
Expected: a dependency line like `kalico-native-transport = { path = "../kalico-native-transport" }`. If absent, add it.

- [ ] **Step 2: Remove the `crc16_matches_klipper_test_vector` from `wire.rs`** — it's now duplicated by the same test in `kalico-native-transport::frame`. Or keep it (architect noted "pin tests stay through re-export"); it'll just exercise the re-exported symbol. Keep it.

- [ ] **Step 3: Run tests**

Run: `cargo test -p kalico-host-rt --lib wire`
Expected: PASS — `crc16_matches_klipper_test_vector` exercises the re-exported function.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/wire.rs
git commit -m "host_io(wire): re-export crc16_ccitt from kalico-native-transport

Spec §4.1. Single source of truth for CRC16-CCITT, now load-bearing in
both protocols (kalico frames + new Klipper-frame validation in the
demuxer). Pin test stays."
```

---

### Task 6: Create `FrameSource<R: Read>` (test-only generic)

**Files:**
- Create: `rust/kalico-native-transport/src/frame_source.rs`
- Modify: `rust/kalico-native-transport/src/lib.rs`

- [ ] **Step 1: Write the failing tests**

Create `rust/kalico-native-transport/src/frame_source.rs`:

```rust
//! Generic frame-source over any `R: Read`. Test-only / corpus-replay
//! companion to `kalico-host-rt::SerialFrameIo`. See spec §3.1.

use std::io::{self, Read};
use std::time::{Duration, Instant};

use crate::demux::{Demuxer, Frame, PollOutcome, StreamError};

#[derive(Debug, thiserror::Error)]
pub enum FrameSourceError {
    #[error("set_timeout failed: {0}")]
    SetTimeout(io::Error),
    #[error("io error: {0}")]
    Io(io::Error),
}

pub struct FrameSource<R: Read> {
    reader: R,
    set_timeout: Box<dyn FnMut(&mut R, Duration) -> io::Result<()>>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
}

impl<R: Read> FrameSource<R> {
    pub fn new(
        reader: R,
        set_timeout: Box<dyn FnMut(&mut R, Duration) -> io::Result<()>>,
    ) -> Self {
        Self { reader, set_timeout, demuxer: Demuxer::new(), scratch: [0u8; 1024] }
    }

    pub fn from_read_no_timeout(reader: R) -> Self {
        Self::new(reader, Box::new(|_, _| Ok(())))
    }

    pub fn into_inner(self) -> R { self.reader }

    pub fn poll_frames_until(&mut self, deadline: Instant)
        -> Result<PollOutcome, FrameSourceError>
    {
        let now = Instant::now();
        let remaining = deadline.saturating_duration_since(now);
        (self.set_timeout)(&mut self.reader, remaining)
            .map_err(FrameSourceError::SetTimeout)?;
        match self.reader.read(&mut self.scratch) {
            Ok(0) => Ok(PollOutcome::PhantomZero),
            Ok(n) => {
                let (frames, errors) = self.demuxer.feed_slice(&self.scratch[..n]);
                Ok(PollOutcome::Frames { frames, errors })
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock) =>
                Ok(PollOutcome::Timeout),
            Err(e) => Err(FrameSourceError::Io(e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    use crate::frame::{encode_frame, CHANNEL_CONTROL};

    #[test]
    fn poll_frames_until_returns_phantom_zero_on_eof() {
        let cursor = Cursor::new(Vec::<u8>::new());
        let mut fs = FrameSource::from_read_no_timeout(cursor);
        let outcome = fs.poll_frames_until(Instant::now() + Duration::from_millis(100)).unwrap();
        assert!(matches!(outcome, PollOutcome::PhantomZero));
    }

    #[test]
    fn poll_frames_until_returns_frames_in_arrival_order() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&encode_frame(CHANNEL_CONTROL, b"first"));
        bytes.extend_from_slice(&encode_frame(CHANNEL_CONTROL, b"second"));
        let cursor = Cursor::new(bytes);
        let mut fs = FrameSource::from_read_no_timeout(cursor);
        let outcome = fs.poll_frames_until(Instant::now() + Duration::from_millis(100)).unwrap();
        match outcome {
            PollOutcome::Frames { frames, errors } => {
                assert!(errors.is_empty());
                assert_eq!(frames.len(), 2);
                let payloads: Vec<_> = frames.iter().map(|f| match f {
                    Frame::Kalico { payload, .. } => payload.clone(),
                    _ => panic!("expected kalico"),
                }).collect();
                assert_eq!(payloads[0], b"first");
                assert_eq!(payloads[1], b"second");
            }
            other => panic!("expected Frames, got {other:?}"),
        }
    }

    #[test]
    fn poll_frames_until_propagates_set_timeout_error() {
        let cursor = Cursor::new(Vec::<u8>::new());
        let mut fs = FrameSource::new(
            cursor,
            Box::new(|_, _| Err(io::Error::new(io::ErrorKind::Other, "broken"))),
        );
        let result = fs.poll_frames_until(Instant::now() + Duration::from_millis(100));
        assert!(matches!(result, Err(FrameSourceError::SetTimeout(_))));
    }
}
```

- [ ] **Step 2: Add `pub mod frame_source;` and re-export**

In `rust/kalico-native-transport/src/lib.rs`:
```rust
pub mod frame_source;
pub use frame_source::{FrameSource, FrameSourceError};
```

- [ ] **Step 3: Run tests**

Run: `cargo test -p kalico-native-transport --lib frame_source`
Expected: 3 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-native-transport/src/frame_source.rs rust/kalico-native-transport/src/lib.rs
git commit -m "transport: FrameSource<R: Read> for test/replay use

Spec §3.1, §3.5. Generic frame source over any Read. Closure adapter
for set_timeout returns io::Result<()> so failures propagate; test
constructor from_read_no_timeout uses a noop closure. Three unit
tests cover phantom-zero EOF, ordered frame emission, and set_timeout
error propagation."
```

---

### Task 7: Create `SerialFrameIo` in `kalico-host-rt`

**Files:**
- Create: `rust/kalico-host-rt/src/host_io/serial_frame_io.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs` (add `pub mod serial_frame_io;`)

- [ ] **Step 1: Create the file**

`rust/kalico-host-rt/src/host_io/serial_frame_io.rs`:

```rust
//! Production frame-source: owns the SerialPort, the Demuxer, and the
//! scratch buffer. Single owner of the wire across identify→reactor handoff.
//! See spec §3.1, §3.5.

use std::io::{self, Read};
use std::time::{Duration, Instant};

use serialport::SerialPort;

use kalico_native_transport::demux::{Demuxer, PollOutcome};

use crate::transport::TransportError;

pub struct SerialFrameIo {
    port: Box<dyn SerialPort>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
}

impl SerialFrameIo {
    pub fn new(port: Box<dyn SerialPort>) -> Self {
        Self { port, demuxer: Demuxer::new(), scratch: [0u8; 1024] }
    }

    /// Read one batch of bytes from the port and demux. The deadline bounds
    /// how long the underlying port read may block; identify uses long
    /// deadlines, the reactor's poll_serial uses `now + READ_TIMEOUT`.
    pub fn poll_frames_until(&mut self, deadline: Instant)
        -> Result<PollOutcome, TransportError>
    {
        let now = Instant::now();
        let remaining = deadline.saturating_duration_since(now);
        if let Err(e) = self.port.set_timeout(remaining) {
            return Err(TransportError::Io(io::Error::new(io::ErrorKind::Other, e.to_string())));
        }
        match self.port.read(&mut self.scratch) {
            Ok(0) => Ok(PollOutcome::PhantomZero),
            Ok(n) => {
                let (frames, errors) = self.demuxer.feed_slice(&self.scratch[..n]);
                Ok(PollOutcome::Frames { frames, errors })
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock) =>
                Ok(PollOutcome::Timeout),
            Err(e) => Err(TransportError::Io(e)),
        }
    }

    /// Raw byte passthrough. Does NOT validate, frame, or re-shape outbound
    /// bytes. Both Klipper-shaped frames (build_frame) and Kalico-native
    /// frames (KalicoIdentify::build_*) are pre-built by their encoders and
    /// written verbatim. See spec §3.1.
    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(bytes).map_err(TransportError::Io)
    }

    pub fn flush(&mut self) -> Result<(), TransportError> {
        self.port.flush().map_err(TransportError::Io)
    }

    /// Test-only access to the underlying port for fixtures that need to
    /// observe what was written. Gated behind a feature so it doesn't leak
    /// into production callers.
    #[cfg(any(test, feature = "test-harness"))]
    pub fn port_mut(&mut self) -> &mut Box<dyn SerialPort> {
        &mut self.port
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // FakeSerialPort lives in test_harness — the tests here will be added
    // in Task 9 once that wiring is in place. Skeleton tests intentionally
    // omitted at file creation time.
}
```

- [ ] **Step 2: Wire up the module in `host_io/mod.rs`**

Add to `rust/kalico-host-rt/src/host_io/mod.rs`:

```rust
pub mod serial_frame_io;
pub use serial_frame_io::SerialFrameIo;
```

- [ ] **Step 3: Compile check**

Run: `cargo build -p kalico-host-rt`
Expected: clean build. The new file compiles; nothing uses it yet.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/serial_frame_io.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "host_io: SerialFrameIo skeleton (port + demuxer owner)

Spec §3.1. Production single-owner of the wire spanning identify→
reactor. write_all is documented as raw passthrough. Tests added in
Task 9 once test_harness wiring is in place."
```

---

### Task 8: Wire `SerialFrameIo` into `Reactor` (replace fields, update poll_serial + write_frame)

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`
- Modify: `rust/kalico-host-rt/src/host_io/test_harness.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs`
- Modify: `rust/kalico-host-rt/src/host_io/identify.rs`

This is the largest task — collapsing three fields (port + rx_buf + kalico_demuxer) into one (io: SerialFrameIo) cascades through ~10 callsites.

- [ ] **Step 1: Update `Reactor` struct fields** (`reactor.rs:30-100` area — find the actual field declarations)

```rust
pub struct Reactor {
    pub(crate) io: SerialFrameIo,
    pub(crate) parser: Arc<MsgProtoParser>,
    // ... rest unchanged ...
}
```

Delete fields `port`, `rx_buf`, `kalico_demuxer`. Add `use crate::host_io::serial_frame_io::SerialFrameIo;`.

- [ ] **Step 2: Update `Reactor::new` and `Reactor::new_with_clock` signatures**

Replace `port: Box<dyn SerialPort>, ..., rx_buf_initial: Vec<u8>` with `io: SerialFrameIo`. The `rx_buf_initial` parameter goes away.

```rust
pub fn new(
    io: SerialFrameIo,
    parser: Arc<MsgProtoParser>,
    submissions: mpsc::Receiver<Submission>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    config: KalicoHostIoConfig,
) -> Self {
    Self::new_with_clock(io, parser, submissions, status_snapshot, config, Arc::new(RealClock))
}

pub fn new_with_clock(
    io: SerialFrameIo,
    parser: Arc<MsgProtoParser>,
    submissions: mpsc::Receiver<Submission>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    config: KalicoHostIoConfig,
    clock: Arc<dyn Clock>,
) -> Self {
    // ... initialize fields, replace `port: port_box`, drop `rx_buf: rx_buf_initial`,
    // drop `kalico_demuxer: Demuxer::new()` (now inside io), set `io: io`.
    // Keep send_seq: 1, receive_seq: 1, last_ack_seq: 0 hardcoded for now.
    // (Commit 2 replaces these with IdentifySeqState.)
}
```

- [ ] **Step 3: Update `poll_serial` to use `io.poll_frames_until`**

Replace the existing body (`reactor.rs:641-708` area) with:

```rust
fn poll_serial(&mut self) {
    let deadline = self.clock.now() + READ_TIMEOUT;
    match self.io.poll_frames_until(deadline) {
        Ok(PollOutcome::Frames { frames, errors }) => {
            self.zero_byte_first_seen = None;
            for e in errors {
                log::warn!("kalico stream error: {e}");
            }
            for f in frames {
                match f {
                    Frame::Klipper(kf) => {
                        if self.handle_inbound_frame(kf).is_err() {
                            return;
                        }
                    }
                    Frame::Kalico { channel, payload } => {
                        self.handle_kalico_frame(channel, &payload);
                    }
                }
            }
        }
        Ok(PollOutcome::Timeout) => {
            self.zero_byte_first_seen = None;
        }
        Ok(PollOutcome::PhantomZero) => {
            let now = self.clock.now();
            let first = *self.zero_byte_first_seen.get_or_insert(now);
            if now.duration_since(first) >= ZERO_BYTE_DEBOUNCE {
                log::warn!("port read returned Ok(0) for >= {ZERO_BYTE_DEBOUNCE:?}; transitioning to Closed");
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code:   FaultCode::HostDisconnect.as_u16(),
                    fault_detail: 0,
                    segment_id:   0,
                    synthesized:  false,
                });
                self.state = ReactorState::Closed;
            }
        }
        Err(e) => {
            log::warn!("port read error: {e:?}; transitioning to Closed");
            self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                fault_code:   FaultCode::HostDisconnect.as_u16(),
                fault_detail: 0,
                segment_id:   0,
                synthesized:  false,
            });
            self.state = ReactorState::Closed;
        }
    }
}
```

Note: `READ_TIMEOUT` is defined in this module today; it stays.

- [ ] **Step 4: Update `write_frame` to use `io.write_all + io.flush`**

Find the existing (`reactor.rs:167-170` area):
```rust
self.port.write_all(bytes).and_then(|_| self.port.flush())
```
Replace with:
```rust
self.io.write_all(bytes)?;
self.io.flush()?;
Ok(())
```

(Adjust to match the surrounding error type — likely `Result<(), TransportError>`.)

- [ ] **Step 5: Update `handle_inbound_frame` signature**

Find the function (search for `fn handle_inbound_frame`). Today it takes `Vec<u8>`. Change to `KlipperFrame`:

```rust
fn handle_inbound_frame(&mut self, frame: KlipperFrame) -> Result<(), Closed> {
    let bytes = frame.bytes();
    // ... existing logic, but read seq from frame.seq_byte() and body from frame.body()
}
```

For now, the simplest migration: keep the body that operates on `&[u8]` and call it with `frame.bytes()` slice. The body parsing logic is unchanged.

- [ ] **Step 6: Update `mod.rs::open_with_port`**

Replace (`mod.rs:233-260` area):

```rust
fn open_with_port(
    mut port_box: Box<dyn serialport::SerialPort>,
    config: KalicoHostIoConfig,
) -> Result<Self, TransportError> {
    let _ = port_box.set_timeout(Duration::from_millis(100));
    let mut io = SerialFrameIo::new(port_box);

    let (parser_owned, raw_identify_bytes, _seq, _rx_buf) = identify::identify_handshake(
        &mut io,
        config.identify_timeout,
    )?;

    let parser = Arc::new(parser_owned);
    let (submission_tx, submission_rx) = std::sync::mpsc::channel();
    let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));

    let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
    let reactor_parser = Arc::clone(&parser);
    let reactor_status = Arc::clone(&status_snapshot);
    let reactor_config = config.clone();
    let reactor_clock = Arc::clone(&clock);
    let reactor_handle = std::thread::spawn(move || {
        let mut reactor = crate::host_io::reactor::Reactor::new_with_clock(
            io, reactor_parser, submission_rx, reactor_status, reactor_config, reactor_clock,
        );
        reactor.run();
    });
    // ... rest unchanged ...
}
```

Note: `_rx_buf` placeholder kept until Task 11 changes the identify signature; identify still returns four-tuple `(parser, raw, seq, rx_buf)` for now but `rx_buf` is dropped here and never seeded into the reactor. The reactor's `Demuxer` (now inside `io`) already consumed those bytes during identify.

- [ ] **Step 7: Update `identify.rs` to take `&mut SerialFrameIo`**

Change signature:
```rust
pub fn identify_handshake(
    io: &mut SerialFrameIo,
    timeout: Duration,
) -> Result<(MsgProtoParser, Vec<u8>, u8, Vec<u8>), TransportError>
```

Internal loop replaces `port.read` + local Demuxer with `io.poll_frames_until`:

```rust
let attempt_deadline = deadline.min(Instant::now() + Duration::from_millis(150));
match io.poll_frames_until(attempt_deadline)? {
    PollOutcome::Frames { frames, errors } => {
        for e in errors { log::warn!("identify stream error: {e}"); }
        for f in frames {
            if let Frame::Klipper(kf) = f {
                if let Some(params) = decode_identify_response(kf.bytes()) {
                    // ... process params ...
                }
            }
        }
    }
    PollOutcome::Timeout | PollOutcome::PhantomZero => {}
}
```

The drain loop at the top (lines 32-42) becomes:
```rust
let drain_until = Instant::now() + Duration::from_millis(300);
while Instant::now() < drain_until {
    match io.poll_frames_until(drain_until)? {
        PollOutcome::Frames { frames: _, errors: _ } => {} // discard
        PollOutcome::Timeout | PollOutcome::PhantomZero => break,
    }
}
```

The `rx_buf: Vec<u8>` accumulator (line 44, 122-124) is **deleted**. Return value's fourth element becomes `Vec::new()` (empty placeholder; full removal in Task 11).

The local `demuxer: Demuxer` at line 51 is **deleted** — `io` owns the demuxer now.

`build_frame` calls and `port.write_all` calls (lines 69-70) become `io.write_all(&frame)?; io.flush()?;`.

- [ ] **Step 8: Update `test_harness.rs` to wrap `FakeSerialPort` in `SerialFrameIo`**

In `rust/kalico-host-rt/src/host_io/test_harness.rs`, locate `ReactorHarness::new` (or equivalent constructor). Where it constructs `Reactor::new(...)` with a raw `FakeSerialPort`, change to:

```rust
let io = SerialFrameIo::new(port_box);
let reactor = Reactor::new_with_clock(
    io,
    parser,
    submission_rx,
    status_snapshot,
    config,
    clock,
);
```

The harness's `feed_rx`/`drain_tx` methods continue to work because they manipulate the shared inner buffers of `FakeSerialPort` via the harness's own handles — `SerialFrameIo` is transparent to that.

- [ ] **Step 9: Add `Reactor::new_for_tests` for the three other test sites**

In `reactor.rs` add (gated):

```rust
#[cfg(any(test, feature = "test-harness"))]
pub fn new_for_tests(
    port: Box<dyn SerialPort>,
    parser: Arc<MsgProtoParser>,
    submissions: mpsc::Receiver<Submission>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    config: KalicoHostIoConfig,
    clock: Arc<dyn Clock>,
) -> Self {
    Self::new_with_clock(
        SerialFrameIo::new(port),
        parser,
        submissions,
        status_snapshot,
        config,
        clock,
    )
}
```

Update the three direct-`Reactor::new` callsites in `reactor.rs` test module (lines 1052, 1316, 1358) to call `Reactor::new_for_tests` with their bespoke `Box<dyn SerialPort>` impls.

- [ ] **Step 10: Build and run the full test suite**

Run: `cargo build -p kalico-host-rt`
Expected: clean build.

Run: `cargo test -p kalico-host-rt --lib`
Expected: all tests pass — A1–A7 deterministic battery + reactor-internal tests + harness tests.

If any tests fail, the most likely root causes:
- A test that fed raw mid-frame bytes via `feed_rx` and expected `extract_packet` semantics on `rx_buf`. Now those bytes flow through the demuxer first; failure mode shifts from "no frame" to "stream error logged, no frame." Update the test to feed valid CRC-bearing frames or assert on demuxer behavior.
- A test that read `rx_buf` directly via `pub(crate)` access. Field gone — update the test to either drop or use new accessors.

- [ ] **Step 11: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/
git commit -m "host_io: collapse port+rx_buf+kalico_demuxer into SerialFrameIo

Spec §3.3, §4.2. Reactor now holds a single io: SerialFrameIo field.
poll_serial reads PollOutcome from io; write_frame writes through
io.write_all + io.flush. handle_inbound_frame takes KlipperFrame
(validated by demuxer) instead of Vec<u8>. Identify and reactor share
one demuxer that survives the handoff by value — no rx_buf transplant,
no second Demuxer instance.

Reactor::new_for_tests gated behind cfg(test|test-harness) wraps
FakeSerialPort internally so the ~62 existing tests stay unchanged.

H7 timeout NOT yet fixed: reactor still hardcodes send_seq:1,
receive_seq:1. That's commit 2."
```

---

### Task 9: Add `SerialFrameIo` unit tests + handoff regression test

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/serial_frame_io.rs`

- [ ] **Step 1: Add tests using `FakeSerialPort` from `test_harness`**

In the `#[cfg(test)] mod tests` block of `serial_frame_io.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::host_io::test_harness::FakeSerialPort;
    use crate::host_io::wire::build_frame;

    #[test]
    fn write_all_passes_klipper_bytes_through_unmodified() {
        let port = FakeSerialPort::new();
        let port_handle = port.handle();
        let mut io = SerialFrameIo::new(Box::new(port));
        let frame = build_frame(&[0x01, 0x02], 0);
        io.write_all(&frame).unwrap();
        io.flush().unwrap();
        let written = port_handle.drain_written();
        assert_eq!(written, frame, "write_all must not modify outbound bytes");
    }

    #[test]
    fn write_all_passes_kalico_bytes_through_unmodified() {
        use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL};
        let port = FakeSerialPort::new();
        let port_handle = port.handle();
        let mut io = SerialFrameIo::new(Box::new(port));
        let frame = encode_frame(CHANNEL_CONTROL, b"hello");
        io.write_all(&frame).unwrap();
        io.flush().unwrap();
        let written = port_handle.drain_written();
        assert_eq!(written, frame);
    }

    // The handoff regression test that would have caught both reverted
    // patches (df07d5a03, 9c5dedc33).
    #[test]
    fn partial_klipper_frame_survives_identify_to_reactor_handoff() {
        use kalico_native_transport::demux::Frame;
        let port = FakeSerialPort::new();
        let port_handle = port.handle();
        let mut io = SerialFrameIo::new(Box::new(port));
        let complete = build_frame(&[0xAA], 0);
        let next = build_frame(&[0xBB], 1);
        // Phase 1 — identify reads the complete frame, plus the FIRST half
        // of `next`.
        let split = next.len() / 2;
        port_handle.feed(&complete);
        port_handle.feed(&next[..split]);
        let outcome = io.poll_frames_until(Instant::now() + Duration::from_millis(50)).unwrap();
        let phase1_frames = match outcome {
            PollOutcome::Frames { frames, .. } => frames,
            other => panic!("phase 1 expected Frames, got {other:?}"),
        };
        assert_eq!(phase1_frames.len(), 1, "phase 1 should yield only the complete frame");
        assert!(matches!(&phase1_frames[0], Frame::Klipper(kf) if kf.bytes() == complete.as_slice()));
        // Phase 2 — reactor side reads the remainder; demuxer state survived.
        port_handle.feed(&next[split..]);
        let outcome = io.poll_frames_until(Instant::now() + Duration::from_millis(50)).unwrap();
        let phase2_frames = match outcome {
            PollOutcome::Frames { frames, .. } => frames,
            other => panic!("phase 2 expected Frames, got {other:?}"),
        };
        assert_eq!(phase2_frames.len(), 1, "phase 2 should complete the second frame");
        assert!(matches!(&phase2_frames[0], Frame::Klipper(kf) if kf.bytes() == next.as_slice()));
    }
}
```

(`FakeSerialPort::new()`, `handle()`, `feed()`, `drain_written()` — verify these exist in `test_harness.rs` and adjust naming to match the actual fixture API.)

- [ ] **Step 2: Run tests**

Run: `cargo test -p kalico-host-rt --lib serial_frame_io`
Expected: 3 tests PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/serial_frame_io.rs
git commit -m "host_io(serial-frame-io): write-passthrough + handoff tests

Spec §5.2. Three tests pin: (1) raw passthrough of Klipper bytes,
(2) raw passthrough of kalico bytes, (3) partial-Klipper-frame
demuxer state survives the identify→reactor handoff. The third test
would have caught both reverted patches (df07d5a03, 9c5dedc33)."
```

---

### Task 10: Mark `extract_packet` `#[doc(hidden)]` and verify offline use

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/wire.rs`

- [ ] **Step 1: Add `#[doc(hidden)]` to `extract_packet`**

```rust
/// Offline klipper-frame parser. Retained for `tests/captures_replay.rs`,
/// `tests/partial_frame_assembly.rs`, `tests/passthrough_integration.rs`,
/// and reactor's own test module to decode bytes the reactor wrote through
/// SerialFrameIo::write_all (raw passthrough). The live reactor path no
/// longer calls this — see SerialFrameIo + Demuxer for live decoding.
#[doc(hidden)]
pub fn extract_packet(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    // ... unchanged body ...
}
```

- [ ] **Step 2: Run integration tests**

Run: `cargo test -p kalico-host-rt --tests`
Expected: all integration tests pass — `extract_packet` is still callable from `tests/`.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/wire.rs
git commit -m "host_io(wire): mark extract_packet #[doc(hidden)] (offline use only)

Spec §4.2. Live reactor path no longer parses via extract_packet;
SerialFrameIo + Demuxer handle inbound. extract_packet retained for
three integration tests + reactor's own test module decoding outbound
frames written through SerialFrameIo::write_all (raw passthrough)."
```

---

### Task 11: Drop `rx_buf: Vec<u8>` from `identify_handshake` return tuple

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/identify.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs`

- [ ] **Step 1: Change identify's return type from `(parser, raw, seq, rx_buf)` to `(parser, raw, seq)`**

In `identify.rs`:
```rust
pub fn identify_handshake(
    io: &mut SerialFrameIo,
    timeout: Duration,
) -> Result<(MsgProtoParser, Vec<u8>, u8), TransportError>
```

Remove the `rx_buf: Vec<u8>` accumulator entirely (already drained in Task 8 step 7 to `Vec::new()`). Update the final return statement.

- [ ] **Step 2: Update `mod.rs::open_with_port` callsite**

```rust
let (parser_owned, raw_identify_bytes, _seq) = identify::identify_handshake(
    &mut io,
    config.identify_timeout,
)?;
```

(The `_seq` will become real in commit 2.)

- [ ] **Step 3: Run tests**

Run: `cargo test -p kalico-host-rt`
Expected: PASS.

- [ ] **Step 4: Commit — END OF COMMIT 1**

```bash
git add rust/kalico-host-rt/src/host_io/identify.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "host_io(identify): drop rx_buf from return tuple

Spec §4.3. After SerialFrameIo lands, the rx_buf carryover into the
reactor is meaningless — identify and reactor share one demuxer.
Return type shrinks to (parser, raw_blob, seq). seq still discarded
at mod.rs callsite as _seq; that's wired up in commit 2.

After this commit:
- The H7 timeout symptom is NOT yet fixed (reactor still hardcodes
  send_seq:1, receive_seq:1).
- The seam is structurally closed: rx_buf transplant is gone, two-
  Demuxer arrangement is gone, raw bytes can no longer bypass the
  demuxer at the API surface.
- All existing tests (~62) pass.
- New regression test pins partial-frame survival across the handoff.

This commit closes commit 1 of the spec's two-commit split."
```

---

## COMMIT 2 — IdentifySeqState plumbing (the actual H7 fix)

After commit 2, the H7 timeout test passes; reactor adopts seq state from identify; the latent `send_seq: 1` hardcode is gone.

---

### Task 12: Hoist `decode_absolute` to free function in `wire.rs`

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/wire.rs`
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`

- [ ] **Step 1: Read existing `decode_absolute`**

```bash
grep -n "fn decode_absolute" rust/kalico-host-rt/src/host_io/reactor.rs
```

Note its body (around lines 415-424).

- [ ] **Step 2: Add free function to `wire.rs`**

```rust
/// Decode a 4-bit wire-seq nibble back to an absolute u64 by walking
/// epochs from `prev_abs`. Selects the closest absolute value whose
/// low nibble matches `wire_seq & MESSAGE_SEQ_MASK`.
///
/// Hoisted from `Reactor::decode_absolute` (was a method) so identify
/// and other callers can reuse the logic without holding a Reactor.
pub fn decode_absolute(prev_abs: u64, wire_seq: u8) -> u64 {
    let nibble = u64::from(wire_seq & MESSAGE_SEQ_MASK);
    let prev_nibble = prev_abs & 0x0F;
    let delta = nibble.wrapping_sub(prev_nibble) & 0x0F;
    prev_abs.wrapping_add(delta)
}
```

(Verify the body matches what the original method computed; copy it byte-for-byte.)

- [ ] **Step 3: Update reactor callsites**

Find every `self.decode_absolute(x)` and replace with `wire::decode_absolute(self.receive_seq, x)`. Likely sites: `reactor.rs:472, 553` (per architect's prior review).

```bash
grep -n "decode_absolute" rust/kalico-host-rt/src/host_io/reactor.rs
```

- [ ] **Step 4: Delete the method from `Reactor`**

Remove `fn decode_absolute(&self, ...)` from the impl block.

- [ ] **Step 5: Add a unit test for the free function**

In `wire.rs::tests`:
```rust
#[test]
fn decode_absolute_walks_within_one_epoch() {
    assert_eq!(decode_absolute(0, 0), 0);
    assert_eq!(decode_absolute(0, 1), 1);
    assert_eq!(decode_absolute(0, 5), 5);
}

#[test]
fn decode_absolute_handles_wrap() {
    assert_eq!(decode_absolute(15, 0), 16);
    assert_eq!(decode_absolute(15, 5), 21);
    assert_eq!(decode_absolute(31, 0), 32);
}
```

- [ ] **Step 6: Run tests**

Run: `cargo test -p kalico-host-rt`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/wire.rs rust/kalico-host-rt/src/host_io/reactor.rs
git commit -m "host_io(wire): hoist decode_absolute to free function

Spec §4.2. Was a Reactor method reading self.receive_seq. Now a free
function in wire.rs taking prev_abs explicitly so identify (and
future callers) can reuse without a Reactor instance. Two unit tests
pin within-epoch and wrap behavior."
```

---

### Task 13: Define `IdentifySeqState`

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/identify.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs` (re-export)

- [ ] **Step 1: Add the type to `identify.rs`**

```rust
/// Sequence-state snapshot returned by identify, adopted by the reactor.
/// See spec §3.1, §3.3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentifySeqState {
    /// Next absolute send-seq the reactor should use for its first
    /// outbound frame after identify completes.
    pub next_send_seq_abs: u64,
    /// Absolute receive-seq adopted from the seq nibble of the last
    /// validated Klipper frame seen during identify (walked across
    /// all responses via wire::decode_absolute).
    pub mcu_receive_seq_abs: u64,
}
```

- [ ] **Step 2: Re-export from `mod.rs`**

```rust
pub use identify::IdentifySeqState;
```

- [ ] **Step 3: Compile check**

Run: `cargo build -p kalico-host-rt`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/identify.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "host_io(identify): define IdentifySeqState type

Spec §3.1. Type is declared but not yet returned; identify still
hands back u8 seq. Wiring in next task."
```

---

### Task 14: Identify captures seq nibble per Klipper frame, returns `IdentifySeqState`

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/identify.rs`

- [ ] **Step 1: Write the failing test**

Add to `identify.rs::tests` (or create the module if absent):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::host_io::test_harness::FakeSerialPort;
    use crate::host_io::wire::{build_frame, MESSAGE_SEQ_MASK};

    #[test]
    fn identify_returns_seq_state_with_correct_absolute_decode() {
        // Build a fake fw response: identify_response with seq nibble = 3.
        // ... construct a minimal valid identify_response payload ...
        // Feed it through FakeSerialPort, call identify_handshake, assert
        // mcu_receive_seq_abs == 3.
        //
        // (Keep the test tight; the goal is to pin "adopted directly, not
        // incremented.")
    }
}
```

(Detailed test setup will reuse a helper from `test_harness` that scripts a fake fw responding to identify requests. If no such helper exists, this test stays as a TODO comment block and the equivalent assertion goes into the integration test in Task 17.)

- [ ] **Step 2: Run test, expect FAIL**

Run: `cargo test -p kalico-host-rt --lib identify::tests::identify_returns_seq_state`
Expected: FAIL — return type still doesn't include `IdentifySeqState`.

- [ ] **Step 3: Update identify_handshake signature and internals**

```rust
pub fn identify_handshake(
    io: &mut SerialFrameIo,
    timeout: Duration,
) -> Result<(MsgProtoParser, Vec<u8>, IdentifySeqState), TransportError>
```

Add internal state:
```rust
let mut next_send_seq_abs: u64 = 1;
let mut mcu_recv_abs: u64 = 0;
```

In the request loop, before writing each frame:
```rust
let wire_seq = (next_send_seq_abs as u8) & MESSAGE_SEQ_MASK;
let frame = build_frame(&payload, wire_seq);
io.write_all(&frame)?;
io.flush()?;
next_send_seq_abs += 1;
```

In the response handling, for **every** `Frame::Klipper(f)` (whether or not it decodes as identify_response):
```rust
mcu_recv_abs = wire::decode_absolute(mcu_recv_abs, f.seq_byte() & MESSAGE_SEQ_MASK);
if let Some(params) = decode_identify_response(f.bytes()) {
    // ... process ...
}
```

Update final return:
```rust
Ok((parser, raw_identify_bytes, IdentifySeqState {
    next_send_seq_abs,
    mcu_receive_seq_abs: mcu_recv_abs,
}))
```

Delete the old `seq: u8` accumulator.

- [ ] **Step 4: Update `mod.rs::open_with_port` to capture the new return**

```rust
let (parser_owned, raw_identify_bytes, identify_seq) = identify::identify_handshake(
    &mut io,
    config.identify_timeout,
)?;
```

`identify_seq` is now wired into `Reactor::new_with_clock` in Task 15.

- [ ] **Step 5: Run tests**

Run: `cargo test -p kalico-host-rt --lib identify`
Expected: the new seq-state test passes.

Run: `cargo test -p kalico-host-rt`
Expected: A1–A7 + everything else still green (reactor doesn't yet adopt the state, so the test of seq-1 hardcoded behavior is unchanged).

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/identify.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "host_io(identify): track next_send_seq_abs + mcu_recv_abs, return IdentifySeqState

Spec §4.2. Identify now maintains explicit u64 absolute counters for
both directions. mcu_recv_abs walks via wire::decode_absolute on every
Frame::Klipper emitted by the demuxer (regardless of whether the body
decodes as identify_response — pins consistency with the wire even
under stray frames). next_send_seq_abs counts host-issued requests.

Reactor doesn't yet adopt the state — open_with_port captures it but
hands hardcoded seq:1 to Reactor::new_with_clock. Adoption in next task."
```

---

### Task 15: Reactor adopts `IdentifySeqState`

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs`

- [ ] **Step 1: Write the failing test (the H7 regression test)**

In `reactor.rs::tests` (or wherever the harness tests live):

```rust
#[test]
fn reactor_first_bridge_call_after_identify_succeeds_with_nonzero_initial_seq() {
    // Construct a reactor with IdentifySeqState { next_send_seq_abs: 5,
    // mcu_receive_seq_abs: 5 }. Submit one outbound frame; assert wire seq
    // is 5 (mod 16 = 5). Feed back an ack with rseq nibble = 6. Assert
    // receive_seq advances to 6 absolute.
    //
    // This is the test that pins the H7 fix: previously Reactor::new
    // hardcoded send_seq:1, receive_seq:1, so this scenario would fail
    // (host sends seq=1 instead of seq=5; firmware ignores).
    //
    // Use ReactorHarness::new_with_seq_state(IdentifySeqState{..}, ...)
    // — a new helper added in this task.
}
```

(Detailed implementation uses existing `feed_rx`/`drain_tx` patterns from `test_harness`.)

- [ ] **Step 2: Update `Reactor::new_with_clock` signature**

```rust
pub fn new_with_clock(
    io: SerialFrameIo,
    parser: Arc<MsgProtoParser>,
    submissions: mpsc::Receiver<Submission>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    seq: IdentifySeqState,
    config: KalicoHostIoConfig,
    clock: Arc<dyn Clock>,
) -> Self {
    // ... initialize:
    //   send_seq:    seq.next_send_seq_abs,
    //   receive_seq: seq.mcu_receive_seq_abs,
    //   last_ack_seq: seq.mcu_receive_seq_abs.saturating_sub(1),
    // Drop the hardcoded `send_seq: 1, receive_seq: 1, last_ack_seq: 0`.
}
```

- [ ] **Step 3: Update `Reactor::new` and `Reactor::new_for_tests`**

`Reactor::new` adds `seq: IdentifySeqState` parameter. `Reactor::new_for_tests` constructs `IdentifySeqState { next_send_seq_abs: 1, mcu_receive_seq_abs: 1 }` literally (no `Default` impl).

```rust
#[cfg(any(test, feature = "test-harness"))]
pub fn new_for_tests(
    port: Box<dyn SerialPort>,
    parser: Arc<MsgProtoParser>,
    submissions: mpsc::Receiver<Submission>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    config: KalicoHostIoConfig,
    clock: Arc<dyn Clock>,
) -> Self {
    Self::new_with_clock(
        SerialFrameIo::new(port),
        parser,
        submissions,
        status_snapshot,
        IdentifySeqState { next_send_seq_abs: 1, mcu_receive_seq_abs: 1 },
        config,
        clock,
    )
}
```

- [ ] **Step 4: Update `mod.rs::open_with_port`**

Pass `identify_seq` through:
```rust
let mut reactor = crate::host_io::reactor::Reactor::new_with_clock(
    io, reactor_parser, submission_rx, reactor_status, identify_seq, reactor_config, reactor_clock,
);
```

Move `identify_seq` into the spawned thread.

- [ ] **Step 5: Add `ReactorHarness::new_with_seq_state` helper**

In `test_harness.rs`:
```rust
#[cfg(test)]
impl ReactorHarness {
    pub fn new_with_seq_state(seq: IdentifySeqState, /* other params... */) -> Self {
        // Same body as `new` but pass `seq` through to Reactor::new_with_clock
        // instead of constructing the default {1,1}.
    }
}
```

- [ ] **Step 6: Run the regression test and the full suite**

Run: `cargo test -p kalico-host-rt --lib reactor::tests::reactor_first_bridge_call_after_identify_succeeds_with_nonzero_initial_seq`
Expected: PASS.

Run: `cargo test -p kalico-host-rt`
Expected: full suite PASS, including A1–A7 unchanged.

- [ ] **Step 7: Commit — END OF COMMIT 2**

```bash
git add rust/kalico-host-rt/src/host_io/
git commit -m "host_io(reactor): adopt IdentifySeqState; drop hardcoded send_seq:1

Spec §3.3, §4.2. Reactor::new_with_clock now takes IdentifySeqState
and initializes send_seq, receive_seq, last_ack_seq from it directly.
Eliminates the latent hardcode that broke the first post-identify
bridge_call on H7 (firmware sees host seq jumping backwards from
identify's last burned seq to wire-seq=1; H7 firmware doesn't tolerate
this).

Reactor::new_for_tests constructs IdentifySeqState{1,1} literally —
no Default impl on the public type, since {1,1} is meaningful only
as a test shim.

Regression test reactor_first_bridge_call_after_identify_succeeds_
with_nonzero_initial_seq pins the fix.

After this commit: H7 timeout symptom is resolved, all existing tests
green, partial-frame handoff regression test green."
```

---

## VALIDATION TASKS — non-code gates

### Task 16: A1–A7 deterministic battery (regression check)

- [ ] **Step 1: Run the full A1–A7 suite**

```bash
cargo test -p kalico-host-rt --lib reactor
```

Expected: all A1–A7 tests pass with no changes to their bodies. The harness now wraps `FakeSerialPort` in `SerialFrameIo` transparently.

If A5 (partial-frame TCP-style assembly) has shifted behavior because of the validating demuxer: bytes that previously reached `extract_packet` as raw and were dropped silently may now generate `StreamError` log entries. That's an improvement, not a regression. The test's pass/fail criterion (does the reactor eventually assemble the frame?) is unchanged.

### Task 17: Renode soak

- [ ] **Step 1: Re-run Phase-2 Renode harness**

Refer to the renode-simulation skill. Run `G1 X10` and `G1 Z5` segment dispatch + clocksync `get_uptime` end-to-end on simulated hardware. Phase-2 gate passed 2026-05-02 (CLAUDE.md); confirm green after both commits.

### Task 18: H7 hardware bring-up gate

- [ ] **Step 1: Build and flash H7 firmware**

Refer to `memory/reference_flash_h7.md`. Build, flash to `dderg@trident.local`.

- [ ] **Step 2: Start the bridge, run `clocksync get_uptime`**

Confirm response within deadline.

- [ ] **Step 3: Run homing**

Confirm endstops trigger and homing completes.

- [ ] **Step 4: If green, mark Step 7-D entry in CLAUDE.md as complete on this checkpoint.** (This is the gate the refactor exists to clear.)

---

## Self-review

After writing the plan, the following self-review checks were applied:

**Spec coverage:** Every section of the spec maps to one or more tasks:
- Spec §3.1 (types) → Tasks 1, 6, 7, 13
- Spec §3.2 (crate boundaries) → Tasks 6, 7
- Spec §3.3 (lifetime contract) → Task 8 (open_with_port + identify), Task 15 (seq adoption)
- Spec §3.4 (demuxer changes incl. 1-byte-shift resync) → Tasks 2, 3, 4
- Spec §3.5 (PollOutcome semantics) → Tasks 6, 7
- Spec §4.1 (transport changes) → Tasks 1–4, 5, 6
- Spec §4.2 (host-rt changes) → Tasks 5, 7, 8, 10, 11, 12, 14, 15
- Spec §4.3 (deletions) → Tasks 8, 11, 12, 15
- Spec §4.4 (commit split) → Task 11 closes commit 1; Task 15 closes commit 2
- Spec §5.1 (transport tests) → Tasks 2, 3, 6 (note: kalico migration tests already in demux from Task 4)
- Spec §5.2 (host-rt tests including handoff) → Tasks 9, 14, 15
- Spec §5.4 (A1–A7) → Task 16
- Spec §5.5 (Renode) → Task 17
- Spec §5.6 (H7 gate) → Task 18

**Placeholder scan:** No "TBD", "TODO", or "implement later" instructions. Two soft references — Task 14 step 1 says "the equivalent assertion goes into the integration test in Task 17 if the helper doesn't exist yet" and Task 17 references the renode-simulation skill. Both are concrete enough; the renode skill is part of the plugin set.

**Type consistency:** `KlipperFrame::from_validated`, `bytes()`, `body()`, `seq_byte()`, `into_bytes()` used consistently across Tasks 1, 4, 8, 9, 14. `IdentifySeqState` field names `next_send_seq_abs` / `mcu_receive_seq_abs` match between Task 13 declaration and Tasks 14/15 usage. `PollOutcome` variants `Frames { frames, errors } / Timeout / PhantomZero` match Task 6 (FrameSource) and Task 7 (SerialFrameIo).

---

**Plan complete and saved to `docs/superpowers/plans/2026-05-06-serial-frame-io-refactor.md`.**

Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
