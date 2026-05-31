# Simple MCU Contract Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the curve pool + segment architecture with a flat per-axis piece ring, making the MCU a dumb polynomial playback engine.

**Architecture:** Each axis gets its own SPSC ring buffer of 32-byte piece entries. The ISR walks pieces sequentially per axis, evaluating Horner polynomials and driving steppers. The host uploads pieces directly into rings via `PushPieces`. All coordination lives on the host.

**Tech Stack:** Rust (`no_std`, `#[repr(C)]`), C (dispatch/FFI boundary), Kconfig

**Spec:** `docs/superpowers/specs/2026-05-27-simple-mcu-contract-design.md`

---

## File Map

**New files:**
- `rust/runtime/src/piece_ring.rs` — PieceEntry struct + PieceRing SPSC buffer
- `rust/runtime/tests/piece_ring.rs` — ring buffer unit tests
- `rust/runtime/tests/piece_tick.rs` — ISR piece-advancement integration tests

**Major modifications:**
- `rust/runtime/src/stepping_state.rs` — AxisConfig loses curve_handle, gains ring reference
- `rust/runtime/src/engine.rs` — gut/rewrite: Engine becomes piece-ring walker
- `rust/runtime/src/tick.rs` — `get_piece_for_time` + simplified tick loop (dispatch_pulse/dispatch_phase stay)
- `rust/runtime/src/state.rs` — SharedState loses segment-related fields
- `rust/runtime/src/lib.rs` — module declarations
- `rust/kalico-protocol/src/messages.rs` — remove old messages, add PushPieces
- `rust/kalico-c-api/src/runtime_ffi.rs` — new FFI entry points
- `src/kalico_dispatch.c` — handle PushPieces, remove old handlers
- `src/Kconfig` — replace pool options with PIECE_RING_SIZE

**Removed files:**
- `rust/runtime/src/curve_pool.rs` (+ `curve_pool/tests.rs`)
- `rust/runtime/src/cubic_curve.rs`
- `rust/runtime/src/c_segment_queue.rs`
- `rust/runtime/src/segment.rs` (+ `segment/tests.rs`)
- `rust/runtime/src/config.rs` (+ `config/tests.rs`) — EMode, McuAxisConfig, KinematicTag
- `rust/runtime/src/kinematics.rs` (+ `kinematics/tests.rs`)
- `rust/runtime/src/reclaim.rs`
- `rust/runtime/src/slot.rs` (+ `slot/tests.rs`)
- `rust/runtime/src/stream.rs`
- `src/kalico_segment_queue.c`, `src/kalico_segment_queue.h`
- `rust/runtime/tests/arm_segment.rs`
- `rust/runtime/tests/cubic_curve_load.rs`
- `rust/runtime/tests/e_follower_absolute.rs`
- `rust/runtime/tests/exhaustion_post_pass.rs`
- `rust/runtime/tests/loom_curve_pool_alloc.rs`
- `rust/runtime/tests/loom_spsc_split.rs`
- `rust/runtime/tests/tick_integration.rs` (rewritten as `piece_tick.rs`)
- `rust/runtime/tests/tick_piece_advance.rs` (rewritten into `piece_tick.rs`)
- `rust/runtime/tests/step_push_emits_pieces_for_g5_move.rs`

**Unchanged (kept as-is):**
- `rust/runtime/src/monomial.rs` — Horner evaluation stays
- `rust/runtime/src/step.rs` — StepMotorState stays
- `rust/runtime/src/step_queue.rs` — per-axis step queue stays
- `rust/runtime/src/per_axis_timer.rs` — step queue consumer stays
- `rust/runtime/src/phase_lut.rs` — phase lookup table stays
- `rust/runtime/src/modulator.rs` — coil current math stays
- `rust/runtime/src/endstop.rs` — endstop infrastructure stays
- `rust/runtime/src/trace.rs` — trace ring stays
- `rust/runtime/src/clock.rs` — cycle counter widening stays
- `rust/runtime/src/fault_helpers.rs` — fault publication stays
- `rust/runtime/src/bezier_root.rs` — sub-sample timing stays
- `rust/runtime/src/sub_sample_timing.rs` — stays
- `rust/kalico-native-transport/` — transport layer unchanged

---

### Task 1: PieceEntry Struct

**Files:**
- Create: `rust/runtime/src/piece_ring.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod piece_ring;`)

- [ ] **Step 1: Write the failing test**

Create `rust/runtime/tests/piece_ring.rs`:

```rust
use runtime::piece_ring::PieceEntry;

#[test]
fn piece_entry_is_32_bytes() {
    assert_eq!(core::mem::size_of::<PieceEntry>(), 32);
}

#[test]
fn piece_entry_is_8_byte_aligned() {
    assert_eq!(core::mem::align_of::<PieceEntry>(), 8);
}

#[test]
fn piece_entry_to_monomial_constant_piece() {
    // Bernstein [5.0, 5.0, 5.0, 5.0] = constant at 5.0mm
    let entry = PieceEntry {
        start_time: 1000,
        coeffs: [5.0, 5.0, 5.0, 5.0],
        duration: 0.001,
        _reserved: 0,
    };
    let (mono, vel) = entry.to_monomial();
    // c0=5.0, c1=0, c2=0, c3=0 (constant)
    assert!((mono[0] - 5.0).abs() < 1e-6);
    assert!(mono[1].abs() < 1e-6);
    assert!(mono[2].abs() < 1e-6);
    assert!(mono[3].abs() < 1e-6);
    // velocity all zero
    assert!(vel[0].abs() < 1e-6);
    assert!(vel[1].abs() < 1e-6);
    assert!(vel[2].abs() < 1e-6);
}

#[test]
fn piece_entry_to_monomial_linear() {
    // Bernstein [0.0, 1/3, 2/3, 1.0] = linear t, duration 0.01s
    // In unit interval: P(t) = t. Monomial: c0=0, c1=1, c2=0, c3=0
    // Duration-rescaled: c1' = c1/d = 1/0.01 = 100
    let entry = PieceEntry {
        start_time: 0,
        coeffs: [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0],
        duration: 0.01,
        _reserved: 0,
    };
    let (mono, vel) = entry.to_monomial();
    assert!((mono[0] - 0.0).abs() < 1e-4);
    assert!((mono[1] - 100.0).abs() < 1e-2);
    assert!(mono[2].abs() < 1e-2);
    assert!(mono[3].abs() < 1e-2);
    // vel: vc0 = c1 = 100
    assert!((vel[0] - 100.0).abs() < 1e-2);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p runtime --test piece_ring -- 2>&1 | head -20`
Expected: compilation error — `piece_ring` module doesn't exist.

- [ ] **Step 3: Write PieceEntry struct**

Create `rust/runtime/src/piece_ring.rs`:

```rust
use crate::monomial::bernstein_to_monomial_with_duration;

#[derive(Clone, Copy, Debug)]
#[repr(C, align(8))]
pub struct PieceEntry {
    pub start_time: u64,
    pub coeffs: [f32; 4],
    pub duration: f32,
    pub _reserved: u32,
}

const _: () = assert!(core::mem::size_of::<PieceEntry>() == 32);
const _: () = assert!(core::mem::align_of::<PieceEntry>() == 8);

impl PieceEntry {
    pub fn to_monomial(&self) -> ([f32; 4], [f32; 3]) {
        let m = bernstein_to_monomial_with_duration(self.coeffs, self.duration);
        (m.coeffs, m.vel_coeffs)
    }

    pub fn end_time(&self, clock_freq: f32) -> u64 {
        self.start_time + (self.duration * clock_freq) as u64
    }
}
```

Add to `rust/runtime/src/lib.rs`:
```rust
pub mod piece_ring;
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo test -p runtime --test piece_ring -v`
Expected: all 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/piece_ring.rs rust/runtime/src/lib.rs rust/runtime/tests/piece_ring.rs
git commit -m "feat: add PieceEntry struct (32-byte, repr(C), aligned)"
```

---

### Task 2: PieceRing SPSC Buffer

**Files:**
- Modify: `rust/runtime/src/piece_ring.rs`
- Modify: `rust/runtime/tests/piece_ring.rs`

- [ ] **Step 1: Write failing tests for ring operations**

Append to `rust/runtime/tests/piece_ring.rs`:

```rust
use runtime::piece_ring::{PieceEntry, PieceRing};

fn make_piece(start: u64, duration: f32) -> PieceEntry {
    PieceEntry {
        start_time: start,
        coeffs: [0.0, 0.0, 0.0, 0.0],
        duration,
        _reserved: 0,
    }
}

#[test]
fn ring_new_empty() {
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 8];
    let ring = PieceRing::new(&mut storage);
    assert_eq!(ring.len(), 0);
    assert_eq!(ring.capacity(), 8);
    assert!(ring.is_empty());
}

#[test]
fn ring_push_and_peek() {
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 8];
    let mut ring = PieceRing::new(&mut storage);
    let piece = make_piece(1000, 0.001);
    assert!(ring.push(piece).is_ok());
    assert_eq!(ring.len(), 1);
    let front = ring.peek().unwrap();
    assert_eq!(front.start_time, 1000);
}

#[test]
fn ring_pop_advances_read() {
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 8];
    let mut ring = PieceRing::new(&mut storage);
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    assert_eq!(ring.len(), 2);
    ring.pop();
    assert_eq!(ring.len(), 1);
    assert_eq!(ring.peek().unwrap().start_time, 200);
}

#[test]
fn ring_full_rejects_push() {
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 4];
    let mut ring = PieceRing::new(&mut storage);
    for i in 0..4 {
        assert!(ring.push(make_piece(i * 100, 0.001)).is_ok());
    }
    assert!(ring.push(make_piece(400, 0.001)).is_err());
    assert_eq!(ring.len(), 4);
}

#[test]
fn ring_wrap_around() {
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 4];
    let mut ring = PieceRing::new(&mut storage);
    // Fill and drain partially
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    ring.pop();
    ring.pop();
    // Now head and tail are at index 2. Push 4 more (wraps around).
    for i in 0..4 {
        assert!(ring.push(make_piece((i + 3) * 100, 0.001)).is_ok());
    }
    assert_eq!(ring.len(), 4);
    assert_eq!(ring.peek().unwrap().start_time, 300);
}

#[test]
fn ring_consumed_count_monotonic() {
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 4];
    let mut ring = PieceRing::new(&mut storage);
    assert_eq!(ring.consumed_count(), 0);
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    ring.pop();
    assert_eq!(ring.consumed_count(), 1);
    ring.pop();
    assert_eq!(ring.consumed_count(), 2);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p runtime --test piece_ring -- 2>&1 | head -10`
Expected: compilation error — `PieceRing` not defined.

- [ ] **Step 3: Implement PieceRing**

Append to `rust/runtime/src/piece_ring.rs`:

```rust
pub struct PieceRing<'a> {
    buf: &'a mut [PieceEntry],
    head: usize, // next write position
    tail: usize, // next read position
    count: usize,
    consumed: u32,
}

impl<'a> PieceRing<'a> {
    pub fn new(storage: &'a mut [PieceEntry]) -> Self {
        Self {
            buf: storage,
            head: 0,
            tail: 0,
            count: 0,
            consumed: 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn is_full(&self) -> bool {
        self.count == self.buf.len()
    }

    pub fn push(&mut self, entry: PieceEntry) -> Result<(), ()> {
        if self.is_full() {
            return Err(());
        }
        self.buf[self.head] = entry;
        self.head = (self.head + 1) % self.buf.len();
        self.count += 1;
        Ok(())
    }

    pub fn peek(&self) -> Option<&PieceEntry> {
        if self.is_empty() {
            None
        } else {
            Some(&self.buf[self.tail])
        }
    }

    pub fn pop(&mut self) {
        if !self.is_empty() {
            self.tail = (self.tail + 1) % self.buf.len();
            self.count -= 1;
            self.consumed = self.consumed.wrapping_add(1);
        }
    }

    pub fn consumed_count(&self) -> u32 {
        self.consumed
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo test -p runtime --test piece_ring -v`
Expected: all ring tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/piece_ring.rs rust/runtime/tests/piece_ring.rs
git commit -m "feat: add PieceRing SPSC buffer with consumed count"
```

---

### Task 3: Protocol Messages — PushPieces

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Modify: `rust/kalico-protocol/src/messages/tests.rs`

- [ ] **Step 1: Write failing test for PushPieces encode/decode**

Add to `rust/kalico-protocol/src/messages/tests.rs`:

```rust
#[test]
fn push_pieces_roundtrip_single_piece() {
    use crate::messages::{PushPieces, PushPiecesResponse};
    use crate::codec::{Decode, Encode, Cursor};

    let msg = PushPieces {
        axis_idx: 2,
        piece_count: 1,
        pieces_bytes: vec![0u8; 32], // one 32-byte piece
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPieces::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.axis_idx, 2);
    assert_eq!(decoded.piece_count, 1);
    assert_eq!(decoded.pieces_bytes.len(), 32);
}

#[test]
fn push_pieces_response_roundtrip() {
    use crate::messages::PushPiecesResponse;
    use crate::codec::{Decode, Encode, Cursor};

    let msg = PushPiecesResponse { result: 0 };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPiecesResponse::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.result, 0);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p kalico-protocol -- push_pieces 2>&1 | head -10`
Expected: compilation error — `PushPieces` doesn't exist.

- [ ] **Step 3: Implement PushPieces message**

Add to `rust/kalico-protocol/src/messages.rs` (in the MessageKind enum and as struct + impls):

```rust
// In MessageKind enum:
PushPieces = 0x0020,
PushPiecesResponse = 0x0021,

// Struct definitions:
#[derive(Debug, Clone, PartialEq)]
pub struct PushPieces {
    pub axis_idx: u8,
    pub piece_count: u8,
    pub pieces_bytes: Vec<u8>, // piece_count * 32 bytes
}

impl Encode for PushPieces {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axis_idx);
        put_u8(out, self.piece_count);
        out.extend_from_slice(&self.pieces_bytes);
    }
}

impl Decode for PushPieces {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let axis_idx = get_u8(c)?;
        let piece_count = get_u8(c)?;
        let byte_len = (piece_count as usize) * 32;
        let pieces_bytes = c.take_bytes(byte_len)?.to_vec();
        Ok(Self { axis_idx, piece_count, pieces_bytes })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushPiecesResponse {
    pub result: i32,
}

impl Encode for PushPiecesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for PushPiecesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self { result: get_i32(c)? })
    }
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-protocol -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs rust/kalico-protocol/src/messages/tests.rs
git commit -m "feat: add PushPieces/PushPiecesResponse protocol messages"
```

---

### Task 4: Protocol — Update RuntimeCapsResponse and StatusHeartbeat

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Modify: `rust/kalico-protocol/src/messages/tests.rs`

- [ ] **Step 1: Write failing tests**

```rust
#[test]
fn runtime_caps_response_new_format() {
    use crate::messages::RuntimeCapsResponse;
    use crate::codec::{Decode, Encode, Cursor};

    let msg = RuntimeCapsResponse { total_piece_memory: 63488 };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 4); // single u32
    let mut cursor = Cursor::new(&buf);
    let decoded = RuntimeCapsResponse::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.total_piece_memory, 63488);
}

#[test]
fn status_heartbeat_with_consumed_counts() {
    use crate::messages::StatusHeartbeat;
    use crate::codec::{Decode, Encode, Cursor};

    let msg = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        consumed_counts: vec![42, 42, 10],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.engine_state, 1);
    assert_eq!(decoded.fault_code, 0);
    assert_eq!(decoded.consumed_counts, vec![42, 42, 10]);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p kalico-protocol -- runtime_caps_response_new_format status_heartbeat_with_consumed 2>&1 | head -10`

- [ ] **Step 3: Update RuntimeCapsResponse and add StatusHeartbeat**

Replace `RuntimeCapsResponse` struct:
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeCapsResponse {
    pub total_piece_memory: u32,
}

impl Encode for RuntimeCapsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u32(out, self.total_piece_memory);
    }
}

impl Decode for RuntimeCapsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self { total_piece_memory: get_u32(c)? })
    }
}
```

Add `StatusHeartbeat`:
```rust
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusHeartbeat {
    pub engine_state: u8,
    pub fault_code: u8,
    pub consumed_counts: Vec<u32>, // one per configured axis, in registration order
}

impl Encode for StatusHeartbeat {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.engine_state);
        put_u8(out, self.fault_code);
        put_u8(out, self.consumed_counts.len() as u8);
        for &c in &self.consumed_counts {
            put_u32(out, c);
        }
    }
}

impl Decode for StatusHeartbeat {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let engine_state = get_u8(c)?;
        let fault_code = get_u8(c)?;
        let num_axes = get_u8(c)? as usize;
        let mut consumed_counts = Vec::with_capacity(num_axes);
        for _ in 0..num_axes {
            consumed_counts.push(get_u32(c)?);
        }
        Ok(Self { engine_state, fault_code, consumed_counts })
    }
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-protocol -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs rust/kalico-protocol/src/messages/tests.rs
git commit -m "feat: update RuntimeCapsResponse, add StatusHeartbeat message"
```

---

### Task 5: Remove Dead Code — Curve Pool, Segments, Kinematics

**Files:**
- Remove: `rust/runtime/src/curve_pool.rs`, `rust/runtime/src/curve_pool/tests.rs`
- Remove: `rust/runtime/src/cubic_curve.rs`
- Remove: `rust/runtime/src/c_segment_queue.rs`
- Remove: `rust/runtime/src/segment.rs`, `rust/runtime/src/segment/tests.rs`
- Remove: `rust/runtime/src/config.rs`, `rust/runtime/src/config/tests.rs`
- Remove: `rust/runtime/src/kinematics.rs`, `rust/runtime/src/kinematics/tests.rs`
- Remove: `rust/runtime/src/reclaim.rs`
- Remove: `rust/runtime/src/slot.rs`, `rust/runtime/src/slot/tests.rs`
- Remove: `rust/runtime/src/stream.rs`
- Remove: stale integration tests that reference removed modules
- Modify: `rust/runtime/src/lib.rs` — remove module declarations

- [ ] **Step 1: Remove module declarations from lib.rs**

In `rust/runtime/src/lib.rs`, remove these `pub mod` lines:
- `pub mod curve_pool;`
- `pub mod cubic_curve;`
- `pub mod c_segment_queue;`
- `pub mod segment;`
- `pub mod config;`
- `pub mod kinematics;`
- `pub mod reclaim;`
- `pub mod slot;`
- `pub mod stream;`

- [ ] **Step 2: Delete the source files**

```bash
rm rust/runtime/src/curve_pool.rs rust/runtime/src/curve_pool/tests.rs
rmdir rust/runtime/src/curve_pool
rm rust/runtime/src/cubic_curve.rs
rm rust/runtime/src/c_segment_queue.rs
rm rust/runtime/src/segment.rs rust/runtime/src/segment/tests.rs
rmdir rust/runtime/src/segment
rm rust/runtime/src/config.rs rust/runtime/src/config/tests.rs
rmdir rust/runtime/src/config
rm rust/runtime/src/kinematics.rs rust/runtime/src/kinematics/tests.rs
rmdir rust/runtime/src/kinematics
rm rust/runtime/src/reclaim.rs
rm rust/runtime/src/slot.rs rust/runtime/src/slot/tests.rs
rmdir rust/runtime/src/slot
rm rust/runtime/src/stream.rs
```

- [ ] **Step 3: Delete stale integration tests**

```bash
rm rust/runtime/tests/arm_segment.rs
rm rust/runtime/tests/cubic_curve_load.rs
rm rust/runtime/tests/e_follower_absolute.rs
rm rust/runtime/tests/exhaustion_post_pass.rs
rm rust/runtime/tests/loom_curve_pool_alloc.rs
rm rust/runtime/tests/loom_spsc_split.rs
rm rust/runtime/tests/step_push_emits_pieces_for_g5_move.rs
rm rust/runtime/tests/tick_integration.rs
rm rust/runtime/tests/tick_piece_advance.rs
```

- [ ] **Step 4: Fix remaining compilation errors**

The engine.rs, tick.rs, state.rs, and stepping_state.rs files reference the removed modules. At this point they will not compile. That's expected — Tasks 6-7 rewrite them. For now, stub out imports in engine.rs/tick.rs so the crate compiles (the bodies get rewritten next).

This step is intentionally imprecise — the implementer must chase compiler errors until `cargo check -p runtime` passes. The key removals from `stepping_state.rs`:
- Remove `use crate::curve_pool::CurveHandle;`
- Remove `pub curve_handle: Option<CurveHandle>` from `AxisConfig`
- Remove the corresponding field from `new_unconfigured()`

From `engine.rs`: gut the file, leaving only `RuntimeStatus` enum and a minimal `Engine` struct placeholder that compiles.

- [ ] **Step 5: Verify the crate compiles**

Run: `cd rust && cargo check -p runtime`
Expected: compiles (tests may fail — that's fine, they'll be rewritten).

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "remove: curve pool, segments, kinematics, dead types"
```

---

### Task 6: Rewrite Engine — Piece Ring Walker

**Files:**
- Rewrite: `rust/runtime/src/engine.rs`
- Modify: `rust/runtime/src/stepping_state.rs`
- Modify: `rust/runtime/src/state.rs`

- [ ] **Step 1: Define the new AxisConfig with PieceRing**

Rewrite `rust/runtime/src/stepping_state.rs`:
- Keep: `StepMode`, `StepperRef`, `StepperBindingRust`, `TMC_CS_OID_NONE`, `MAX_STEPPERS_PER_AXIS`
- Remove: `N_AXES` (dynamic now), `TickCaches` (will be per-axis in new engine)
- Replace `AxisConfig` with new struct that holds a ring reference index + ISR working state:

```rust
pub struct AxisState {
    pub mode: AtomicU8,
    pub steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    pub microstep_distance: f32,
    // ISR working state (not in ring):
    pub mono_coeffs: [f32; 4],   // cached monomial position coeffs
    pub vel_coeffs: [f32; 3],    // cached velocity coeffs
    pub piece_end_cycles: u64,   // end time of current piece
    pub piece_start_cycles: u64, // start time of current piece
    pub piece_duration: f32,     // duration in seconds
    pub last_step_count: i32,
    pub has_piece: bool,         // true if currently evaluating a piece
    pub ring_idx: u8,            // index into the ring array on Engine
    // Sub-sample timing:
    pub p_prev: f32,
    pub v_prev: f32,
}
```

- [ ] **Step 2: Rewrite Engine struct**

Rewrite `rust/runtime/src/engine.rs`:

```rust
use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};
use crate::clock::TickCounter;
use crate::piece_ring::{PieceEntry, PieceRing};
use crate::stepping_state::AxisState;
use crate::state::SharedState;
use crate::trace::{TraceSample, TRACE_RING_N};
use heapless::spsc::Producer;

pub const MAX_AXES: usize = 8;
pub const FAULT_TOLERANCE_TICKS: u64 = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RuntimeStatus {
    Idle = 0,
    Running = 1,
    Fault = 2,
}

pub struct Engine {
    pub axes: [Option<AxisState>; MAX_AXES],
    pub num_axes: u8,
    pub status: AtomicU8,
    pub last_error: AtomicI32,
    pub tick_counter: TickCounter,
    pub clock_freq: f32,
    step_state: [crate::step::StepMotorState; MAX_AXES],
}
```

- [ ] **Step 3: Implement `Engine::tick`**

```rust
impl Engine {
    pub fn tick(
        &mut self,
        now: u64,
        shared: &SharedState,
        rings: &mut [PieceRing<'_>],
        trace_prod: &mut Producer<'_, TraceSample, TRACE_RING_N>,
    ) {
        for i in 0..self.num_axes as usize {
            let axis = match &mut self.axes[i] {
                Some(a) => a,
                None => continue,
            };
            if axis.microstep_distance == 0.0 {
                continue; // unconfigured
            }
            let ring = &mut rings[axis.ring_idx as usize];
            let piece = Self::get_piece_for_time(axis, ring, now, self.clock_freq);
            if piece.is_none() {
                continue; // idle
            }
            let t_local = (now - axis.piece_start_cycles) as f32 / self.clock_freq;
            let position = crate::monomial::eval_position_from_coeffs(&axis.mono_coeffs, t_local);
            let velocity = crate::monomial::eval_velocity_from_coeffs(&axis.vel_coeffs, t_local);
            // dispatch stepping (pulse or phase) — delegates to existing dispatch_axis
            crate::tick::dispatch_axis(axis, position, velocity, shared, &mut self.step_state[i]);
        }
    }

    fn get_piece_for_time(
        axis: &mut AxisState,
        ring: &mut PieceRing<'_>,
        now: u64,
        clock_freq: f32,
    ) -> Option<()> {
        if axis.has_piece && now < axis.piece_end_cycles {
            return Some(());
        }
        // Try to load next piece
        let next = ring.peek()?;
        if now < next.start_time {
            axis.has_piece = false;
            return None; // not yet
        }
        let tolerance = FAULT_TOLERANCE_TICKS * (clock_freq as u64 / 40_000);
        if now.saturating_sub(next.start_time) > tolerance {
            // FAULT: piece start is in the past
            axis.has_piece = false;
            return None; // caller checks fault flag
        }
        // Arm the new piece
        let (mono, vel) = next.to_monomial();
        axis.piece_start_cycles = next.start_time;
        axis.piece_end_cycles = next.end_time(clock_freq);
        axis.piece_duration = next.duration;
        axis.mono_coeffs = mono;
        axis.vel_coeffs = vel;
        axis.has_piece = true;
        // Free previous slot (the pop releases the slot we just moved off of)
        ring.pop();
        Some(())
    }
}
```

- [ ] **Step 4: Verify compilation**

Run: `cd rust && cargo check -p runtime`
Expected: compiles (some warnings about unused items are fine).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: rewrite Engine as piece-ring walker"
```

---

### Task 7: PushPieces FFI Handler

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `src/kalico_dispatch.c`

- [ ] **Step 1: Add Rust FFI entry point**

Add to `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[no_mangle]
pub unsafe extern "C" fn kalico_runtime_push_pieces(
    rt: *mut RuntimeContext,
    axis_idx: u8,
    piece_count: u8,
    pieces_ptr: *const u8,
    pieces_len: u16,
) -> i32 {
    let rt = &mut *rt;
    if axis_idx as usize >= rt.engine.num_axes as usize {
        return KALICO_ERR_INVALID_ARG;
    }
    if pieces_len as usize != (piece_count as usize) * 32 {
        return KALICO_ERR_INVALID_ARG;
    }
    let ring = &mut rt.rings[axis_idx as usize];
    let slice = core::slice::from_raw_parts(pieces_ptr, pieces_len as usize);
    for chunk in slice.chunks_exact(32) {
        let entry = core::ptr::read_unaligned(chunk.as_ptr() as *const PieceEntry);
        if ring.push(entry).is_err() {
            return KALICO_ERR_RING_FULL;
        }
    }
    KALICO_OK
}
```

- [ ] **Step 2: Add C-side dispatch handler**

In `src/kalico_dispatch.c`, add handler for `PUSH_PIECES` (0x0020):

```c
static void handle_push_pieces(uint32_t correlation_id, const uint8_t *body, uint16_t body_len) {
    if (body_len < 2) {
        send_error_response(correlation_id, MSG_PUSH_PIECES_RESPONSE, KALICO_ERR_INVALID_ARG);
        return;
    }
    uint8_t axis_idx = body[0];
    uint8_t piece_count = body[1];
    uint16_t expected_len = 2 + (uint16_t)piece_count * 32;
    if (body_len != expected_len) {
        send_error_response(correlation_id, MSG_PUSH_PIECES_RESPONSE, KALICO_ERR_INVALID_ARG);
        return;
    }
    int32_t result = kalico_runtime_push_pieces(
        runtime_handle, axis_idx, piece_count, body + 2, body_len - 2);
    // Send response
    uint8_t resp[4];
    resp[0] = (uint8_t)(result);
    resp[1] = (uint8_t)(result >> 8);
    resp[2] = (uint8_t)(result >> 16);
    resp[3] = (uint8_t)(result >> 24);
    send_response(correlation_id, MSG_PUSH_PIECES_RESPONSE, resp, 4);
}
```

- [ ] **Step 3: Verify compilation**

Run: `cd rust && cargo check -p kalico-c-api`
Expected: compiles.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: PushPieces FFI handler + C dispatch"
```

---

### Task 8: ConfigureAxis with Ring Allocation

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/runtime/src/engine.rs`

- [ ] **Step 1: Write test for configure_axis allocating a ring**

Add to `rust/runtime/tests/piece_ring.rs`:

```rust
#[test]
fn configure_axis_allocates_ring() {
    // This tests the engine-level configure_axis logic
    use runtime::engine::Engine;
    use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

    let mut engine = Engine::new(550_000_000.0);
    let bindings = [StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    let result = engine.configure_axis(0, StepMode::Pulse, 0.0125, 256, &bindings);
    assert_eq!(result, 0); // KALICO_OK
    assert_eq!(engine.num_axes, 1);
}
```

- [ ] **Step 2: Implement `Engine::configure_axis`**

```rust
impl Engine {
    pub fn configure_axis(
        &mut self,
        axis_idx: u8,
        mode: StepMode,
        microstep_distance: f32,
        ring_depth: u16,
        bindings: &[StepperBindingRust],
    ) -> i32 {
        if axis_idx as usize >= MAX_AXES {
            return KALICO_ERR_INVALID_ARG;
        }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
            return KALICO_ERR_INVALID_ARG;
        }
        // Allocate ring from pool memory (implementation detail:
        // the ring storage is pre-allocated in the RuntimeContext)
        let axis = AxisState::new(mode, microstep_distance, axis_idx, bindings);
        self.axes[axis_idx as usize] = Some(axis);
        if axis_idx >= self.num_axes {
            self.num_axes = axis_idx + 1;
        }
        KALICO_OK
    }
}
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p runtime --test piece_ring -- configure_axis -v`
Expected: PASS.

- [ ] **Step 4: Update FFI for configure_axis**

Update `kalico_runtime_configure_axis` in `runtime_ffi.rs` to pass `ring_depth` parameter and use the new Engine API.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: ConfigureAxis allocates per-axis ring from pool"
```

---

### Task 9: ISR Tick Integration Test

**Files:**
- Create: `rust/runtime/tests/piece_tick.rs`

- [ ] **Step 1: Write integration test — single axis constant piece**

```rust
use runtime::engine::Engine;
use runtime::piece_ring::{PieceEntry, PieceRing};
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

#[test]
fn tick_evaluates_constant_piece() {
    let clock_freq: f32 = 550_000_000.0;
    let mut engine = Engine::new(clock_freq);

    // Configure axis 0 with pulse stepping, 80 steps/mm
    let bindings = [StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    engine.configure_axis(0, StepMode::Pulse, 1.0 / 80.0, 64, &bindings);

    // Create ring and push a constant piece at position 10.0mm
    let mut storage = [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; 64];
    let mut ring = PieceRing::new(&mut storage);
    ring.push(PieceEntry {
        start_time: 1000,
        coeffs: [10.0, 10.0, 10.0, 10.0], // constant Bernstein = 10.0
        duration: 0.001, // 1ms piece
        _reserved: 0,
    }).unwrap();

    let mut rings = [&mut ring];

    // Tick at start_time + 1 tick
    let now = 1000 + (clock_freq as u64 / 40_000); // one tick after start
    engine.tick(now, /* shared */, &mut rings, /* trace */);

    // The axis should have armed the piece
    let axis = engine.axes[0].as_ref().unwrap();
    assert!(axis.has_piece);
}
```

- [ ] **Step 2: Run to verify compilation and behavior**

Run: `cd rust && cargo test -p runtime --test piece_tick -v`

- [ ] **Step 3: Write test — piece advancement**

```rust
#[test]
fn tick_advances_to_next_piece() {
    // Push two consecutive pieces, tick past the first one's end,
    // verify the engine transitions to the second piece.
    // ... (full implementation)
}
```

- [ ] **Step 4: Write test — fault on piece-start-in-past**

```rust
#[test]
fn tick_faults_on_piece_start_in_past() {
    // Push a piece with start_time far in the past,
    // verify the engine triggers a fault.
    // ... (full implementation)
}
```

- [ ] **Step 5: Make all tests pass and commit**

```bash
git add -A
git commit -m "test: ISR tick integration tests for piece ring walker"
```

---

### Task 10: StatusHeartbeat Implementation

**Files:**
- Modify: `rust/runtime/src/engine.rs`
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `src/runtime_tick.c` (the 10 Hz drain task)

- [ ] **Step 1: Add `Engine::get_status` method**

```rust
impl Engine {
    pub fn get_status(&self, rings: &[PieceRing<'_>]) -> (u8, u8, &[u32]) {
        let state = self.status.load(Ordering::Relaxed);
        let fault = self.last_error.load(Ordering::Relaxed) as u8;
        // Collect consumed counts per axis
        // (stored in a fixed array on the engine, updated by tick)
        (state, fault, &self.consumed_counts[..self.num_axes as usize])
    }
}
```

- [ ] **Step 2: Wire into C-side 10 Hz drain task**

The existing `runtime_status_drain` in `src/runtime_tick.c` calls a Rust FFI function to get status and emits it as a frame. Update to call new `kalico_runtime_get_heartbeat` FFI.

- [ ] **Step 3: Test heartbeat encoding**

Run: `cd rust && cargo test -p kalico-protocol -- status_heartbeat -v`
Expected: PASS (from Task 4's tests).

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: StatusHeartbeat with per-axis consumed counts"
```

---

### Task 11: Kconfig and C-Side Cleanup

**Files:**
- Modify: `src/Kconfig`
- Remove: `src/kalico_segment_queue.c`, `src/kalico_segment_queue.h`
- Modify: `src/kalico_dispatch.c` — remove old handlers
- Modify: `rust/runtime/build.rs` — read new config option

- [ ] **Step 1: Update Kconfig**

Remove `CONFIG_RUNTIME_CURVE_POOL_N` and `CONFIG_RUNTIME_MAX_PIECES_PER_CURVE`. Add:

```kconfig
config RUNTIME_PIECE_RING_SIZE
    int "Total piece ring memory (bytes)"
    default 63488 if RUNTIME_TARGET_LARGE
    default 16384 if RUNTIME_TARGET_SMALL
    default 16384
    help
      Total static memory allocated for piece ring buffers.
      Divided equally among configured axes at runtime.
      Each piece entry is 32 bytes. With 4 axes on H7:
      63488 / 32 / 4 = 496 pieces per axis.
```

- [ ] **Step 2: Update build.rs**

In `rust/runtime/build.rs`, replace the old pool lookups with:

```rust
let ring_size = lookup("KALICO_RUNTIME_PIECE_RING_SIZE", "63488");
// emit: pub const PIECE_RING_SIZE: usize = {ring_size};
```

- [ ] **Step 3: Remove C segment queue files**

```bash
rm src/kalico_segment_queue.c src/kalico_segment_queue.h
```

- [ ] **Step 4: Remove old dispatch handlers from kalico_dispatch.c**

Remove `handle_load_curve_cubic`, `handle_push_segment`, `handle_reset_curve_pool` functions and their routing entries.

- [ ] **Step 5: Verify build**

Run: `cd rust && cargo build -p runtime && cargo build -p kalico-c-api`
Expected: compiles.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "chore: Kconfig PIECE_RING_SIZE, remove C segment queue, old dispatch handlers"
```

---

### Task 12: Remove Old Protocol Messages

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`

- [ ] **Step 1: Remove old message types**

Remove from `MessageKind` enum and their struct/impl blocks:
- `LoadCurveCubic` (0x0010) / `LoadCurveResponse` (0x0011)
- `PushSegment` (0x0020) / `PushSegmentResponse` (0x0021) — note: `PushPieces` now owns 0x0020/0x0021
- `ResetCurvePool` (0x0050) / `ResetCurvePoolResponse` (0x0051)
- `CreditFreed` (0x0081)
- Old `StatusEvent` (0x0080) — replaced by `StatusHeartbeat`

Update wire ID assignments:
- `PushPieces` = 0x0020, `PushPiecesResponse` = 0x0021
- `StatusHeartbeat` = 0x0080

- [ ] **Step 2: Fix any compilation errors in dependent crates**

Run: `cd rust && cargo check --workspace`
Chase errors in `kalico-host-rt`, `kalico-native-transport`, `kalico-c-api` that reference removed types.

- [ ] **Step 3: Run protocol tests**

Run: `cd rust && cargo test -p kalico-protocol -v`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "remove: old protocol messages (LoadCurveCubic, PushSegment, ResetCurvePool, CreditFreed)"
```

---

### Task 13: Final Integration — Workspace Builds Clean

**Files:**
- Various — fix any remaining compilation errors across workspace

- [ ] **Step 1: Full workspace check**

Run: `cd rust && cargo build --workspace 2>&1 | head -50`
Fix any errors.

- [ ] **Step 2: Run all tests**

Run: `cd rust && cargo test -p runtime -p kalico-protocol`
Expected: PASS (new tests). Old tests that referenced removed code should already be deleted.

- [ ] **Step 3: Run clippy**

Run: `cd rust && cargo clippy --workspace --all-targets -- -D warnings`
Fix any warnings.

- [ ] **Step 4: Run fmt**

Run: `cd rust && cargo fmt --all -- --check`
Fix formatting.

- [ ] **Step 5: Commit any final fixups**

```bash
git add -A
git commit -m "chore: workspace builds clean, clippy + fmt pass"
```

---

### Task 14: Calculate Sane Defaults for Ring Sizing

**Files:**
- Modify: `docs/superpowers/specs/2026-05-27-simple-mcu-contract-design.md` §8

- [ ] **Step 1: Calculate H7 ring depth**

H7 (BTT Octopus Pro): 1 MB SRAM. Budget ~62 KB for piece rings.
- 4 axes (A, B, Z, E in CoreXY): `63488 / 32 / 4 = 496 pieces per axis`
- At ~1 ms/piece: 496 ms look-ahead. At 10 Hz heartbeat, host has ~5 heartbeats to react. Comfortable.

- [ ] **Step 2: Calculate F4 ring depth**

F4: 128 KB SRAM. Budget ~16 KB for piece rings.
- 1 axis (Z): `16384 / 32 / 1 = 512 pieces per axis`
- 512 ms look-ahead. Comfortable.
- 2 axes: `16384 / 32 / 2 = 256 pieces per axis` = 256 ms. Still fine.

- [ ] **Step 3: Update spec §8 with calculated values**

Document the defaults and rationale in the spec.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "docs: ring sizing defaults for H7 and F4"
```
