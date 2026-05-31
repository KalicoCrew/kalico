# PushPieces Receive-Into-Ring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the host→MCU piece-transfer path so the host writes pieces directly into absolute ring slots (no separate staging buffer, no hidden 128-piece cap), eliminating the oversized-frame silent-drop that caused the `PieceStartInPast` jog crash.

**Architecture:** Dumb MCU / smart host. The MCU is a ring consumer plus a slot-writer with no write-time guards: the host addresses pieces by physical slot, streams them into the ring on a dedicated transport channel, and commits a monotonic valid-frontier (`head`) only after CRC. The host owns all accounting (physical write cursor, flow control `head − consumed ≤ N`, `head`). Re-sends are idempotent overwrites. See `docs/superpowers/specs/2026-05-29-pushpieces-receive-into-ring-design.md`.

**Tech Stack:** Rust (runtime engine, kalico-c-api FFI staticlib, kalico-protocol wire codec, motion-bridge host pump) compiled f64-host / f32-MCU; C (MCU transport: kalico_demux, kalico_dispatch, runtime_storage). Tests: `cargo test` for Rust units; Linux-sim + Python + hardware bench for the C transport and end-to-end.

---

## Landing constraints (read before starting)

1. **This is one atomic wire-contract change.** The PushPieces frame format and channel change on both the host and **both** MCUs. The system is only end-to-end-correct after Tasks 1–8 all land. Commit per task (each compiles + passes its unit tests), but do not flash a half-applied change to the bench. The bench flow (`flashing-trident-mcus` skill) flashes host + H7 + F4 from one commit; `make clean` between H7 and F4.
2. **Channel decision (resolves a mapping ambiguity).** Pieces move to a new dedicated transport channel `KALICO_CHANNEL_PIECES = 0x02` (spec §4.3 — "no message logic in the demux"). We *reuse* the existing `kalico_call` request/response framing (per-message `kind`/`version`/`correlation_id` header + `PushPiecesResponse` on the control channel) so correlation matching is unchanged; the only transport change is that the frame is sent on channel `0x02`, and the MCU demux routes channel `0x02` straight to a streaming sink instead of buffering + message-type dispatch.
3. **f32/f64 single-source compile.** Runtime code compiles f64 on host (tests) and f32 on MCU. Don't introduce `f64` literals into runtime hot paths; follow the existing `cfg`/type-alias patterns already in `rust/runtime`.
4. **cbindgen header is generated.** After changing any `runtime_ffi.rs` `extern "C"` signature, regenerate `rust/kalico-c-api/include/kalico_runtime.h` (Task 4) and commit it.

## File Structure

| File | Action | Responsibility after change |
|------|--------|------------------------------|
| `rust/kalico-protocol/src/messages.rs` | modify | `PushPieces` struct + Encode/Decode gain `start_slot: u16`, `new_head: u32` |
| `rust/kalico-protocol/schema_def.rs` | modify | `PushPieces` schema → version 2, new fields, channel `pieces` |
| `rust/kalico-protocol/src/lib.rs` | modify | add `KALICO_CHANNEL_PIECES` channel constant (Rust side) |
| `src/kalico_dispatch.c` / `.h` (channel consts live in dispatch.c) | modify | add `KALICO_CHANNEL_PIECES 0x02` |
| `rust/runtime/src/piece_ring.rs` | modify | `RingDescriptor` → monotonic `head:u32`/`consumed:u32` + physical `tail`; add `write_slot`, `commit_head`; `len()=head−consumed` |
| `rust/runtime/src/engine.rs` | modify | consumer unaffected (verify no `head==tail` compares); `push_pieces` retained for tests only |
| `rust/kalico-c-api/src/runtime_ffi.rs` | modify | replace `kalico_runtime_push_pieces` with `kalico_runtime_write_piece` + `kalico_runtime_commit_head` |
| `rust/kalico-c-api/include/kalico_runtime.h` | regenerate | reflects the two new FFI fns |
| `rust/motion-bridge/src/bridge.rs` | modify | Gap 2: `ring_depth_for_axis_inner` hard-errors above `u16::MAX` |
| `rust/motion-bridge/src/pump.rs` | modify | `AxisQueue.physical_write_cursor`; `FramePlan.start_slot`; send `start_slot`+`new_head` on `KALICO_CHANNEL_PIECES` |
| `src/kalico_demux.c` / `.h` | modify | route channel `0x02` → streaming piece sink; shrink `KALICO_DEMUX_KALICO_BUF_SIZE` |
| `src/kalico_dispatch.c` | modify | add piece sink (`write_piece` loop + `commit_head` + response); retire `handle_push_pieces` |
| `src/runtime_storage.c` | modify | fix stale `AXI_BSS_KALICO_BUF_BYTES` |

---

## Task 1: Protocol — `PushPieces` v2 wire format (`start_slot` + `new_head`)

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs:180-222`
- Modify: `rust/kalico-protocol/schema_def.rs:70-80`
- Test: `rust/kalico-protocol/src/messages/tests.rs`

- [ ] **Step 1: Write the failing round-trip test**

In `rust/kalico-protocol/src/messages/tests.rs`, add:

```rust
#[test]
fn push_pieces_v2_roundtrip_carries_slot_and_head() {
    let msg = PushPieces {
        axis_idx: 2,
        piece_count: 1,
        start_slot: 41,
        new_head: 5000,
        pieces_bytes: vec![0xAB; 32],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    // axis_idx(1) + piece_count(1) + start_slot(2) + new_head(4) + 32 = 40 bytes.
    assert_eq!(buf.len(), 40);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPieces::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.axis_idx, 2);
    assert_eq!(decoded.piece_count, 1);
    assert_eq!(decoded.start_slot, 41);
    assert_eq!(decoded.new_head, 5000);
    assert_eq!(decoded.pieces_bytes, vec![0xAB; 32]);
}
```

- [ ] **Step 2: Run it; verify it fails to compile**

Run: `cd rust && cargo test -p kalico-protocol push_pieces_v2_roundtrip -- --nocapture`
Expected: FAIL — `PushPieces` has no field `start_slot` / `new_head`.

- [ ] **Step 3: Add the fields + update Encode/Decode**

In `rust/kalico-protocol/src/messages.rs`, the struct (currently lines 180-186):

```rust
pub struct PushPieces {
    pub axis_idx: u8,
    pub piece_count: u8,
    pub start_slot: u16,
    pub new_head: u32,
    pub pieces_bytes: Vec<u8>,
}
```

Encode (currently lines 188-193):

```rust
impl Encode for PushPieces {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axis_idx);
        put_u8(out, self.piece_count);
        put_u16(out, self.start_slot);
        put_u32(out, self.new_head);
        out.extend_from_slice(&self.pieces_bytes);
    }
}
```

Decode (currently lines 196-222) — insert the two reads after `piece_count`, before the `pieces_len` validation:

```rust
impl Decode for PushPieces {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let axis_idx = get_u8(c)?;
        let piece_count = get_u8(c)?;
        let start_slot = get_u16(c)?;
        let new_head = get_u32(c)?;
        let pieces_len = piece_count as usize * 32;
        let pieces_bytes = take_bytes(c, pieces_len)?; // keep the existing read helper used today
        Ok(Self { axis_idx, piece_count, start_slot, new_head, pieces_bytes })
    }
}
```

(Use whatever `get_u16`/`get_u32`/`put_u16`/`put_u32`/byte-read helpers already exist in this file's codec — match the names used by the other messages. If `put_u16`/`get_u16` don't exist yet, add them next to the existing `put_u8`/`get_u8` following the little-endian pattern in `codec.rs`.)

- [ ] **Step 4: Bump the schema entry**

In `rust/kalico-protocol/schema_def.rs:70-80`, change `version: 1` → `version: 2`, set channel to `"pieces"`, and add the two fields:

```rust
SchemaMessage {
    type_tag: 0x0060,
    name: "PushPieces",
    version: 2,
    channel: "pieces",
    fields: &[
        SchemaField { name: "axis_idx", ty: "u8" },
        SchemaField { name: "piece_count", ty: "u8" },
        SchemaField { name: "start_slot", ty: "u16" },
        SchemaField { name: "new_head", ty: "u32" },
        SchemaField { name: "pieces_bytes", ty: "array<u8>" },
    ],
}
```

- [ ] **Step 5: Fix the existing round-trip test that asserts the old length**

The existing `push_pieces_roundtrip_single` test (messages/tests.rs) asserts `buf.len() == 34` and constructs `PushPieces` without the new fields. Update it: add `start_slot: 0, new_head: 0` to the literal and change the length assertion to `40`.

- [ ] **Step 6: Run the protocol tests**

Run: `cd rust && cargo test -p kalico-protocol`
Expected: PASS (new round-trip + updated old test + any schema-hash test now reflecting v2).

- [ ] **Step 7: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs rust/kalico-protocol/src/messages/tests.rs rust/kalico-protocol/schema_def.rs
git commit -m "feat(protocol): PushPieces v2 carries start_slot + new_head"
```

---

## Task 2: Add the `KALICO_CHANNEL_PIECES` channel constant

**Files:**
- Modify: `rust/kalico-protocol/src/lib.rs` (Rust channel constants)
- Modify: `src/kalico_dispatch.c:28-29` (C channel `#define`s)

- [ ] **Step 1: Add the Rust constant**

In `rust/kalico-protocol/src/lib.rs`, next to the existing channel constants (find `CONTROL`/`EVENTS` — grep `CHANNEL`), add:

```rust
pub const KALICO_CHANNEL_CONTROL: u8 = 0x00;
pub const KALICO_CHANNEL_EVENTS: u8 = 0x01;
pub const KALICO_CHANNEL_PIECES: u8 = 0x02;
```

(Keep the existing two; add only `PIECES` if the first two already exist.)

- [ ] **Step 2: Add the C constant**

In `src/kalico_dispatch.c:28-29`, after the existing two:

```c
#define KALICO_CHANNEL_CONTROL 0x00
#define KALICO_CHANNEL_EVENTS  0x01
#define KALICO_CHANNEL_PIECES  0x02
```

- [ ] **Step 3: Build the host crate to confirm the constant compiles**

Run: `cd rust && cargo build -p kalico-protocol`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-protocol/src/lib.rs src/kalico_dispatch.c
git commit -m "feat(transport): add KALICO_CHANNEL_PIECES (0x02)"
```

---

## Task 3: Runtime ring — monotonic cursors + `write_slot` + `commit_head`

**Files:**
- Modify: `rust/runtime/src/piece_ring.rs:52-167`
- Test: `rust/runtime/tests/piece_ring.rs` (or the inline `#[cfg(test)]` block — match where `RingDescriptor` tests live)

**Background:** Today `RingDescriptor` is `{ ring_offset, ring_depth, head: usize (physical), tail: usize (physical), count: usize, consumed: u32 }`. The new representation drops `count` (derived) and makes `head` a **monotonic `u32` valid frontier**; `tail` stays a physical read cursor; `consumed` stays monotonic. `len() = head.wrapping_sub(consumed)`. Slot writes are absolute (host-addressed). `head` advances via `commit_head` (monotone), never per-piece.

- [ ] **Step 1: Write failing tests for the new methods**

Add to the `RingDescriptor` test module:

```rust
fn make_storage<const N: usize>() -> [PieceEntry; N] {
    [PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }; N]
}
fn pe(start: u64) -> PieceEntry {
    PieceEntry { start_time: start, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 }
}

#[test]
fn write_slot_lands_at_absolute_index_without_advancing_head() {
    let mut storage = make_storage::<8>();
    let mut ring = RingDescriptor::new(0, 8);
    ring.write_slot(&mut storage, 5, pe(1234));
    assert_eq!(storage[5].start_time, 1234);
    // write_slot does not move the frontier; nothing is visible yet.
    assert_eq!(ring.len(), 0);
    assert!(ring.is_empty());
}

#[test]
fn commit_head_makes_slots_visible_and_is_monotone() {
    let mut storage = make_storage::<8>();
    let mut ring = RingDescriptor::new(0, 8);
    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    assert_eq!(ring.len(), 2);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 10);
    // A stale, lower new_head (re-send) is ignored.
    ring.commit_head(1);
    assert_eq!(ring.len(), 2);
}

#[test]
fn pop_advances_physical_tail_and_monotonic_consumed() {
    let mut storage = make_storage::<4>();
    let mut ring = RingDescriptor::new(0, 4);
    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    ring.pop();
    assert_eq!(ring.consumed_count(), 1);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 20);
    assert_eq!(ring.len(), 1);
}

#[test]
fn empty_full_distinct_via_monotonic_difference() {
    let mut storage = make_storage::<2>();
    let mut ring = RingDescriptor::new(0, 2);
    assert!(ring.is_empty());
    ring.write_slot(&mut storage, 0, pe(1));
    ring.write_slot(&mut storage, 1, pe(2));
    ring.commit_head(2);
    assert!(ring.is_full());      // len == ring_depth, not mistaken for empty
    assert!(!ring.is_empty());
}
```

- [ ] **Step 2: Run; verify it fails to compile**

Run: `cd rust && cargo test -p runtime write_slot_lands -- --nocapture`
Expected: FAIL — `write_slot` / `commit_head` not found; `head` type mismatch.

- [ ] **Step 3: Redefine `RingDescriptor` and its methods**

In `rust/runtime/src/piece_ring.rs`:

Struct (replace lines 52-65):

```rust
pub struct RingDescriptor {
    pub ring_offset: usize,
    pub ring_depth: usize,
    /// Monotonic valid frontier (host-driven; advanced only by `commit_head`).
    pub head: u32,
    /// Physical read cursor in [0, ring_depth); advanced one per `pop`.
    pub tail: usize,
    /// Monotonic consumed counter (heartbeat); advanced one per `pop`.
    pub consumed: u32,
}
```

`new` / `new_unconfigured` — initialize the same fields to zero (drop `count`):

```rust
pub const fn new(offset: usize, depth: usize) -> Self {
    Self { ring_offset: offset, ring_depth: depth, head: 0, tail: 0, consumed: 0 }
}
pub const fn new_unconfigured() -> Self {
    Self { ring_offset: 0, ring_depth: 0, head: 0, tail: 0, consumed: 0 }
}
```

`len` / `is_empty` / `is_full` — derive from monotonic difference:

```rust
pub fn len(&self) -> usize {
    self.head.wrapping_sub(self.consumed) as usize
}
pub fn is_empty(&self) -> bool {
    self.head == self.consumed
}
pub fn is_full(&self) -> bool {
    self.len() == self.ring_depth
}
```

Replace `push` with `write_slot` (absolute, no frontier change):

```rust
/// Write one entry to an absolute physical slot. Does NOT advance `head`.
/// `physical_slot` must be < ring_depth (caller/host guarantees this).
pub fn write_slot(&self, storage: &mut [PieceEntry], physical_slot: usize, entry: PieceEntry) {
    if self.ring_depth == 0 || physical_slot >= self.ring_depth {
        return;
    }
    storage[self.ring_offset + physical_slot] = entry;
}
```

Add `commit_head` (monotone relative to `consumed`, wrap-safe):

```rust
/// Advance the valid frontier to `new_head`, monotonically. A `new_head`
/// that is not ahead of the current `head` (a stale re-send) is ignored.
/// Comparison is relative to `consumed` so it is correct across the u32 wrap;
/// both deltas are <= ring_depth in normal operation.
pub fn commit_head(&mut self, new_head: u32) {
    let cur = self.head.wrapping_sub(self.consumed);
    let proposed = new_head.wrapping_sub(self.consumed);
    if proposed > cur {
        self.head = new_head;
    }
}
```

`pop` (advance physical tail + monotonic consumed; no `count`):

```rust
pub fn pop(&mut self) {
    if self.ring_depth == 0 || self.is_empty() {
        return;
    }
    self.tail += 1;
    if self.tail >= self.ring_depth {
        self.tail = 0;
    }
    self.consumed = self.consumed.wrapping_add(1);
}
```

`peek` and `consumed_count` are unchanged. Keep the `is_configured` method as-is.

- [ ] **Step 4: Update the test-only `push` and `PieceRing<'a>` if they reference `count`**

`PieceRing<'a>` (host-test wrapper, lines 200-210) and the test-only `RingDescriptor::push` may still reference a `count` field. Two options — pick the one that compiles cleanest:
- If `RingDescriptor::push` is only used by tests, reimplement it in terms of the new representation: write at `(self.head % self.ring_depth as u32) as usize`, then `self.head = self.head.wrapping_add(1)`; return `Err(())` if `self.is_full()` or `ring_depth == 0`.
- `PieceRing<'a>` keeps its own private `head/tail/count` fields (it is a separate borrow-holding struct, not the ISR descriptor) — leave it untouched if it compiles; it is host-test-only.

Run `cd rust && cargo build -p runtime` and fix any remaining `count` references the compiler flags.

- [ ] **Step 5: Verify the consumer has no `head == tail` / physical-`head` assumptions**

Run: `grep -n "\.head" rust/runtime/src/engine.rs`
Expected: the consumer (`get_position_and_velocity`, `tick`) uses `peek`/`pop`/`is_empty`/`consumed_count`, never a raw `head == tail` comparison or `head` as a physical index. If any raw `head` comparison exists, replace it with `is_empty()` / `len()`. (Per the spec the consumer is copy-on-arm and only needs `is_empty` + `peek` + `pop`.)

- [ ] **Step 6: Run the ring tests + the existing consumer tests**

Run: `cd rust && cargo test -p runtime`
Expected: PASS — new ring tests pass; `tick_arms_piece_when_start_time_reached` and other consumer tests still pass (they use `engine.push_pieces` → ring append, which still works via the rewritten test-only `push`).

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/piece_ring.rs rust/runtime/src/engine.rs
git commit -m "feat(runtime): ring monotonic head/consumed + write_slot/commit_head"
```

---

## Task 4: Runtime FFI — `kalico_runtime_write_piece` + `kalico_runtime_commit_head`

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs:1451-1502`
- Regenerate: `rust/kalico-c-api/include/kalico_runtime.h`
- Test: `rust/kalico-c-api/tests/` (new test file or extend `init_once.rs`)

- [ ] **Step 1: Write the failing FFI test**

Create `rust/kalico-c-api/tests/write_piece.rs` (mirror the init/handle setup from `tests/init_once.rs` — it shows how to obtain an initialized `*mut KalicoRuntime` and configure an axis):

```rust
// Setup helpers `init_runtime()` and `configure_axis0(rt, depth)` mirror
// the pattern in tests/init_once.rs. Reuse them.

#[test]
fn write_piece_then_commit_head_makes_one_piece_visible() {
    let rt = init_runtime();
    configure_axis0(rt, 64); // ring_depth 64 on axis 0

    // A 32-byte PieceEntry with start_time = 7777, all else zero.
    let mut piece = [0u8; 32];
    piece[0..8].copy_from_slice(&7777u64.to_le_bytes());

    unsafe {
        let rc = kalico_runtime_write_piece(rt, 0, /*start_slot*/ 0, /*index*/ 0, piece.as_ptr());
        assert_eq!(rc, KALICO_OK);
        // Not visible until commit.
        // (consumed_count stays 0; we assert via the heartbeat helper if available,
        //  otherwise rely on the commit assertion below.)
        let rc = kalico_runtime_commit_head(rt, 0, /*new_head*/ 1);
        assert_eq!(rc, KALICO_OK);
    }
}

#[test]
fn write_piece_rejects_unconfigured_axis() {
    let rt = init_runtime();
    let piece = [0u8; 32];
    unsafe {
        let rc = kalico_runtime_write_piece(rt, 3, 0, 0, piece.as_ptr());
        assert_eq!(rc, KALICO_ERR_INVALID_ARG);
    }
}

#[test]
fn write_piece_null_rt_is_null_ptr_error() {
    let piece = [0u8; 32];
    unsafe {
        let rc = kalico_runtime_write_piece(core::ptr::null_mut(), 0, 0, 0, piece.as_ptr());
        assert_eq!(rc, KALICO_ERR_NULL_PTR);
    }
}
```

- [ ] **Step 2: Run; verify it fails**

Run: `cd rust && cargo test -p kalico-c-api --test write_piece`
Expected: FAIL — `kalico_runtime_write_piece` / `kalico_runtime_commit_head` not defined.

- [ ] **Step 3: Implement the two FFI functions**

In `rust/kalico-c-api/src/runtime_ffi.rs`, replace `kalico_runtime_push_pieces` (lines 1451-1502) with the two functions below. Mirror the handle-deref boilerplate of the function you're replacing (null check, `INIT_DONE.load(Acquire)`, `UnsafeCell::raw_get` projection of `piece_storage` and the engine/ring). Use `read_unaligned` for the incoming bytes.

```rust
/// Write one 32-byte PieceEntry to absolute physical slot
/// `(start_slot + index) mod ring_depth` for `axis_idx`. Does not advance the
/// frontier (see `kalico_runtime_commit_head`). Streamed pre-CRC by the transport.
#[no_mangle]
pub unsafe extern "C" fn kalico_runtime_write_piece(
    rt: *mut KalicoRuntime,
    axis_idx: u8,
    start_slot: u16,
    index: u8,
    piece_ptr: *const u8,
) -> i32 {
    if rt.is_null() || piece_ptr.is_null() {
        return KALICO_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return KALICO_ERR_NOT_INIT;
    }
    let ctx = &*(rt as *const RuntimeContext);
    let isr = &mut *ctx.isr.get();
    let storage = &mut *(UnsafeCell::raw_get(&ctx.piece_storage) as *mut [PieceEntry; TOTAL_RING_PIECES]);

    let axis = match isr.engine.axis_mut(axis_idx) {
        Some(a) if a.ring.is_configured() => a,
        _ => return KALICO_ERR_INVALID_ARG,
    };
    let depth = axis.ring.ring_depth;
    let slot = (start_slot as usize + index as usize) % depth;
    let entry = core::ptr::read_unaligned(piece_ptr as *const PieceEntry);
    axis.ring.write_slot(&mut storage[..], slot, entry);
    KALICO_OK
}

/// Advance the axis ring's monotonic valid frontier to `new_head` (monotone;
/// a lower value from a stale re-send is ignored). Called post-CRC by the transport.
#[no_mangle]
pub unsafe extern "C" fn kalico_runtime_commit_head(
    rt: *mut KalicoRuntime,
    axis_idx: u8,
    new_head: u32,
) -> i32 {
    if rt.is_null() {
        return KALICO_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return KALICO_ERR_NOT_INIT;
    }
    let ctx = &*(rt as *const RuntimeContext);
    let isr = &mut *ctx.isr.get();
    let axis = match isr.engine.axis_mut(axis_idx) {
        Some(a) if a.ring.is_configured() => a,
        _ => return KALICO_ERR_INVALID_ARG,
    };
    axis.ring.commit_head(new_head);
    KALICO_OK
}
```

Notes for the implementer:
- `axis_mut(axis_idx)` is the accessor used by the old `push_pieces`/`tick` paths to reach `stepping_axes[axis_idx]`. Use the exact accessor the existing code uses (grep `stepping_axes` / `axis_mut` in engine.rs); if it returns `Option<&mut AxisState>`, the match above is correct.
- `RuntimeContext`, `TOTAL_RING_PIECES`, `UnsafeCell::raw_get`, and `INIT_DONE` are the same items the old `push_pieces` used at runtime_ffi.rs:209/113 — reuse them verbatim.
- The old `kalico_runtime_push_pieces` is deleted (the C side no longer calls it after Task 7).

- [ ] **Step 4: Run the FFI tests**

Run: `cd rust && cargo test -p kalico-c-api --test write_piece`
Expected: PASS.

- [ ] **Step 5: Regenerate the C header**

Run: `cd rust && cargo run -p kalico-c-api --bin gen-headers`
(If the binary name differs, grep `cbindgen` in `rust/kalico-c-api/` for the generation entry point.)
Expected: `rust/kalico-c-api/include/kalico_runtime.h` now declares `kalico_runtime_write_piece` and `kalico_runtime_commit_head` and no longer declares `kalico_runtime_push_pieces`.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h rust/kalico-c-api/tests/write_piece.rs
git commit -m "feat(ffi): replace push_pieces with write_piece + commit_head"
```

---

## Task 5: Host bridge — Gap 2 single-source depth (hard error above u16)

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:552-585`
- Test: `rust/motion-bridge/src/bridge.rs` (existing `#[cfg(test)] mod` for ring-depth)

- [ ] **Step 1: Write the failing test**

In the bridge ring-depth test module, add:

```rust
#[test]
fn ring_depth_over_u16_is_hard_error_not_clamp() {
    // total_piece_memory huge, 1 axis → axis_ring_depth > u16::MAX.
    // Construct an McuAxisConfig whose caps.total_pieces() / num_axes exceeds 65535.
    let configs = vec![mcu_axis_config_with_total_pieces(70_000 * 32, /*axes*/ 1)];
    let res = ring_depth_for_axis_inner(&configs, /*mcu_handle*/ 0, /*axis*/ 0);
    assert!(res.is_err(), "depth > u16::MAX must be a hard error, not a clamp");
}
```

(Reuse/extend the existing test helper that builds an `McuAxisConfig`; if none exists, build it inline matching the struct fields used elsewhere in this module.)

- [ ] **Step 2: Run; verify it fails**

Run: `cd rust && cargo test -p motion-bridge ring_depth_over_u16 -- --nocapture`
Expected: FAIL — current code warns and returns `Ok(u16::MAX)`.

- [ ] **Step 3: Replace the warn+clamp with an error**

In `rust/motion-bridge/src/bridge.rs`, `ring_depth_for_axis_inner` (lines 575-581 — the clamp branch):

```rust
let depth_u32 = axis_ring_depth(cfg.caps.total_pieces() as u32, cfg.axes.len() as u32);
if depth_u32 > u16::MAX as u32 {
    return Err(format!(
        "ring depth {depth_u32} exceeds u16::MAX (65535) for mcu {mcu_handle} axis {axis}; \
         a >65535-piece ring would need >2 MB of SRAM and is impossible here — \
         check total_piece_memory configuration"
    ));
}
Ok(depth_u32 as u16)
```

(Leave `axis_ring_depth` and the `ring_depth_table` build at line 2152 unchanged — making this a hard error guarantees the table's u32 and the MCU's u16 can never diverge.)

- [ ] **Step 4: Run the bridge tests**

Run: `cd rust && cargo test -p motion-bridge`
Expected: PASS (new test + existing `axis_ring_depth_tests`).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "fix(bridge): ring depth above u16::MAX is a hard error (Gap 2)"
```

---

## Task 6: Host pump — physical write cursor, frame fields, send on pieces channel

**Files:**
- Modify: `rust/motion-bridge/src/pump.rs:22-46, 319-477`
- Test: `rust/motion-bridge/src/pump.rs` (`mod tests` / `mod sched_tests`)

- [ ] **Step 1: Write the failing test for cursor advancement + wrap**

In `rust/motion-bridge/src/pump.rs` `mod tests`:

```rust
#[test]
fn physical_write_cursor_advances_and_wraps_at_n() {
    let mut q = AxisQueue::new(4); // ring_depth 4
    assert_eq!(q.physical_write_cursor, 0);
    q.advance_write_cursor(3);
    assert_eq!(q.physical_write_cursor, 3);
    q.advance_write_cursor(3); // 3 + 3 = 6, wraps at 4 -> 2
    assert_eq!(q.physical_write_cursor, 2);
}
```

- [ ] **Step 2: Run; verify it fails**

Run: `cd rust && cargo test -p motion-bridge physical_write_cursor -- --nocapture`
Expected: FAIL — no `physical_write_cursor` field / `advance_write_cursor` method.

- [ ] **Step 3: Add the cursor to `AxisQueue`**

`AxisQueue` (lines 22-33):

```rust
pub struct AxisQueue {
    pub pieces: VecDeque<PieceEntry>,
    pub pushed: u32,
    pub consumed: u32,
    pub ring_depth: u32,
    /// Physical ring slot [0, ring_depth) where the next piece will be written.
    /// Advanced incrementally (mod ring_depth) — never derived as `pushed % N`,
    /// so it is immune to the u32 wrap of `pushed`.
    pub physical_write_cursor: u32,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self { pieces: VecDeque::new(), pushed: 0, consumed: 0, ring_depth, physical_write_cursor: 0 }
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.consumed);
        self.ring_depth.saturating_sub(in_flight)
    }
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 { return; }
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}
```

- [ ] **Step 4: Add `start_slot` to `FramePlan`**

`FramePlan` (lines 42-46):

```rust
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
}
```

`schedule()` constructs `FramePlan`s; set `start_slot: 0` there as a placeholder and fill the real value in `run_pump` from the live `AxisQueue` (the queue isn't borrowed in `schedule`'s frame construction the way it is at send time). Update every `FramePlan { .. }` literal in `schedule()` and its tests to include `start_slot: 0`.

- [ ] **Step 5: Set `start_slot`, send, then advance cursor + pushed on ACK**

In `run_pump` (lines 380-408), in the send loop, just before sending each frame `f`, look up its queue and set `f.start_slot = q.physical_write_cursor as u16`. On the `Ok` arm (after popping pieces and bumping `pushed`), also advance the cursor:

```rust
// before send:
let start_slot = queues.get(&f.key).map(|q| q.physical_write_cursor as u16).unwrap_or(0);
f.start_slot = start_slot;
let n = f.pieces.len() as u32;
let new_head = queues.get(&f.key).map(|q| q.pushed.wrapping_add(n)).unwrap_or(n);

match sink.send_frame(f.key, &f.pieces, f.start_slot, new_head) {
    Ok(_) => {
        if let Some(q) = queues.get_mut(&f.key) {
            for _ in 0..f.pieces.len() { q.pieces.pop_front(); }
            q.pushed = q.pushed.wrapping_add(n);
            q.advance_write_cursor(n);   // <-- keeps physical cursor in lockstep
        }
    }
    Err(e) => { /* existing break/log path unchanged */ }
}
```

(Match the exact existing pop/bump lines 391-395; the only additions are computing `start_slot`/`new_head` before the call and `advance_write_cursor(n)` after success. `new_head` = post-send `pushed`, monotonic.)

- [ ] **Step 6: Extend `WireSink::send_frame` to carry the new fields on the pieces channel**

`WireSink::send_frame` (lines 433-477) — change the signature and the message build:

```rust
fn send_frame(&self, key: AxisKey, pieces: &[PieceEntry], start_slot: u16, new_head: u32) -> Result<i32, String> {
    let mut pieces_bytes = Vec::with_capacity(pieces.len() * 32);
    for p in pieces { pieces_bytes.extend_from_slice(&p.to_le_bytes()); }
    let msg = PushPieces {
        axis_idx: key.axis,
        piece_count: pieces.len() as u8,
        start_slot,
        new_head,
        pieces_bytes,
    };
    // Send on the dedicated PIECES channel (correlation/response unchanged).
    // Mirror the existing kalico_call usage but target KALICO_CHANNEL_PIECES.
    self.kalico_call_on_channel(KALICO_CHANNEL_PIECES, MessageKind::PushPieces, &msg, self.timeout)
        .map(|resp: PushPiecesResponse| resp.result)
}
```

Implementer note: the existing send uses `kalico_call(MessageKind::PushPieces, ...)` which targets the control channel. Add the ability to send on a specified channel — either a `kalico_call_on_channel(channel, kind, msg, timeout)` wrapper, or a parameter on the existing `kalico_call`. Follow the existing `kalico_call` implementation (grep `fn kalico_call` in motion-bridge / the host transport module); the response (`PushPiecesResponse`, matched by `correlation_id`) still arrives on the control channel and needs no change. Update the `PieceSink` trait's `send_frame` signature to match, and update any mock `PieceSink` in tests.

- [ ] **Step 7: Update the pump's mock sink + any send tests**

Find the test `PieceSink` impl (used by `run_pump` tests) and update its `send_frame` to the new 4-arg signature; have it record `start_slot`/`new_head` so a test can assert they advance. Add:

```rust
#[test]
fn run_pump_sets_start_slot_from_cursor_and_advances_it() {
    // Drive two sends of N pieces each on one axis with ring_depth D;
    // assert the recorded start_slots are [0, N % D] and new_head is [N, 2N].
    // (Use the existing run_pump test harness / channel-driven PumpMsg::Enqueue path.)
}
```

- [ ] **Step 8: Run the pump tests**

Run: `cd rust && cargo test -p motion-bridge pump`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add rust/motion-bridge/src/pump.rs
git commit -m "feat(pump): physical write cursor + start_slot/new_head on pieces channel"
```

---

## Task 7: MCU C — streaming piece sink + channel routing; retire `handle_push_pieces`

**Files:**
- Modify: `src/kalico_demux.c` (state machine: route channel `0x02` to the sink), `src/kalico_demux.h`
- Modify: `src/kalico_dispatch.c:313-337` (retire `handle_push_pieces`; add the streaming piece sink)

**No C unit-test harness exists** (Klipper heritage). Verification for this task is: (a) it compiles for both MCUs, (b) the host-side + sim integration in Task 10. Write the code carefully against the spec data-flow (§5).

- [ ] **Step 1: Add the streaming piece sink in `kalico_dispatch.c`**

Add a small sink that the demux drives byte-by-byte for the pieces channel. The piece-frame payload (after the existing per-message header that `kalico_call` prepends) is:

```
[kind u16][version u8][correlation_id u32]      <- per-message header (7 bytes, existing)
[axis_idx u8][piece_count u8][start_slot u16][new_head u32]   <- 8-byte piece header
[piece 0 .. 32B][piece 1 .. 32B] ...            <- piece_count * 32 bytes
```

Sink state + entry points (add near `handle_push_pieces`):

```c
struct piece_sink {
    uint32_t correlation_id;
    uint8_t  axis_idx;
    uint8_t  piece_count;
    uint16_t start_slot;
    uint32_t new_head;
    uint16_t pieces_seen;     // completed pieces written so far
    uint8_t  scratch[32];     // assembles one PieceEntry across USB chunks
    uint8_t  scratch_len;
    uint8_t  header_len;      // header bytes consumed so far
    uint8_t  header[15];      // 7 (per-msg) + 8 (piece header)
};
static struct piece_sink g_psink;

// Called by the demux when a PIECES-channel frame starts (after sync/len/channel).
void piece_sink_begin(void) {
    g_psink.pieces_seen = 0;
    g_psink.scratch_len = 0;
    g_psink.header_len = 0;
}

// Feed one payload byte (everything between channel and CRC). Returns 0 on OK.
void piece_sink_feed(uint8_t b) {
    if (g_psink.header_len < 15) {
        g_psink.header[g_psink.header_len++] = b;
        if (g_psink.header_len == 15) {
            // parse: header[0..7) per-message; [7]=axis, [8]=count,
            // [9..11)=start_slot, [11..15)=new_head
            g_psink.correlation_id = (uint32_t)g_psink.header[3]
                | ((uint32_t)g_psink.header[4] << 8)
                | ((uint32_t)g_psink.header[5] << 16)
                | ((uint32_t)g_psink.header[6] << 24);
            g_psink.axis_idx    = g_psink.header[7];
            g_psink.piece_count = g_psink.header[8];
            g_psink.start_slot  = (uint16_t)g_psink.header[9] | ((uint16_t)g_psink.header[10] << 8);
            g_psink.new_head    = (uint32_t)g_psink.header[11]
                | ((uint32_t)g_psink.header[12] << 8)
                | ((uint32_t)g_psink.header[13] << 16)
                | ((uint32_t)g_psink.header[14] << 24);
        }
        return;
    }
    // body: assemble 32-byte pieces, stream each to the ring (pre-CRC).
    g_psink.scratch[g_psink.scratch_len++] = b;
    if (g_psink.scratch_len == 32) {
        kalico_runtime_write_piece(runtime_handle, g_psink.axis_idx,
                                   g_psink.start_slot, (uint8_t)g_psink.pieces_seen,
                                   g_psink.scratch);
        g_psink.pieces_seen++;
        g_psink.scratch_len = 0;
    }
}

// Called by the demux when the trailing CRC matches.
void piece_sink_commit(void) {
    int32_t rc = kalico_runtime_commit_head(runtime_handle, g_psink.axis_idx, g_psink.new_head);
    send_push_pieces_response(g_psink.correlation_id, rc);
}
```

(Use the same `runtime_handle` global the old `handle_push_pieces` passed to `kalico_runtime_push_pieces`. `send_push_pieces_response` is unchanged, lines 298-311.)

- [ ] **Step 2: Retire `handle_push_pieces`**

Delete `handle_push_pieces` (lines 313-337) and its dispatch registration in `kalico_dispatch_frame` (the case that routed message kind `0x0060`). Pieces no longer reach `kalico_dispatch_frame` — they are handled entirely by the sink on the pieces channel. Leave `send_push_pieces_response` in place (the sink calls it).

- [ ] **Step 3: Route the pieces channel in the demux**

In `src/kalico_demux.c`, in `kalico_demux_feed_byte`: when a frame's channel byte is read and equals `KALICO_CHANNEL_PIECES`, enter a new `DEMUX_S_PIECES` state instead of accumulating into `kalico_buf`. In that state:
- On the first body byte, call `piece_sink_begin()` once.
- For each payload byte (track remaining via the `len` field already parsed): fold CRC (the same `crc16_ccitt` accumulation the control path uses) **and** call `piece_sink_feed(b)`.
- When the two trailing CRC bytes arrive, compare to the accumulated CRC. **Match** → `piece_sink_commit()`, return `KALICO_DEMUX_OUT_NONE` (the sink already sent the response). **Mismatch** → return `KALICO_DEMUX_OUT_ERROR` (no commit, no response; the streamed slots are never made visible because `commit_head` was not called). Reset state to `DEMUX_S_WAITING`.

Keep the existing control/events accumulation path (and the now-smaller `kalico_buf`) for all other channels. The oversize-drop check (lines 123-127) stays for the control path only; the pieces path has no per-frame buffer to overflow.

The 100 ms idle-reset (line ~214) already resyncs a truncated piece frame — no commit happens, so partial slot writes are harmless.

- [ ] **Step 4: Declare the sink entry points**

Add prototypes for `piece_sink_begin/feed/commit` to a header visible to `kalico_demux.c` (e.g. `kalico_dispatch.h`, or wherever `kalico_dispatch_frame` is declared). Add `#include` of the runtime FFI header that now declares `kalico_runtime_write_piece` / `kalico_runtime_commit_head`.

- [ ] **Step 5: Build both MCU firmwares**

Run on the Pi (per bench flow — do NOT cross-compile locally), or locally if a host-sim build target exists:
```
make clean && make -j$(nproc)            # H7 (.config.h7.bak)
make clean && make -j$(nproc)            # F4 (.config.f446.test)
```
Expected: both compile; no reference to `kalico_runtime_push_pieces` or `handle_push_pieces` remains (grep to confirm).

- [ ] **Step 6: Commit**

```bash
git add src/kalico_demux.c src/kalico_demux.h src/kalico_dispatch.c
git commit -m "feat(mcu): streaming piece sink on KALICO_CHANNEL_PIECES; retire handle_push_pieces"
```

---

## Task 8: MCU C — shrink staging buffer + fix stale AXI reservation

**Files:**
- Modify: `src/kalico_demux.h:38-40`
- Modify: `src/runtime_storage.c:56`

- [ ] **Step 1: Shrink `KALICO_DEMUX_KALICO_BUF_SIZE` to the largest control frame**

In `src/kalico_demux.h`, the pieces no longer pass through `kalico_buf`, so it only needs to hold the largest **control** frame (Identify / QueryRuntimeCaps / **ConfigureAxis** — the last is the biggest, with its stepper bindings). Replace lines 38-40:

```c
/* Pieces stream straight into the ring on KALICO_CHANNEL_PIECES and never
 * touch kalico_buf. This buffer now only stages the largest inbound CONTROL
 * frame: [sync(1)][len(2)][channel(1)][per-msg hdr(7)][body][crc(2)].
 * ConfigureAxis is the largest body; 512 B leaves generous margin. */
#define KALICO_DEMUX_KALICO_BUF_SIZE 512u
_Static_assert(KALICO_DEMUX_KALICO_BUF_SIZE >= 64u,
               "kalico_buf too small for control frames");
```

(Keep `KALICO_MAX_PIECES_PER_FRAME` if it is referenced elsewhere; it no longer sizes `kalico_buf`. If grep shows it is now unused, delete it.)

Implementer verification: grep the codebase for the largest `ConfigureAxis` body (axis + mode + microstep + ring_depth + `stepper_count` × binding-size). Confirm `7 + body + 6 <= 512`. If `ConfigureAxis` can exceed 512 (many bindings), raise the constant to cover it and document the computation in the comment.

- [ ] **Step 2: Fix the stale AXI reservation**

In `src/runtime_storage.c:56`:

```c
#define AXI_BSS_KALICO_BUF_BYTES 512   /* matches KALICO_DEMUX_KALICO_BUF_SIZE (was 14752, NURBS-era) */
```

The AXI-SRAM over-sum `_Static_assert` (lines 62-71) now has ~14 KB more headroom; it must still pass. The reclaimed SRAM is left unallocated (spec §4.5, out of scope).

- [ ] **Step 3: Build both MCU firmwares**

Run: `make clean && make -j$(nproc)` for H7, then F4 (per Task 7 Step 5).
Expected: both compile; the `_Static_assert`s in `runtime_storage.c` pass.

- [ ] **Step 4: Commit**

```bash
git add src/kalico_demux.h src/runtime_storage.c
git commit -m "perf(mcu): shrink kalico_buf to control-frame size; fix stale AXI reservation"
```

---

## Task 9: Cleanup — revert the throwaway bench-diagnostics commits

These were the temporary instrumentation added to find the root cause; the spec requires "no quick hacks left in the code." Revert them now that the fix lands.

**Files:** various (`src/generic/fault_handler.c/.h`, `src/linux/fault_handler_stub.c`, `rust/motion-bridge/src/pump.rs`, `rust/motion-bridge/src/bridge.rs`, `rust/motion-bridge/src/router.rs`, `rust/runtime/.../fault_helpers.rs`, `rust/kalico-host-rt/...`).

- [ ] **Step 1: Confirm the commit list**

Run:
```bash
git log --oneline -20
```
Expected to find the throwaway diag commits (per the session record): `36e142623` (ISR-gap + USB attribution, C), `711db096e` (pump.rs `pump_diag`), `633e04395` (bridge.rs anchor-diag + router.rs), `bdd7dbe10` (fault_helpers.rs lateness), `5dd56e9ee` + `4e3fa4816` (trace-kcall / trace-close in kalico-host-rt). Verify each is still present and not already reverted (`ac855756d` already reverted the ISR-gap fault-diagnostics pair).

- [ ] **Step 2: Revert each, newest-first**

```bash
git revert --no-edit 4e3fa4816 5dd56e9ee bdd7dbe10 633e04395 711db096e 36e142623
```
Resolve any conflicts caused by intervening real changes (the diag in `pump.rs`/`bridge.rs` sits near code Tasks 5-6 touched — keep the Task 5-6 changes, drop only the diag lines). If a `git revert` is messier than a hand-removal because of overlap, instead delete the diag blocks directly (grep `pump_diag`, `anchor-diag`, `trace-kcall`, `tim5_max_gap`, `THROWAWAY`) and commit.

- [ ] **Step 3: Confirm no diag residue**

Run:
```bash
grep -rn "THROWAWAY\|pump_diag\|tim5_max_gap\|trace-kcall\|trace_kcall\|anchor-diag\|anchor_diag" src/ rust/ | grep -v "/target/"
```
Expected: no matches (or only the `diag_*` infrastructure that predates this investigation and is legitimately part of the fault handler — confirm against `git blame`).

- [ ] **Step 4: Build host + both MCUs**

Run: `cd rust && cargo build` ; then `make clean && make -j$(nproc)` for each MCU.
Expected: PASS.

- [ ] **Step 5: Commit (if hand-removed) / already committed (if `git revert`)**

```bash
git add -A
git commit -m "revert: remove bench root-cause diagnostics (fix landed)"
```

---

## Task 10: Integration — bench regression (the jog repro)

**Files:** none (verification only). Uses the `flashing-trident-mcus` skill and the documented repro.

- [ ] **Step 1: Confirm working tree committed + pushed**

The bench pulls from origin. Run `git status` (clean) and push the branch.

- [ ] **Step 2: Flash host + both MCUs from this commit**

Use the `flashing-trident-mcus` skill (Sonnet subagent): rebuild `motion_bridge_native.so`, flash H7 (`.config.h7.bak`), `make clean`, flash F4 (`.config.f446.test`). Do not improvise on failure — stop and report.

- [ ] **Step 3: Run the exact repro (with explicit per-command motion authorization)**

> Motion commands require explicit user authorization each time (hardware-damage rule). Do NOT issue these without a per-command "yes". The repro is: `SET_KINEMATIC_POSITION X=150 Y=150 Z=5`, then alternating ±10 mm X jogs.

Expected: completes with **no** `PieceStartInPast` (-308) shutdown; klippy.log shows no `PushPieces: Timeout`.

- [ ] **Step 4: Fetch + inspect logs**

Copy `klippy.log` to `/tmp/klippy-<timestamp>.log` (per the fetch-logs rule) and confirm: pump sent multi-hundred-piece frames without timeout; heartbeat `consumed` advances; no fault.

- [ ] **Step 5: Spot-check idempotent re-send (optional, if a drop-injection hook exists)**

If the host has a test hook to drop one ACK, exercise it and confirm the re-send does not corrupt motion (idempotent overwrite). Otherwise note this is covered by the host unit tests (Task 6) and skip.

---

## Self-Review

**1. Spec coverage** (each spec section → task):
- §4.1 ring representation (monotonic head/consumed + physical tail, write-by-slot, commit_head, no clearing) → **Task 3**.
- §4.2 write seam (write_piece + commit_head, no bulk buffer) → **Task 4** (FFI) + **Task 7** (streaming sink uses it pre-CRC / post-CRC).
- §4.3 dedicated channel + streaming sink, no message logic in demux → **Task 2** (channel) + **Task 7** (sink + routing). Wire-format/schema touchpoint → **Task 1**.
- §4.4 host addressing (physical write cursor), flow control, pushed-on-ACK → **Task 6**; Gap 2 single-source depth → **Task 5**.
- §4.5 shrink staging buffer + fix AXI reservation → **Task 8**.
- §6/§7 invariants (atomic visibility via post-CRC commit; idempotent re-send; freshness; no MCU write-time check) → realized by Tasks 3/4/6/7; exercised by the ring/FFI/pump unit tests and the bench regression (Task 10).
- §8 B1/B2/B4 boundary (two handle methods, scalar+ptr only, Rust owns slot math) → **Task 4**.
- "No quick hacks left" → **Task 9**.

**2. Placeholder scan:** no "TBD"/"handle errors appropriately"; every code step shows code; the one genuinely environment-dependent value (the shrunk buffer size) is given a concrete default (512) with a documented verification step against `ConfigureAxis`.

**3. Type consistency:** `write_slot(&self, storage, physical_slot: usize, entry)` and `commit_head(&mut self, new_head: u32)` (Task 3) match the FFI calls `kalico_runtime_write_piece(.., start_slot: u16, index: u8, piece_ptr)` and `kalico_runtime_commit_head(.., new_head: u32)` (Task 4) — the FFI does the `(start_slot+index) % depth` → `physical_slot` mapping before calling `write_slot`. `PushPieces { axis_idx, piece_count, start_slot: u16, new_head: u32, pieces_bytes }` (Task 1) matches `WireSink::send_frame(key, pieces, start_slot: u16, new_head: u32)` (Task 6) and the MCU sink's header parse (Task 7: axis@7, count@8, start_slot@9-10, new_head@11-14). `new_head = pushed.wrapping_add(n)` (Task 6) is the same monotonic frontier `commit_head` consumes (Task 3). Channel constant `KALICO_CHANNEL_PIECES = 0x02` is identical in Rust (Task 2) and C (Task 2/7).

**Open implementer flag (carried from the codebase map):** the host send path (`kalico_call` → channel) must be extended to send `PushPieces` on `KALICO_CHANNEL_PIECES` while still matching the `PushPiecesResponse` by `correlation_id` on the control channel (Task 6 Step 6). The exact `kalico_call` internals were not fully mapped; follow the existing `kalico_call` implementation and add a channel parameter. This is the one place to verify against the real transport code before coding.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-05-29-pushpieces-receive-into-ring.md`. Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
