# Push-Pieces Wiring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Rust work MUST be done by the `rust-engineer` subagent.

**Goal:** Make the host stream `PushPieces` frames to each MCU so motion actually plays, replacing the dead segment/curve/handle dispatch path.

**Architecture:** A per-segment **enqueue adapter** (runs in the planner's dispatch closure) flattens each `ShapedSegment` into absolute-timed `PieceEntry`s per `(mcu, axis)` and hands them to a standalone **pump** thread over a channel. The pump merges all axes by `start_time`, pushes in strict time order with per-ring flow control (stall on a full head), and is fed `consumed_counts` by a routed `StatusHeartbeat`. A single shared host-time anchor `T0` (projected per-MCU via the existing clock-sync) keeps MCUs in sync; it re-anchors only when the planner timeline jumps backward.

**Tech Stack:** Rust (workspace crates `runtime`, `kalico-protocol`, `kalico-host-rt`, `motion-bridge`, `nurbs`, `trajectory`), C (MCU firmware `src/`), Python (klippy).

**Spec:** `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md`

---

## Orientation (read before starting)

Key code facts (verified 2026-05-28):

- **`PieceEntry`** — `rust/runtime/src/piece_ring.rs:305`, `#[repr(C, align(8))]`, 32 bytes: `start_time: u64`, `coeffs: [f32;4]` (Bernstein), `duration: f32`, `_reserved: u32`. No wire serializer yet (Task 1 adds it).
- **`PushPieces`** — `rust/kalico-protocol/src/messages.rs:181`. Body: `axis_idx: u8`, `piece_count: u8`, `pieces_bytes: Vec<u8>` (`piece_count × 32`). `PushPiecesResponse { result: i32 }` (0 = OK, negative = error).
- **Wire send** — `KalicoHostIo::kalico_call(MessageKind, body: Vec<u8>, timeout) -> Result<(MessageKind, Vec<u8>), TransportError>` (`rust/kalico-host-rt/src/host_io/mod.rs:755`). Blocking round-trip.
- **`StatusHeartbeat`** — `messages.rs:288`: `engine_state: u8`, `fault_code: u8`, `consumed_counts: Vec<u32>`. Wire tag `0x0083`. Currently **dropped** by `lift_event_to_runtime_event` (`rust/kalico-host-rt/src/host_io/kalico_native.rs`, the `_ =>` arm).
- **Clock projection** — `PassthroughRouter::host_time_to_mcu_clock(mcu, host_secs) -> Result<u64,_>` (`rust/kalico-host-rt/src/passthrough_queue/router.rs:437`) projects an arbitrary host time to that MCU's clock via the live clock-sync estimate. `router.clock.now()` is the shared host clock.
- **Curve → pieces** — `nurbs::bezier::extract_bezier_pieces(&ScalarNurbs<f64>) -> Vec<BezierPiece<f64>>` (`rust/nurbs/src/bezier.rs:493`). `BezierPiece { u_start, u_end, .. }` with `.to_bernstein() -> Vec<T>` (`bezier.rs:9,69`).
- **CoreXY transform** — `nurbs::algebra::add_with_knot_union(&a, &b)` and `scalar_multiply(&c, -1.0)` (`rust/nurbs/src/algebra.rs:101,9`).
- **`ShapedSegment`** — `rust/trajectory/src/lib.rs:150`: `axes: [ScalarNurbs<f64>; 3]` (X/Y/Z), `e_mode`, `extrusion_per_xy_mm`, `e_independent: Option<ScalarNurbs<f64>>`, `t_start: f64`, `t_end: f64`.
- **Dispatch closure** — `init_planner` (`rust/motion-bridge/src/bridge.rs` ~1880–2423) builds `dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>` and hands it to `PlannerHandle::spawn(cfg, dispatch)`. The closure body (~2065–2413) is the segment-era code we replace; the TODO at `bridge.rs:2397` is the missing send.
- **`McuAxisConfig`** — `rust/motion-bridge/src/dispatch.rs:43`: `mcu_id: u32`, `axes: Vec<usize>`, `kinematics: u8`, `caps: McuCaps`. `KINEMATICS_COREXY = 0`.
- **Per-axis config (live path)** — Python `motion_toolhead.py:1143` sends `kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u extrusion_per_xy_mm=%u stepper_count=%c steppers=%*s`; C handler `command_kalico_configure_axis` (`src/stepper.c:222`) **hardcodes `ring_depth = 64`** at line 282 (`TODO: wire from host`). FFI `kalico_runtime_configure_axis(..., ring_depth: u16, ...)` already accepts it (`rust/kalico-c-api/src/runtime_ffi.rs:1353`).
- **Caps** — `McuCaps { total_piece_memory: u32 }` (`dispatch.rs:62`); `total_pieces() = total_piece_memory/32` (`dispatch.rs:89`). Bridge holds `runtime_caps: Option<RuntimeCapsResponse>` per MCU (`bridge.rs:96`), `FALLBACK_RUNTIME_CAPS` = 62 KB.

**Build/test commands** (run from repo root unless noted):
- Rust unit/integration tests: `cargo test -p <crate>` (e.g. `cargo test -p motion-bridge`).
- Host feature where a crate is dual-target: `cargo test -p runtime --features host`.
- MCU firmware: built on the Pi per `feedback_bench_firmware_flow` — commit → push → pull → `make` on the Pi. Do **not** cross-compile locally.

**Sequencing:** Build the new path (Tasks 1–9) and verify motion at the existing `ring_depth=64`, then wire real ring sizing (Task 9 step set), then remove the dead segment-era path (Task 10). The dead path is inert, so the build stays green throughout — never delete-then-rebuild.

---

## Phase A — Foundations

### Task 1: `PieceEntry` wire serializer + shared result-code constants

**Files:**
- Modify: `rust/runtime/src/piece_ring.rs` (add `to_le_bytes` to `PieceEntry`)
- Modify: `rust/kalico-protocol/src/lib.rs` (add `result_codes` module)
- Test: inline `#[cfg(test)]` in both

- [ ] **Step 1: Write the failing test for `PieceEntry::to_le_bytes`**

Add to `rust/runtime/src/piece_ring.rs` inside (or appended to) its test module:

```rust
#[test]
fn piece_entry_to_le_bytes_matches_field_layout() {
    let p = PieceEntry {
        start_time: 0x0102_0304_0506_0708,
        coeffs: [1.0, 2.0, 3.0, 4.0],
        duration: 0.5,
        _reserved: 0,
    };
    let b = p.to_le_bytes();
    assert_eq!(b.len(), 32);
    assert_eq!(&b[0..8], &0x0102_0304_0506_0708u64.to_le_bytes());
    assert_eq!(&b[8..12], &1.0f32.to_le_bytes());
    assert_eq!(&b[12..16], &2.0f32.to_le_bytes());
    assert_eq!(&b[16..20], &3.0f32.to_le_bytes());
    assert_eq!(&b[20..24], &4.0f32.to_le_bytes());
    assert_eq!(&b[24..28], &0.5f32.to_le_bytes());
    assert_eq!(&b[28..32], &0u32.to_le_bytes());
}
```

- [ ] **Step 2: Run it — expect FAIL (no method `to_le_bytes`)**

Run: `cargo test -p runtime --features host piece_entry_to_le_bytes`
Expected: compile error, `no method named to_le_bytes`.

- [ ] **Step 3: Implement `to_le_bytes`**

Add inside `impl PieceEntry` in `rust/runtime/src/piece_ring.rs`:

```rust
/// Serialize to the 32-byte little-endian wire form. Field order matches
/// the `#[repr(C, align(8))]` layout, so on a little-endian host these
/// bytes are byte-identical to the C struct the MCU reads.
#[inline]
pub fn to_le_bytes(&self) -> [u8; 32] {
    let mut b = [0u8; 32];
    b[0..8].copy_from_slice(&self.start_time.to_le_bytes());
    b[8..12].copy_from_slice(&self.coeffs[0].to_le_bytes());
    b[12..16].copy_from_slice(&self.coeffs[1].to_le_bytes());
    b[16..20].copy_from_slice(&self.coeffs[2].to_le_bytes());
    b[20..24].copy_from_slice(&self.coeffs[3].to_le_bytes());
    b[24..28].copy_from_slice(&self.duration.to_le_bytes());
    b[28..32].copy_from_slice(&self._reserved.to_le_bytes());
    b
}
```

- [ ] **Step 4: Run it — expect PASS**

Run: `cargo test -p runtime --features host piece_entry_to_le_bytes`
Expected: PASS.

- [ ] **Step 5: Add shared result-code constants in `kalico-protocol`**

Add to `rust/kalico-protocol/src/lib.rs`:

```rust
/// `PushPiecesResponse.result` codes, shared host ↔ MCU. These mirror the
/// `KALICO_ERR_*` values the C side returns from `handle_push_pieces`
/// (`src/kalico_dispatch.c`). Keep the two in sync — a renumber on one side
/// without the other silently desyncs the wire contract.
pub mod result_codes {
    pub const OK: i32 = 0;
    pub const RING_FULL: i32 = -3;       // KALICO_ERR_RING_FULL
    pub const INVALID_AXIS: i32 = -2;    // KALICO_ERR_INVALID_ARG (axis path)
}
```

Then verify the actual C values before committing: `grep -n "KALICO_ERR_RING_FULL\|KALICO_ERR_INVALID_ARG\|KALICO_OK" src/*.h rust/kalico-c-api/include/*.h`. Set the constants to the real numeric values found; adjust the comments if the names differ.

- [ ] **Step 6: Pin the constants with a test**

Add to `rust/kalico-protocol/src/lib.rs` (or its test module):

```rust
#[test]
fn result_codes_are_stable() {
    assert_eq!(result_codes::OK, 0);
    assert!(result_codes::RING_FULL < 0);
    assert!(result_codes::INVALID_AXIS < 0);
    assert_ne!(result_codes::RING_FULL, result_codes::INVALID_AXIS);
}
```

- [ ] **Step 7: Run + commit**

Run: `cargo test -p runtime --features host && cargo test -p kalico-protocol`
Expected: PASS.

```bash
git add rust/runtime/src/piece_ring.rs rust/kalico-protocol/src/lib.rs
git commit -m "feat: PieceEntry::to_le_bytes wire serializer + shared PushPieces result codes"
```

---

### Task 2: Route `StatusHeartbeat` to a host-side callback

The heartbeat is decoded but discarded. Add a `RuntimeEvent::Heartbeat` variant, lift it in the kalico-native path, and expose `attach_heartbeat_callback` on `KalicoHostIo` mirroring the existing `attach_credit_counter` / `attach_retirement_callback` pattern, so the bridge can route `consumed_counts` into the pump.

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/runtime_events.rs` (add `Heartbeat` variant)
- Modify: `rust/kalico-host-rt/src/host_io/kalico_native.rs` (`lift_event_to_runtime_event`)
- Modify: `rust/kalico-host-rt/src/host_io/events.rs` (dispatch + callback slot)
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs` (`attach_heartbeat_callback`)
- Test: `rust/kalico-host-rt/src/host_io/events/dispatch_tests.rs`

- [ ] **Step 1: Read the existing credit-callback wiring as the template**

Run: `grep -n "attach_credit_counter\|credit_counter\|AttachCreditCounter\|on_credit_freed" rust/kalico-host-rt/src/host_io/mod.rs rust/kalico-host-rt/src/host_io/events.rs rust/kalico-host-rt/src/host_io/reactor.rs`
This is the pattern to mirror exactly (a `ReactorCommand::Attach…` + an `Option<callback>` slot on `EventDispatcher`).

- [ ] **Step 2: Add the `Heartbeat` variant**

In `rust/kalico-host-rt/src/host_io/runtime_events.rs`, add to `enum RuntimeEvent`:

```rust
    /// Per-axis consumed-piece counts from `StatusHeartbeat` (0x0083),
    /// used by the host pump for flow control.
    Heartbeat {
        consumed_counts: Vec<u32>,
    },
```

- [ ] **Step 3: Write the failing test for heartbeat lift**

In `rust/kalico-host-rt/src/host_io/kalico_native.rs` test module (or `events/dispatch_tests.rs`), add a test that builds a `StatusHeartbeat` body and asserts `lift_event_to_runtime_event(MessageKind::StatusHeartbeat, &body)` yields the consumed counts rather than `Ignored`:

```rust
#[test]
fn status_heartbeat_lifts_to_runtime_event() {
    use kalico_protocol::messages::StatusHeartbeat;
    use kalico_protocol::codec::Encode; // trait lives in codec, not messages
    let hb = StatusHeartbeat { engine_state: 1, fault_code: 0, consumed_counts: vec![7, 0, 3] };
    let mut body = Vec::new();
    hb.encode(&mut body);
    let mut st = KalicoNativeState::default(); // or the test ctor used elsewhere in this file
    match lift_event_to_runtime_event(&mut st, MessageKind::StatusHeartbeat, &body) {
        KalicoDispatchResult::Event(RuntimeEvent::Heartbeat { consumed_counts }) => {
            assert_eq!(consumed_counts, vec![7, 0, 3]);
        }
        other => panic!("expected Heartbeat event, got {other:?}"),
    }
}
```

(Use whatever `KalicoNativeState` constructor the other tests in this file use; if none, pass the value the function actually needs — `lift_event_to_runtime_event` ignores state for events.)

- [ ] **Step 4: Run it — expect FAIL**

Run: `cargo test -p kalico-host-rt status_heartbeat_lifts`
Expected: FAIL — currently returns `Ignored` via the `_ =>` arm.

- [ ] **Step 5: Add the lift arm**

In `lift_event_to_runtime_event` (`kalico_native.rs`), add before the `_ =>` arm:

```rust
        MessageKind::StatusHeartbeat => match KStatusHeartbeat::decode(body) {
            Ok(hb) => KalicoDispatchResult::Event(RuntimeEvent::Heartbeat {
                consumed_counts: hb.consumed_counts,
            }),
            Err(e) => {
                log::warn!("kalico StatusHeartbeat decode failed: {e:?}");
                KalicoDispatchResult::Ignored
            }
        },
```

Add the import alias at the top of the file (matching how `KFaultEvent` is aliased): `use kalico_protocol::messages::StatusHeartbeat as KStatusHeartbeat;` and ensure `Decode` is in scope. Also fix the stale doc comment at `kalico_native.rs:14` to stop claiming the heartbeat is already lifted — it now is.

- [ ] **Step 6: Run it — expect PASS**

Run: `cargo test -p kalico-host-rt status_heartbeat_lifts`
Expected: PASS.

- [ ] **Step 7: Add `attach_heartbeat_callback` mirroring the credit path**

Following the credit pattern found in Step 1, add to `EventDispatcher` an `Option<Arc<dyn Fn(&[u32]) + Send + Sync>>` slot (`heartbeat_callback`), invoke it in the dispatch match arm for `RuntimeEvent::Heartbeat`, add a `ReactorCommand::AttachHeartbeatCallback(Arc<…>)` and its reactor handler (mirror `AttachCreditCounter` at `reactor.rs:1113`), and add the public method on `KalicoHostIo`.

The `RuntimeEvent::Heartbeat` arm fires the callback and is **pump-private** — like `CreditFreed`, it is consumed by the dispatcher and not forwarded to the general `runtime_rx` channel that other runtime-event consumers read (spec §3.4: heartbeats "needn't surface to general runtime-event consumers"). Match the credit arm's forward/consume behavior exactly.

```rust
/// Register a callback fired on every StatusHeartbeat with the per-axis
/// consumed-piece counts. Runs on the reactor/event thread — must be
/// non-blocking (it only sends on a channel; see motion-bridge pump).
pub fn attach_heartbeat_callback(&self, cb: std::sync::Arc<dyn Fn(&[u32]) + Send + Sync>) {
    let _ = self.submission_tx.send(ReactorCommand::AttachHeartbeatCallback(cb));
}
```

- [ ] **Step 8: Write a test that the callback fires**

In `events/dispatch_tests.rs`, mirror an existing credit-callback dispatch test: install a heartbeat callback that records into an `Arc<Mutex<Vec<Vec<u32>>>>`, dispatch a `RuntimeEvent::Heartbeat { consumed_counts: vec![5,1] }`, assert the recorder sees `[5,1]`.

- [ ] **Step 9: Run + commit**

Run: `cargo test -p kalico-host-rt`
Expected: PASS.

```bash
git add rust/kalico-host-rt/src/host_io/
git commit -m "feat: route StatusHeartbeat to RuntimeEvent::Heartbeat + attach_heartbeat_callback"
```

---

## Phase B — Pump core (`rust/motion-bridge/src/pump.rs`)

The pump is pure logic + a thin thread. Tasks 3–4 are pure and fully unit-tested with no I/O; Task 5 adds the thread and the real wire sink.

### Task 3: `AxisQueue` + wrapping room accounting

**Files:**
- Create: `rust/motion-bridge/src/pump.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (`pub mod pump;`)
- Test: inline `#[cfg(test)]` in `pump.rs`

- [ ] **Step 1: Write the failing tests**

Create `rust/motion-bridge/src/pump.rs`:

```rust
//! Host-side piece pump: merges per-(mcu,axis) piece queues by absolute
//! start_time and streams PushPieces frames in strict time order with
//! per-ring flow control. See
//! `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md`.

use std::collections::VecDeque;
use runtime::piece_ring::PieceEntry;

/// Destination ring identity.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

/// One axis's outbound queue plus flow-control accounting. `pushed` and
/// `consumed` are wrapping u32 mirrors of the MCU's monotonic ring counter
/// (spec §3.3) — never reset on a time re-anchor, only on an MCU ring reset.
pub struct AxisQueue {
    pub pieces: VecDeque<PieceEntry>,
    pub pushed: u32,
    pub consumed: u32,
    pub ring_depth: u32,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self { pieces: VecDeque::new(), pushed: 0, consumed: 0, ring_depth }
    }
    /// Free ring slots = depth − in-flight, where in-flight = pushed − consumed
    /// (wrapping). Saturates at 0.
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.consumed);
        self.ring_depth.saturating_sub(in_flight)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn piece(start: u64) -> PieceEntry {
        PieceEntry { start_time: start, coeffs: [0.0; 4], duration: 0.001, _reserved: 0 }
    }

    #[test]
    fn room_full_then_drains() {
        let mut q = AxisQueue::new(4);
        assert_eq!(q.room(), 4);
        q.pushed = 4;
        assert_eq!(q.room(), 0);          // full
        q.consumed = 1;
        assert_eq!(q.room(), 1);          // one freed
    }

    #[test]
    fn room_correct_across_u32_wrap() {
        let mut q = AxisQueue::new(8);
        q.pushed = 2;                      // wrapped past u32::MAX
        q.consumed = u32::MAX;             // consumed is "behind" pushed by 3
        // in_flight = 2 - (u32::MAX) wrapping = 3
        assert_eq!(q.room(), 5);
    }
}
```

- [ ] **Step 2: Register the module**

In `rust/motion-bridge/src/lib.rs`, add `pub mod pump;` (alphabetically near `pub mod planner;`).

- [ ] **Step 3: Run — expect PASS** (pure code + tests written together)

Run: `cargo test -p motion-bridge pump::tests`
Expected: PASS (both tests).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/pump.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(pump): AxisQueue + wrapping-u32 room accounting"
```

---

### Task 4: Scheduler — earliest head, stall-on-full, batch plan

The scheduler is a pure function over the queue map that decides what to send next. It returns a **plan** (which frames to emit), so it's testable without any I/O. The run-loop (Task 5) executes plans.

**Files:**
- Modify: `rust/motion-bridge/src/pump.rs`
- Test: inline

- [ ] **Step 1: Write the failing tests**

Add to `pump.rs`:

```rust
use std::collections::BTreeMap;

/// A planned outbound frame: one axis's contiguous run of pieces.
#[derive(Debug, PartialEq)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
}

/// Outcome of one scheduling decision.
#[derive(Debug, PartialEq)]
pub enum Schedule {
    /// Send these frames (all on one MCU, a contiguous prefix of global order).
    Send(Vec<FramePlan>),
    /// Global head's ring is full — wait for a heartbeat. Do not send anything.
    StallFull(AxisKey),
    /// No pieces queued anywhere.
    Idle,
}

/// Decide the next action over the queue map. Does **not** mutate the queues;
/// the caller applies the returned plan (pops pieces, bumps `pushed`).
/// `max_per_frame` caps a single PushPieces frame's piece_count (u8 wire field).
pub fn schedule(queues: &BTreeMap<AxisKey, AxisQueue>, max_per_frame: usize) -> Schedule {
    // earliest non-empty queue head, tie-broken by (mcu_id, axis)
    let head = queues
        .iter()
        .filter(|(_, q)| !q.pieces.is_empty())
        .min_by(|(ka, qa), (kb, qb)| {
            qa.pieces.front().unwrap().start_time
                .cmp(&qb.pieces.front().unwrap().start_time)
                .then(ka.cmp(kb))
        });
    let (&head_key, head_q) = match head {
        None => return Schedule::Idle,
        Some(h) => h,
    };
    if head_q.room() == 0 {
        return Schedule::StallFull(head_key);
    }

    // Greedily take the contiguous prefix of global time order that stays on
    // head_key.mcu_id and has room. Simulate room locally so a single
    // scheduling pass never plans more than each ring can hold.
    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new(); // key -> count planned
    loop {
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                let already = taken.get(k).copied().unwrap_or(0);
                q.pieces.get(already).map(|p| (k, already, p.start_time))
            })
            .min_by(|(ka, _, sa), (kb, _, sb)| sa.cmp(sb).then(ka.cmp(kb)));
        let (&k, _idx, _start) = match next {
            Some(n) => n,
            None => break,
        };
        if k.mcu_id != head_key.mcu_id {
            break; // next-earliest is a different MCU — stop the batch
        }
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        if already >= room || already >= max_per_frame {
            break; // this ring is locally full (or frame cap hit) for this pass
        }
        *taken.entry(k).or_insert(0) += 1;
    }

    let frames: Vec<FramePlan> = taken
        .into_iter()
        .filter(|(_, n)| *n > 0)
        .map(|(k, n)| FramePlan {
            key: k,
            pieces: queues[&k].pieces.iter().take(n).copied().collect(),
        })
        .collect();
    Schedule::Send(frames)
}

#[cfg(test)]
mod sched_tests {
    use super::*;

    fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
        let mut q = AxisQueue::new(ring_depth);
        for &s in starts {
            q.pieces.push_back(PieceEntry { start_time: s, coeffs: [0.0;4], duration: 0.001, _reserved: 0 });
        }
        q
    }

    #[test]
    fn idle_when_empty() {
        let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        assert_eq!(schedule(&queues, 255), Schedule::Idle);
    }

    #[test]
    fn stalls_when_global_head_ring_full() {
        let mut queues = BTreeMap::new();
        // mcuA/x earliest but full; mcuB/x later but has room → must STALL, not skip.
        let mut a = q_with(2, &[10]);
        a.pushed = 2; // full
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
        assert_eq!(schedule(&queues, 255), Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 }));
    }

    #[test]
    fn batches_contiguous_same_mcu_prefix_only() {
        let mut queues = BTreeMap::new();
        // global order: A/x@0, A/y@1, B/x@2, A/x@3
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
        queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
        let s = schedule(&queues, 255);
        // batch stops at B/x@2 → A/x gets [0] only (A/x@3 is after the B boundary),
        // A/y gets [1]. B/x not included.
        match s {
            Schedule::Send(frames) => {
                let ax: Vec<_> = frames.iter().map(|f| (f.key, f.pieces.len())).collect();
                assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 0 }, 1)));
                assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 1 }, 1)));
                assert!(!ax.iter().any(|(k, _)| k.mcu_id == 2));
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn frame_cap_splits() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
        let s = schedule(&queues, 2);
        match s {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].pieces.len(), 2); // capped at 2 this pass
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }
}
```

- [ ] **Step 2: Run — expect PASS** (impl + tests written together; the impl above is complete)

Run: `cargo test -p motion-bridge pump::sched_tests`
Expected: PASS (all four).

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/pump.rs
git commit -m "feat(pump): strict-time-order scheduler with stall-on-full-head + batching"
```

---

### Task 5: Pump run-loop — channels, `PieceSink`, real wire send

The run-loop owns the queue map (no shared lock), selects over a pieces-in channel and a heartbeat-in channel, and on each wake applies `schedule()` and sends via a `PieceSink`. The real sink calls `kalico_call`; tests use a recording sink.

**Files:**
- Modify: `rust/motion-bridge/src/pump.rs`
- Test: inline + `rust/motion-bridge/tests/pump_loop.rs`

- [ ] **Step 1: Define the inbound messages and the sink trait**

Add to `pump.rs`:

```rust
/// Pieces handed to the pump for one (mcu, axis), in time order.
pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    /// Set when this batch begins a fresh stream (timeline re-anchor): the
    /// pump leaves flow-control counters alone (spec §3.3) — this flag exists
    /// only so future logic can react; for now it is informational.
    pub fresh_stream: bool,
}

/// Per-MCU heartbeat: consumed counts indexed by axis.
pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub consumed_counts: Vec<u32>,
}

/// Inbound to the pump loop.
pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Shutdown,
}

/// Sends one axis's frame to the wire. Returns the MCU's result code
/// (`result_codes::OK` on success).
pub trait PieceSink: Send {
    fn send_frame(&self, key: AxisKey, pieces: &[PieceEntry]) -> Result<i32, String>;
}
```

- [ ] **Step 2: Write the failing run-loop test with a recording sink**

Create `rust/motion-bridge/tests/pump_loop.rs`:

```rust
use std::sync::{Arc, Mutex};
use std::sync::mpsc;
use motion_bridge::pump::{AxisKey, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, run_pump};
use runtime::piece_ring::PieceEntry;

struct RecordingSink(Arc<Mutex<Vec<(AxisKey, usize)>>>);
impl PieceSink for RecordingSink {
    fn send_frame(&self, key: AxisKey, pieces: &[PieceEntry]) -> Result<i32, String> {
        self.0.lock().unwrap().push((key, pieces.len()));
        Ok(0)
    }
}

fn p(start: u64) -> PieceEntry {
    PieceEntry { start_time: start, coeffs: [0.0;4], duration: 0.001, _reserved: 0 }
}

#[test]
fn pump_sends_in_time_order_and_stops_when_full() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    // ring_depth lookup: every (mcu,axis) gets depth 2.
    let depth = |_k: AxisKey| 2u32;
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || run_pump(rx, sink, depth));

    // Two pieces to mcu1/axis0; depth 2 → both fit, no heartbeat needed yet.
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 }, pieces: vec![p(0), p(1)], fresh_stream: true,
    })).unwrap();
    // A third piece — should NOT send until a heartbeat frees room.
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 }, pieces: vec![p(2)], fresh_stream: false,
    })).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(rec.lock().unwrap().len(), 1, "first frame (2 pieces) sent, third stalled");
    assert_eq!(rec.lock().unwrap()[0], (AxisKey { mcu_id: 1, axis: 0 }, 2));

    // Heartbeat: consumed 2 → room frees → third piece goes out.
    tx.send(PumpMsg::Heartbeat(HeartbeatMsg { mcu_id: 1, consumed_counts: vec![2] })).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(rec.lock().unwrap().len(), 2);
    assert_eq!(rec.lock().unwrap()[1], (AxisKey { mcu_id: 1, axis: 0 }, 1));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
```

- [ ] **Step 3: Run — expect FAIL (no `run_pump`)**

Run: `cargo test -p motion-bridge --test pump_loop`
Expected: compile error, `run_pump` not found.

- [ ] **Step 4: Implement `run_pump`**

Add to `pump.rs`. The loop: drain all currently-available messages (applying enqueues and heartbeat updates to the owned queue map), then repeatedly `schedule()` + send + apply until `Idle` or `StallFull`, then block on the next message.

```rust
use std::sync::mpsc::{Receiver, RecvError};

/// Run the pump until `Shutdown`. `ring_depth_of` supplies each ring's depth
/// the first time its key is seen. `sink` performs the actual wire send.
pub fn run_pump<S, F>(rx: Receiver<PumpMsg>, sink: S, ring_depth_of: F)
where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
{
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    const MAX_PER_FRAME: usize = 255; // u8 wire piece_count

    let mut apply = |msg: PumpMsg, queues: &mut BTreeMap<AxisKey, AxisQueue>| -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Enqueue(EnqueueMsg { key, pieces, fresh_stream: _ }) => {
                let q = queues.entry(key).or_insert_with(|| AxisQueue::new(ring_depth_of(key)));
                q.pieces.extend(pieces);
            }
            PumpMsg::Heartbeat(HeartbeatMsg { mcu_id, consumed_counts }) => {
                for (axis, &c) in consumed_counts.iter().enumerate() {
                    let key = AxisKey { mcu_id, axis: axis as u8 };
                    if let Some(q) = queues.get_mut(&key) {
                        q.consumed = c;
                    }
                }
            }
        }
        true
    };

    // Block for the first message, then process bursts.
    loop {
        let first = match rx.recv() {
            Ok(m) => m,
            Err(RecvError) => return,
        };
        if !apply(first, &mut queues) {
            return;
        }
        // Drain anything else already queued (coalesce bursts before sending).
        while let Ok(m) = rx.try_recv() {
            if !apply(m, &mut queues) {
                return;
            }
        }
        // Send as far as the schedule allows. A send failure breaks all the
        // way back to `recv()` (labeled break) instead of re-running schedule
        // on the still-queued pieces — otherwise a persistent failure spins a
        // tight busy-loop. The next inbound message (heartbeat/enqueue) retries.
        'send: loop {
            match schedule(&queues, MAX_PER_FRAME) {
                Schedule::Idle | Schedule::StallFull(_) => break 'send,
                Schedule::Send(frames) => {
                    if frames.is_empty() {
                        break 'send;
                    }
                    for f in frames {
                        match sink.send_frame(f.key, &f.pieces) {
                            Ok(_) => {
                                let q = queues.get_mut(&f.key).expect("planned key exists");
                                for _ in 0..f.pieces.len() {
                                    q.pieces.pop_front();
                                }
                                q.pushed = q.pushed.wrapping_add(f.pieces.len() as u32);
                            }
                            Err(e) => {
                                log::error!("pump send_frame failed for {:?}: {e}", f.key);
                                // Leave the pieces queued; retry on next message.
                                break 'send;
                            }
                        }
                    }
                }
            }
        }
    }
}
```

- [ ] **Step 5: Run — expect PASS**

Run: `cargo test -p motion-bridge --test pump_loop`
Expected: PASS.

- [ ] **Step 6: Implement the real `KalicoHostIo` sink**

Add to `pump.rs` (gated so tests don't need a live IO):

```rust
use std::sync::Weak;
use std::collections::HashMap;
use std::time::Duration;
use kalico_host_rt::host_io::KalicoHostIo;
use kalico_protocol::messages::{PushPieces, MessageKind};
use kalico_protocol::codec::{Encode, Decode};

/// Production sink: one `kalico_call(PushPieces)` per frame.
///
/// Holds `Weak<KalicoHostIo>` — NOT `Arc` — mirroring the dispatch_ios design
/// at `bridge.rs:1986`: `detach_serial` drops the strong `Arc` to tear down
/// the reactor, so the pump must not pin the IO alive. An upgrade failure
/// means the MCU was detached; the frame is dropped with an error (the pump
/// logs and leaves the pieces queued — a detached MCU is a stream teardown).
pub struct WireSink {
    pub ios: HashMap<u32, Weak<KalicoHostIo>>,
    pub timeout: Duration,
}

impl PieceSink for WireSink {
    fn send_frame(&self, key: AxisKey, pieces: &[PieceEntry]) -> Result<i32, String> {
        let io = self.ios.get(&key.mcu_id)
            .and_then(Weak::upgrade)
            .ok_or_else(|| format!("KalicoHostIo for mcu {} detached", key.mcu_id))?;
        let mut pieces_bytes = Vec::with_capacity(pieces.len() * 32);
        for p in pieces {
            pieces_bytes.extend_from_slice(&p.to_le_bytes());
        }
        let msg = PushPieces {
            axis_idx: key.axis,
            piece_count: pieces.len() as u8,
            pieces_bytes,
        };
        let mut body = Vec::new();
        msg.encode(&mut body);
        let (_kind, resp) = io.kalico_call(MessageKind::PushPieces, body, self.timeout)
            .map_err(|e| format!("kalico_call PushPieces: {e:?}"))?;
        let r = kalico_protocol::messages::PushPiecesResponse::decode(&resp)
            .map_err(|e| format!("decode PushPiecesResponse: {e:?}"))?;
        if r.result != kalico_protocol::result_codes::OK {
            return Err(format!("MCU rejected PushPieces (mcu {} axis {}): {}", key.mcu_id, key.axis, r.result));
        }
        Ok(r.result)
    }
}
```

Import the codec traits so the methods resolve: `use kalico_protocol::codec::{Encode, Decode};`. `Decode::decode(&[u8])` is the convenience wrapper over `decode_from(&mut Cursor)` (confirmed `codec.rs:65-68`), so `PushPiecesResponse::decode(&resp)` and `msg.encode(&mut body)` are both correct as written.

- [ ] **Step 7: Run + commit**

Run: `cargo test -p motion-bridge`
Expected: PASS.

```bash
git add rust/motion-bridge/src/pump.rs rust/motion-bridge/tests/pump_loop.rs
git commit -m "feat(pump): channel-fed run-loop + WireSink (PushPieces via kalico_call)"
```

---

## Phase C — Anchor + enqueue adapter

### Task 6: Stream anchor — shared `T0`, contiguous vs reset

**Files:**
- Create: `rust/motion-bridge/src/anchor.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (`pub mod anchor;`)
- Test: inline

- [ ] **Step 1: Write the failing tests**

Create `rust/motion-bridge/src/anchor.rs`:

```rust
//! Shared host-time anchor mapping planner time → host time. One `T0` per
//! stream, re-established only when the planner timeline jumps backward
//! (a reset). See spec §3.2.1.

const CONTIGUITY_EPS: f64 = 1e-6; // seconds; planner timestamps compare to each other
const DEFAULT_LEAD_SECS: f64 = 0.25;

pub struct Anchor {
    /// Host-time instant (seconds) that planner t = 0 maps to. `None` until
    /// the first segment establishes it.
    t0: Option<f64>,
    /// Previous segment's planner t_end (seconds).
    last_t_end: f64,
    lead_secs: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self { t0: None, last_t_end: 0.0, lead_secs: DEFAULT_LEAD_SECS }
    }

    /// Map a segment to host time. `host_now` is the shared host clock now
    /// (seconds). Returns `T0` such that piece host time = `T0 + u_start`.
    /// Re-anchors when `seg_t_start` is not contiguous with the previous
    /// segment's `t_end` (a backward jump = fresh stream).
    ///
    /// Returns `(t0, fresh_stream)`.
    pub fn anchor_segment(&mut self, seg_t_start: f64, seg_t_end: f64, host_now: f64) -> (f64, bool) {
        let fresh = match self.t0 {
            None => true,
            Some(_) => seg_t_start + CONTIGUITY_EPS < self.last_t_end, // backward jump
        };
        if fresh {
            self.t0 = Some(host_now + self.lead_secs - seg_t_start);
        }
        self.last_t_end = seg_t_end;
        (self.t0.unwrap(), fresh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_segment_lands_lead_ahead() {
        let mut a = Anchor::new();
        let (t0, fresh) = a.anchor_segment(0.0, 1.0, 100.0);
        assert!(fresh);
        // piece at u_start=0 → host time t0 + 0 = now + lead
        assert!((t0 + 0.0 - (100.0 + 0.25)).abs() < 1e-9);
    }

    #[test]
    fn contiguous_segment_keeps_t0() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
        // next segment starts where the last ended → same T0, host_now advanced
        let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 100.9);
        assert!(!fresh);
        assert_eq!(t0_a, t0_b);
    }

    #[test]
    fn backward_jump_reanchors() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0);
        // timeline reset to ~0 after a long idle; host_now jumped way forward
        let (t0_b, fresh) = a.anchor_segment(0.0, 1.0, 130.0);
        assert!(fresh);
        assert_ne!(t0_a, t0_b);
        assert!((t0_b - (130.0 + 0.25)).abs() < 1e-9);
    }
}
```

- [ ] **Step 2: Register + run — expect PASS**

Add `pub mod anchor;` to `lib.rs`.
Run: `cargo test -p motion-bridge anchor::tests`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/anchor.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(anchor): shared T0, contiguous-vs-reset re-anchor rule"
```

---

### Task 7: Enqueue adapter — `ShapedSegment` → per-(mcu,axis) pieces

Flattens one shaped segment into `EnqueueMsg`s. Pure (takes the router-projection as a closure so it's testable without a live router). CoreXY transform relocated from `dispatch.rs`.

**Files:**
- Create: `rust/motion-bridge/src/enqueue.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (`pub mod enqueue;`)
- Test: inline

- [ ] **Step 1: Write the failing tests**

Create `rust/motion-bridge/src/enqueue.rs`:

```rust
//! Per-segment enqueue adapter: flatten a ShapedSegment into absolute-timed
//! PieceEntry batches per (mcu, axis). Replaces dispatch::build_push_params.
//! See spec §3.2.

use crate::pump::{AxisKey, EnqueueMsg};
use crate::dispatch::{McuAxisConfig, KINEMATICS_COREXY, AXIS_X, AXIS_Y};
use nurbs::ScalarNurbs;
use runtime::piece_ring::PieceEntry;
use trajectory::ShapedSegment;

/// Build per-(mcu,axis) enqueue messages for one shaped segment.
///
/// `project(mcu_id, host_secs) -> mcu_clock` converts a host-time instant to
/// that MCU's absolute clock (the router's `host_time_to_mcu_clock`). `t0` is
/// the shared anchor (host seconds); a piece at planner time `u_start` has
/// host time `t0 + u_start`. `fresh_stream` is forwarded onto each message.
pub fn enqueue_segment<P>(
    seg: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t0: f64,
    fresh_stream: bool,
    project: P,
) -> Vec<EnqueueMsg>
where
    P: Fn(u32, f64) -> u64,
{
    let mut out = Vec::new();
    for cfg in mcu_configs {
        // CoreXY: pre-compute motor-frame A=X+Y, B=X−Y for MCUs driving both.
        let corexy = cfg.kinematics == KINEMATICS_COREXY
            && cfg.axes.contains(&AXIS_X)
            && cfg.axes.contains(&AXIS_Y);
        let motor = if corexy {
            let x = &seg.axes[AXIS_X];
            let y = &seg.axes[AXIS_Y];
            let a = nurbs::algebra::add_with_knot_union(x, y)
                .expect("post-union add (motor-A)");
            let neg_y = nurbs::algebra::scalar_multiply(y, -1.0_f64);
            let b = nurbs::algebra::add_with_knot_union(x, &neg_y)
                .expect("post-union add (motor-B)");
            Some((a, b))
        } else {
            None
        };

        for &axis_idx in &cfg.axes {
            if axis_idx >= seg.axes.len() {
                continue;
            }
            let curve: &ScalarNurbs<f64> = match (&motor, axis_idx) {
                (Some((a, _)), AXIS_X) => a,
                (Some((_, b)), AXIS_Y) => b,
                _ => &seg.axes[axis_idx],
            };
            let pieces = flatten_axis(curve, t0, cfg.mcu_id, axis_idx as u8, &project);
            if !pieces.is_empty() {
                out.push(EnqueueMsg {
                    key: AxisKey { mcu_id: cfg.mcu_id, axis: axis_idx as u8 },
                    pieces,
                    fresh_stream,
                });
            }
        }
    }
    out
}

fn flatten_axis<P>(
    curve: &ScalarNurbs<f64>,
    t0: f64,
    mcu_id: u32,
    _axis: u8,
    project: &P,
) -> Vec<PieceEntry>
where
    P: Fn(u32, f64) -> u64,
{
    let bps = nurbs::bezier::extract_bezier_pieces(curve);
    let mut out = Vec::with_capacity(bps.len());
    for bp in &bps {
        let bern = bp.to_bernstein();
        let mut coeffs = [0.0_f32; 4];
        for k in 0..4.min(bern.len()) {
            coeffs[k] = bern[k] as f32;
        }
        if bern.len() < 4 && !bern.is_empty() {
            let last = bern[bern.len() - 1] as f32;
            for k in bern.len()..4 {
                coeffs[k] = last;
            }
        }
        let start_time = project(mcu_id, t0 + bp.u_start);
        out.push(PieceEntry {
            start_time,
            coeffs,
            duration: (bp.u_end - bp.u_start) as f32,
            _reserved: 0,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dispatch::McuCaps;

    // A single cubic Bézier moving p0→p1 over the time domain [0,1] s, built
    // from the real public ctors: BezierPiece::from_bernstein (bezier.rs:101)
    // → bezier_pieces_to_nurbs (bezier.rs:532). The four Bernstein coeffs of a
    // degree-3 curve linear in value are [p0, p0+Δ/3, p0+2Δ/3, p1]; the last
    // equals the endpoint value (used by the CoreXY assertion below).
    fn linear_axis(p0: f64, p1: f64) -> ScalarNurbs<f64> {
        let d = p1 - p0;
        let bern = [p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p1];
        let piece = nurbs::bezier::BezierPiece::from_bernstein(&bern, 0.0_f64, 1.0_f64);
        nurbs::bezier::bezier_pieces_to_nurbs(&[piece])
    }

    fn seg_x_move() -> ShapedSegment {
        ShapedSegment {
            axes: [linear_axis(0.0, 10.0), linear_axis(0.0, 0.0), linear_axis(0.0, 0.0)],
            e_mode: geometry::segment::EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        }
    }

    #[test]
    fn cartesian_x_axis_yields_pieces_with_projected_start_time() {
        let cfg = vec![McuAxisConfig {
            mcu_id: 7,
            axes: vec![AXIS_X, AXIS_Y, 2],
            kinematics: 1, // Cartesian (no CoreXY transform)
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        }];
        // project: host_secs * 1000 (fake 1 kHz MCU clock), truncated.
        let msgs = enqueue_segment(&seg_x_move(), &cfg, 100.0, true, |_mcu, hs| (hs * 1000.0) as u64);
        let x = msgs.iter().find(|m| m.key == AxisKey { mcu_id: 7, axis: 0 }).expect("X enqueued");
        assert!(!x.pieces.is_empty());
        // first piece host time = t0 + u_start(0) = 100.0 → 100_000 ticks
        assert_eq!(x.pieces[0].start_time, 100_000);
        assert!(x.pieces.iter().all(|p| p.duration > 0.0));
        // Y and Z carry a constant curve → one (or more) constant pieces, still enqueued.
        assert!(msgs.iter().any(|m| m.key.axis == 1));
    }

    #[test]
    fn corexy_x_slot_is_x_plus_y() {
        let cfg = vec![McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        }];
        // X moves 0→10, Y moves 0→4 → motor A (slot 0) end = 14, B (slot 1) end = 6.
        let seg = ShapedSegment {
            axes: [linear_axis(0.0, 10.0), linear_axis(0.0, 4.0), linear_axis(0.0, 0.0)],
            e_mode: geometry::segment::EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        };
        let msgs = enqueue_segment(&seg, &cfg, 0.0, true, |_mcu, hs| (hs * 1000.0) as u64);
        let a = msgs.iter().find(|m| m.key.axis == 0).unwrap();
        // last Bernstein coeff of motor-A end ≈ 14
        assert!((a.pieces.last().unwrap().coeffs[3] - 14.0).abs() < 1e-3);
    }
}
```

> **Implementer note:** the `ScalarNurbs`/`BezierPiece` ctors above are host-feature-gated (`#[cfg(feature = "host")]` in `rust/nurbs/src/scalar.rs:18`). `motion-bridge` already pulls `nurbs` with the host feature for its other curve code, so the test compiles under `cargo test -p motion-bridge`; if a feature error appears, confirm `nurbs = { ..., features = ["host"] }` (or a `host` passthrough) in `rust/motion-bridge/Cargo.toml`.

- [ ] **Step 2: Register + run — expect PASS** (impl written with tests)

Add `pub mod enqueue;` to `lib.rs`.
Run: `cargo test -p motion-bridge enqueue::tests`
Expected: PASS (after wiring the real `ScalarNurbs` ctor).

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/enqueue.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(enqueue): ShapedSegment -> per-(mcu,axis) PieceEntry adapter with CoreXY transform"
```

---

## Phase D — Integration into `init_planner`

### Task 8: Spawn the pump, rewrite the dispatch closure, route heartbeats

This replaces the segment-era dispatch closure body with: anchor the segment, run the enqueue adapter, send the resulting `EnqueueMsg`s to the pump channel. It spawns the pump thread and attaches the heartbeat callback (which forwards `consumed_counts` to the pump). The old credit/slot setup in `init_planner` stays for now (inert) and is removed in Task 10.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (`init_planner`, ~1880–2423; the `Connection`/lifecycle structs for the pump handle)
- Test: `rust/motion-bridge/tests/streaming_replan.rs` (extend) or a new `tests/push_pieces_integration.rs`

- [ ] **Step 1: Read the current dispatch closure end-to-end**

Run: `sed -n '1880,2424p' rust/motion-bridge/src/bridge.rs` and read it fully. Identify: where `mcu_configs` is built (~1890), where `host_ios` map is built (~1941), where the closure captures state, the per-segment clock derivation (~2185–2305), and the `set(PlannerHandle::spawn(cfg, dispatch))` call (~2420).

- [ ] **Step 2: Build the pump channel + thread before the closure**

After `host_ios` is built (~1960), add (real code, not pseudocode):

```rust
// ── Push-pieces pump ───────────────────────────────────────────────────
use crate::pump::{run_pump, AxisKey, PumpMsg, WireSink};
use std::sync::{mpsc, Weak}; // Arc is already imported at the top of bridge.rs

let (pump_tx, pump_rx) = mpsc::channel::<PumpMsg>();

// Ring depth per (mcu,axis): total_piece_memory / 32 / num_axes_on_mcu.
// (Task 9 makes the MCU actually allocate this; until then the MCU C side
// uses its hardcoded 64, so clamp host depth to min(computed, 64) so room
// accounting never lets the host overfill the real ring.)
let ring_depth_table: HashMap<AxisKey, u32> = {
    let mut t = HashMap::new();
    for cfg in &mcu_configs {
        let total = cfg.caps.total_pieces() as u32;
        let n = cfg.axes.len().max(1) as u32;
        let depth = (total / n).min(64); // see Task 9 — drop the .min(64) clamp once C honors ring_depth
        for &axis in &cfg.axes {
            t.insert(AxisKey { mcu_id: cfg.mcu_id, axis: axis as u8 }, depth);
        }
    }
    t
};

// Downgrade to Weak so the pump never pins an IO alive across detach_serial
// (mirrors dispatch_ios at bridge.rs:1986). Build this BEFORE the dispatch_ios
// loop consumes/moves host_ios, or clone the map.
let wire_ios: HashMap<u32, Weak<KalicoHostIo>> = host_ios
    .iter()
    .map(|(&id, io)| (id, Arc::downgrade(io)))
    .collect();
let pump_timeout = Duration::from_secs(5);
let ring_depth_table_for_pump = ring_depth_table.clone();
let pump_thread = std::thread::Builder::new()
    .name("push-pieces-pump".into())
    .spawn(move || {
        let sink = WireSink { ios: wire_ios, timeout: pump_timeout };
        run_pump(pump_rx, sink, move |k| {
            ring_depth_table_for_pump.get(&k).copied().unwrap_or(64)
        });
    })
    .expect("spawn pump thread");
```

Store `pump_tx` (clone for the closure and for shutdown) and `pump_thread` on the `Connection`/planner-lifecycle struct so `attach_serial`/teardown can `send(PumpMsg::Shutdown)` and `join`. (Mirror how `clock_sync_thread` is stored/joined at `bridge.rs:850,991`.)

- [ ] **Step 3: Attach the heartbeat callback to each MCU's IO**

In the per-MCU setup loop (where `attach_credit_counter` is called, ~2011), add:

```rust
{
    let pump_tx_hb = pump_tx.clone();
    let mcu_id = cfg_mcu.mcu_id;
    io.attach_heartbeat_callback(Arc::new(move |consumed: &[u32]| {
        let _ = pump_tx_hb.send(PumpMsg::Heartbeat(crate::pump::HeartbeatMsg {
            mcu_id,
            consumed_counts: consumed.to_vec(),
        }));
    }));
}
```

- [ ] **Step 4: Replace the dispatch closure body**

Replace the closure body (~2065–2413) with the anchor + enqueue + send. Keep the per-MCU clock-sync wait (`compute_ack_clock`) that establishes the router estimate, but the per-piece conversion now uses `host_time_to_mcu_clock`. New closure:

> **Append-only firewall (spec §2.1) — do not violate.** This closure's only input is the segment the planner hands it; the planner only ever invokes it for *committed* segments (drained `emit_committed`/`commit_decel_to_zero` output). The pump must hold no reference to `ShaperState` or the planner's speculative buffer. Do NOT add a "let the pump peek at the planner buffer to deepen lookahead" path — that reintroduces piece retraction the wire cannot express.

```rust
let mut anchor = crate::anchor::Anchor::new();
let router_for_cb = Arc::clone(&router_arc);
let mcu_configs_for_cb = mcu_configs.clone();
let pump_tx_for_cb = pump_tx.clone();

let dispatch: Arc<dyn Fn(&trajectory::ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
    Arc::new(move |seg: &trajectory::ShapedSegment| -> Result<(), DispatchError> {
        // Shared host "now" (seconds) from the router's single clock.
        let host_now = {
            let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
            r.host_now_secs() // add this thin accessor on PassthroughRouter if absent: instant_to_f64(self.clock.now())
        };
        let (t0, fresh) = anchor.anchor_segment(seg.t_start, seg.t_end, host_now);

        let project = |mcu_id: u32, host_secs: f64| -> u64 {
            let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
            // McuHandle's inner u32 is private; reconstruct via the bridge's
            // existing helper (crate::types::mcu_handle_from_raw, imported at
            // bridge.rs:31, which wraps McuHandle::from_raw). mcu_id is the same
            // raw handle the configs/dispatch_ios key on.
            r.host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
                .unwrap_or(0)
        };

        let msgs = crate::enqueue::enqueue_segment(seg, &mcu_configs_for_cb, t0, fresh, project);
        for m in msgs {
            pump_tx_for_cb.send(crate::pump::PumpMsg::Enqueue(m))
                .map_err(|_| DispatchError::ConnectionDropped(0))?;
        }
        Ok(())
    });
```

> **Add the `host_now_secs` accessor.** `instant_to_f64` (`router.rs:116`) and the per-MCU clock fields are private, so expose `pub fn host_now_secs(&self) -> f64 { instant_to_f64(self.clock.now()) }` on `PassthroughRouter` (no such accessor exists yet — the existing callers compute it inline inside private methods). This is the only new router method needed; `host_time_to_mcu_clock` (`router.rs:437`) already exists and is `pub`.
>
> **Lock discipline:** `project` re-locks the router for every axis of every MCU. If `router_arc` is contended, hoist the projection: lock once per segment, snapshot each MCU's `(clock_offset, clock_freq, last_clock)`, drop the lock, and project arithmetically in the closure (the math is `last_clock + ((host_secs − offset)·freq).max(0)`, `router.rs:447-449`). Start with the simple re-lock; optimize only if the bench shows contention.

- [ ] **Step 5: Write an integration test**

Create `rust/motion-bridge/tests/push_pieces_integration.rs` that drives a planner with a mock/loopback IO (mirror the harness in `tests/streaming_replan.rs`) and asserts that submitting a move results in `PushPieces` frames observed at the IO layer with monotonic per-axis `start_time`s, and that withholding heartbeats stalls further pushes after `ring_depth`. If `streaming_replan.rs` already has a loopback fixture, extend it; otherwise reuse its setup.

- [ ] **Step 6: Run + commit**

Run: `cargo test -p motion-bridge`
Expected: PASS.

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/tests/push_pieces_integration.rs
git commit -m "feat(bridge): spawn pump, rewrite dispatch closure to enqueue adapter, route heartbeats"
```

---

### Task 9: Wire real `ring_depth` through `kalico_configure_axis`

Replace the hardcoded `ring_depth = 64` in C with a host-supplied value, and have the host compute it from `total_piece_memory / num_axes`. Then drop the `.min(64)` clamp added in Task 8.

**Files:**
- Modify: `src/stepper.c` (`command_kalico_configure_axis`, ~222–290)
- Modify: `klippy/motion_toolhead.py` (~1143 command lookup + ~1205 send)
- Modify: `rust/motion-bridge/src/bridge.rs` (drop the `.min(64)` clamp from Task 8)
- (FFI already accepts `ring_depth: u16` — no change.)

- [ ] **Step 1: Add `ring_depth` to the command format (C)**

In `src/stepper.c`, change the command declaration to add a `ring_depth` field. Find the `DECL_COMMAND` for `kalico_configure_axis` (the format string must match Python's lookup exactly) and add `ring_depth=%hu` (u16). Then in `command_kalico_configure_axis`, read it from the correct `args[]` index (it shifts the blob args by one) and replace lines 281–282:

```c
    // ring_depth: PieceEntry slots for this axis, supplied by the host
    // (total_piece_memory / 32 / num_axes_on_mcu). 0 is invalid.
    uint16_t ring_depth = (uint16_t)args[<new_index>];
    if (ring_depth == 0)
        shutdown("configure_axis ring_depth must be nonzero");
```

Adjust every `args[N]` after the inserted field. Update the `DECL_COMMAND` format string and re-check the arg order against the handler.

- [ ] **Step 2: Send `ring_depth` from Python**

In `klippy/motion_toolhead.py`, update the `lookup_command` format string (~1143) to include `ring_depth=%hu` in the same position as the C declaration, and add the value to the `configure_axis_cmd.send([...])` call (~1205). Compute it from the MCU's reported `total_piece_memory` and the number of axes this MCU drives:

```python
# total_piece_memory comes from the runtime caps the bridge already queries;
# expose it to motion_toolhead or recompute: ring_depth = total_pieces // n_axes.
ring_depth = max(1, (total_piece_memory // 32) // num_axes_on_this_mcu)
```

Determine where `total_piece_memory` / `num_axes_on_this_mcu` are available in this function (the bridge already holds `runtime_caps`; expose via a bridge getter if needed). If the value isn't readily available Python-side, pass it from the bridge during `init_planner` setup or have the bridge send `kalico_configure_axis` itself instead of Python — pick whichever matches how config is already driven, and keep one config path.

- [ ] **Step 3: Drop the host clamp**

In `rust/motion-bridge/src/bridge.rs` (Task 8 Step 2), change `(total / n).min(64)` to `(total / n).max(1)` — the MCU now allocates the real depth.

- [ ] **Step 4: Build + flash + smoke-test on the bench**

Per `feedback_bench_firmware_flow`: commit, push, pull on the Pi, `make -j$(nproc)` for both MCUs (`make clean` between H7 and F446 per `feedback_always_make_clean`), flash both. Then boot klippy and confirm no `configure_axis` shutdown and that `kalico_configure_axis` is accepted (check `~/printer_data/logs/klippy.log`).

- [ ] **Step 5: Commit**

```bash
git add src/stepper.c klippy/motion_toolhead.py rust/motion-bridge/src/bridge.rs
git commit -m "feat: wire ring_depth host->MCU via kalico_configure_axis; size from total_piece_memory"
```

---

## Phase E — Removal

### Task 10: Remove the dead segment-era host path

Now that the pump path is live and verified, delete the inert credit/slot/segment machinery. Work bottom-up (remove consumers before types) so the build never breaks mid-edit.

**Files (all under `rust/`):** `motion-bridge/src/dispatch.rs`, `motion-bridge/src/cap_check.rs`, `motion-bridge/src/slot_pool.rs`, `kalico-host-rt/src/producer.rs`, `kalico-host-rt/src/credit.rs`, `kalico-host-rt/src/host_io/{mod.rs,events.rs,reactor.rs}`, `motion-bridge/src/bridge.rs`.

- [ ] **Step 1: Inventory the references**

Run:
```
grep -rn "build_push_params\|split_plan_if_needed\|split_recursive\|McuPushPlan\|SegmentPushParams\|CurveLoadParams\|fits_curve_load\|UNUSED_HANDLE\|is_trivially_constant\|de_casteljau_split\|extract_time_window\|CreditCounter\|attach_credit_counter\|on_credit_freed\|SharedSlotPool\|attach_retirement_callback\|retire_through_segment\|CREDIT_SEED_CAPACITY\|e_mode\|extrusion_ratio" rust/ src/
```
This is the work list. Anything still referenced by the live pump path stays (it shouldn't — verify each).

- [ ] **Step 2: Remove `init_planner`'s credit/slot setup**

In `bridge.rs`, delete the per-MCU `CreditCounter::new` + `attach_credit_counter` + `SharedSlotPool` + `attach_retirement_callback` block (~2005–2042) and the `self.credit_counters` / `self.slot_pools` fields and their clears. Remove `CREDIT_SEED_CAPACITY`.

- [ ] **Step 3: Remove the dispatch/producer/cap_check/credit/slot modules**

Delete `dispatch::build_push_params`, `split_plan_if_needed`/`split_recursive`, `McuPushPlan`, `set_handle`, `UNUSED_HANDLE`, `is_trivially_constant`, `de_casteljau_split`, `extract_time_window`. Keep only what the enqueue adapter imports from `dispatch.rs` (`McuAxisConfig`, `McuCaps`, `KINEMATICS_COREXY`, `AXIS_*`). Delete `cap_check.rs` (`fits_curve_load`), `producer.rs`’s `CurveLoadParams`/`SegmentPushParams`, `credit.rs`, `slot_pool.rs`, and `host_io` credit/retirement wiring (`attach_credit_counter`, `attach_retirement_callback`, `EventDispatcher::on_credit_freed`, the `ReactorCommand::Attach{CreditCounter,RetirementCallback}` arms). Remove `pub mod` lines for deleted modules in the respective `lib.rs`.

- [ ] **Step 4: Drop `e_mode`/`extrusion_ratio` from any remaining dispatch-path struct** (they were on `SegmentPushParams`, removed in Step 3 — confirm nothing else carries them on the host send path).

- [ ] **Step 5: Build the whole workspace**

Run: `cargo build --workspace && cargo test -p motion-bridge -p kalico-host-rt`
Expected: green. Fix any now-dead test files (delete tests of removed APIs).

- [ ] **Step 6: Commit**

```bash
git add -A rust/
git commit -m "remove: dead segment-era host path (credit, slot pool, build_push_params, cap_check, e_mode/extrusion_ratio)"
```

---

## Phase F — Bench validation (spec §9)

### Task 11: End-to-end motion on the bench

Manual, observable (no extrusion — E has no curve yet, §6.1). Per `feedback_no_gcode_without_permission`, ask the user before issuing any motion G-code.

- [ ] **Step 1: Flash both MCUs** with the current branch (`feedback_flash_both_mcus`, `feedback_bench_firmware_flow`).
- [ ] **Step 2:** Single travel move on CoreXY → X/Y MCU moves; Z holds (one constant piece/segment); E silent. Confirm via motion + `klippy.log`.
- [ ] **Step 3:** Move involving Z → Z pieces on F446 while X/Y stream to H7.
- [ ] **Step 4:** Sustained streaming past the initial ring fill → both MCUs stay fed, neither races ahead nor halts (validates heartbeat routing — without it motion stops when the first ring drains). This is the original "one MCU starves" bug.
- [ ] **Step 5:** Clean stop at the end of a move sequence → decel to zero; no `PIECE_START_IN_PAST` fault, no abrupt halt.
- [ ] **Step 6:** Jog after an idle gap → no delay, no past-piece fault (validates the §3.2.1 reset/re-anchor + the Task 9 planner-reset dependency).
- [ ] **Step 7:** Capture `~/printer_data/logs/klippy.log` (`feedback_fetch_logs_to_tmp`) and confirm no faults; record results.

---

## Cross-spec dependency (track separately)

**Planner timeline reset on quiescence (spec §3.2.1):** `commit_decel_to_zero` (`rust/trajectory/src/streaming/emit.rs:241`) must reset the timeline toward 0 on a true stop (reseed at the stop position, zero the cursors), so idle→motion surfaces as `t_start → 0` and the anchor re-anchors. Today it only advances cursors. The anchor (Task 6) already handles the backward jump; this trajectory-crate change makes the jump actually happen. If it is not yet implemented when Task 11 Step 6 runs, that step will fault — implement the reset (its own small task in the `trajectory` crate) before validating jog-after-idle. **This is a `trajectory`-crate change, separate from this plan's host scope; flag it to the owner.**

**MCU-side credit removal:** the companion segment-era MCU removal spec deletes `CreditFreed` emission. Sequence merges so neither side lands a dangling reference (spec §6.2).
