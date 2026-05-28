# Push-Pieces Wiring

**Date:** 2026-05-28
**Branch:** `simple-mcu-contract` (from `sota-motion`)
**Goal:** Make the host actually drive motion by streaming `PushPieces` frames to each MCU, replacing the dead segment / curve / handle dispatch path left behind by the simple-MCU-contract rewrite. This is the missing wire-send at `bridge.rs:2397` ("builds sub-plans but does not send them").

**Explicitly out of scope:** flush and cancel/homing. Both are separate work pieces (see §8). This spec is only about getting time-synchronized motion onto the wire so the system can be tested end to end.

---

## 1. Background

The MCU contract was already simplified to a dumb per-axis polynomial playback engine (see `2026-05-27-simple-mcu-contract-design.md`): each axis owns a ring of 32-byte `PieceEntry` cubic-Bézier fragments and plays them back by absolute MCU-clock `start_time`. The wire surface is `ConfigureAxis` (allocate per-axis rings) and `PushPieces` (append pieces), with `StatusHeartbeat` reporting per-axis monotonic `consumed_counts`.

The **host** never caught up. The dispatch closure (`bridge.rs`) still builds the old `McuPushPlan` (segment / curve / packed-handle form via `build_push_params` + `split_plan_if_needed`) and then does nothing with it — the `LoadCurveCubic` + `PushSegment` wire path it targeted was deleted. So no pieces ever reach the MCU.

This spec replaces that whole layer with: flatten each shaped segment into per-axis absolute-timed pieces, and stream them in strict time order, flow-controlled by the heartbeat.

### 1.1 Verified facts this design rests on

- **`PieceEntry` layout** (`rust/runtime/src/piece_ring.rs`): `start_time: u64` (MCU clock cycles), `coeffs: [f32; 4]` (Bernstein control points), `duration: f32` (seconds), `_reserved: u32`. 32 bytes total.
- **Idle = hold.** `engine.rs::get_position_and_velocity` returns `None` when an axis has no current piece or the next piece has not started; the tick loop `continue`s, leaving `p_prev` untouched and emitting no steps. An axis with no piece freezes at its last position — it does **not** snap to zero. This covers genuinely pieceless cases (an axis the segment has no curve for, e.g. E on a pure-travel move; or end-of-stream gaps). It is **not** relied on as an optimization for non-moving axes — see §3.2.
- **Every segment carries all three kinematic axes.** `emit_shaped` always builds `ShapedSegment.axes[0..2]` (X/Y/Z); a non-moving axis gets a *constant* curve, which `extract_bezier_pieces` yields as one constant piece. So a standing-still axis produces one cheap piece per segment, not zero. E is separate and may be absent on travel moves.
- **Wire send exists.** `KalicoHostIo::kalico_call(MessageKind::PushPieces, body, timeout)` sends one control-channel frame and awaits the matching `PushPiecesResponse`. `kalico_protocol::messages::PushPieces` already has an `Encode` impl. No new transport code is required.
- **Clock conversion exists.** The bridge already maps planner-local time to per-MCU clock via the clock-sync + planner-epoch machinery it used for segment `t_start`/`t_end` (`PassthroughRouter::host_time_to_mcu_clock`). We apply the same mapping per piece instead of per segment.

## 2. Core model

Each axis is, after shaping and the C¹ refit, a chain of cubic Bézier pieces — each exact on its absolute-time interval `[u_start, u_end]`. Axes are refit **independently**, so their piece boundaries do **not** line up across axes, and that is fine: nothing downstream requires aligned boundaries.

The two reasons we do **not** impose a common time grid across axes:

1. **Pause** is a host-side concern that lives entirely above piece generation: to pause, the host plans the current G-code move down to zero velocity and stops emitting. The MCU is never told to pause.
2. **Cancel** (a future trsync trip) stops every MCU regardless of where its pieces are.

So pieces stay in their natural per-axis form. The only cross-axis discipline is in *what order we push them* (§4), not in how they are cut.

A cubic Bézier piece represents a degree-3 polynomial exactly on its interval. You can always *split* a piece (de Casteljau, exact) but never *merge* two across a boundary (would exceed degree 3). This design never needs to do either — it ships the pieces the refit produced, unmodified.

## 3. Components

Three pieces, in the order data flows.

### 3.1 Ring allocation — `ConfigureAxis` (startup, config-time)

Pure config-time bookkeeping, independent of everything else in this spec. Per MCU:

- Read `total_piece_memory` (bytes) from `RuntimeCapsResponse`.
- `total_pieces = total_piece_memory / 32`.
- Divide equally across the axes that MCU drives: `ring_depth = total_pieces / num_axes_on_mcu`.
- Send one `ConfigureAxis` per axis carrying that `ring_depth` (plus the existing stepping-mode / microstep / stepper-binding fields, unchanged).

This logic knows nothing about motion, static axes, or piece content. It allocates a ring for **every** axis the MCU drives — whether an axis actually moves during a given print is a runtime send-time decision (§3.2, §4) and has no bearing on allocation.

### 3.2 Enqueue adapter (per `ShapedSegment`, planner thread)

Replaces `build_push_params` + `split_plan_if_needed` + `McuPushPlan` entirely. For each `ShapedSegment` the planner emits:

1. **CoreXY transform** (preserved, relocated from `dispatch.rs`): for a CoreXY MCU driving both X and Y, combine into motor-frame A = X + Y (slot 0) and B = X − Y (slot 1) via the existing `nurbs::algebra::add_with_knot_union`. All other axes pass through.
2. For each axis the segment provides a curve for:
   - `extract_bezier_pieces` → per piece:
     - `start_time = host_time_to_mcu_clock(u_start)` (per-MCU; absolute clock cycles).
     - `duration = (u_end − u_start)` as f32 seconds.
     - `coeffs = ` the four Bernstein control points as f32.
     - `_reserved = 0`.
   - Append the resulting `PieceEntry`s, in time order, to that `(mcu, axis)` queue.

No segment IDs, no packed handles, no slot allocation, no sub-splitting.

**No static/skip judgment anywhere.** The adapter extracts and enqueues whatever pieces the segment's curves yield. A non-moving axis carries a constant curve → one constant piece, which is sent like any other; an axis with no curve (e.g. E on a travel move) yields nothing, naturally. Sending the explicit constant piece is simpler *and* safer than skipping: position continuity is carried in the piece itself rather than depending on the engine's hold behavior. The pump (§3.3) likewise never inspects piece content — it only sees queued `PieceEntry`s and their `start_time`s.

### 3.3 Pump (async; the real new logic)

A single global pusher, woken by **both** "new pieces enqueued" and "heartbeat updated `consumed_counts`". It cannot live inside the per-segment closure because it must react to heartbeats arriving asynchronously between segments.

It maintains, per `(mcu, axis)`:
- the queue of not-yet-pushed `PieceEntry`s (from §3.2),
- `pushed` — count the host has sent,
- `consumed` — latest value from the heartbeat,
- `ring_depth` — from §3.1.

`room = ring_depth − (pushed − consumed)`.

**Scheduling — strict global start-time order with stall-on-full-head:**

```
loop:
  head = the (mcu, axis) whose next unpushed piece has the smallest start_time
  if no head:                      # all queues drained
      wait for new pieces
      continue
  if room(head) == 0:              # the head's ring is full
      wait for a heartbeat         # DO NOT push anything else
      continue
  # head has room — push it, coalescing a contiguous same-MCU run
  batch = longest prefix of the global time order that is all on head.mcu
          and whose per-axis rings still have room
  send PushPieces frame(s) for batch   # one frame per axis (wire is per-axis)
```

The stall is the entire safety property: a hung (non-consuming) MCU fills its ring, the global head lands on it, and the pusher waits — so no other MCU can run ahead of a stuck one. Halting the system on a hung MCU is the correct, safe outcome; a higher layer (cancel / shutdown) deals with the hang.

**Never send a doomed push.** Room is computed from host-side accounting, so a `PushPieces` is only ever sent when it will fit. `RING_FULL` in `PushPiecesResponse` stays a pure safety net that should not fire in normal operation — the host never spends MCU compute on a rejection.

**Batching** is the one efficiency carve-out and cannot break ordering: the batch is always a contiguous prefix of the global time order that happens to be on one MCU, split into one `PushPieces` frame per axis (the wire format carries `axis_idx, piece_count, pieces[]`). It ends the instant the next piece in global order is on a different MCU, or a ring in the run hits `room == 0`.

A consequence, stated for clarity: the system's effective buffer depth is governed by whichever ring fills first **in time** — the axis emitting the most pieces per second (X/Y under shaping), not the stationary ones. This is automatic and intended.

### 3.4 Heartbeat handling

On each `StatusHeartbeat`: read per-axis `consumed_counts`, update each `(mcu, axis)` `consumed`, wake the pump. (`StatusHeartbeat` decoding already exists; this wires its `consumed_counts` into the pump's accounting.)

## 4. Wire send detail

Per `PushPieces` frame: `kalico_call(MessageKind::PushPieces, body, timeout)`, where `body` encodes `axis_idx: u8, piece_count: u8, pieces[count × 32 bytes]` via the existing `PushPieces` `Encode`. The response is `PushPiecesResponse` (`OK` / `RING_FULL` / `INVALID_AXIS`); in normal operation it is always `OK` (host-side room accounting).

`kalico_call` is a blocking round-trip. For the MVP the pump issues them serially. Pipelining (fire-and-forget plus response reconciliation) is a throughput tuning lever, not a design change, and is left for later if the round-trip latency proves to bound piece throughput.

## 5. Scheduling / timing notes

- **Initial start-time lead.** The first piece of a fresh stream must have a `start_time` far enough in the future that it is loaded before the ISR reaches it. This is covered by the planner running ahead of realtime (its existing look-ahead) mapped through `host_time_to_mcu_clock`, combined with the pump pushing earliest-first so near-future pieces always go out first. `PIECE_START_IN_PAST` (a hard fault, §4.2 of the contract spec) is the backstop if the host ever falls behind.
- **No underrun in normal operation.** Strict time-order pushing keeps every axis buffered to ≈ the same wall-clock frontier (bounded by the tightest ring). Underrun only occurs if an MCU stops consuming — which is the hung-MCU case the stall rule deliberately turns into a system halt.
- **All axes of a segment arrive together.** The planner emits each `ShapedSegment` covering all axes over one `[t_start, t_end]` window, so the enqueue adapter never presents the pump with a partial timeline; the global merge always has a complete picture up to the latest enqueued segment.

## 6. Removal (folded into this work)

The new pump cannot coexist cleanly with the credit / slot machinery, and the segment-era host types are dead under the piece-ring contract. Remove:

- `dispatch.rs`: `build_push_params`, `split_plan_if_needed` / `split_recursive`, `McuPushPlan`, `SegmentPushParams` references, packed-handle constants (`UNUSED_HANDLE`) and `set_handle`, and `is_trivially_constant`. Keep the CoreXY transform logic (relocated into the enqueue adapter); `de_casteljau_split` / `extract_time_window` are unused once sub-splitting is gone — remove them too.
- `producer.rs`: `CurveLoadParams`, `SegmentPushParams`, and the now-unused conversion helpers.
- `credit.rs` and the `CreditCounter` wiring in `bridge.rs` (`attach_credit_counter`, `CREDIT_SEED_CAPACITY`).
- The slot pool (`SharedSlotPool`) and retirement-callback wiring (`attach_retirement_callback`, `on_credit_freed`, `retire_through_segment`).
- The `e_mode` / `extrusion_ratio` fields on the dispatch path. **E becomes an ordinary axis**: the planner emits E pieces like any other axis, and the MCU has no follower math (E-follows-XY was removed from the MCU precisely because it belongs in the planner). Whether the planner's E-follower *curve generation* is complete is downstream of this spec; the push path treats E uniformly.

Removal should leave the build green with the new path in place — not a separate "delete then rebuild" step.

## 7. What stays unchanged

- The planner, shaper, TOPP-RA, and the per-segment `ShapedSegment` emission. Segments remain a **planner** concept (the unit over which dynamics and the fit are defined); they are simply never a wire concept.
- `ConfigureAxis`'s stepping-mode / microstep / stepper-binding payload.
- Clock-sync and the planner-epoch mapping the bridge already maintains.
- The MCU firmware (this is host-only work; the MCU already implements `ConfigureAxis` / `PushPieces` / `StatusHeartbeat`).

## 8. Deferred (separate specs)

- **Flush:** purely host-side. The host knows from `consumed_counts` when every pushed piece has played; a flush is "block until consumed == pushed on all axes". Gradual stop (plan-to-zero-velocity) lives above piece generation. No MCU message.
- **Cancel + cross-MCU homing:** a generic `Cancel` wire command (stop all axes now, clear rings, hold), the trsync→engine seam, host relay of a trip to all MCUs, and host-side reconstruction of every axis's position by evaluating the retained piece polynomials at the trip timestamp. This is the architectural follow-up that this spec's piece stream is a prerequisite for.

## 9. Testing

End-to-end, on the bench (the user can verify motion directly):

- A single travel move on CoreXY produces motion on the X/Y MCU; Z gets one constant piece per segment (held), E is silent (no curve on a travel move).
- A move involving Z produces Z motion pieces on the F4 while X/Y stream to the H7.
- Sustained streaming (many moves) keeps both MCUs fed without one racing ahead of the other — the original bug.
- Heartbeat `consumed_counts` advance and the pump refills as rings drain.

Unit / integration (host crates):

- Enqueue adapter: a moving axis yields time-ordered `PieceEntry`s with correct absolute `start_time` and `duration`; a non-moving axis yields one constant piece per segment; CoreXY A/B are the X±Y combination.
- Pump scheduler: strict global start-time order; stall when the head's ring is full (no skip-ahead to a ring with room); same-MCU runs coalesce; no frame is sent when `room == 0`.
- Ring allocation: `total_piece_memory` divides equally across a MCU's driven axes.
