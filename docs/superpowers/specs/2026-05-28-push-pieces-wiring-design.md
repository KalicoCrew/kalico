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
- **A late piece is a hard fault, not a soft pickup.** `engine.rs:654-659`: once `now − start_time > FAULT_TOLERANCE` (2 ISR ticks, ≈ tens of µs), the engine raises `PIECE_START_IN_PAST` — per contract §6 it stops all steppers, emits `FaultEvent`, and refuses further `PushPieces` until reset/reconfigure. There is no stutter-and-recover; a piece that arrives after its `start_time` halts the machine.
- **Wire send exists.** `KalicoHostIo::kalico_call(MessageKind::PushPieces, body, timeout)` sends one control-channel frame and awaits the matching `PushPiecesResponse`. `kalico_protocol::messages::PushPieces` already has an `Encode` impl. No new transport code is required. **But this is a blocking round-trip** (`host_io/mod.rs:755`, blocks at `:780`) — see §3.5.
- **Heartbeat is currently DROPPED, not consumed — this is new routing, and skipping it is a guaranteed early halt.** The `StatusHeartbeat` *struct* decodes, but `lift_event_to_runtime_event` (`kalico_native.rs`) routes it into the `_ =>` "unexpected event kind" arm and discards it; there is no `RuntimeEvent::Heartbeat` variant (`runtime_events.rs:52` has only `Fault`). The doc comment at `kalico_native.rs:14` claims it lifts `StatusHeartbeat` — it does not. Consequence: with `consumed` never updating, after the host pushes `ring_depth` pieces `room = ring_depth − (ring_depth − 0) = 0`, the pump stalls "waiting for a heartbeat" — while heartbeats physically arrive and are shredded at the door. Motion halts the instant the initial ring plays out. So this is real work (add a dispatch arm + a route to the pump), not "wire a value in" — see §3.4.
- **Clock conversion — the live anchor is NOT `host_time_to_mcu_clock`.** That function is test-only (`router_transport/tests.rs:45,57`; live code only mentions it in comments). The live dispatch path anchors via `PassthroughRouter::compute_ack_clock(mcu)` for the per-MCU "now" (`bridge.rs:2206`), then places segment times as `mcu_base_clock + (t · freq)` (`bridge.rs:2293-2297`). Per-piece conversion must use the *same* anchor: `start_time = mcu_base_clock + round(u_start · freq)`. This anchor is also what must re-establish on every planner `reset()` (set_position / stream open) — fix the citation and the re-anchor together; they are one anchor.
- **`consumed` is a wrapping `u32`.** `piece_ring.rs:64` (`consumed: u32`), `:160` (`wrapping_add`). The host's `room = ring_depth − (pushed − consumed)` must be computed in **wrapping `u32`**, and host-side `pushed` must be the same width (or be reset per epoch). Harmless on a short bench run (2³² pieces is enormous) but a silent landmine on a long-lived host process — free to get right now.

## 2. Core model

Each axis is, after shaping and the C¹ refit, a chain of cubic Bézier pieces — each exact on its absolute-time interval `[u_start, u_end]`. Axes are refit **independently**, so their piece boundaries do **not** line up across axes, and that is fine: nothing downstream requires aligned boundaries.

The two reasons we do **not** impose a common time grid across axes:

1. **Pause** is a host-side concern that lives entirely above piece generation: to pause, the host plans the current G-code move down to zero velocity and stops emitting. The MCU is never told to pause.
2. **Cancel** (a future trsync trip) stops every MCU regardless of where its pieces are.

So pieces stay in their natural per-axis form. The only cross-axis discipline is in *what order we push them* (§4), not in how they are cut.

A cubic Bézier piece represents a degree-3 polynomial exactly on its interval. You can always *split* a piece (de Casteljau, exact) but never *merge* two across a boundary (would exceed degree 3). This design never needs to do either — it ships the pieces the refit produced, unmodified.

### 2.1 Source: the committed dispatch stream — append-only by construction

The host does **not** receive finished moves. The streaming shaper only dispatches shaped output up to a moving commit boundary `target = t_decel_start − max_h` (`streaming/emit.rs:82`). Everything past that boundary — the trailing decel ramp — is **speculative**: a later replan may rewrite it (`replace_uncommitted_axis_pieces`), or the quiescence timer fires and `commit_decel_to_zero` (`streaming/emit.rs:241`) adopts it as the real stop. Both paths emit their now-committed segments through the **same dispatch closure** this spec consumes.

This is the architectural firewall that makes append-only push correct, and it is structural, not a discipline:

> **The pump's sole input is the committed dispatch stream — the drained output of `emit_committed` / `commit_decel_to_zero`, delivered via the dispatch closure. The pump holds no handle to `ShaperState` internals (`axes[i].pieces` past `t_dispatched`, `planned_fitted`). The speculative tail is not reachable from the pump.**

Append-only correctness then follows automatically: the only pieces that *exist* to push are already committed, hence immutable. The wire protocol has no "replace piece N" operation and needs none — there is no code path by which the pump could push a piece a later replan might change. Framed this way, the failure mode (reaching forward into the planner buffer to deepen lookahead) is **impossible by construction**, not merely forbidden — and it pre-empts a future "optimization" that would hand the pump a peek at the speculative plan to fill rings deeper. Any such change would be reintroducing the retraction problem, and the boundary is where that must be refused.

A direct consequence for the pump's lifecycle: "all queues drained up to the boundary" is **not** end-of-motion. The actual zero-velocity stop ramp is held back until quiescence commits it, arriving later as its own segments (latest in time — the earliest-first scheduler emits them last). The pump stays alive across a drained-to-boundary state; it does not tear down. This is exactly the host-side "pause = bring the axis to zero velocity at the end of the move" behavior — it *is* the quiescence commit, already wired through the dispatch closure.

## 3. Components

Three pieces, in the order data flows.

### 3.1 Ring allocation — `ConfigureAxis` (startup, config-time)

Pure config-time bookkeeping, independent of everything else in this spec. Per MCU:

- Read `total_piece_memory` (bytes) from `RuntimeCapsResponse`.
- `total_pieces = total_piece_memory / 32`.
- Divide equally across the axes that MCU drives: `ring_depth = total_pieces / num_axes_on_mcu`.
- Send one `ConfigureAxis` per axis carrying that `ring_depth` (plus the existing stepping-mode / microstep / stepper-binding fields, unchanged).

This logic knows nothing about motion, static axes, or piece content. It allocates a ring for **every** axis the MCU drives — whether an axis actually moves during a given print is a runtime send-time decision (§3.2, §4) and has no bearing on allocation.

**Sizing sanity check (budget 1 of §5.1).** Memory is the allocation driver, but the resulting depth must be validated against the steady-stream refill budget: `ring_depth × typical_piece_duration` (ring-depth-in-time) must comfortably exceed the heartbeat RTT, so the ring does not drain between refills. The defaults in the contract spec (≈496 pieces/axis on H7, ≈0.5 s at ~1 ms/piece, vs a 10 Hz / 100 ms heartbeat) clear this by ~5×. If a board's memory budget ever yields a ring too shallow to cover `max_h` + heartbeat RTT, that is a configuration error to surface at startup, not a runtime degradation to absorb silently. Ring depth does **not** address budget 2 (the stop case) — see §5.1.

### 3.2 Enqueue adapter (per `ShapedSegment`, planner thread)

Replaces `build_push_params` + `split_plan_if_needed` + `McuPushPlan` entirely. For each `ShapedSegment` the planner emits:

1. **CoreXY transform** (preserved, relocated from `dispatch.rs`): for a CoreXY MCU driving both X and Y, combine into motor-frame A = X + Y (slot 0) and B = X − Y (slot 1) via the existing `nurbs::algebra::add_with_knot_union`. All other axes pass through.
2. For each axis the segment provides a curve for:
   - `extract_bezier_pieces` → per piece:
     - `start_time = mcu_base_clock + round(u_start · freq)` — the **live anchor** (`compute_ack_clock` + `mcu_base_clock + t·freq`, §1.1), per-MCU, in absolute clock cycles. **Not** `host_time_to_mcu_clock` (test-only). Re-anchored on planner `reset()`.
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
- `pushed: u32` — count the host has committed to the wire,
- `consumed: u32` — latest value from the heartbeat,
- `ring_depth` — from §3.1.

`room = ring_depth − (pushed −_wrapping consumed)`, computed in **wrapping `u32`** to match the MCU's `consumed` (§1.1). `pushed` increments when a frame's pieces are **committed to the wire** (the room is reserved at that point), not when the `PushPiecesResponse` returns — the response only confirms; host-side accounting is authoritative (the host never sends a doomed push). On planner `reset()`/epoch change, `pushed` and `consumed` reset together with the clock re-anchor (§3.2).

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

**Tie-break.** When two heads share the same `start_time` (common: CoreXY A and B are the X±Y combination over identical windows, so their pieces are co-timed), break ties deterministically by `(mcu_id, axis_idx)`. Determinism matters only for reproducibility — co-timed pieces on different rings are independent, so any fixed order is correct.

A consequence, stated for clarity: the system's effective buffer depth is governed by whichever ring fills first **in time** — the axis emitting the most pieces per second (X/Y under shaping), not the stationary ones. This is automatic and intended.

### 3.4 Heartbeat handling — new routing, not a value-wire

Today `StatusHeartbeat` is decoded as a struct but **discarded** (§1.1): `lift_event_to_runtime_event` drops it into the `_ =>` arm. This work must:

1. Add a `MessageKind::StatusHeartbeat` arm to `lift_event_to_runtime_event` (`kalico_native.rs`) that decodes the body and produces a routed result instead of `Ignored`.
2. Carry it to the pump. Given the concurrency model (§3.5), the cleanest route is a dedicated heartbeat channel/callback into the pump (a `RuntimeEvent::Heartbeat` variant is the alternative, but heartbeats are pump-private and need not surface to the general runtime-event consumers).

On each heartbeat the pump updates per-`(mcu, axis)` `consumed` and re-evaluates the stall (a freed ring may unblock the global head). Skipping this is the guaranteed early halt of §1.1, not an edge case.

### 3.5 Pump concurrency

Three actors touch the per-`(mcu, axis)` state: the **planner thread** (enqueue, §3.2), the **pump**, and the **heartbeat handler** (§3.4, runs on the reactor/event path). Two non-negotiables:

1. **The pump must never hold a lock across the blocking `kalico_call`.** `kalico_call` blocks on the wire round-trip (`host_io/mod.rs:780`). Holding any shared lock across it serializes the planner thread and the heartbeat handler behind wire latency — under sustained motion that throttles the planner to the wire RTT and starves the heartbeat updates the pump itself depends on (deadlock-adjacent).
2. **The dual wakeup must be lost-wakeup-safe.** The pump wakes on *both* "new pieces enqueued" and "heartbeat freed room". A naive condvar can miss a wake that fires between the stall check and the wait.

**Recommended model: the pump owns the queues and all ring accounting; it is fed by channels.** The planner sends flattened pieces over a channel; the heartbeat handler sends `(mcu, axis, consumed)` updates over a channel. The pump's run loop selects over both, mutating its own state with no shared lock — so neither the planner nor the heartbeat handler ever blocks on the pump, and the lost-wakeup problem dissolves into channel readiness. The only blocking call (`kalico_call`) happens while the pump holds nothing the other two actors need.

## 4. Wire send detail

Per `PushPieces` frame: `kalico_call(MessageKind::PushPieces, body, timeout)`, where `body` encodes `axis_idx: u8, piece_count: u8, pieces[count × 32 bytes]` via the existing `PushPieces` `Encode`. The response is `PushPiecesResponse`.

**Result codes must be shared, named constants** (host ↔ MCU), not magic numbers: `OK` / `RING_FULL` / `INVALID_AXIS` (the MCU side returns these from `handle_push_pieces`; see `KALICO_ERR_*` usage in `kalico_dispatch.c`). Define them once in `kalico_protocol` and reference from both sides so a renumber can't desync. In normal operation the result is always `OK` (host-side room accounting); `RING_FULL` is the safety net.

**Throughput sanity (CLAUDE.md constraint #1).** `kalico_call` is a blocking round-trip, so a serial pump caps piece throughput at `1 / RTT` frames/s per MCU. With batching (§3.3) a frame can carry many pieces of one axis, so the relevant rate is *frames*, not pieces: a USB RTT of ~0.5–1 ms gives ~1000–2000 frames/s, and shaped XY emits on the order of hundreds–low-thousands of pieces/s aggregate — so coalesced frames keep the serial pump comfortably ahead for the bench bring-up. If a future high-piece-rate workload approaches the ceiling, pipelining (fire-and-forget plus response reconciliation against the heartbeat) is the lever — a throughput optimization, not a design change. This is the one number to validate on the bench before declaring the path adequate, not an assumption to carry silently.

## 5. Scheduling / timing notes

- **Initial start-time lead.** The first piece of a fresh stream must have a `start_time` far enough in the future that it is loaded before the ISR reaches it. This is covered by the planner running ahead of realtime (its existing look-ahead) mapped through the live anchor (§3.2; note the existing `lead_cycles_init ≈ freq · 0.25` lead at `bridge.rs:2200`), combined with the pump pushing earliest-first so near-future pieces always go out first.
- **All axes of a segment arrive together.** The planner emits each `ShapedSegment` covering all axes over one `[t_start, t_end]` window, so the enqueue adapter never presents the pump with a partial timeline; the global merge always has a complete picture up to the latest enqueued segment.

### 5.1 Underrun is a halt — two independent budgets

If the MCU drains its ring to the committed frontier, the axis idle-holds (§1.1). But the frontier sits at `t_decel_start − max_h` — **mid-cruise, nonzero velocity** — so the freeze is already an infinite-jerk stop. Worse, when the held-back ramp finally arrives its `start_time` is now in the MCU's past, which is a **hard fault** (`PIECE_START_IN_PAST`, §1.1), not a soft pickup: the machine halts and needs a reconfigure. This is a halt-class invariant the design depends on, inherited from the streaming shaper (the old segment dispatch had it too). We do **not** fix it here — but the design relies on two separate budgets staying satisfied, and ring depth only covers one:

1. **Steady-stream refill (mid-print, moves keep coming).** Each new move's replan advances `t_decel_start`, so the committed frontier keeps moving forward; the only risk is heartbeat-latency refill lag. **Binding check: ring-depth-in-time > heartbeat RTT.** This is the §3.1 sizing sanity check, and it is the *only* underrun ring depth defends against.
2. **Boundary / stop underrun (genuine stop, dwell, end-of-print, or input starvation).** The frontier stalls at the last move's `t_decel_start − max_h` and advances only when quiescence fires `commit_decel_to_zero`. **Ring depth does nothing here** — there is no piece past the frontier to push, no matter how deep the ring. The binding budget is planner-side: `T_commit + emit + push + RTT` must be less than the buffered time remaining at the boundary, with `FAULT_TOLERANCE` (2 ISR ticks ≈ tens of µs) as the absolute floor.

Plain version: deep rings are a bigger water tank — they ride out a slow refill. But at a stop the tap itself is closed until the planner decides the print is done; a bigger tank just delays the moment you notice no water is coming. What saves the stop case is the planner reopening the tap (committing the decel) before the tank runs dry — a timer budget, not a tank-size budget.

§3.1 sizing must therefore sanity-check ring-depth-in-time against `max_h` + heartbeat RTT (budget 1); budget 2 is a planner concern (`T_commit` vs buffered-time-at-boundary) and is named here only so the dependency is explicit, not silently assumed.

## 6. Removal (folded into this work) — unwiring a LIVE path, not deleting orphans

Important correction to an earlier framing: the credit / slot machinery is **not dead code**. It is **live-wired and only inert** — the MCU simply stopped emitting `CreditFreed`, so the path never fires, but every link is present and compiled in:

- `attach_credit_counter` (`host_io/mod.rs:543`, called from `bridge.rs:2011`),
- the reactor route `ReactorCommand::AttachCreditCounter` (`reactor.rs:1113`),
- `EventDispatcher::on_credit_freed` (`events.rs:268`),
- plus the `credit/tests.rs` and `events/dispatch_tests.rs` suites.

So removal means **carefully unwiring a live path**, and the order matters — pull the wiring before (or together with) the types, or the build breaks mid-removal. The §6 "leave the build green" requirement is doing real work here, not a platitude.

Remove (host side):

- `dispatch.rs`: `build_push_params`, `split_plan_if_needed` / `split_recursive`, `McuPushPlan`, `SegmentPushParams` references, packed-handle constants (`UNUSED_HANDLE`) and `set_handle`, and `is_trivially_constant`. Keep the CoreXY transform logic (relocated into the enqueue adapter); `de_casteljau_split` / `extract_time_window` are unused once sub-splitting is gone — remove them too.
- `producer.rs`: `CurveLoadParams`, `SegmentPushParams`, and the now-unused conversion helpers.
- **`cap_check.rs`: `fits_curve_load`** (`cap_check.rs:19`) — an additional `CurveLoadParams` consumer the earlier list missed. Removing `CurveLoadParams` without this leaves a dangling reference.
- `credit.rs` and the `CreditCounter` wiring above (`attach_credit_counter`, the reactor command, `CREDIT_SEED_CAPACITY`).
- The slot pool (`SharedSlotPool`) and retirement-callback wiring (`attach_retirement_callback`, `EventDispatcher::on_credit_freed`, `retire_through_segment`).
- The `e_mode` / `extrusion_ratio` fields on the dispatch path (see §6.1 on E).
- **Replace the hardcoded `total_pieces() / 4`** (`bridge.rs:2021` and `:2365`) with the per-MCU `num_axes` division of §3.1. The `/4` was a stand-in for "assume 4 axes"; ring sizing must use the actual axis count for that MCU.

### 6.1 E is a follower ratio today, not a curve — do not overclaim "printable"

`emit_shaped` carries E as a follower *ratio* (`extrusion_per_xy_mm`), **not** as a Bézier curve on an E axis. So "the planner emits E pieces like any other axis" is the *target*, not the current reality. Under this spec:

- **Travel moves work** (X/Y/Z have curves; E has none → nothing enqueued).
- **Extruding moves produce no E motion** — there is no E curve to extract pieces from.

Closing that is a new pipeline stage: arc-length integration of the shaped XY into an E position curve (the host-side half of "E follows XY", which was correctly removed from the MCU because it belongs in the planner). That stage is **out of scope here** and is its own spec. §9's success criteria are therefore **travel/motion only** — this spec does not claim a printable extruding path.

### 6.2 Cross-spec sequencing

The credit subsystem spans both this spec (host side) and the companion segment-era MCU-side removal spec. Sequence them so neither lands a dangling reference: the host unwiring here must not assume MCU-side `CreditFreed` removal has happened, and vice versa. Coordinate the merge order or gate one on the other.

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

**Scope of "works": travel/motion only.** This spec does not deliver an extruding path (§6.1) — E has no curve yet. Success criteria are about getting time-synchronized *motion* onto the wire, not printing.

End-to-end, on the bench (the user can verify motion directly):

- A single travel move on CoreXY produces motion on the X/Y MCU; Z gets one constant piece per segment (held), E is silent (no curve on a travel move).
- A move involving Z produces Z motion pieces on the F4 while X/Y stream to the H7.
- **Sustained streaming past the initial ring fill** keeps both MCUs fed without one racing ahead or halting — this exercises the heartbeat-routing fix (§3.4); without it, motion stops the instant the first ring drains. This is the original bug *and* the new first-light risk in one test.
- Heartbeat `consumed_counts` advance and the pump refills as rings drain.
- A clean stop (end of a move sequence) decelerates to zero — the quiescence ramp (§2.1) arrives and is pushed; no `PIECE_START_IN_PAST` fault, no abrupt halt.

Unit / integration (host crates):

- Enqueue adapter: a moving axis yields time-ordered `PieceEntry`s with correct absolute `start_time` (live anchor, §3.2) and `duration`; a non-moving axis yields one constant piece per segment; CoreXY A/B are the X±Y combination (transform applied *before* piece extraction).
- Pump scheduler: strict global start-time order; stall when the head's ring is full (no skip-ahead to a ring with room); same-MCU runs coalesce; no frame is sent when `room == 0`; deterministic `(mcu_id, axis_idx)` tie-break on equal `start_time`.
- Room accounting: wrapping-`u32` arithmetic stays correct across a `consumed`/`pushed` wrap (don't require 2³² real pieces — unit-test the wrap directly).
- Heartbeat routing: a `StatusHeartbeat` frame reaches the pump and advances `consumed` (regression guard against the `_ =>` discard).
- Result codes: host and MCU agree on the named `OK`/`RING_FULL`/`INVALID_AXIS` constants (shared source, §4).
- Ring allocation: `total_piece_memory` divides by the actual per-MCU `num_axes` (not the old `/4`).
- Concurrency: the pump never holds a shared lock across `kalico_call` (§3.5) — assert via the channel-fed ownership model rather than a runtime check where possible.
