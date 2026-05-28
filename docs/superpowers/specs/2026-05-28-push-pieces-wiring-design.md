# Push-Pieces Wiring

**Date:** 2026-05-28
**Branch:** `simple-mcu-contract` (from `sota-motion`)
**Goal:** Make the host drive motion by streaming `PushPieces` frames to each MCU, replacing the dead segment / curve / handle dispatch path. This is the missing wire-send at `bridge.rs:2397` ("builds sub-plans but does not send them").

**Out of scope:** flush and cancel/homing (§8). This spec only gets time-synchronized motion onto the wire so the system can be tested end to end.

---

## 1. Background

The MCU is a dumb per-axis polynomial playback engine (`2026-05-27-simple-mcu-contract-design.md`): each axis owns a ring of 32-byte `PieceEntry` cubic-Bézier fragments and plays them by absolute MCU-clock `start_time`. The wire surface is `ConfigureAxis` (allocate per-axis rings), `PushPieces` (append pieces), and `StatusHeartbeat` (per-axis monotonic `consumed_counts`).

The host never caught up: the dispatch closure still builds the old `McuPushPlan` (via `build_push_params` + `split_plan_if_needed`) and does nothing with it — the `LoadCurveCubic` / `PushSegment` wire path it targeted is gone. So no pieces reach the MCU. This spec replaces that layer with: flatten each shaped segment into per-axis absolute-timed pieces, then stream them in strict time order, flow-controlled by the heartbeat.

### 1.1 Facts the design rests on (verified against code)

- **`PieceEntry`** (`piece_ring.rs:305`): `start_time: u64` (clock cycles), `coeffs: [f32; 4]` (Bernstein), `duration: f32` (seconds), `_reserved: u32`. 32 bytes.
- **Idle = hold.** `engine.rs::get_position_and_velocity` returns `None` when an axis has no current piece (or the next hasn't started); the tick loop `continue`s, `p_prev` untouched, no steps. An axis with no piece freezes — it does **not** snap to zero. (Covers genuinely pieceless axes, e.g. E on a travel move.)
- **Every segment carries X/Y/Z.** `emit_shaped` always builds `ShapedSegment.axes[0..2]`; a non-moving axis gets a *constant* curve → one constant piece. E is separate and may be absent.
- **A late piece aborts the print.** When `now − start_time > FAULT_TOLERANCE` (2 ticks; `engine.rs:654`), the MCU faults and the print stops (via foreground Klipper `shutdown()` — not a clean instantaneous engine stop, and uncoordinated across MCUs, but the end state is "stopped, aborted"). The mechanism is not this spec's concern; the obligation is to never let underrun happen (§5.1).
- **Wire send exists, but blocks.** `KalicoHostIo::kalico_call(MessageKind::PushPieces, body, timeout)` sends one frame and awaits `PushPiecesResponse`; `PushPieces` already has an `Encode` impl. It is a blocking round-trip (`host_io/mod.rs:780`) — see §3.5.
- **Heartbeat is currently DROPPED.** `StatusHeartbeat` decodes as a struct but `lift_event_to_runtime_event` (`kalico_native.rs`) routes it to the `_ =>` discard arm; there is no `RuntimeEvent::Heartbeat`. Routing it is new work (§3.4), and skipping it is a guaranteed halt: with `consumed` frozen, `room` hits 0 after the first `ring_depth` pieces and the pump stalls forever while heartbeats are thrown away.
- **Clock sync already exists — we consume it, we don't build it.** A dedicated subsystem maintains each MCU's clock estimate `(clock_offset, clock_freq, last_clock)` in the router, fed by both klippy's clocksync (`set_clock_est`) and the bridge's own periodic `runtime_clock_sync_request` driver (`spawn_periodic_clock_sync` → `ClockSyncEstimator` → `set_clock_est_from_sample`, `bridge.rs:153`). All MCUs share **one** host clock (`router.clock`, a single `Arc<dyn Clock>`), and `clock_offset` is expressed in that same host-time domain. Projecting a host time `t_host` to an MCU's clock is `last_clock + (t_host − offset)·freq`: `compute_ack_clock(mcu)` (`router.rs:422`) does it for `t_host = now`; `host_time_to_mcu_clock(mcu, t_host)` (`router.rs:437`) for arbitrary `t_host` (live math — only the binding is currently test-only). §3.2.1 adds no clock machinery; it anchors one shared host-time instant and projects it per-MCU through this existing estimate.
- **`consumed` is a wrapping `u32`** (`piece_ring.rs:64,160`). Host `room` and `pushed` must use matching wrapping `u32` (§3.3).

## 2. Core model

After shaping + C¹ refit, each axis is a chain of cubic Bézier pieces, each exact on its absolute-time interval `[u_start, u_end]`. Axes are refit independently, so piece boundaries do **not** align across axes — and nothing requires they do. A cubic piece can be *split* exactly (de Casteljau) but never *merged* across a boundary; this design does neither — it ships the refit's pieces unmodified.

No common time grid is needed because the only cross-axis coordination is *push order* (§3.3), not piece cuts. Pause and cancel both live above piece generation (§8) and don't depend on aligned boundaries.

### 2.1 Source: the committed dispatch stream is append-only by construction

The host does not receive finished moves. The streaming shaper dispatches only up to a moving commit boundary `target = t_decel_start − max_h` (`streaming/emit.rs:82`); the trailing decel ramp past it is **speculative** — a later replan may rewrite it, or the quiescence timer commits it via `commit_decel_to_zero` (`streaming/emit.rs:241`). Both commit paths emit through the **same dispatch closure** this spec consumes.

> **The pump's only input is the committed dispatch stream (the drained output of `emit_committed` / `commit_decel_to_zero`). It holds no handle to `ShaperState` internals; the speculative tail is unreachable from it.**

So append-only correctness is structural, not a discipline: the only pieces that exist to push are already committed, hence immutable. The wire has no "replace piece N" and needs none. This pre-empts any future "let the pump peek at the planner buffer to deepen lookahead" optimization — that would reintroduce retraction and must be refused at this boundary.

Lifecycle consequence: "all queues drained to the boundary" is **not** end-of-motion. The zero-velocity stop ramp arrives later (as the quiescence commit's segments, latest in time → pushed last). The pump stays alive across a drained-to-boundary state.

## 3. Components

### 3.1 Ring allocation — `ConfigureAxis` (startup)

Config-time bookkeeping, independent of everything else. Per MCU:

- `total_pieces = total_piece_memory / 32` (from `RuntimeCapsResponse`).
- `ring_depth = total_pieces / num_axes_on_mcu` (equal division across the axes that MCU drives).
- Send one `ConfigureAxis` per axis with that `ring_depth` (stepping-mode / microstep / stepper-binding fields unchanged).

It allocates a ring for **every** axis the MCU drives, regardless of whether that axis moves in any given print. The resulting depth must clear the steady-stream refill budget (§5.1 budget 1): `ring_depth × typical_piece_duration` must comfortably exceed the heartbeat RTT. Defaults (≈496 pieces/axis on H7 ≈ 0.5 s, vs 100 ms heartbeat) clear it ~5×; a board too shallow to cover `max_h` + heartbeat RTT is a startup config error, not a runtime degradation.

### 3.2 Enqueue adapter (per `ShapedSegment`, planner thread)

Replaces `build_push_params` + `split_plan_if_needed` + `McuPushPlan`. For each segment:

1. **CoreXY transform** (relocated from `dispatch.rs`): for a CoreXY MCU driving X and Y, combine into motor-frame A = X + Y (slot 0), B = X − Y (slot 1) via `nurbs::algebra::add_with_knot_union`. Other axes pass through.
2. For each axis the segment provides a curve for, `extract_bezier_pieces` → per piece:
   - `start_time = project(mcu, T0 + u_start)` — the piece's host time projected into this MCU's clock (§3.2.1).
   - `duration = u_end − u_start` (f32 seconds), `coeffs =` the four Bernstein points (f32), `_reserved = 0`.
   - Append in time order to that `(mcu, axis)` queue.

No segment IDs, handles, slots, or sub-splitting. **No static/skip judgment**: a non-moving axis's constant curve yields one piece, sent like any other (position continuity carried in the piece, not dependent on idle-hold); an axis with no curve yields nothing. The pump never inspects piece content.

#### 3.2.1 Absolute `start_time` and the shared anchor

The MCU compares `start_time` only against absolute `now` (earlier → idle, too-late → fault, §1.1). There is no relative / ASAP / "0 = now" encoding; `0` is just a past clock value → instant fault. The host always stamps a real future absolute clock, and pieces are self-contained (contract §4.3).

The host holds **one shared anchor `T0`** — a single **host-time** instant (in the router's shared clock domain, §1.1) mapping planner `t = 0`. Every piece's `start_time = project(mcu, T0 + u_start)`, projected through that MCU's existing clock estimate (§1.1). Because `T0` lives in the shared host domain and only the projection is per-MCU, all MCUs begin a given segment at the same real moment **within clock-sync error** (sub-ms — fine for motion). This is the *only* new clock requirement: anchor **one** shared `T0` and project it, rather than the old path's per-MCU independent anchoring (separate `compute_ack_clock` + lead + rebase per MCU, which let MCUs drift apart). The clock-sync subsystem itself is reused as-is.

**Anchoring rule** (compares planner timestamps to each other, never to wall-clock — the host does no real-time checks):

- The adapter remembers the previous segment's planner `t_end`.
- **Contiguous** (`t_start ≈ last_t_end`): keep `T0`. `N+1.start = N.start + N.duration` falls out.
- **Reset** (`t_start` jumps backward to ~0): capture the shared host clock once as `host_now = router.clock.now()` and set `T0 = host_now + lead − t_start` (host seconds; `lead ≈ 0.25 s`, the existing value). The new stream's first segment then lands `lead` ahead, synchronously on every MCU via the per-MCU projection.

A backward jump in planner time is the unambiguous "fresh stream" signal, and it covers every reset uniformly: explicit `reset()` (stream-open, set_position, homing, underrun, force_idle, reconnect) and the quiescence reset all return the timeline toward 0.

**Planner-side dependency** (trajectory crate — outside this spec's host scope, but required): `commit_decel_to_zero` must reset the timeline toward 0 on a true stop (reseed at the stop position, zero the time cursors). Today it only advances the cursors, so idle→motion cumulates silently and a frozen anchor would stamp the next move in the past → fault. With the reset, "fresh stream ⇔ timeline at 0" is a single invariant. (If the planner wants continuity across a hold instead, it emits stationary pieces and the timeline stays contiguous — the host needs no special case either way.)

### 3.3 Pump (async — the real new logic)

A single global pusher, woken by new-pieces and by heartbeat. It can't live in the per-segment closure because it must react to heartbeats arriving between segments.

Per `(mcu, axis)` it tracks: the queue of unpushed pieces, `pushed: u32`, `consumed: u32` (from heartbeat), and `ring_depth`. `room = ring_depth − (pushed −_wrap consumed)`, **wrapping `u32`** (§1.1). `pushed` increments when pieces are committed to the wire (room reserved then), not on response — host accounting is authoritative.

These counters track the MCU's **monotonic** ring counter and are **independent of the time re-anchor** (§3.2.1): a planner timeline reset does not reset them. At an idle→motion re-anchor the ring has drained, so `consumed ≈ pushed` already and `room` is full — no reset needed, and zeroing `pushed` against the still-climbing MCU counter would corrupt `room`. They reset only on an actual MCU ring reset (epoch change from MCU restart/reconfigure, where `consumed` genuinely returns to 0).

**Scheduling — strict global start-time order, stall on a full head:**

```
loop:
  head = (mcu, axis) whose next unpushed piece has the smallest start_time
  if no head:            wait for new pieces; continue
  if room(head) == 0:    wait for a heartbeat; continue   # DO NOT push anything else
  batch = longest prefix of global time order on head.mcu with room remaining
  send PushPieces frame(s) for batch   # one frame per axis
```

The stall is the safety property: a non-consuming MCU fills its ring, the global head lands on it, everyone waits — no MCU can run ahead of a stuck one. Halting on a hung MCU is the correct outcome (cancel/shutdown is a higher layer).

**Batching** cannot break ordering: the batch is always a contiguous prefix of the global order that happens to be on one MCU, split into one frame per axis (`axis_idx, piece_count, pieces[]`). It ends when the next piece is on a different MCU or a ring hits `room == 0`.

**Tie-break** on equal `start_time` (common: CoreXY A/B are co-timed): deterministic `(mcu_id, axis_idx)`. Any fixed order is correct — co-timed pieces on different rings are independent; determinism is only for reproducibility.

Effective buffer depth is governed by whichever ring fills first *in time* — the axis with the most pieces/s (X/Y under shaping), not the idle ones. Automatic and intended.

### 3.4 Heartbeat handling (new routing)

`StatusHeartbeat` is currently discarded (§1.1). This work:

1. Adds a `MessageKind::StatusHeartbeat` arm to `lift_event_to_runtime_event` that decodes the body instead of returning `Ignored`.
2. Routes it to the pump — a dedicated heartbeat channel into the pump (per §3.5), since heartbeats are pump-private and needn't surface to general runtime-event consumers.

On each heartbeat the pump updates per-`(mcu, axis)` `consumed` and re-evaluates the stall (a freed ring may unblock the head).

### 3.5 Pump concurrency

Three actors touch ring state: the planner thread (enqueue), the pump, and the heartbeat handler (reactor path). Two non-negotiables:

1. **Never hold a lock across the blocking `kalico_call`** (`host_io/mod.rs:780`) — it would serialize the planner and heartbeat handler behind wire RTT and starve the very heartbeats the pump depends on.
2. **The dual wakeup must be lost-wakeup-safe** — a naive condvar can miss a wake firing between the stall check and the wait.

**Model: the pump owns the queues and all accounting, fed by channels.** Planner sends pieces over one channel; the heartbeat handler sends `(mcu, axis, consumed)` over another. The pump selects over both and mutates only its own state — nobody blocks on the pump, the lost-wakeup problem becomes channel readiness, and the only blocking call (`kalico_call`) holds nothing the others need.

## 4. Wire send

Per frame: `kalico_call(MessageKind::PushPieces, body, timeout)`; `body` is `axis_idx: u8, piece_count: u8, pieces[count × 32]` via the existing `Encode`. Response `PushPiecesResponse`.

**Result codes** must be shared named constants (host ↔ MCU), defined once in `kalico_protocol` and referenced from both sides so a renumber can't desync: `OK` / `RING_FULL` / `INVALID_AXIS` (MCU returns these from `handle_push_pieces`, cf. `KALICO_ERR_*` in `kalico_dispatch.c`). Normal operation is always `OK`; `RING_FULL` is the safety net.

**Throughput (CLAUDE.md constraint #1).** A serial blocking pump caps at `1/RTT` *frames*/s per MCU. With batching the rate is frames, not pieces: ~0.5–1 ms USB RTT → ~1000–2000 frames/s, vs shaped XY emitting hundreds–low-thousands of pieces/s aggregate — coalesced frames keep a serial pump comfortably ahead for bring-up. Validate this number on the bench before declaring the path adequate. If a future workload approaches the ceiling, pipelining (fire-and-forget + response reconciliation) is the lever — a throughput tweak, not a design change.

## 5. Timing

- **Initial lead.** A fresh stream's first piece must be far enough ahead to load before the ISR reaches it — covered by the anchor's `lead ≈ 0.25 s` (§3.2.1) plus earliest-first pushing.
- **All axes arrive together.** Each `ShapedSegment` covers all axes over one `[t_start, t_end]`, so the pump always has a complete timeline up to the latest enqueued segment.

### 5.1 Underrun is a halt — two budgets, ring depth covers only one

If the ring drains to the committed frontier, the axis idle-holds — but the frontier sits at `t_decel_start − max_h`, mid-cruise at nonzero velocity, so the freeze is an infinite-jerk stop; and when the held-back ramp finally arrives it's now in the past → `PIECE_START_IN_PAST` (§1.1). This is a halt-class invariant inherited from the streaming shaper; we don't fix it here, but two budgets must hold:

1. **Steady-stream refill** (moves keep coming): each replan advances `t_decel_start`, so the frontier moves; the only risk is heartbeat-latency refill lag. **Binding: ring-depth-in-time > heartbeat RTT** (the §3.1 check). The *only* underrun ring depth defends.
2. **Stop / dwell / starvation:** the frontier stalls at the last move's boundary until quiescence fires `commit_decel_to_zero`. **Ring depth does nothing** — there's no piece past the frontier to push. Binding budget is planner-side: `T_commit + emit + push + RTT` < buffered time at the boundary, floored by `FAULT_TOLERANCE`.

Plain version: deep rings are a bigger tank that rides out a slow refill, but at a stop the tap is closed until the planner commits the decel — a bigger tank just delays noticing. Budget 2 is a planner concern, named here only so the dependency is explicit.

## 6. Removal (folded in) — unwiring a live path

The credit / slot machinery is **inert, not dead**: the MCU stopped emitting `CreditFreed`, so it never fires, but every link is compiled in — `attach_credit_counter` (`host_io/mod.rs:543`, `bridge.rs:2011`), `ReactorCommand::AttachCreditCounter` (`reactor.rs:1113`), `EventDispatcher::on_credit_freed` (`events.rs:268`), plus `credit/tests.rs` and `events/dispatch_tests.rs`. Removal must unwire before (or with) deleting types, or the build breaks mid-removal.

Host-side:

- `dispatch.rs`: `build_push_params`, `split_plan_if_needed`/`split_recursive`, `McuPushPlan`, `SegmentPushParams` refs, `UNUSED_HANDLE`/`set_handle`, `is_trivially_constant`. Keep CoreXY transform (relocated). `de_casteljau_split`/`extract_time_window` become unused → remove.
- `producer.rs`: `CurveLoadParams`, `SegmentPushParams`, unused helpers.
- `cap_check.rs`: `fits_curve_load` (`:19`) — another `CurveLoadParams` consumer; removing the type without this dangles.
- `credit.rs` + wiring (`attach_credit_counter`, the reactor command, `CREDIT_SEED_CAPACITY`).
- Slot pool (`SharedSlotPool`) + retirement wiring (`attach_retirement_callback`, `on_credit_freed`, `retire_through_segment`).
- `e_mode` / `extrusion_ratio` on the dispatch path (§6.1).
- The hardcoded `total_pieces() / 4` (`bridge.rs:2021,2365`) → per-MCU `num_axes` division (§3.1).

Removal leaves the build green with the new path in place — not delete-then-rebuild.

### 6.1 E is a follower ratio, not a curve

`emit_shaped` carries E as `extrusion_per_xy_mm`, not a Bézier curve on an E axis. So "the planner emits E pieces like any axis" is the target, not today's reality: travel moves work (E has no curve → nothing enqueued); extruding moves produce no E motion. The fix — arc-length integration of shaped XY into an E position curve (the host half of "E follows XY") — is its own spec. §9 success criteria are **motion-only**; no printable extrusion claim.

### 6.2 Cross-spec sequencing

The credit subsystem spans this spec (host) and the companion MCU-side removal spec. Sequence the merges so neither lands a dangling reference — the host unwiring must not assume MCU-side `CreditFreed` removal, or vice versa.

## 7. Unchanged

- Planner, shaper, TOPP-RA, and per-segment `ShapedSegment` emission — segments stay a planner concept, never a wire concept.
- `ConfigureAxis`'s stepping/microstep/binding payload; clock-sync and the planner-epoch mapping.
- MCU firmware — it already implements `ConfigureAxis` / `PushPieces` / `StatusHeartbeat`. (The §3.2.1 quiescence-reset is a host-adjacent *planner* change, not firmware.)

## 8. Deferred

- **Flush:** host-side only — block until `consumed == pushed` on all axes. Gradual stop (plan-to-zero) lives above piece generation. No MCU message.
- **Cancel + cross-MCU homing:** a generic `Cancel` wire command, the trsync→engine seam, host relay of a trip to all MCUs, and host reconstruction of position from the retained piece polynomials at the trip time. The piece stream here is its prerequisite.

## 9. Testing

**Scope: motion only** (no extrusion, §6.1).

Bench (motion directly observable):

- Travel move on CoreXY → motion on X/Y MCU; Z gets one constant piece/segment (held); E silent.
- Move with Z → Z pieces on F4 while X/Y stream to H7.
- **Sustained streaming past the initial ring fill** → both MCUs stay fed, neither races ahead nor halts. Exercises the heartbeat-routing fix (§3.4) — without it motion stops when the first ring drains. (Original bug + new first-light risk in one test.)
- Clean stop → decel to zero; quiescence ramp arrives and is pushed; no `PIECE_START_IN_PAST`.
- Jog after an idle gap → no delay and no past-piece fault (exercises the §3.2.1 reset/re-anchor).

Unit / integration:

- Enqueue adapter: moving axis → time-ordered pieces with correct absolute `start_time` + `duration`; non-moving axis → one constant piece; CoreXY A/B = X±Y (transform before extraction).
- Anchor: contiguous `t_start` keeps `T0`; backward jump establishes a fresh `T0` landing at `now + lead`; same `T0` across MCUs.
- Pump: strict global order; stall on full head (no skip-ahead); same-MCU coalescing; no send when `room == 0`; deterministic tie-break.
- Room accounting: wrapping-`u32` correctness across a wrap (test the wrap directly).
- Heartbeat routing: a `StatusHeartbeat` reaches the pump and advances `consumed` (regression guard against the `_ =>` discard).
- Result codes: host/MCU agree on shared `OK`/`RING_FULL`/`INVALID_AXIS`.
- Ring allocation: divides by actual per-MCU `num_axes`, not `/4`.
- Concurrency: pump holds no shared lock across `kalico_call` (assert via the channel-owned model).
