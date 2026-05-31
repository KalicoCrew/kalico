# Monotonic planner clock: never rewind, never re-anchor

**Date:** 2026-05-30
**Status:** Design — pending implementation
**Branch:** `simple-mcu-contract`
**Supersedes:** `2026-05-30-idle-reanchor-design.md` (Choice B — reset-and-re-anchor)

## Problem

A jog followed by an idle gap and a second jog hard-faults the MCUs with
`-308 PieceStartInPast` ("the ISR reached a piece whose start_time is more than 2
ISR ticks — ~50 µs — in the past; the MCU was not fed in time"). The same fault
reproduces after `M400` followed by a new move.

The previous fix (`2026-05-30-idle-reanchor-design.md`, Choice B) rewound the
planner timeline to 0 on quiescence and re-established `t0`, so the next move
would re-anchor to `host_now + LEAD`. It did not work, and the on-hardware traces
(`klippy.log.2026-05-30_20-13-27`, journalctl `[idle-reanchor]`/`[anchor]`) show
why.

### Root cause

The re-anchor fires on a **50 ms quiescence timer** (`T_COMMIT`), but pieces are
scheduled **`LEAD` = 250 ms ahead**. So 50 ms after a move is submitted, its
motion is still in the MCU's *future* — the toolhead has not started, let alone
stopped. `T_COMMIT` commits the decel-to-zero tail to the wire **and resets the
planner clock** while those pieces are still pending. When the next move arrives
within ~one move-duration and re-anchors to `host_now + LEAD`, that lands its
first piece *behind* the previous move's committed tail on the MCU clock:

```
jog4: host_now=158.451  → t0 = 158.701  → committed pieces occupy MCU-time [158.701 … 158.984]
      T_COMMIT fires (flushes jog4 decel tail), clock reset
jog5: host_now=158.565  → re-anchor t0 = 158.815  → first piece at 158.815
      158.815 < 158.984  → 168 ms behind jog4's committed tail → -308
```

The decode confirms it: `fault_code=65228=0xFECC`, `(0xFECC as i16) = -308`;
`fault_detail=0x20000` → axis 2 (a *held* axis, adopted first by the ISR).

The deeper defect: re-anchoring resets `seg_t_start` **backward** to 0 and
compensates by moving `t0` forward. That backward jump in planner-time is a
**monotonicity violation** — the original `Anchor` guaranteed piece stamps only
ever increase by holding `t0` fixed and letting planner-time march forward.
`-308` is fundamentally a "MCU was not fed in time / this piece is in the past"
guard; the re-anchor manufactures exactly that condition whenever the MCU buffer
has not actually drained.

The spec's load-bearing false premise (`2026-05-30-idle-reanchor` § Behavioral
notes) was: *"T_COMMIT firing already means … the toolhead stopped."* In
MCU-clock terms it has not — `T_COMMIT` (50 ms) ≪ `LEAD` (250 ms).

## Intended architecture

Stop treating planner-time as a resettable accumulator. Make it a **monotonic
clock**: seconds since the stream's sync origin, never paused, never rewound on
idle. `t0` is established **once** per stream and never re-anchored. A move
self-places with one rule; the held-back decel and Flush become clock-derived;
the MCU's retirement messages are reserved for their real job (ring-buffer flow
control), not timing.

This makes the entire `-308`-on-re-anchor fault class **structurally impossible**:
if planner-time only ever advances, a piece stamp can never land behind one
already on the wire.

### Why the clock is trustworthy (the load-bearing fact)

The planner thread's monotonic clock and the projection's host-time input are the
**same OS monotonic clock**:

- `host_now_secs()` = `instant_to_f64(self.clock.now())` — raw monotonic seconds
  off a process-lifetime anchor; "only deltas are meaningful"
  (`rust/kalico-host-rt/src/passthrough_queue/router.rs:114-123, 436-438`). It is
  **not** MCU-synced print time.
- `host_time_to_mcu_clock(host_secs)` maps that value to MCU ticks via
  `(host_secs − clock_offset) * clock_freq` (`router.rs:443-455`), where
  `clock_offset`/`clock_freq` are the **continuously re-estimated** host↔MCU
  linear fit.

Therefore the host↔MCU crystal skew (`SYNC_ERROR`) lives entirely in the
projection coefficients and is re-grounded on every projection. Planner-time
never carries it; `elapsed_since_sync` measured on the planner thread *is* the
projection's host-time frame, at the same rate, with zero drift between them by
construction.

### Precision under an unbounded clock (verified safe)

Planner-time now tracks wall-clock for the whole session (hours → thousands of
seconds). This is safe: **no f32 quantity carries absolute planner-time.**

- `PieceEntry.start_time` is `u64` absolute MCU ticks, computed `t0 + u_start` in
  **f64** then `* clock_freq` → `u64` (`enqueue.rs:160`, `router.rs:452`).
- `coeffs: [f32; 4]` are **mm-space positions** (bed-bounded; ~15 µm ulp at
  150 mm) — do not scale with session length.
- `duration: f32` is an f64 subtraction `u_end − u_start` cast after the fact —
  **piece-local** (~ms), magnitude-bounded.

At planner-time ≈ 3700 s, f64 ulp ≈ 0.45 ps vs the ~50 µs `-308` tolerance — a
margin of ~10¹¹. And the projection's `delta` operand is bounded by the ~500 ms
clock-sync refresh window, so the f64→u64 cast keeps sub-tick precision.

## Design

### A. Move placement — one rule

A move's start in planner-time is:

```
seg_t_start = max(t_appended, elapsed_since_sync)
```

- **Queued-ahead** (`t_appended > elapsed_since_sync`): `t_appended` wins →
  contiguous **blend**. This is continuous printing and rapid jogs: the new move
  continues the existing trajectory at the current dispatch cursor.
- **Drained** (`elapsed_since_sync > t_appended`): the clock wins → the move
  lands at "now", i.e. `host_now + LEAD` via the unchanged `Anchor`. This is a
  genuine idle.

The crossover *is* the drain point, computed from the planner's own clock with no
router round-trip. `elapsed_since_sync = sync_instant.elapsed()` where
`sync_instant` is captured at the stream's first dispatch (see § F).

The drained branch advances by inserting a **rest-hold**: bump `t_appended` up to
`elapsed_since_sync` with a degenerate "park at current position, v = 0" segment
(host-side only — not dispatched; the MCU simply stays put). Because the toolhead
is genuinely at rest, the shaper history window stays valid with **no reseed** —
the hold *is* the `[−h, 0]` rest extension that the old `reset()` had to rebuild.

### B. Decel-commit safety timer — clock-derived

The held-back decel-to-zero must still reach the MCU before the on-wire buffer
drains (otherwise the MCU underruns at the last cruise piece). The MCU begins
planner-time 0 at `elapsed_since_sync = LEAD` and plays forward 1:1, so its
playhead is `elapsed_since_sync − LEAD`; the on-wire buffer (ending at
`t_dispatched`) drains at `elapsed_since_sync = t_dispatched + LEAD`. The commit
deadline sits `SAFETY_MARGIN` before that, computed from the planner's own clock:

```
commit_deadline (elapsed_since_sync) = t_dispatched + LEAD − SAFETY_MARGIN
next_recv_timeout = (t_dispatched + LEAD − SAFETY_MARGIN) − elapsed_since_sync
```

- A follow-on move **before** the deadline blends (§ A). The held-back tail is
  **host-side only** — `emit_committed` holds everything past
  `t_decel_start − max_h` in `planned_fitted`/`axes[i].pieces` and never passes it
  to the dispatch closure (`emit.rs:82-179`); `append_and_replan` →
  `replace_uncommitted_axis_pieces` overwrites it in place (`state.rs:738-775`).
  Nothing on the wire to retract.
- **Silence past** the deadline commits the decel-to-zero (`commit_decel_to_zero`,
  `emit.rs:241-342`) → the toolhead stops cleanly. After the commit,
  `t_dispatched == t_appended`.

This replaces the fixed 50 ms `T_COMMIT`-since-last-append timer. As long as moves
keep arriving and extending `t_dispatched`, the deadline keeps pushing out and the
decel never commits — continuous motion, no spurious stops (including no spurious
mid-print stop on a transient sub-`LEAD` host stall).

### C. `t0` and the `Anchor` — established once, never re-anchored

`t0` is set on the stream's first dispatch (`Anchor::anchor_segment` with
`t0 == None`) and held for the life of the stream. Because § A only ever pushes
`seg_t_start` forward, the `Anchor` never sees a backward jump during normal
operation and never re-anchors. **`anchor.rs` is unchanged** — its backward-jump
branch now serves only the genuine resets in § D.

### D. Genuine resets are retained

`ShaperState::reset()` still runs on **stream-open, homing, `SET_KINEMATIC_POSITION`,
engine `Underrun`, `force_idle`, and klippy reconnect**. These truly drain the MCU
and restart the stream, so zeroing planner-time and re-establishing `t0` (and the
sync origin, § F) is correct there; the MCU queue is empty, so the resulting
backward jump cannot overlap committed pieces. **Only the idle re-anchor is
removed.**

### E. Flush / M400 — time governs *when*, counts govern *how full*

The MCU's per-axis retirement counts (10 Hz `StatusHeartbeat` 0x0083,
`CONFIG_CLOCK_FREQ / 10`, `src/runtime_tick.c:465`) have exactly one job:
**ring-buffer flow control**. That already lives in the pump — `AxisQueue` tracks
`pushed`/`retired`, computes `free_slots = ring_depth − (pushed − retired)`, and
stalls a full ring waiting for a heartbeat (`pump.rs:19-54, :88`). **This is
unchanged.**

Flush completion ("is the motion done?") becomes **time-based**:

```
on Flush (M400 / wait_moves):
    commit the held-back decel        # toolhead stops at t_appended
    wait until elapsed_since_sync ≥ t_appended + LEAD
    return
```

- `t_appended + LEAD` is the wall-clock instant the last committed piece finishes
  executing — known analytically the moment the decel is committed. No 10 Hz
  polling, no `retired == sent` barrier, no 60 s blind timeout.
- **Safety comes from the fault path, not retirement.** A stalled / lagging MCU
  surfaces independently as a `FaultEvent` / `-308` → shutdown. Absent a fault,
  motion runs on schedule and the clock deadline is exact. (Per the project's
  "fail loudly" rule, a genuine failure is a loud fault, not a silent timeout.)
- **No timeline reset.** The monotonic clock keeps advancing through the wait, so
  the next move self-places via § A and continues automatically. This deletes the
  never-implemented "Flush / M400 handled separately" gap (`planner.rs:753-759`)
  and kills the M400-then-move `-308` with the same mechanism as the idle case.

`DrainSync` is **not removed.** Its `add_sent` / `set_retired` accounting is the
pump's flow-control mechanism (untouched), and `set_position` (`bridge.rs:2737`)
still uses `wait_drained` for a real retirement barrier before re-seeding motor
state. What changes is only the **M400 completion semantics**: the clock wait
(`t_appended + LEAD`) is the authoritative "motion done" signal; the post-flush
`wait_drained` in `drain_motion` becomes redundant (it returns immediately once
the clock wait has passed) and may be kept as a belt-and-suspenders check or
dropped.

### F. Sync-origin pinning

The planner's `sync_instant` (an `Instant`) and the `Anchor`'s `t0` (an
`host_now_secs` value) must denote the **same logical moment** — the stream's
first dispatch — so that `t0 + seg_t_start ≈ host_now + LEAD` holds throughout.
Both read the same OS monotonic clock; the only residual is the sub-millisecond
skew between the planner thread capturing `Instant::now()` and the dispatch
callback reading `host_now_secs()`. That residual is absorbed by `LEAD` (250 ms).

On every genuine reset (§ D), `sync_instant` is re-captured at the next first
dispatch, in lockstep with the `Anchor` re-establishing `t0`.

### G. `SAFETY_MARGIN` sizing

`SAFETY_MARGIN` is the lead time the decel-commit (§ B) needs so the decel-to-zero
reaches the MCU before its on-wire buffer (`t_dispatched`) drains. It must cover
the worst case of: decel shaping + dispatch closure + pump enqueue + wire
transmission + one idle-gap's worth of `Instant`-vs-MCU skew. It must be
comfortably less than `LEAD` (so the commit lands ahead of drain) and large enough
to never underrun. **Proposed starting value: 50 ms**, to be validated on-hardware
against the bench MCUs; tune from the worst observed dispatch→wire latency.

## Out of scope

Two real smells surfaced by the Flush read, neither a timing-correctness issue;
keep them separate:

- **`wait_moves` returns at dispatch, not wire/MCU** (`bridge.rs:2378`,
  `planner.rs:875-912`). The `wait_moves` (non-M400) path signals completion when
  pieces are handed to the pump, not when sent/executed. Used by drip moves /
  dwell / inter-move barriers. Evaluate separately whether those callers need a
  stronger barrier.
- **The 60 s blind `DrainSync` timeout** (`drain.rs`). Removed implicitly when
  `DrainSync` is retired from the Flush path, but if any caller still needs a
  retirement barrier, it should get a clock-informed deadline rather than a fixed
  60 s.

## Files touched (anticipated)

- `rust/motion-bridge/src/planner.rs` — `run_loop`: replace the `T_COMMIT`
  fixed-timer arm with the clock-derived decel-commit deadline (§ B); add the
  `max(t_appended, elapsed_since_sync)` placement + rest-hold advance (§ A);
  capture/re-capture `sync_instant` (§ F); rewrite the `Flush` arm as the
  time-based wait (§ E); remove the idle reset.
- `rust/trajectory/src/streaming/state.rs` — add the rest-hold advance primitive
  (`advance_idle`, § A), reusing the existing `axis_position_at` settled-position
  read. `current_position()` is **retained** (public API with tests; its
  underlying read is what the rest-hold reuses).
- `rust/motion-bridge/src/drain.rs` — **unchanged.** `DrainSync` stays (pump
  flow-control accounting + `set_position`'s barrier). Only M400's completion
  semantics move to the clock (§ E).
- `rust/motion-bridge/src/bridge.rs` — `wait_moves` / `drain_motion` re-pointed
  at the time-based Flush; heartbeat callback wiring for flow control unchanged.
- `rust/motion-bridge/src/anchor.rs` — **unchanged** (backward-jump branch now
  serves only genuine resets).
- `rust/motion-bridge/src/planner.rs` + `anchor.rs` — remove the temporary
  `[idle-reanchor]` / `[anchor]` diag traces (commit 381e8f7eb).

## Testing

- **Unit (placement):** `max(t_appended, elapsed_since_sync)` selects blend when
  queued-ahead and advance-to-now when drained; the advance inserts a rest-hold
  that leaves position continuous (no teleport) and the shaper history valid.
- **Unit (decel deadline):** with moves arriving before
  `t_dispatched − SAFETY_MARGIN` no decel commits (continuous); with silence past
  it the decel-to-zero commits exactly once and `t_dispatched == t_appended`.
- **Unit (monotonicity):** across an idle gap then a new move, every emitted piece
  stamp is ≥ the previous — never a backward jump. This is the regression guard
  for the original fault.
- **Unit (Flush timing):** Flush commits the decel and returns no earlier than
  `elapsed_since_sync == t_appended + LEAD`; a subsequent move self-places via § A
  with a stamp `≥ host_now + LEAD` (not in the past).
- **Unit (flow control untouched):** pump `free_slots`/stall behavior under
  `pushed`/`retired` is unaffected.
- **Hardware:** the two-jog repro (`SET_KINEMATIC_POSITION`; jog `Y=-1`; pause;
  jog `Y=-25`) and rapid same-axis jogs (the 114 ms-gap sequence from
  `klippy.log.2026-05-30_20-13-16`) complete without `-308`; `M400` followed by a
  new move completes without `-308`. Build + flash both MCUs per the bench flow.
