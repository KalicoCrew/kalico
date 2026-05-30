# Idle re-anchor: rewind the planner clock on quiescence

**Date:** 2026-05-30
**Status:** SUPERSEDED by `2026-05-30-monotonic-planner-clock-design.md` — the
reset-and-re-anchor approach (Choice B) did not work on hardware; the fault and
the corrected architecture are analyzed in the successor spec.
**Branch:** `simple-mcu-contract`

## Problem

A jog followed by an idle gap and a second jog hard-faults the MCUs with
`-308 PieceStartInPast` ("ISR reached a piece whose start_time is more than 2
ISR ticks in the past — MCU was not fed in time").

Observed on-hardware (`klippy.log.2026-05-30_11-29-33`):

| t (host, s) | event |
|---|---|
| 86116.03 | `SET_KINEMATIC_POSITION X=150 Y=150 Z=50` |
| 86117.57 | jog #1 `_CLIENT_LINEAR_MOVE Y=-1 F=6000` → **works** |
| 86119.84 | jog #2 `_CLIENT_LINEAR_MOVE Y=-25 F=6000` → **fault** |

Both MCUs latched `-308`: `bottom` (F446) on `fault_detail=0x20000` → axis 2 (Z),
`mcu` (H723) on `fault_detail=0` → axis 0 (X). The faulting axes are the *held*
(non-moving) axes; the move itself was Y-only. The fault decode: wire
`fault_code=65228 = 0xFECC`, and `(0xFECC as i16) = -308 = FaultCode::PieceStartInPast`;
`fault_detail = (axis_idx & 0xFF) << 16`.

### Root cause

The host maps planner-time → MCU clock through `Anchor` (`rust/motion-bridge/src/anchor.rs`):
a piece at planner-time `u` is scheduled at host wall-clock `t0 + u`, then projected
onto the MCU clock. `t0` is established once per stream and only re-established on a
**backward** planner-time jump.

The planner timeline (`ShaperState::t_appended` and the per-segment `t_start`/`t_end`)
is a monotonic accumulator. It is zeroed only by `ShaperState::reset(...)`, which today
runs on `kalico_stream_open`, homing / `SET_KINEMATIC_POSITION`, engine `Underrun`,
`force_idle`, and klippy reconnect — **never on a plain idle gap between moves.**

So:

```
jog1: first move → t0 = host_now + LEAD - seg_t_start  → pieces land LEAD ahead. OK.
      planner_time advances 0 → ~0.05
[idle 2.3 s — planner_time stays ~0.05, host_now races to 86120.1]
jog2: forward-contiguous (seg_t_start ≈ 0.05 ≥ last_t_end), so NOT a backward jump.
      stale t0 reused → piece scheduled at (t0 + 0.05) ≈ 86118.05, but host_now ≈ 86120.1
      → ~2 s in the MCU's past → PieceStartInPast on every axis as the ISR adopts it.
```

`LEAD = 0.25 s`; the MCU tolerance is 2 ISR ticks (~50 µs at 20–40 kHz), so a ~2 s
lag faults deterministically. Jog #1 works only because it is the first move after
stream open (`t0 == None` → fresh). The held axes fault rather than Y because the
engine checks axes in index order and latches on the first stale one it adopts.

## Intended architecture

After an idle, the host must schedule the next move the **same way as the first
move**: re-anchor to `host_now + LEAD`. The mechanism already exists — the `Anchor`'s
backward-jump branch — but nothing rewinds the planner timeline to *trigger* it on
idle. The fix restores that trigger.

## Approaches considered

- **A — Re-anchor inside `Anchor`.** Add a second `fresh` condition: re-anchor when
  the projected start has fallen behind `host_now` (`t0 + seg_t_start < host_now`).
  Smallest change, but detects idle indirectly ("did the schedule fall behind reality?")
  and would also fire under mid-print host starvation, silently inserting a hold.
  Rejected: hacky, conflates idle with falling-behind.
- **B — Rewind the planner clock on quiescence (chosen).** When the quiescence-commit
  fires (toolhead already decelerated to a stop), zero the planner timeline so the next
  move is a backward jump and the *existing* `Anchor` logic re-anchors it. Matches the
  documented original intent (the `backward_jump_reanchors` test comment: "timeline reset
  to ~0 after a long idle").
- **C — Re-anchor early in `Anchor` with a margin** (`< host_now + LEAD`). Same family
  as A, same objection.

B is chosen: it puts idle detection where idle is actually known (the quiescence timer)
and reuses the correctly-shaped `reset()` primitive instead of adding a second,
indirectly-triggered code path.

## Design (Choice B)

### Trigger: the `T_COMMIT` quiescence timer

The planner run-loop (`rust/motion-bridge/src/planner.rs`) arms a 50 ms quiescence timer
(`T_COMMIT`) after every `Move`. When it fires (no follow-on move arrived), the existing
`run_commit_and_dispatch` flushes the held-back decel-to-zero tail to the wire — i.e. the
toolhead is now physically stopped — and the loop sets `last_append_time = None`.

This is the path the buggy jog-then-jog sequence takes (jog #1's decel committed via
`T_COMMIT` well before jog #2 arrived). It is the only path in scope.

### Action: `state.reset(current_position)` after the commit

```
on T_COMMIT fire:                       # planner.rs run-loop, RecvTimeoutError::Timeout arm
    run_commit_and_dispatch(...)        # EXISTING: flush decel-to-zero tail; toolhead stopped
    let pos = state.current_position()  # NEW accessor (see below)
    state.reset(pos)                    # EXISTING primitive: zero the timeline, reseed at pos
    last_append_time = None             # EXISTING
```

After this, the next `Move` arrives with `seg_t_start == 0`. The `Anchor` sees
`0 + CONTIGUITY_EPS < last_t_end` → backward-jump branch → re-anchors to
`host_now + LEAD`. **No change to `Anchor` is required.**

### Why `reset()` (and why it needs a position)

Rewinding the clock is conceptually position-free. But the smooth shaper keeps a rolling
**history window** of the last `h` seconds of motion (`h` = kernel support), and those
past pieces are stored in the axis queues **stamped in the same time frame as the clock**
(`ShaperState::new`/`reset` seed each axis with a `(pos, v=0)` rest extension covering
`[-(h + δ_safety), 0]`). Zeroing the clock to 0 orphans that window — its timestamps no
longer line up with the new origin — so it must be rebuilt as "parked at `pos`, v=0 for
the last `h` seconds." That rebuild is the one step that needs the position.

`ShaperState::reset(home_pos)` already does exactly this bundle (zero
`t_appended`/`t_decel_start`/`t_shaped`/`t_dispatched`, clear `uncommitted_moves` /
`pending_dispatch` / `planned_*`, reseed each axis at `home_pos`). It is the correctly
shaped primitive; a narrower "clock-only" rewind would leave a stale history window and
corrupt the shaping of the first post-idle move.

### New code: `ShaperState::current_position() -> [f64; 4]`

At quiescence the settled position is the planner's own state (not an external value as
in `force_idle`, which is passed `recovered_pos` from the MCU). The accessor evaluates
each axis's committed curve at the settled cursor (`t_appended == t_dispatched` once the
decel-to-zero is committed). The machinery exists: `read_path_speed_at(t, …)` reads
velocity off the axis Bézier curves and `split_partially_committed_at_t_dispatched` reads
toolhead position at `t_dispatched` (`state.rs`), so a position read at the settled cursor
is a small, precedented addition.

### `last_move_time` is unaffected

`reset()`'s existing callers (homing, `force_idle`) zero `t_appended` **without** touching
klippy's `last_move_time_bits` atomic — the planner clock and klippy's inline-event clock
are already decoupled. They are two clocks for two jobs: the `Anchor` maps planner-time →
MCU clock for pieces; `last_move_time` is klippy's own frame for scheduling inline events
(M106, SET_PIN AT_TIME), mapped to the wire by klippy's separate `est_print_time`
machinery. B follows this precedent and introduces no new desync.

### `anchor.rs` is unchanged

B requires no change to `rust/motion-bridge/src/anchor.rs` — the existing backward-jump
branch does the re-anchoring once the planner clock is rewound. (An exploratory Choice-A
edit was made and then reverted; the file matches `HEAD`.)

## Out of scope

- **M400 / `Flush` / `wait_moves` barriers.** A long idle following a `Flush` (which also
  commits to a stop and disarms `T_COMMIT`) would re-open the same fault, so a complete fix
  eventually needs the rewind there too. The current `Flush`/M400 path is suspected not to
  work well and will be designed separately. **This spec deliberately handles only
  `T_COMMIT`.**
- `ClockSyncRearm` (commits while motion continues — no idle transition; not a rewind site).

## Behavioral notes

- `T_COMMIT` is 50 ms, so the planner clock rewinds after *any* ≥50 ms gap in move
  submission — but `T_COMMIT` firing already means the decel-to-zero was committed and the
  toolhead stopped under the existing design. **B adds no new stops**; it only changes what
  happens to the clock after a stop that was already occurring. Continuous prints (moves
  <50 ms apart) never trip it.

## Testing

- **Unit (planner / state):** after a `T_COMMIT` commit, `state.t_appended == 0` and each
  axis is reseeded at the pre-rewind settled position (position continuity across the
  rewind — no teleport).
- **Unit (`current_position`):** for a known committed trajectory, `current_position()`
  returns the endpoint of the last move (matches the decel-to-zero settle point).
- **Integration (anchor interaction):** Move → `T_COMMIT` rewind → next Move yields pieces
  whose projected `start_time` is `host_now + LEAD` ahead (not in the past). This is the
  regression guard for the original fault.
- **Hardware:** the two-jog repro (`SET_KINEMATIC_POSITION`; jog `Y=-1`; pause; jog `Y=-25`)
  must complete without `-308`. Build + flash both MCUs per the bench flow.

## Files touched

- `rust/trajectory/src/streaming/state.rs` — add `current_position()`.
- `rust/motion-bridge/src/planner.rs` — call `state.reset(state.current_position())` in the
  `T_COMMIT` (`RecvTimeoutError::Timeout`) arm, after `run_commit_and_dispatch`.

`rust/motion-bridge/src/anchor.rs` is intentionally **not** touched.
