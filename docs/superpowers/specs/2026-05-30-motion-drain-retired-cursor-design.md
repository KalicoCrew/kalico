# Motion drain via retired cursor — design

**Date:** 2026-05-30
**Branch:** `simple-mcu-contract`
**Status:** design, pending implementation plan

## Problem

This branch added time-seeding on move start and per-MCU `set_position`
re-seeding (`runtime_seed_position`). Both run **without** waiting for motion
already in flight to finish. There is also no real `M400` in bridge mode.

Concretely, three host operations can corrupt an in-progress motion:

- **`set_position` / time-seed** (`bridge.rs:2688`) sends `runtime_seed_position`
  fire-and-forget. Its own comment admits "ordering against in-flight pieces is
  out of scope." Re-seeding while an axis is still stepping stomps a moving axis.
- **`M400`** in bridge mode resolves to `wait_moves()`, which is only a host
  *dispatch* barrier (`planner.flush()` shapes the decel-to-zero tail and hands
  pieces to the pump) — it does not wait for the MCU to execute them.
- **`wait_moves_and_mcu()`** (`motion_toolhead.py:710`), the method homing calls
  (`homing.py:377`) to mean "MCU is actually done," is currently an empty alias
  of `flush_step_generation()` — a clock-extrapolation guess, not ground truth.

The host needs a real "wait until the MCU has physically finished executing
everything I sent" barrier — non-blocking in the sense that the reactor
(heaters, heartbeats, temperature reporting) keeps running while the **G-code
thread** stops until motion completes.

## Root cause: the reported cursor counts the wrong event

The per-axis counter the MCU reports in its heartbeat (`consumed_counts`) is
incremented at **arm-time**, not **retire-time**:

- The sole live increment is `RingDescriptor::pop()` → `piece_ring.rs:242`.
- The engine calls `pop()` in Branch 4 of `get_position_and_velocity`
  (`engine.rs:727`) **when it arms a piece** — caching its coefficients into the
  ISR working set *before* a single step of that piece has played.
- The piece's time window `[piece_start_cycles, piece_end_cycles)` is still
  entirely ahead at that moment (Branch 1, `engine.rs:692`, keeps evaluating it
  until `now >= piece_end_cycles`).

So `consumed == sent` means "the last piece just **started**," not "motion
finished." A naive "ring fully drained ⟹ stopped" host check fires up to one
full piece-duration early, while the toolhead is still moving. (Confirmed by
workflow investigation + adversarial verification, 2026-05-30.)

## Design

**The MCU is dumb; the host is smart.** The host already knows everything it
sent. The only fix needed is to make the one cursor the MCU reports mean
"retired" (window fully elapsed) instead of "armed," then have the host block
until `retired == sent` per axis. Nothing flows back from the pump — the host
owns both numbers.

### Part 1 — MCU: relocate + rename the cursor to retire semantics

One counter, renamed `consumed` → `retired`, incremented when a piece's window
**ends**, not when it is armed.

- Move the `pop()` / counter increment out of Branch 4 (arm-time). The engine
  arms a piece by peeking + caching coefficients (as today) but does **not**
  advance the cursor at that point.
- Advance the cursor when the engine **leaves** a piece because its window has
  elapsed — i.e. when Branch 1 (`now < piece_end_cycles`) stops holding and the
  loop moves on. Each fully-crossed piece (including multiple in one sub-tick
  burst) increments the cursor exactly once.
- Consequence, intended: the currently-playing piece's slot stays occupied
  until it retires, so host-side flow-control room is conservative by exactly
  one slot per axis (~1 in ~992). Accepted — simpler contract, and the host
  physically cannot overwrite a slot mid-play.
- Rename the reported field `consumed_counts` → `retired_counts`
  (`messages.rs:304`, `engine.rs:360 consumed_counts()`,
  `runtime_ffi.rs:1771`, `piece_ring.rs:247 consumed_count()`). **No wire-format
  change** — same per-axis `u32` array, different value and name.

**Contract established:** `retired == sent` on an axis ⟺ that axis has
physically finished all motion the host sent it. Clock-independent — the MCU
asserts completion directly; the host never extrapolates the MCU clock.

### Part 2 — Host: `sent` cursor + drain barrier

The host maintains, per `(mcu, axis)`:

- **`sent`** — incremented by the host each time it hands a piece to the wire.
  The pump already tracks this as `pushed` (`pump.rs:30`); reuse it.
- **`retired`** — mirrored from the heartbeat's `retired_counts`, exactly as
  `consumed` is mirrored today (`pump.rs:31`, updated in the `Heartbeat` arm).

`drain_motion()` (new bridge method, releases the GIL):

1. `planner.flush()` — shapes + dispatches the decel-to-zero tail to the wire
   (exists today; the flush already brings motion to a stop, no new braking
   segment needed).
2. Block the G-code thread until `retired == sent` for every axis. The
   heartbeat path runs on `KalicoHostIo`'s own reactor thread (off the GIL), so
   heaters/heartbeats keep flowing while G-code waits.

Where the `sent`/`retired` comparison physically lives (pump-thread-side vs. a
host-readable snapshot) is an implementation detail for the plan — the
*contract* is "host compares two numbers it already has." No new pump→bridge
return channel, no `PumpMsg::Drain`.

### Part 3 — Wire it into the three entry points

- **`wait_moves_and_mcu()`** (`motion_toolhead.py:710`) → `drain_motion()`.
- **`M400`** in bridge mode → route to `drain_motion()` instead of the
  dispatch-only `wait_moves()`.
- **`set_position` / time-seed** (`bridge.rs:2688`) → `drain_motion()` **before**
  sending `runtime_seed_position`; replace the "ordering out of scope" comment
  with the real guarantee.

Plain `wait_moves()` stays the cheap dispatch-only barrier (callers that only
need pieces on the wire, e.g. velocity-limit updates) — matching the existing
Klipper `wait_moves` ≠ `flush_step_generation` split; draining on every minor
flush would needlessly stall throughput.

## Error handling

Per project policy (CLAUDE.md "fail loudly") and the user's direction, all edge
cases **crash immediately** rather than recover — handled if/when they actually
occur:

- A drain that observes an enqueue arriving while it is waiting (should be
  impossible: the G-code thread is blocked) → hard error, not a silent
  barrier-extend.
- `retired` overtaking `sent`, or any cursor invariant violation → hard error.
- A wedged MCU that never reaches `retired == sent` → loud timeout failure
  (reuse the existing wedge-guard duration style), not an indefinite hang.

## Out of scope

- No new "bring to a stop" segment generation — `planner.flush()` already
  emits the decel-to-zero tail.
- No host-side MCU-clock extrapolation for completion (explicitly rejected: the
  host cannot know the MCU/host clock relationship for certain; the MCU's
  retire assertion is definitive).
- No second counter and no separate "consumed" reporting — one cursor, one
  meaning.

## Affected files

MCU: `rust/runtime/src/engine.rs`, `rust/runtime/src/piece_ring.rs`,
`rust/kalico-c-api/src/runtime_ffi.rs`, `rust/kalico-protocol/src/messages.rs`.
Host: `rust/motion-bridge/src/pump.rs`, `rust/motion-bridge/src/bridge.rs`,
`klippy/motion_toolhead.py` (and the bridge-mode `M400` path).

Both MCUs (H7 + F446) must be reflashed for the new cursor semantics; the host
`.so` rebuilt.
