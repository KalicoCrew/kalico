# Executed-Motion History Service (`motion_state_at`)

Spec A of the beacon-support program
([survey](../../kalico-rewrite/beacon-fork-survey.md)). Everything here
lives in this repo; external consumers (the beacon_klipper fork) only call
the published query.

## Goal

A bounded, always-on history of every motion curve the bridge dispatched,
queryable as: *what was the commanded toolhead state (position, velocity,
acceleration) per axis at past time T* — exact, resync-immune, with no
step-history or trapq concepts.

Consumers, in order of arrival:

1. Homing trip reconstruction (existing — becomes the first client of the
   general ring; `homing_trajectory` is deleted).
2. Probe streaming: mapping sensor samples (stamped in the probe MCU's
   clock) to toolhead positions — the backbone of Beacon/Cartographer
   scanning (spec D swaps their `_get_position_at_time` to this query).
3. Contact homing cruise-phase validation (`accel(T) == 0` check, spec B).
4. `kinematics/extruder.py:find_past_position` — currently broken
   (calls deleted `get_past_mcu_position`); E *is* the coordinate, so it
   maps directly.
5. Future closed-loop axes (servo XYZ, closed-loop E for on-the-fly
   pressure-advance tuning): commanded-state-at-T is the reference signal
   feedback is compared against.

## Non-goals

- Motor-space (per-stepper step count) answers. `extras/angle.py` wants
  them for encoder calibration; it gets an explicit
  "not supported on the bridge motion engine" error instead of today's
  `AttributeError`. A toolhead→motor transform can be layered later if
  that hardware ever matters.
- Measured (closed-loop feedback) state. This service reports *commanded*
  state; a future feedback store can mirror its shape.
- Mainline API emulation (`get_trapq`, `trapq_extract_old`,
  `get_past_mcu_position` lookalikes) — per the fork decision.

## Architecture

### Recording

One site: the dispatch callback at `bridge.rs:2715-2723`, where
`enqueue_segment` has just produced per-(MCU, axis) messages whose
`m.pieces` are the exact `PieceEntry` values the MCU will execute. Today
they are mirrored into `homing_trajectory` only when a drip cohort is
active; instead, **every** piece is recorded into the history ring keyed
by `m.key` (`AxisKey { mcu_id, axis }`) — homing or not, all axes the
dispatcher serves (X/Y/Z/E and whatever it grows), no axis special-cases.

### Storage: host-only entry, ABI-frozen wire type untouched

`PieceEntry` is `#[repr(C)]`, 32-byte size-asserted, and consumed by the
MCU ISR (`motion_core.rs:101` advances pieces via
`end_time = start_time + (duration_f32 × cycles_per_second) as u64`). It
cannot change. The ring therefore stores a host-only entry computed once
at record time:

```rust
struct HistoryPiece {
    start_clock: u64,   // PieceEntry.start_time — the MCU's schedule
    end_clock: u64,     // PieceEntry::end_time(nominal_freq) — see below
    coeffs: [f32; 4],   // Bernstein control points (position, mm)
}
```

`end_clock` is computed with the **per-MCU nominal clock frequency** (the
same constant the ISR receives as `cycles_per_second`), replicating the
ISR's f32 formula bit-for-bit — not the router's sync-estimated frequency.
This is the resync-proofing fix: both endpoints of every piece are then
ground truth in the axis MCU's own clock domain, and a within-domain query
touches zero sync state, ever. (The current homing reconstruction
recomputes piece ends at query time with the sync estimate
(`homing.rs:72,77` via `ack_clock_and_freq`) — a ppm-scale wart this
deletes.)

Ring: fixed-capacity `VecDeque<HistoryPiece>` per `AxisKey`,
`HISTORY_CAPACITY = 4096` entries (~128 KB/axis worst case). Eviction is
oldest-first on overflow; the *effective* retained window is whatever the
capacity holds at the current piece rate, and queries older than the
oldest retained piece fail with the actual window bounds in the error
(self-regulating, fail-loud — no silent approximation). Pieces are
recorded in dispatch order; `start_clock` is monotonically non-decreasing
per key, so lookup is binary search.

### Hold-state: idle is known, not an error

Probes routinely sample at standstill (settling samples, calibration
while parked — possibly parked far longer than the ring's window). Per
`AxisKey` the service keeps an **endpoint register** that survives ring
eviction:

```rust
struct AxisEndpoint { clock: u64, position: f64 }
```

updated on every recorded piece (`end_clock`, Bernstein endpoint
`coeffs[3]`) and by position rebase (below). Query resolution for time T:

1. T inside a retained piece → evaluate curve and derivatives at
   `u = (T − start) / (end − start)`.
2. T in a gap between pieces, or after the last piece → hold-state of the
   nearest preceding endpoint: its position, velocity 0, acceleration 0.
3. T before the oldest retained piece (and before the endpoint register's
   coverage) → error with retained window bounds.
4. T in the future — beyond the MCU's latest acked clock — → error
   ("query in the future"). Pieces dispatched ahead (lead time) are in
   the ring but only addressable once the clock reaches them.

### Position rebase invalidates history

`set_position` (homing, G92) rebases the toolhead coordinate frame;
pieces dispatched before it are in the old frame and answering across the
rebase would be silently wrong. On `bridge.set_position` / planner stream
reset: clear all rings, set every endpoint register to the new position
at the current clock. Queries into pre-rebase time then fail naturally as
case 3. Nothing legitimate crosses a rebase: no scanning flow spans a
set_position, and homing trip reconstruction runs before homing.py calls
`toolhead.set_position`.

### Query

Rust core (in `rust/motion-bridge/src/motion_history.rs`):

```rust
pub struct AxisState { pub position: f64, pub velocity: f64, pub acceleration: f64 }

/// T given directly in `axis_key.mcu_id`'s clock domain — exact path.
fn state_at_clock(key: AxisKey, clock: u64) -> Result<AxisState, HistoryError>;
```

Evaluation in f64 from the f32 control points (the
`eval_bernstein_cubic` precedent); velocity and acceleration from the
Bernstein derivative forms scaled by `1/duration` and `1/duration²`,
where duration-in-seconds uses the nominal frequency (estimate-free).

Bridge FFI / Python surface (on the `motion_bridge` object):

```python
bridge.motion_state_at(print_time=...)          # Python extras
bridge.motion_state_at(mcu=h, clock=c)          # probe-sample stamps
# → {"x": (pos, vel, accel), "y": (...), "z": (...), "e": (...)}  per
#   axis present in the dispatch config; raises on any per-axis error.
```

The FFI takes only `(mcu, clock)`. The `print_time=` form is resolved in
the Python binding: `mcu.print_time_to_clock()` on the queried MCU's
existing clocksync, then the clock-based call. Cross-domain resolution
(source MCU ≠ axis MCU) reuses the existing homing path
(`homing.rs:109-143`): source clock → host seconds → each target MCU's
clock via the router (`clock_to_host_secs` / `host_time_to_mcu_clock`).
All MCUs — including a probe's — are router-claimed
(`serialhdl.py:386-406`), so no new plumbing. Sync-estimate error applies
only to this domain *crossing* (µs-scale for sub-second-old events ⇒
sub-µm at scan speeds); within-domain evaluation stays exact.

### Migrations in the same change

- `reconstruct_axis_position` evaluates against the general ring;
  `homing_trajectory` (bridge.rs:471,699,2718,3033,3257) is deleted.
  Behavioral delta: piece-end normalization switches from sync-estimated
  to nominal frequency (strictly more correct).
- `kinematics/extruder.py:find_past_position` → E-axis query via
  `motion_state_at(print_time=...)`.
- `extras/angle.py:get_past_mcu_position` call → explicit fail-loud
  error (non-goal above).

## Error handling

`HistoryError` variants, all loud, all carrying enough to diagnose:
`BeforeRetainedWindow { queried, window_start, window_end }`,
`QueryInFuture { queried, latest_acked }`,
`NoHistoryForAxis(AxisKey)`,
`ClockUnsynced { … }` (cross-domain only, mirrors
`ReconstructError::ClockUnsynced`),
`UnknownNominalFreq { mcu_id }`.
Python binding raises `command_error` with the formatted message.

## Testing

Separate test files per house rules.

Unit (Rust, `motion_history` tests):
- Eval correctness: position/velocity/acceleration against analytically
  known cubics, including piece boundaries (`u = 0`, `u = 1`).
- **Resync immunity**: record pieces, mutate the router's frequency
  estimate, assert bit-identical within-domain results.
- ISR parity: `HistoryPiece.end_clock` equals
  `PieceEntry::end_time(nominal)` for a sweep of durations/frequencies
  (f32 truncation semantics preserved).
- Hold-state: gaps between retained pieces; after-last-piece holds (long
  parks evict nothing — eviction is capacity-driven); rebase-then-idle
  sampling answered by the endpoint register with an empty ring.
- Rebase: set_position clears rings, updates registers, pre-rebase
  queries fail as BeforeRetainedWindow.
- Window/future errors; eviction under overflow reports true window.
- Cross-MCU conversion against a mocked router sync state.

Integration (sim):
- Scan-shaped test: stream of timestamped queries during dispatched
  motion compared against the planner's own trajectory output.
- Homing regression: existing homing sim variants pass unchanged after
  the `homing_trajectory` → ring migration.
- `find_past_position` smoke test through the Python binding.

## Out of scope (future specs)

- Software-trip homing arms, credit deadman, probing primitive — spec B.
- Probe interface restoration — spec C.
- Any beacon_klipper changes — spec D.
- Measured-state (feedback) history for closed-loop control — future.
