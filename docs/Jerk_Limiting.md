# Jerk limiting

This document describes Kalico's optional jerk-limited motion, including the
"notch" mode that parks a shaper zero on a fixed structural resonance.

Jerk limiting is opt-in. With `unified_planner` off (the default) motion is
unchanged.

## Overview

By default the motion planner drives each axis with a trapezoidal velocity
profile: acceleration jumps instantly from zero to `max_accel` at the start of
a move and back to zero at the end. Those instantaneous acceleration steps
excite the machine's mechanical resonances, which show up as ringing or
"ghosting" on printed surfaces.

With jerk limiting enabled the planner instead emits short constant-acceleration
slices whose acceleration changes are bounded by the configured jerk step. This
is a discretized S-curve approximation rather than mathematically continuous
acceleration.

The lookahead is jerk-aware and conservative: it plans boundary speeds using an
analytic model that slightly overestimates the distance required by the emitted
slices, so it should not approve a profile the emitter cannot render.

## The notch law

A jerk-limited acceleration ramp that does not saturate `max_accel` is
*triangular* in `a(t)` — it rises to a peak and falls back over a rise time
`T`. A triangular acceleration pulse is an input shaper: it has a spectral zero
at `f = 1/T`. Any structural mode at that frequency is not excited by the move.

With a **fixed** jerk `J`, the rise time is `T = sqrt(dv/J)` for a velocity
change `dv`, so the zero sits at `sqrt(J/dv)` — it *slides* with every move's
`dv`. Short moves get a high-frequency notch, long moves a low-frequency one.
That cannot cancel a resonance at a fixed frequency.

Setting the jerk per ramp to

```
J = dv * f_n^2
```

instead pins the rise time at `T = 1/f_n` for every move, so the shaper zero
stays on the mode at `f_n` regardless of `dv`. A useful side effect is that the
peak acceleration self-scales:

```
a_peak   = dv * f_n          (linear in the velocity change)
distance = (v0 + v1) / f_n   (the "runway" a ramp needs)
```

When `dv > max_accel / f_n`, the triangular pulse would exceed `max_accel`.
Notch mode then switches to a designed saturated trapezoid: it sets
`J = max_accel * f_n`, holds the ramp edge time at `1/f_n`, and adds a
constant-acceleration plateau for the remaining velocity change. The ideal ramp
edge still has a zero at `f_n` while the configured acceleration limit is
honored.

The notch frequencies and ramp velocity change determine jerk directly. The
emitted zero-order-held slices approximate the ideal pulse, and insufficient
runway moves the response away from the requested notch.

## Configuration

Add to the `[printer]` section (see also `config/sample-jerk-limiting.cfg`):

```
[printer]
# ... existing options ...
unified_planner: True
unified_notch_freq: 55
```

- `unified_planner` (default: False)
  Enable jerk-limited motion.

- `unified_notch_freq` (default: 0)
  Mode frequency in Hz to park the accel-ramp's shaper zero on, via
  `J = dv * f_n^2`. `0` disables the notch law.

- `unified_notch_freq_x`, `unified_notch_freq_y` (default: 0)
  Optional per-axis notch modes in Hz. Setting two **different** modes selects a
  two-zero ramp — see [Two modes in one ramp](#two-modes-in-one-ramp) below.
  There is no separate enable: `f_x == f_y` is exactly `unified_notch_freq`, and
  omitting both reproduces it too. These two must be set **together** — setting
  exactly one is a config error (use `unified_notch_freq` to notch both axes at
  one frequency, or set both `_x` and `_y`). Use this when X and Y have
  different measured modes — e.g. a bed-slinger whose heavy Y rings low and
  lighter X higher.

- `unified_max_da` (default: 0)
  Optional cap in mm/s^2 on the positive acceleration step emitted between
  slices. `0` disables the cap. The emitter shrinks slice time to keep jerk-up
  steps within this cap — and unlike `unified_jerk_dt` that shrinking has no
  floor, so a small value here is the quickest way to ask for an unrenderable
  ramp. Settings that would need more than 4096 slices to ramp
  `0 -> max_velocity` are rejected at startup and by `SET_VELOCITY_LIMIT`,
  rather than left to fail on a `G1` mid-print. `M204` is deliberately not
  checked: slicers emit it per feature, and failing there would be the
  shutdown this is avoiding.

- `unified_jerk_dt` (default: 0.001)
  Integration time step in seconds for the emitted ramp. Smaller values give a
  smoother ramp and more motion-queue entries. The minimum is `0.0001`. It must
  provide at least ten slices across the fastest configured notch edge:
  `unified_jerk_dt * max(f_x, f_y) <= 0.1`.

### Two modes in one ramp

The ramp shapes the *scalar path speed*, so both axes see the same `a(t)` scaled
by the move's direction: `a_x = rx*a(t)`, `a_y = ry*a(t)`. That is often read as
"one move can only null one frequency", and it is why this used to place a single
zero at a direction-weighted mean of `f_x` and `f_y` — landing it on **neither**
mode on a diagonal.

It is not true. Spectral zeros multiply under convolution, and the single-zero
triangle is already `rect(1/f_n) * rect(1/f_n)`. Widening one rect makes the
pulse a **trapezoid**:

```
a(t)   = rect(1/f_hi) * rect(1/f_lo)
|A(f)| = |sinc(f/f_hi) * sinc(f/f_lo)|      -> exact zeros at BOTH modes
```

and since both axes share that `a(t)`, both axes get both zeros, on every
heading. Concretely, for `f_x = 55`, `f_y = 75`, `dv = 200 mm/s` (residual is
`|A(f)|/|a|₁`, so `0` is a perfect null):

| ramp | duration | peak accel | residual @55 | residual @75 |
|---|---|---|---|---|
| one zero @55 | 36.3 ms | 10997 | **0.0003** | 0.0451 |
| one zero @75 | 26.6 ms | 14995 | 0.1044 | **0.0004** |
| one zero @65 (the old blend) | 30.7 ms | 12998 | 0.0306 | 0.0162 |
| **two zeros, 55 + 75** | **31.4 ms** | **11000** | **0.0004** | **0.0003** |

The blend leaves 3.1% at X and 1.6% at Y; the trapezoid leaves ~0.04% at both,
for 2% more ramp time and 15% *less* peak acceleration. The laws are

```
J        = dv * f_lo * f_hi           T_rise  = 1/f_hi
a_peak   = dv * f_lo                  T_total = 1/f_lo + 1/f_hi
distance = 0.5*(v0+v1) * T_total
```

which collapse term-by-term to the single-zero triangle when `f_lo == f_hi`.
Runway and throughput are governed by the pair's **harmonic mean**
`f_eq = 2/(1/f_lo + 1/f_hi)` wherever this document says `f_n`.

Two consequences worth knowing:

- Ramp shape no longer depends on heading, so cross-move spanning links along a
  curve on geometry alone (see `unified_span_ramps`).
- If `a_peak = dv * f_lo` would exceed `max_accel`, the plateau is forced wider
  than `1/f_lo` and only **one** zero fits. Recovering both would need a
  third-order (S-edge) pulse the emitter cannot render, so the planner keeps the
  zero on whichever mode it helps more — which of the two that is depends on
  `dv`, so it is decided per ramp — and logs `second_notch_saturated` when a mode
  is genuinely left excited. Raising `max_accel`, or spanning the ramp across
  more moves so each `dv` is smaller, removes it.

Per-axis *input shaping* still differs: it filters each axis independently and so
is not bound to a shared pulse at all, at the cost of operating post-hoc on the
committed path.

## Live tuning

`SET_UNIFIED` changes the settings without a restart (moves already queued are
flushed first, so the change applies to subsequently planned moves):

```
SET_UNIFIED ENABLE=1 NOTCH_FREQ=55
SET_UNIFIED NOTCH_FREQ_X=70 NOTCH_FREQ_Y=55   # per-axis modes
SET_UNIFIED MAX_DA=100
SET_UNIFIED                       # report current state
```

## Finding the mode frequency

Use `TUNING_TOWER` to sweep `notch_freq` while printing a ringing/ghosting
tower, then read the Z height of the cleanest band back to a frequency.

```
SET_UNIFIED ENABLE=1
TUNING_TOWER COMMAND=SET_UNIFIED PARAMETER=notch_freq \
    START=30 STEP_DELTA=4 STEP_HEIGHT=5 SKIP=2
```

Each 5 mm band prints at `notch_freq = START + STEP_DELTA * floor((z - SKIP) /
STEP_HEIGHT)`. Pick the frequency of the band with the least ringing, then run
a finer sweep centered on it (for example `START=<found-4> STEP_DELTA=1`) to
refine. The notch is a broad `sinc^2`, so nearby frequencies look similar —
once several adjacent bands are indistinguishable, any value in that range
works.

An independent cross-check: measure the spacing of a ghost band on a surface
printed at a known speed. Band spacing is the resonance period, so
`f = print_speed / spacing`.

## Limitations

- **Runway.** A jerk-limited ramp needs `(v0 + v1) / f_n` of travel to complete.
  Lower `f_n` needs more runway, so very fine detail on a low-frequency notch may
  not be shaped. When a move is too short, the *planner* does not accelerate
  across it at all — it holds the entry speed rather than emitting an unshaped
  ramp. If lookahead and emission ever disagree and the requested notched
  profile cannot be rendered, motion fails closed instead of substituting a
  potentially large constant-acceleration step. See **Throughput** below: this
  is the dominant practical effect of the feature.

- **Acceleration limit.** For `dv <= max_accel / f_n`, notch mode uses the
  triangular law `J = dv * f_n^2`. Above that, it uses a saturated trapezoid with
  `J = max_accel * f_n` and a constant-acceleration plateau. This preserves the
  ideal ramp-edge zero at `f_n` while honoring `max_accel`, but large velocity
  changes take longer than they would with an unlimited acceleration setting.

- **Homing and probing.** Homing/probing drip moves continue to use the standard
  trapezoid path. `unified_planner` applies to normal queued motion.

- **One zero.** The accel ramp provides a single shaper zero, so it cancels one
  mode. A second, well-separated mode is not addressed by this feature.

- **Throughput.** Jerk-limited moves take longer than the equivalent trapezoid,
  and on short-segment geometry the cost is much larger than "a bit slower".

  Parking the shaper zero fixes the ramp *duration* at `2 / f_n` no matter how
  small the speed change is, so the ramp distance `(v0 + v1) / f_n` does **not**
  fall to zero as `dv` does — its floor is `2 * v0 / f_n`. A move shorter than
  that cannot change speed at all. Minimum move length needed to change speed,
  at `f_n = 55`:

  | current speed | runway needed |
  | ------------- | ------------- |
  | 20 mm/s       | 0.73 mm       |
  | 50 mm/s       | 1.82 mm       |
  | 100 mm/s      | 3.64 mm       |
  | 200 mm/s      | 7.27 mm       |
  | 300 mm/s      | 10.91 mm      |

  With `unified_span_ramps` off, acceleration also cannot be spread across a run
  of short segments to get around this, because each move's ramp returns to
  `a = 0` at its own end. On a chain of equal-length segments the toolhead then
  converges to roughly

  ```
  v_terminal ~= f_n * segment_length
  ```

  regardless of `max_velocity` and `max_accel` — at `f_n = 55`, about 55 mm/s on
  1 mm segments and 11 mm/s on 0.2 mm segments. Curve-heavy or high-resolution
  sliced geometry is therefore speed-limited by `f_n`, not by the machine.

  **`unified_span_ramps` (on by default) removes this ceiling without moving the
  zero.** The floor is not a property of the notch law — it is a property of
  requiring the ramp to start and end at `a = 0` *inside every move*. Drop that
  requirement and let one ramp span a run of consecutive near-collinear moves,
  and the runway is the run's length instead of a single segment's. The ramp
  itself is unchanged: same `2 / f_n` rise, same triangular `a(t)`, same zero
  sitting exactly on `f_n`. Measured on the emitted acceleration waveform of a
  spanning profile at `f_n = 55` Hz, `|A(f_n)| / |A(0)| = 0.0066`, against `0.41`
  at `f_n / 2` — the null is still on the mode and still sharp.

  Terminal speed on a chain of collinear segments, requested feedrate 300 mm/s,
  `f_n = 55`, `max_accel = 20000`:

  | segment | spanning off | spanning on |
  | ------- | ------------ | ----------- |
  | 0.1 mm  | 5.5 mm/s     | 300 mm/s    |
  | 0.2 mm  | 11 mm/s      | 300 mm/s    |
  | 0.5 mm  | 27.5 mm/s    | 300 mm/s    |
  | 1 mm    | 55 mm/s      | 300 mm/s    |
  | 2 mm    | 110 mm/s     | 300 mm/s    |
  | 5 mm    | 275 mm/s     | 300 mm/s    |

  A run breaks at any direction change beyond `unified_span_max_angle`, at a
  notch-target change, and wherever a corner or feedrate limit would be
  exceeded. Acceleration limits may vary inside a run; the emitter uses the
  lowest limit among its member moves, so the run remains within every
  per-move limit.

  That angle limit matters more than it looks. The ramp shapes the *scalar*
  path speed, and the axes see `a_x = rx·a(t)`. If the heading changes at time
  `t_c` while `a(t)` is still non-zero, the turning axis gets

  ```
  a_x(t) = r1x·a(t) + (r2x - r1x)·a(t)·u(t - t_c)
  ```

  The first term keeps the zero; the second is a *truncated* triangle, which
  has no null at `f_n`. Integrating the ideal pulse gives
  `|A_tail(f_n)| / |a|₁ ≤ 0.159` (worst when the turn lands on the acceleration
  peak), so the residual left on the turning axis is about `2·sin(θ/2)·0.159` —
  measured against the ~0.0066 the emitter's own ZOH discretization already
  leaves on a straight run:

  | heading change | residual at `f_n` | vs. straight-run floor |
  | -------------- | ----------------- | ---------------------- |
  | 0.5°           | 0.0014            | 0.2x                   |
  | 2° (default)   | 0.0058            | 0.9x                   |
  | 5°             | 0.014             | 2.1x                   |
  | 18°            | 0.051             | 7.8x                   |

  This is why the default is 2° and not the much coarser angle a "nearly
  collinear" intuition suggests. A scalar-only spectrum cannot see any of this,
  which is why the test measures each axis separately. Boundary speeds are additionally
  capped by the stock constant-acceleration reach, which keeps every move
  individually feasible at `max_accel`. Set `unified_span_ramps: False` for the
  strict per-move behaviour.

  Slicing at a longer segment length also raises the per-move ceiling directly,
  and is the only lever if spanning is disabled.

  Geometry that turns more than `unified_span_max_angle` at *every* segment
  cannot be spanned, and falls back to the per-move ceiling. Small arcs are the
  case that bites: an r=4 mm circle at 0.2 mm chords turns 2.87° per segment,
  just past the default. Widening `unified_span_max_angle` to 3° recovers the
  full speed there for a residual of 0.0083 — 1.3x the emitter's own floor —
  which is a far better trade than shaping those moves on the wrong frequency
  would be. What is left after that is genuinely sharp short-segment geometry,
  where the corners are already speed-capped by `junction_deviation` and the
  notch runway is usually not the binding constraint.

## How it works

Jerk-limited profiles are emitted as a chain of short constant-acceleration
slices through the existing trapezoid motion queue and step compression — there
are no firmware or MCU changes. The extruder is driven slice-by-slice in
lock-step with the toolhead so pressure advance integrates the real velocity
profile. The planning math lives in `klippy/extras/pathplan.py` and is covered
by `test_pathplan.py`, `test_pathplan_jerk.py`, `test_pathplan_notch.py`,
and `test_notch_dual.py`.
