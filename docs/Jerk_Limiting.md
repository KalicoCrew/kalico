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

With jerk limiting enabled the planner instead emits each speed change as a
piecewise-linear acceleration pulse: at most three constant-jerk segments per
ramp (rise, hold, fall), queued as exact cubic moves rather than approximated by
a staircase of constant-acceleration slices.

The lookahead is jerk-aware and exact: the closed-form ramp distance it plans
boundary speeds with is the distance of the emitted ramp, so it never approves
a profile the emitter cannot render.

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
emitted ramp *is* the ideal pulse, so its zero is exact (to rounding, ~1e-17),
not a discretized approximation of one. Insufficient runway does not move the
zero; it stops the planner from changing speed in that move at all.

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
| one zero @55 | 36.4 ms | 11000 | **0** | 0.0451 |
| one zero @75 | 26.7 ms | 15000 | 0.1041 | **0** |
| one zero @65 (the old blend) | 30.8 ms | 13000 | 0.0306 | 0.0164 |
| **two zeros, 55 + 75** | **31.5 ms** | **11000** | **0** | **0** |

The blend leaves 3.1% at X and 1.6% at Y; the trapezoid leaves nothing at either,
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
  third-order (S-edge) pulse the notch law does not design, so the planner keeps the
  zero on whichever mode it helps more — which of the two that is depends on
  `dv`, so it is decided per ramp — and logs `second_notch_saturated` when a mode
  is genuinely left excited. Raising `max_accel`, or spanning the ramp across
  more moves so each `dv` is smaller, removes it.

Per-axis *input shaping* still differs: it filters each axis independently and so
is not bound to a shared pulse at all, at the cost of operating post-hoc on the
committed path.

### Arc output

Turn arcs on at both ends of the pipeline: arc fitting in the slicer, and
`[gcode_arcs]` in `printer.cfg`.

```
[gcode_arcs]
resolution: 1.0
```

`[gcode_arcs]` is **not** enabled by default. Without it a `G2`/`G3` from the
slicer is an unknown command and the print aborts on the first curve, so if you
switch arc fitting on, switch this on with it.

It matters here for more than accepting the commands. Curved geometry sliced as
sub-millimetre chords is the worst case for the notch law: anything turning more
than `unified_span_max_angle` at *every* segment cannot be spanned and falls
back to the per-move ceiling `v ~= f_n * segment_length` — at `f_n = 55`, about
11 mm/s on 0.2 mm chords. Arc fitting collapses those chords into a single
`G2`/`G3`, and `[gcode_arcs]` re-expands it at `resolution`, which makes the
segment length that sets that ceiling a number chosen in `printer.cfg` rather
than whatever chord tolerance the slicer happened to use. At `resolution: 1.0`
the same curve ceilings at 55 mm/s instead; coarser values raise it
proportionally.

Coarse `resolution` is only safe because `max_deviation` (default 0.05 mm)
still splits any arc whose chords would stray further than that from the
curve. Without it, length-only segmenting turns every arc shorter than
`2 * resolution` into one straight chord from its start to its end: at
`resolution: 3.0` a 5.6 mm, 340° loop collapsed to a 0.33 mm chord carrying
the whole loop's plastic and aborted the print with "Move exceeds maximum
extrusion". Gentle arcs, the ones worth a long chord, are untouched.

This is a different lever from spanning, and it does not help spanning — longer
segments turn *more* per segment, not less. On a tight r=4 mm arc, 0.2 mm chords
turn 2.87° per segment and 1 mm segments turn 14.4°, so neither spans at the 2°
default; the gain is entirely in the per-move ceiling. On gentle curves 1 mm
segments both span and carry a high ceiling, which is the case worth optimising
for.

## Live tuning

`SET_UNIFIED` changes the settings without a restart (moves already queued are
flushed first, so the change applies to subsequently planned moves):

```
SET_UNIFIED ENABLE=1 NOTCH_FREQ=55
SET_UNIFIED NOTCH_FREQ_X=70 NOTCH_FREQ_Y=55   # per-axis modes
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

## Pressure advance

Enabling the notch law means `pressure_advance_smooth_time` should usually be
**lowered**. The pressure advance *value* does not need retuning — linear
advance adds filament proportional to extruder velocity with no acceleration
term, so the coefficient is a property of the melt and nozzle, not of the
planner. Only the smoothing window needs to track the ramp.

The 0.040 s default exists to fix a problem this planner does not have. On a
stock trapezoid, acceleration is a step function, so velocity has a kink at
every ramp boundary and the pressure advance term — proportional to velocity —
inherits it. Differentiating that gives a step in *extruder* velocity, which
would demand unbounded extruder acceleration; the smoothing rounds it off.

A notched ramp is already a trapezoid in acceleration lasting
`1/f_lo + 1/f_hi`, so velocity is C1 and the pressure advance term is already
continuous. There is no discontinuity left to smooth, so the window only
needs to be short against the ramp.

Leaving the default in place is actively harmful once it approaches the ramp
duration: at 55/65 Hz the ramp is `1/55 + 1/65` = 33.6 ms, so a 40 ms window is
**wider than the entire acceleration event**. The correction is then averaged
across more than the whole ramp, which flattens and delays exactly the corner
signal a pressure advance tuning tower is read on — the tower's optimum goes
broad and its location moves between runs.

Aim for roughly a third of the notch period:

| notch                | ramp `1/f_lo + 1/f_hi` | suggested `smooth_time` |
| -------------------- | ---------------------- | ----------------------- |
| 30 Hz                | 66.7 ms                | 0.020                   |
| 40 Hz                | 50.0 ms                | 0.015                   |
| 55 Hz                | 36.4 ms                | 0.012                   |
| 55 / 65 Hz           | 33.6 ms                | 0.010                   |
| 80 Hz                | 25.0 ms                | 0.008                   |
| 100 Hz               | 20.0 ms                | 0.006                   |

```
[extruder]
pressure_advance_smooth_time: 0.01
```

Tune pressure advance itself *after* setting this. A tower read through an
oversized smoothing window measures the window as much as the melt.

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

- **Two zeros at most.** `unified_notch_freq_x`/`_y` give the ramp two zeros
  (see [Two modes in one ramp](#two-modes-in-one-ramp)), and a ramp that
  saturates `max_accel` keeps only one. A third mode is not addressed by this
  feature.

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
  straight spanning profile at `f_n = 55` Hz, `|A(f_n)| / |A(0)|` is ~1e-17
  against `0.41` at `f_n / 2` — the null is still on the mode and still exact.

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
  peak), so the residual left on the turning axis is about `2·sin(θ/2)·0.159`.
  A straight run nulls exactly, so there is no emitter floor for a turn to hide
  under: this limit is a direct residual budget.

  | heading change | residual at `f_n` |
  | -------------- | ----------------- |
  | 0.5°           | 0.0014            |
  | 2° (default)   | 0.0055            |
  | 5°             | 0.014             |
  | 18°            | 0.050             |

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
  just past the default (emitting the arc as `G2`/`G3` and re-expanding it via
  `[gcode_arcs]` raises the per-move ceiling instead — see
  [Arc output](#arc-output)). Widening `unified_span_max_angle` to 3° recovers the
  full speed there for a residual of 0.0083, which is a far better trade than
  shaping those moves on the wrong frequency would be. What is left after that
  is genuinely sharp short-segment geometry,
  where the corners are already speed-capped by `junction_deviation` and the
  notch runway is usually not the binding constraint.

## How it works

Each ramp is queued as at most three constant-jerk segments (rise, hold, fall)
through `trapq_append_jerk`: the host trapq carries a jerk term, and the
integrators used by step generation, pressure advance and the smoothing input
shapers evaluate the cubic position exactly. Step compression and the MCU are
unchanged — there are no firmware changes. A ramp spanning several moves is
split at the move boundaries into the same segments. The extruder is driven
segment-by-segment in lock-step with the toolhead (`process_move_cubic`), so
pressure advance integrates the real velocity profile. The planning math lives
in `klippy/extras/pathplan.py` and is covered by `test_pathplan.py`,
`test_pathplan_cubic.py`, `test_pathplan_notch.py`, `test_notch_dual.py`,
`test_notch_peraxis.py`, `test_toolhead_span.py`, and
`test_toolhead_unified.py`.and `test_notch_dual.py`.
