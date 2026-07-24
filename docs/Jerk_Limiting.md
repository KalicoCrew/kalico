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

With jerk limiting enabled the planner instead ramps the acceleration up and
back down at a bounded rate (the *jerk*), so acceleration is continuous. Each
move's accel/decel phase becomes a short S-curve rather than a hard step.

The lookahead is jerk-aware: it plans the boundary speeds that the jerk-limited
emitter can actually reach, so no move is planned into a profile it cannot
render.

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

Because peak acceleration is now `dv * f_n`, `max_accel` no longer governs
ordinary moves — it only backstops moves too short to shape (see Limitations).
Keep `max_accel` at the machine's real torque limit.

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
  `J = dv * f_n^2`. `0` disables the notch law (a fixed jerk is used instead).

- `unified_notch_freq_x`, `unified_notch_freq_y` (default: 0)
  Optional per-axis notch modes in Hz. The ramp shapes the *scalar path speed*,
  so its single spectral zero lands on both axes at once (`a_x` and `a_y` are the
  same `a(t)` scaled by the move's direction) — two independent per-axis zeros
  cannot coexist on one move. The target is therefore chosen per move, weighted
  by direction:

  ```
  f = (f_x*|rx| + f_y*|ry|) / (|rx| + |ry|)
  ```

  giving `f_x` on a pure-X move, `f_y` on a pure-Y move, and a weighted mean
  (always within `[min(f_x,f_y), max(f_x,f_y)]`) on diagonals. Use this when X
  and Y have different measured modes — e.g. a bed-slinger whose heavy Y rings
  low and lighter X higher: each axis is shaped on its own mode where it
  dominates the move, instead of running everything at the single worst mode.
  These two must be set **together** — setting exactly one is a config error
  (use `unified_notch_freq` to notch both axes at one frequency, or set both
  `_x` and `_y`). Omitting both reproduces the single-`unified_notch_freq`
  behaviour exactly. Diagonals remain a compromise (one zero per move) — that
  irreducible coupling is what
  per-axis *input shaping*, which filters each axis independently, avoids at the
  cost of operating post-hoc on the committed path.

- `unified_max_jerk` (default: 0)
  Fixed jerk cap in mm/s^3. `0` = uncapped. When `unified_notch_freq` is set
  this acts as a *ceiling* on the per-ramp jerk rather than the jerk itself.
  Clamping the jerk down only lengthens the ramp (moves the zero below `f_n`),
  so it is always safe; use it to bound peak acceleration at large `dv`.

- `unified_jerk_dt` (default: 0.001)
  Integration time step in seconds for the emitted ramp. Smaller values give a
  smoother ramp and more motion-queue entries.

## Live tuning

`SET_UNIFIED` changes the settings without a restart (moves already queued are
flushed first, so the change applies to subsequently planned moves):

```
SET_UNIFIED ENABLE=1 NOTCH_FREQ=55
SET_UNIFIED NOTCH_FREQ_X=70 NOTCH_FREQ_Y=55   # per-axis modes
SET_UNIFIED MAX_JERK=400000
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
  A move shorter than that cannot be fully shaped: it falls back to a bounded
  fixed-jerk ramp, and if it is shorter still, to the stock constant-accel
  profile (bounded by `max_accel`). Lower `f_n` needs more runway, so very fine
  detail on a low-frequency notch may not be shaped.

- **One zero.** The accel ramp provides a single shaper zero, so it cancels one
  mode. A second, well-separated mode is not addressed by this feature.

- **Throughput.** Because `a_peak = dv * f_n` is typically below `max_accel`,
  jerk-limited moves take longer than the equivalent trapezoid. This is the
  cost of not exciting the mode.

## How it works

Jerk-limited profiles are emitted as a chain of short constant-acceleration
slices through the existing trapezoid motion queue and step compression — there
are no firmware or MCU changes. The extruder is driven slice-by-slice in
lock-step with the toolhead so pressure advance integrates the real velocity
profile. The planning math lives in `klippy/extras/pathplan.py` and is covered
by `test_pathplan.py`, `test_pathplan_jerk.py`, and `test_pathplan_notch.py`.
