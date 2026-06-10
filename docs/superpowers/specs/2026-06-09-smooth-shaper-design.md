# Smooth shaper: emit a truly smooth, compact shaped trajectory

Date: 2026-06-09
Status: design — pending review

## Problem

The smooth (bell-kernel) input shaper produces motion that is neither smooth nor
compact. For a trivial **100 mm straight move with the shaper applied** it emits
on the order of **~1000 pieces**. Two implementation choices cause this, and they
compound:

1. **`shape_axis` / `convolve_discrete` (`trajectory/src/shaper.rs`)** samples the
   convolution at `OUTPUT_SAMPLES_PER_KERNEL_WIDTH = 12` points per kernel width
   and emits **piecewise-LINEAR** pieces (`coeffs: vec![v0, slope]`, degree 1).
   Count `~= 12 * (move_duration / kernel_width) = 12 * duration * freq / 0.8025`
   — scales with **duration x frequency**, independent of motion complexity.
2. **`refit_to_cubic` (`trajectory/src/refit.rs`)** is *meant* to reduce that.
   `fit_hermite_c1` underneath is a top-down recursive **merge** (fit one cubic
   over a span, split only on tolerance failure) — but it merges **ineffectively
   here**, because it is anchored to the dense linear piece boundaries and
   Hermite-fits the kinked C0 staircase rather than the true smooth convolution.

**Measured (throwaway, single smooth cubic `s(t)`, 100 mm, T = 0.8 s, 40 Hz):**

```
pre-shaper = 1   ->   shape_axis = 479   ->   refit_to_cubic = 429
```

So `shape_axis` inflates 1 -> 479 (the linear-sampling artifact), and the refit
barely dents it (479 -> 429). An effective adaptive fit of the *smooth* signal
reaches ~10 (Python least-squares experiment), so the refit is ~40x off. Both
stages are implicated; the net ~429 is the bloat the user observes.

Worse than the count: **linear position pieces are only C0.** Piecewise-linear
position => piecewise-constant velocity => acceleration *impulses* at every
~1.5 ms joint. The smooth shaper exists precisely to remove acceleration
discontinuities, and the linear representation reintroduces one at every sample.
The dense sampling hides this (each step is tiny), but the emitted curve is a
staircase impersonating a smooth move.

Empirically (throwaway fit experiment, `/tmp/fit_experiment*.py`), the true
complexity of the post-shaper signal is ~10-20 cubic pieces for representative
moves at sub-micron error — roughly **50-60x fewer** than today.

## Goal / success criteria

Make the emitted shaped trajectory **truly smooth** and compact, without leaving
the uniform-cubic primitive:

- **Smoothness (headline):** emitted per-axis trajectory is **C2-continuous**
  (continuous position, velocity, and acceleration) across piece joints — no
  acceleration steps. This is what "smooth shaper" must mean.
- **Compactness:** piece count tracks the motion's actual complexity, not
  `duration x frequency`. A straight move => a handful of pieces; a curved move
  => tens. Target: a 100 mm straight move drops from ~1000 to well under ~50.
- **Accuracy:** max position error <= the existing refit budget
  (`REFIT_TOLERANCE_MM = 1e-4` mm = 0.1 um), which is ~100x below motor
  resolution (~12.5 um at 80 steps/mm) — physically lossless.
- **No architectural disturbance:** stays uniform cubic; does not touch the
  planner, the TOPP solve, the geometry/curve representation, or the MCU contract
  (the MCU still evaluates per-piece cubic polynomials, just tens of smooth ones).

## Why cubic (not exact / not higher degree)

We considered three alternatives and rejected them for this change:

- **Exact analytic convolution.** A *straight* move's exact post-shaper signal is
  piecewise degree-8 (cubic profile ⊛ degree-4 bell). Degree 8 does not fit a
  cubic Bezier exactly and is not piecewise-cubic, so exactness would force
  degree-8 pieces and break the uniform-cubic rule — for zero physical benefit
  over a sub-um cubic fit. Curved moves are non-polynomial regardless, so they
  cannot be exact at any degree.
- **Higher-degree pieces.** For curved moves higher degree only trades
  coefficients for ~3x fewer pieces (a secondary knob), not correctness. Not
  worth breaking uniform cubic; revisit only if piece count proves a bottleneck.
- Therefore: **adaptive cubic merge-fit, applied uniformly to straight and curved
  segments.** Sub-um cubic approximation is lossless in any physical sense.

## Design

Replace the "dense-linear sample + subdivide-only refit" pipeline with an
**adaptive C2 cubic merge-fit** of the smooth shaped signal. The convolution math
itself is correct and unchanged; only the piece-generation/representation changes.

1. **Fit against the true smooth convolution, not the linear samples.** The fit
   target is the convolution *value* `(x ⊛ w)(t)` — which `convolve_discrete`
   already computes accurately per query point via its `fir_at` quadrature
   (`INPUT_SAMPLES_PER_KERNEL_WIDTH = 40`). That value is smooth; only the
   *linear interpolation between the 12 output samples* is C0. The fix keeps the
   accurate evaluator, queries it at whatever density the fit needs, and **drops
   the 12-sample linear output entirely.** Internal sampling density is a
   fit-accuracy parameter, not output granularity.

2. **Adaptive merge-fit (replaces the subdivide-only driver).** Fit cubic Bezier
   pieces over **maximal spans** within tolerance: greedily grow each piece as far
   as it can while max position error stays <= tolerance, place a knot, continue.
   Knots land at the signal's natural breakpoints (profile phase transitions,
   kernel-window crossings) because that is where the fit error forces a break —
   not on a fixed clock.

3. **C2 continuity at joints.** Pieces share position, velocity, and acceleration
   at knots so the emitted motion has no acceleration steps. (The current
   `fit_hermite_c1` is C1 only — continuous velocity but not acceleration; this is
   an upgrade to C2, which the bell-smoothed signal naturally supports.)

### Components touched

- `trajectory/src/shaper.rs` — stop treating the 12-samples/kernel-width linear
  output as the result; expose the shaped signal as a fit target (or evaluate it
  on demand) for the merge-fit. Internal input/output sampling constants become
  fit-accuracy parameters, not output-granularity.
- `trajectory/src/refit.rs` — replace the subdivide-only loop with the adaptive
  C2 merge-fit. (Investigate whether `nurbs::algebra::fit_hermite_c1` can be
  driven to merge / extended to C2, or whether a new greedy merge-fit is cleaner.)
- `trajectory/src/emit_shaped.rs` — wire the new path; unchanged interface to
  `ShapedSegment`.

### Scope boundaries (explicitly out)

- **Cross-segment merging.** This change merges *within* a segment, which is where
  the ~60x lives (a single straight segment is the failing case). Merging across
  segment boundaries (decoupling piece count from slicer segmentation) is a
  separate, later optimization.
- Higher-degree or exact pieces; PH curves; native-parameter timing; MCU-side
  curve evaluation — all out.
- The fail-loud MVC fix already in this branch (`constraints.rs`/`output.rs`/
  `chain.rs`) is unrelated and committed separately.

## Testing

- **Piece count, straight move:** a 100 mm straight move with the shaper applied
  emits well under ~50 pieces (assert << 1000; expect a handful).
- **Piece count, curved move:** a representative curved G5 emits tens, not
  hundreds, at the refit tolerance.
- **Accuracy:** sampled densely *within* pieces (not only at knots), max
  |emitted(t) - convolution(t)| <= `REFIT_TOLERANCE_MM`, where `convolution(t)`
  is the accurate `fir_at` evaluation (the fit target above) — so the assertion
  catches between-knot deviation, not just knot agreement.
- **Smoothness (C2):** at every internal piece joint, position/velocity/
  acceleration match within a relative eps (e.g. 1e-6 of the local
  velocity/acceleration magnitude) — no acceleration step. Directly asserts the
  "truly smooth" property the linear representation violated.
- **Regression:** existing `emit_shaped` / `refit` tests pass; the
  `shaper::long_segment_stability` test still holds.

## Open questions for the plan

- C2 is the target (it is what kills accel steps). Open item is to *measure* the
  piece-count cost of C2 vs C1 on a representative move, so we can revisit only if
  C2 proves surprisingly expensive — not to re-decide up front.
- Reuse `fit_hermite_c1` (extended to merge + C2) vs a fresh greedy merge-fit —
  decide in the plan after reading `nurbs::algebra::fit_hermite_c1`.
- Internal fit-target sampling density: high enough that sampling error is well
  under the refit tolerance; pick a principled multiple of the kernel width.
