---
topic: Sonny-Jeon junction-deviation cornering formula and angle conventions
created: 2026-04-27
last_updated: 2026-04-27
verified_claims:
  - 2026-04-27 VERIFIED — `v_jd² = a · δ · cos(α/2)/(1 − cos(α/2))` with α = arccos(t_left · t_right) (deviation-angle convention) is algebraically equivalent to grbl/Klipper's `v² = a · δ · sin(θ/2)/(1 − sin(θ/2))` with θ = arccos(−t_left · t_right) (interior-angle convention). Verified at α ∈ {1e-6, 5°, 45°, 60°, 90°, 120°, 135°, 170°, π−1e-6} to f64 precision.
  - 2026-04-27 VERIFIED — geometric derivation: inscribed circle tangent to both segments inside chord-error δ has R = δ · cos(α/2)/(1 − cos(α/2)) in deviation-angle convention; v² = a · R yields the formula. Sonny Jeon's original 2011 blog uses the same deviation-angle convention; grbl's C implementation re-conventions via dot-product negation purely as a runtime optimization (avoids one trig call).
sources:
  - https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/ (Sonny Jeon's original 2011 derivation)
  - https://raw.githubusercontent.com/grbl/grbl/master/grbl/planner.c (grbl reference implementation)
  - https://raw.githubusercontent.com/Klipper3d/klipper/master/klippy/toolhead.py (Klipper's port)
---

# Sonny-Jeon Junction-Deviation Cornering Formula

## Summary

The Sonny-Jeon junction-deviation chord-error formula has two equivalent expressions corresponding to two angle conventions. The **deviation-angle convention** (α = 0 collinear, α = π reversal) uses `cos(α/2)`; the **interior-angle convention** (θ = π collinear, θ = 0 reversal) uses `sin(θ/2)`. Sonny Jeon's original 2011 blog post uses the deviation-angle convention; grbl and Klipper implement the interior-angle form because it skips one trig call. The kalico Layer-2 multi-segment spec adopts the deviation-angle form, which is therefore the **original** formulation, not a kalico invention.

## Verified claim — 2026-04-27

From `docs/superpowers/specs/2026-04-27-layer-2-multi-segment-design.md` §2.2 "Sharp-corner sub-case (G1↔G1)":

> `v_jd² = a_centripetal_max · δ_chord · cos(α/2) / (1 − cos(α/2))` where α = arccos(t_left · t_right) with both t_left, t_right forward unit tangents.

Specifically: (a) numerical equivalence to Klipper at multiple angles, (b) limit cases (α→0 → v→∞; α→π → v→0; 90° → v² = (1+√2)·a·δ ≈ 2.414·a·δ), (c) self-consistent angle convention, (d) geometric derivation reproduces the formula.

### Verification

**1. Symbolic equivalence (closed form).** With `α = π − θ`:

- `cos(α/2) = cos((π − θ)/2) = cos(π/2 − θ/2) = sin(θ/2)` (co-function identity).
- `1 − cos(α/2) = 1 − sin(θ/2)`.
- Hence `cos(α/2) / (1 − cos(α/2)) = sin(θ/2) / (1 − sin(θ/2))`.

Plus, given grbl's `cos θ = −(t_left · t_right)` (i.e., negated dot product) and the spec's `cos α = +(t_left · t_right)`:

- `θ = arccos(−x), α = arccos(x), arccos(−x) = π − arccos(x)`, so `θ = π − α`. ✓

**2. Numerical cross-check.** At nine angles spanning `α ∈ [1e-6, π − 1e-6]`, both formulas (with `a · δ = 1`) match to within 1e-15 (f64 round-off floor):

| α (deg) | θ (deg) = 180−α | spec `cos(α/2)/(1−cos(α/2))` | klipper `sin(θ/2)/(1−sin(θ/2))` |
|---|---|---|---|
| ~0    | ~180  | 8.0e12 (effectively ∞) | 8.0e12 |
| 5°    | 175°  | 1049.66 | 1049.66 |
| 45°   | 135°  | 12.137  | 12.137  |
| 60°   | 120°  | 6.464   | 6.464   |
| 90°   | 90°   | 2.414   | 2.414   |
| 120°  | 60°   | 1.000   | 1.000   |
| 135°  | 45°   | 0.620   | 0.620   |
| 170°  | 10°   | 0.0955  | 0.0955  |
| ~180° | ~0    | ~1e-6 (effectively 0)  | ~1e-6 |

**3. Geometric derivation.** Inscribe a circle of radius R tangent to both segments. The center lies on the angle bisector of the **interior** angle (let interior half-angle = β, so β = θ/2 = (π−α)/2). Distance from corner vertex to center along bisector = R/sin(β). Closest point on circle to vertex is along the bisector, distance R/sin(β) − R = R · (1 − sin(β))/sin(β). Setting this equal to δ:

```
R = δ · sin(β) / (1 − sin(β))
  = δ · sin(θ/2) / (1 − sin(θ/2))           [interior-angle form]
  = δ · cos(α/2) / (1 − cos(α/2))           [deviation-angle form, via sin(θ/2) = cos(α/2)]
```

Then `v² = a · R` gives both formulas. ✓

**4. Klipper SCV consistency.** Klipper converts user-set `square_corner_velocity` to a per-axis `junction_deviation` via `δ = scv² · (√2 − 1) / a`. Plugging into the formula at θ = π/2 (interior-angle 90° corner) yields v² = scv² · (√2 + 1) · (√2 − 1) = scv², as required by the SCV definition. ✓

**5. Sonny Jeon's original convention.** The 2011 onehossshay.wordpress.com post uses the deviation-angle convention directly: the post states cos θ = +(v_exit · v_entry) (no negation), giving collinear → θ = 0, and the corresponding formula uses `cos(θ/2)` in numerator and denominator. grbl's C implementation re-conventions to interior-angle form purely as a runtime optimization — it precomputes `−prev·curr` so it can drive `sin²(θ/2) = (1 − cos θ)/2` directly without an `acos` call. **The kalico spec's α-form is therefore the original formulation, not a re-derivation.**

### Sources
- https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/ — Sonny Jeon, "Improving Grbl's Cornering Algorithm" (24 Sep 2011), retrieved 2026-04-27.
- https://raw.githubusercontent.com/grbl/grbl/master/grbl/planner.c — grbl `planner.c`, retrieved 2026-04-27. The relevant lines: `junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];` (negated dot product, interior-angle convention) and `block->max_junction_speed_sqr = max(MINIMUM_JUNCTION_SPEED², (acceleration · junction_deviation · sin_theta_d2)/(1 − sin_theta_d2));`.
- https://raw.githubusercontent.com/Klipper3d/klipper/master/klippy/toolhead.py — Klipper `toolhead.py`, retrieved 2026-04-27. Same convention: `junction_cos_theta = −(...)`, then `sin_theta_d2 = sqrt(max(0.5*(1.0 − junction_cos_theta), 0.))`, then `sin_theta_d2 / (1 − sin_theta_d2) · junction_deviation · accel`.

### Caveats / unchecked assumptions

- **Numerical-stability hazard not in the spec.** The spec writes `α = arccos(t_left · t_right)` then uses `cos(α/2)`. In f64, when `t_left · t_right` overshoots `±1` by a few ULPs (which happens routinely after `normalize()`), `acos` returns NaN. Klipper/grbl avoid this entirely by computing `cos(α/2) = sqrt(max(0, (1 + dot)/2))` directly from the dot product without any `acos` round-trip. The kalico implementation should adopt the same direct half-angle computation; the spec should clamp `dot` or call out the direct route as the recommended implementation. **The math the spec states is correct; the literal computational recipe it implies is brittle.**

- **45° worked-example precision slip.** Spec says `v_jd² ≈ 12.16 · a · δ`; true value is `12.137`. Off by ~0.2%. Comes from rounding `cos(π/8) → 0.9239` and `1 − cos(π/8) → 0.0761` *before* dividing (`0.9239/0.0761 = 12.14`, also ≠ 12.16; the 12.16 is a transcription slip). Cosmetic doc nit; the implementation should target 12.137. The "implementations significantly off this value have a sign/convention bug" intent still holds: Draft-2's wrong `sin(α/2)` form would give 0.62 at α=45°, three orders of magnitude off — easily distinguishable from 12.14 vs 12.16.

- **`ALPHA_COLLINEAR_THRESHOLD` rationale slightly misleading.** Spec says "rather than letting the division blow up". In f64 the division does not blow up at α=1e-3 (denominator ≈ 1.25e-7, perfectly representable). The actual reason for clamping is to keep `v_jd` within the downstream solver's feasible range; "numerical hygiene" is correct, "blow up" overstates it. Cosmetic.

- **Out of scope:** higher-order corner-blend formulations (cubic Bezier blends, Tajima & Sencer 2016 dynamic-limit-aware blends) — those land in build-order Step 8/9 (Layer 3 corner-blend finalization), which replaces the JD chord-error model with explicit blend geometry. The JD formula above only governs the *fallback* path for sharp G1↔G1 corners that the fitter and corner-blend slot didn't handle.

- **Scalar arithmetic only.** Did not verify Rust f64 round-off behavior of the actual implementation (no implementation is checked into this branch yet); the verification is mathematical and against reference C/Python sources. Bit-exact reproduction in the eventual `junction.rs` should be re-verified against fixture-based tests.
