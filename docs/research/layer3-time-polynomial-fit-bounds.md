---
topic: Layer 3 time-domain piecewise polynomial fit of x(t) — error bounds and piece counts
created: 2026-04-29
last_updated: 2026-04-29
verified_claims:
  - 2026-04-29 VERIFIED (conditional) — Adaptive piecewise-degree-4 fit of x(t) achieves <5 µm error in 4-30 pieces per 50mm/500mm/s G5-cubic segment, conditional on (a) piece boundaries being a subset of TOPP-RA grid points, (b) near-zero-velocity boundaries special-cased, (c) v_avg ≈ 500 mm/s.
sources:
  - https://pmc.ncbi.nlm.nih.gov/articles/PMC9129939/
  - https://www.sciencedirect.com/science/article/abs/pii/S0736584518302035
  - https://arxiv.org/pdf/2102.07459
  - https://www.mathworks.com/help/fusion/ref/minsnappolytraj.html
---

# Layer 3 time-domain piecewise polynomial fit of x(t) — error bounds and piece counts

## Summary

Pre-baking x(t) on the host as an adaptive piecewise-degree-4 polynomial (rather than carrying the math-exact piecewise-degree-6 representation directly) is feasible at the 5 µm position-error budget with 4-30 pieces per 50mm/500mm/s G5-cubic segment. Two non-obvious conditions must hold for the bound: piece boundaries must align to (be a subset of) the TOPP-RA grid (otherwise C¹-discontinuities at TOPP-RA breakpoints inject delta-functions into x⁽⁵⁾, blowing up the Taylor-remainder bound), and near-zero-velocity boundaries must be special-cased (the t(s) jacobian diverges as v→0). Under those conditions, dimensional analysis with kalico-target jerk J ≈ 10⁶ mm/s³ and Chebyshev/Taylor approximation theory predicts ~5-15 pieces typical, up to ~25 in jerk-transient-heavy profiles. Below v_avg ≈ 200 mm/s the 4-30 budget breaks at fixed 5 µm tolerance.

## Verified claim — 2026-04-29

> For a typical 50mm G5-cubic segment at v_avg ≈ 500 mm/s, an adaptive piecewise-degree-4 polynomial fit of x(t) (after composing geometry × pure-quadratic-per-TOPP-piece s(t)) achieves max-position error < 5 µm in 4-30 pieces per segment, vs. the math-exact representation.

**Verdict:** VERIFIED, conditional. See conditions and caveats below.

### Verification

**Setup.** On each TOPP-RA piece, b(s) := v² is linear in s, so a := ½·db/ds is constant and s(t) = s_i + v_i·(t-t_i) + ½·a_i·(t-t_i)² (exact pure quadratic). Composition with cubic geometry x(s) gives x(t) of degree 6 per TOPP-RA piece. With Δs = 0.5mm grid, 50mm segment ⇒ N≈100 TOPP-RA pieces; segment time T ≈ 100ms.

**Sub-claim A — degree-4 best-approx of degree-6 on width h.** Standard Chebyshev: ‖p₆ - p₄*‖∞ on [-h/2, h/2] = (h/2)⁶ · 2⁻⁵ = h⁶/2048 (degree-6 part) plus (h/2)⁵·2⁻⁴ = h⁵/512 (degree-5 part). Taylor remainder bound: h⁵·‖x⁽⁵⁾‖∞/3840.

**Sub-claim B — ‖x⁽⁵⁾‖∞ estimate.** Cruise: ~0. Jerk-limited transient regions of duration τ_J = a_max/J ≈ 65ms have third-derivative ~J=10⁶, fourth ~J/τ_J ≈ 1.5·10⁷, fifth ~J/τ_J² ≈ 2.4·10⁸ mm/s⁵. Solving h⁵·2.4·10⁸/3840 ≤ 5·10⁻³ mm gives h ≤ 9.6 ms ⇒ ~10 pieces in transient regions, ~1-2 in cruise. Total 4-20 pieces realistic.

**Sub-claim C — termination of adaptive refinement.** Halving h cuts error 32×; bounded recursion provided ‖x⁽⁵⁾‖∞ is finite. **Two pathological failures:** (1) v→0 cusp ⇒ x⁽⁵⁾(t) → ∞ — must be special-cased at segment endpoints; (2) TOPP-RA grid kinks (sub-claim E) — must be aligned out.

**Sub-claim D — degree choice.** Degree-3 with M_4 ≈ 1.5·10⁷ gives h ≈ 9.4 ms ≈ same as degree-4 — essentially same piece count. Degree-4 is justified by chain-rule structure (degree-3 geometry × degree-2 s(t) ⇒ degree-6 reference) where degree-4 captures jerk + snap content; degree-5 buys ~2× piece reduction at modest evaluation-cost penalty but is bottlenecked by TOPP-RA's C¹-only joints anyway.

**Sub-claim E — TOPP-RA C¹ kinks (CRITICAL).** TOPP-RA b(s) piecewise-linear ⇒ a(s) piecewise-constant ⇒ a(t), jerk(t), snap(t) all discontinuous at TOPP-RA breakpoints. x(t) is C¹ only at those joins. The "math-exact reference is a single degree-6 polynomial" framing is wrong: it is N pieces of degree-6 with C¹ joints. The adaptive fit must merge adjacent TOPP-RA pieces (boundaries ⊂ TOPP-RA grid). Cross-piece average merge ratio K ≈ 100/N_fit pieces ≈ 5-25 — claim's 4-30 range matches.

### Sources

- https://pmc.ncbi.nlm.nih.gov/articles/PMC9129939/ — retrieved 2026-04-29 — real-time NURBS interpolation under multiple constraints; chord-error practice in modern CNC.
- https://www.sciencedirect.com/science/article/abs/pii/S0736584518302035 — retrieved 2026-04-29 — error-bounded B-spline approximation using dominant points; same adaptive-refinement structure.
- https://arxiv.org/pdf/2102.07459 — retrieved 2026-04-29 — minimum-snap trajectory generation, polynomial-degree rationale (k-segment vs degree trade).
- https://www.mathworks.com/help/fusion/ref/minsnappolytraj.html — retrieved 2026-04-29 — MATLAB minsnappolytraj documentation; default degree-7 for snap-bounded confirms our degree-4 choice is on the conservative side for snap.

### Caveats / unchecked assumptions

- ‖x⁽⁵⁾‖∞ estimates are dimensional, not measured against real kalico simulator output. Real values may differ by 2-5×; measure on representative fixtures before committing the piece-count budget.
- Rational quadratic NURBS (G2/G3) excluded per brief regime; rationals add a denominator that amplifies high derivatives.
- Smooth-shaper convolution (next Layer 3 step) raises x(t) degree by kernel degree and multiplies piece count per `bspline-polynomial-convolution.md` — post-shape piece counts are **not** covered here.
- "Piece boundaries ⊂ TOPP-RA grid" is asserted as a correctness condition; it must be enforced by the fitter implementation or the 5 µm bound silently fails.
- Low-feedrate regime v_avg < 200 mm/s with the same 5 µm absolute tolerance falls outside the 4-30 range. Either relax tolerance proportionally to feedrate or accept higher piece counts at slow speeds.
- Best-uniform-approx coefficient ratios assumed standard centered Taylor / Chebyshev-of-the-second-kind constants; small constant-factor variation does not change order-of-magnitude conclusions.
