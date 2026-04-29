---
topic: Linear u(s) approximation error per TOPP-RA grid piece for cubic Béziers
created: 2026-04-29
last_updated: 2026-04-29
verified_claims:
  - 2026-04-29 REFUTED — "Linear u(s) approximation per TOPP-RA grid piece (Δs=0.5 mm) yields sub-µm position error for both Goldapp 1991 quasi-uniform cubic Béziers and arbitrary cubic Béziers." Sub-µm holds only for restricted (R, θ) regimes of arc-style cubics; arbitrary cubics with non-uniform parameterization fail even on benign shapes.
sources:
  - https://www.rgnpublications.com/journals/index.php/cma/article/download/362/350/1390
  - https://www.sciencedirect.com/science/article/abs/pii/0167839691900072
  - https://link.springer.com/article/10.1007/s11431-015-5949-2
  - https://raphlinus.github.io/curves/2018/12/28/bezier-arclength.html
---

# Linear u(s) approximation error per TOPP-RA grid piece for cubic Béziers

## Summary

For a cubic Bézier `x(u)` with `u ∈ [0,1]` and TOPP-RA grid pieces of width Δs = 0.5 mm, the per-piece chord-style linear approximation `u_lin(s) = u_k + (u_{k+1}−u_k)·(s−s_k)/Δs` has worst-case position error

  ε_max ≤ (Δs² / 8) · max_{u ∈ piece} |v'(u) / v²(u)|

(leading order; tight constant for non-degenerate curves) where `v(u) = ‖dx/du‖`. This bound is **finite and small for arc-shaped cubics with sufficient radius**, but **diverges as v→0 anywhere on the piece** and is large whenever the parameterization is non-uniform.

The original claim that "Goldapp 1991 cubic Béziers and arbitrary slicer-emitted cubic Béziers both yield sub-µm error" is **REFUTED** in both regimes:
- Cubic-Hermite-formula (`α = (4/3)·tan(θ/4)·R`, often misattributed to Goldapp) arcs: sub-µm holds only for R ≥ ~5 mm at θ ≤ 60° or R ≥ ~10 mm at θ ≤ 90°. Tight arcs (R ≤ 2 mm) blow the budget.
- Goldapp 1991's own L∞-position-optimal placement is *worse* for parameterization speed-uniformity than the Hermite formula (Goldapp optimizes a different objective).
- Arbitrary cubic Béziers: even mild S-curves and long gentle Béziers exceed 1 µm; degenerate inputs reach 100 µm to 10⁷ µm.

The Step-7-pre spec's framing of T-A's precision must be tightened or restricted to a defined input class.

## Verified claim — 2026-04-29

> For a cubic Bézier (Goldapp 1991 placement *or* arbitrary slicer-emitted), the per-TOPP-RA-grid-piece linear approximation `u(s) ≈ u_k + (u_{k+1}−u_k)/Δs · (s − s_k)` over Δs = 0.5 mm grid pieces yields position error in `x(u_lin(s))` that is **bounded sub-microns**.

**Verdict: REFUTED.**

### Verification

#### Analytical derivation

The linear interpolant of a smooth scalar function `f(s)` over `[s_k, s_{k+1}]` has the standard error bound

  max |f(s) − f_lin(s)| ≤ (Δs)² / 8 · max |f''(s)|

(standard linear-spline interpolation error; tight for Δs² · f''/8 sign-preserving on the interval).

Applied to `f(s) = u(s)`:

  u'(s) = ds/du⁻¹ = 1 / v(u(s))
  u''(s) = − v'(u) · u'(s) / v(u)² = − v'(u) / v(u)³

So

  max |u(s) − u_lin(s)| ≤ Δs² / 8 · max |v'(u) / v(u)³|.

The position error then follows by Taylor expansion of `x` around `u(s)`:

  x(u_lin) − x(u) = x'(u) · (u_lin − u) + O((Δu)²)
  ‖x'(u)‖ = v(u)
  ‖x(u_lin) − x(u)‖ ≤ v · |Δu| ≤ Δs² / 8 · max |v'(u) / v²(u)|     **(*)**

Equation (*) is the load-bearing bound. It is *leading-order tight* for non-degenerate inputs (numerical experiments below match within a small multiplicative factor) and *blows up* whenever v(u) → 0 anywhere on the piece (cusps, near-cusps, clustered control points).

#### Numerical experiments

`/tmp/kalico-verifier/linear_us_error.py` (full code preserved in transcript) computes both the measured L∞ position error (using high-precision Brent inversion of `s(u)`) and the analytical bound (*) on a battery of cubic Béziers. Δs = 0.5 mm throughout.

**Regime 1 — circle-arc cubic Béziers, "Riskus" / cubic-Hermite formula `α = (4/3)·tan(θ/4)·R`:**

| R (mm) | θ (deg) | L (mm) | Measured err (µm) | Bound (*) (µm) |
|---|---|---|---|---|
| 5 | 10 | 0.87 | 0.061 | 0.137 |
| 5 | 30 | 2.62 | 0.334 | 0.418 |
| 5 | 60 | 5.24 | 0.781 | 0.886 |
| 5 | 90 | 7.85 | 1.303 | 1.428 |
| 10 | 90 | 15.71 | 0.683 | 0.714 |
| 50 | 90 | 78.55 | 0.142 | 0.143 |
| 1 | 90 | 1.57 | – | 7.14 |
| 0.5 | 30 | 0.26 | – | 4.18 |

Sub-µm holds only for R ≥ ~5 mm at θ ≤ 60°, or R ≥ ~10 mm at θ ≤ 90°. Tighter arcs fail. Trend: bound scales as 1/R (since v ∝ R, v' ∝ R, so v'/v² ∝ 1/R) and grows monotonically with arc angle.

**Regime 1' — circle-arc cubic Béziers, *actual* Goldapp 1991 formula `α = R·sin(δ)·(√(4+3·tan²δ)−1)/3`:**

| R (mm) | θ (deg) | Bound (*) (µm) | Ratio vs Hermite |
|---|---|---|---|
| 5 | 30 | 120.9 | ×289 |
| 5 | 60 | 34.5 | ×39 |
| 5 | 90 | 6.5 | ×4.5 |
| 50 | 90 | 0.65 | ×4.5 |

Goldapp's placement (which optimizes geometric L∞ position error, *not* parameterization-speed-uniformity) is **289× worse than the Hermite formula at θ=30°** for linear-u(s) error. This contradicts the Step-7-pre spec's framing that "Goldapp 1991 specifically chooses control-point placements to minimize parameterization-speed-non-uniformity" — Goldapp does no such thing; his objective is L∞ position error.

**Regime 2 — arbitrary cubic Béziers:**

| Case | Speed ratio v_max/v_min | Measured err (µm) |
|---|---|---|
| S-curve typical `[0,0]→[5,5]→[10,−5]→[20,0]` | 2.15 | 4.4 |
| Long gentle 50 mm `[0,0]→[10,5]→[40,5]→[50,0]` | 1.79 | 2.5 |
| Near-cusp interior | 1.12 | 1.10 |
| P1=P2 (cubic degenerates to quadratic) | 2.0 | 5.4 |
| Clustered start (P1 ≈ P0+0.014) | 707 | 118 |
| Extreme clustered start (P1 = P0 + 1.4 µm) | 7070 | 119 |
| P1 = P0 (degree drops at start, true cusp) | ∞ | 10⁷ predicted; numerically diverges |

Even the most ordinary slicer-emittable shapes (S-curve, long gentle Bézier) exceed 1 µm by 2-5×. Degenerate inputs (clustered control points, near-cusps) are catastrophic.

#### Worst-case scaling

For arc-style cubic Béziers at angle θ on radius R, the bound (*) takes the form

  ε_max ≈ (Δs² / 8R) · g(θ)

with `g(θ)` slowly increasing in θ (g(30°) ≈ 1, g(60°) ≈ 2, g(90°) ≈ 3.5 for the Hermite formula). Setting ε_max ≤ 1 µm gives the practical floor

  R · 8 / g(θ) ≥ Δs² · 10⁶ µm⁻¹·mm  →  **R ≥ Δs² · g(θ) / 8** (in mm, with ε in µm)

So at Δs = 0.5 mm, R ≥ ~0.25·g(θ) mm in nominal units; numerically R ≥ ~5 mm at θ = 60°, R ≥ ~10 mm at θ = 90° to stay sub-µm. **3D-print features at R = 1-2 mm — lots of them — are outside this regime.**

For arbitrary cubics, no such clean bound exists. The spec needs to either (a) define an admissible-input class explicitly, (b) accept a precision floor higher than 1 µm, or (c) refine the approximation (e.g., quadratic-in-s reparameterization, or sub-piece refinement).

### Sources

- **Rababah 2016, "The Best Uniform Cubic Approximation of Circular Arcs with High Accuracy,"** *Communications in Mathematics and Applications* 7(1):37-46. — retrieved 2026-04-29 via WebFetch (PDF). Confirms that all GCk cubic-arc-approximation methods (Goldapp 1991, de Boor-Höllig-Sabin 1987, Dokken-Dæhlen-Lyche-Mørken 1990, Rababah 2016) optimize *geometric* L∞ error (e(t) = x²+y²−1 or related), not parameterization speed-uniformity. None of them are designed for the linear-u(s) error use case. Provides exact best-uniform formula at θ = 120.5° as Theorem I.
- **Goldapp 1991** (referenced in Rababah 2016 as [6]), "Approximation of circular arcs by cubic polynomials," *CAGD* 8:227-238. — not directly accessed (paywalled at sciencedirect.com/science/article/abs/pii/0167839691900072, returns 403). Indirectly confirmed by Rababah 2016 reference list and by the formula `α = R·sin(δ)·(√(4+3·tan²δ)−1)/3` widely circulated in CAGD secondary sources for elliptical/circular arcs.
- **Real-time Bezier interpolation satisfying chord error constraint for CNC tool path,** *Science China Technological Sciences* (2016) — retrieved 2026-04-29 via WebSearch description. Confirms that practical CNC interpolators handle cubic Béziers' non-arc-length parameterization via *iterative re-parameterization at runtime*, not via per-piece linear approximation. Standard practice in the CNC literature uses Newton iteration or 2nd-order Taylor at the sample point, not chord-style linear interpolation over a coarse grid.
- **Levien 2018 blog "How long is that Bézier?"** — retrieved 2026-04-29. Confirms that arc length of a cubic Bézier requires either Gauss-Legendre quadrature or recursive subdivision, and that arc-length-to-parameter inversion (u(s)) is non-elementary even for cubic Béziers. Aligns with the spec's caveat that u(s) is non-polynomial.

### Caveats / unchecked assumptions

- The bound (*) is *leading order*. For curves with `v_min` of the same order as the variation in `v`, higher-order Taylor terms become non-negligible; numerically the measured error tracks (*) within ~2× until v_min/v_max gets very small.
- The Step-13 compat-layer's Goldapp-output piece-count budget ("~2 cubic pieces per quarter-arc at 0.1 µm L∞ position error") refers to **geometric chord error**, not linear-u(s) error. These are unrelated bounds; conflating them is the root cause of the spec's confusion.
- The kalico-aware slicer's parameterization choices for emitted G5 are unconstrained by this analysis — the slicer could emit reasonable (Hermite-style) or unreasonable (clustered-CP) cubics. The robustness of the live pipeline depends on slicer behavior, which the kalico build cannot assume.
- Δs = 0.5 mm is fixed in this analysis. The bound scales as Δs². Reducing the TOPP-RA grid spacing to 0.1 mm would shrink the bound 25× and reclaim sub-µm in many of the failing regimes — but would inflate the TOPP-RA solve cost ~5× per segment and increase the wire-format burden.
- The 1 µm sub-micron threshold is taken at face value from the brief. The spec's "0.1 µm L∞ position error" budget (line 56 of the Step-7-pre spec) is 10× tighter; under that budget, even the best-case Hermite arc-Béziers at R = 50 mm are borderline (0.142 µm).
- f64 round-off in the compose primitive itself is not the issue here — that's bounded at ~10⁻¹³ mm absolute. The error analyzed is purely the chord-vs-true-curve mismatch from the linear u(s) assumption.
- The "near-cusp interior" case (P0=[0,0,0], P1=[3,1,0], P2=[7,−1,0], P3=[10,0,0]) does not actually have a true cusp — speed ratio is only 1.12 — but still gives 1.1 µm error. The bound (*) catches even this benign case.

### Recommended remediations for the Step-7-pre / Step-7-A spec

Listed for the orchestrator's consideration; not part of the verification verdict itself.

1. **Restrict the input class.** Document the admissible regime explicitly: e.g., "kalico-aware-slicer G5 must emit cubic Béziers with v_max/v_min ≤ 2.0 over the `[0,1]` parameter domain, and arc-style cubics must use the cubic-Hermite formula with R ≥ R_min(θ)." This shifts the sub-µm responsibility onto the slicer contract.
2. **Use quadratic-in-s reparameterization instead of linear.** Replace `u_lin(s)` with `u_quad(s) = u_k + a·(s−s_k) + b·(s−s_k)²` fitted to match exact `u(s_k), u(s_{k+1}), du/ds(s_k)`. Error scales as Δs³·max|u'''(s)|/something, which is several orders of magnitude smaller per-piece. Does not increase compose primitive degree (composition becomes cubic-of-quadratic-of-quadratic = degree 12, but per-piece still polynomial; piece count bumps).
3. **Sub-piece refinement.** Detect pieces where (*) exceeds a chosen tolerance and split them. Adaptive refinement is the standard CAGD response; integrates with TOPP-RA's grid only at the cost of breaking the "TOPP-RA grid drives piece boundaries" invariant of the rest of the pipeline.
4. **Accept a higher precision floor.** Document T-A's precision as "sub-5-µm under typical printable-feature regime" rather than "sub-µm." Practical printers have ~50 µm mechanical tolerance anyway; 1-5 µm trajectory-domain error is well below the floor of mechanical effects.
5. **Drop the "Goldapp specifically chooses control points to minimize parameterization-speed non-uniformity" claim** — it's factually incorrect. Goldapp 1991 minimizes geometric L∞ position error. Use the Riskus / cubic-Hermite formula `α = (4/3)·tan(θ/4)·R` if speed-uniformity matters; it's better than Goldapp on that axis (and worse on the geometric axis).
