# NURBS Multiplication: Mørken Correctness Note

**Date:** 2026-04-26
**Status:** Design note grounding the Fix 1 / Fix 2 / Fix 3 decision tree for `algebra::multiply`
**Trigger:** Multi-piece proptest caught a real bug in `multiply` (over-aggressive `knot_remove_redundant` strips knots past the natural-multiplicity threshold given by Mørken's formula).

## Context

Layer 0 of the Kalico-next motion planner provides host-side NURBS algebra (`multiply`, `convolve`) used by Layer 3's smooth-shaper bake and shaper-aware TOPP-RA. The current `multiply` implementation faithfully follows Piegl & Tiller §5.6.3 (Bezier extract → per-piece polynomial product → recompose → Tiller A5.8 chord-error tolerance prune). It is correct for single-piece inputs. It is **wrong** for multi-piece inputs that share an interior breakpoint, because Tiller A5.8 answers "is this knot geometrically removable within tolerance?" rather than "what is the natural multiplicity of the product at this knot?".

The two questions usually agree. They diverge in exactly the case the proptest caught: when one factor has C⁰ continuity (multiplicity-1 interior knot at degree 1) at a shared breakpoint, the product is genuinely C⁰ there, but the post-Bezier-recompose curve has full-multiplicity (degree+1) at every interior knot, and A5.8's chord-error metric may accept a removal that crosses below the natural-multiplicity threshold. The result smears the kink and disagrees with the pointwise product at the breakpoint by tens of percent.

## The canonical formula

**Mørken 1991 Theorem 3.1** (re-popularized by **Patrizi & Sestini, arXiv:2601.17432, Jan 2026**):

For the product `h(u) = f(u) · g(u)` of two splines `f ∈ S_{τ¹}^{p₁}` and `g ∈ S_{τ²}^{p₂}`, the result is a spline `h ∈ S_t^p` of degree `p = p₁ + p₂` defined on a knot vector `t` whose breakpoints are the union of the breakpoints of `τ¹` and `τ²`, with multiplicities

```
μ(tᵢ) = max{ p₁ + μ²(tᵢ),  p₂ + μ¹(tᵢ) }     if μ¹(tᵢ) > 0 and μ²(tᵢ) > 0
μ(tᵢ) = p₁ + μ²(tᵢ)                           if μ¹(tᵢ) = 0 and μ²(tᵢ) > 0
μ(tᵢ) = p₂ + μ¹(tᵢ)                           if μ¹(tᵢ) > 0 and μ²(tᵢ) = 0
```

Equivalently, the **natural continuity** at a shared knot is

```
C^(p − μ(tᵢ)) = C^( min{p₁ − μ¹(tᵢ),  p₂ − μ²(tᵢ)} )
```

— the minimum of the two factors' continuities at that knot.

### Worked example (the failing proptest case)

- `a`: degree d_a=3, μ¹(0.1)=1 (so a is C² at 0.1)
- `b`: degree d_b=1, μ²(0.1)=1 (so b is C⁰ at 0.1)
- Both > 0 → μ(0.1) = max{3+1, 1+1} = **4**
- Product degree p = 4 → continuity = C^(4−4) = C⁰ ✓ (b's C⁰ kink dominates)

Our current `multiply` produces a NURBS with multiplicity 5 at u=0.1 (full Bezier multiplicity from `bezier_pieces_to_nurbs`), then `knot_remove_redundant` peels it down. The correct stopping point is multiplicity **4**. Anything below 4 destroys the C⁰ break and gives wrong evaluation at u=0.1. The chord-error tolerance test is allowing too many removals — pushing multiplicity below 4.

## Worked example (TOPP-RA velocity profiles)

For shaper-aware TOPP-RA, `v_shaped(s)² ≤ constraint(s) / |dx/ds|²` style constraints involve `multiply(v, v)` where v comes from TOPP-RA. v(s) is continuous (C⁰) but generally not C¹ at constraint-switching points (where one limit becomes binding while another releases).

For v² where v has C⁰ at the kink (d_v=1, m_v=1):
- μ(v²) at the kink = max(d_v + m_v, d_v + m_v) = d_v + 1 = 2
- Product degree = 2*d_v = 2
- Continuity at the kink = C^(2 − 2) = C⁰

This is correct: square of a tent function is two parabola pieces with a kink at the same point.

For higher-degree v (e.g., a cubic-fitted velocity profile, d_v=3, m_v=1):
- μ(v²) at the kink = max(3 + 1, 3 + 1) = 4
- Product degree = 6
- Continuity at the kink = C^(6 − 4) = C²

The kink propagates as a "second-derivative kink" in v² — eval is still smooth, second derivative has a corner. Mørken handles this consistently regardless of v's degree.

## Why P&T's tolerance-prune is structurally wrong

Tiller A5.8 (P&T §5.4) asks "is this knot geometrically removable within control-point-space chord error `tol`?". That's a *different* question from "is the natural multiplicity less than current multiplicity?".

- The Bezier round-trip discards the original knot multiplicities. After `bezier_pieces_to_nurbs`, the curve has μ = degree+1 at every interior breakpoint regardless of source.
- The tolerance test then peels back. If the per-piece coefficient values happen to align such that *one more* removal is within tolerance, A5.8 takes it. But that one more removal crosses the natural-multiplicity threshold and changes the C^k class of the curve.
- The chord-error metric is in cp-distance, not in evaluation error at the knot. A removal that drops μ from 5 → 4 → 3 may keep cp-distance small while genuinely shifting eval at u₀ (because removing the multiplicity-restoring knot reshapes the local basis support).

**A5.8 is the right algorithm for "geometric simplification". It is the wrong algorithm for "expose natural smoothness of an algebraic product".** The natural multiplicity is *known a priori* from the input multiplicities and degrees (Mørken Eq. (1)); there is no tolerance involved.

## Fix tree

### Fix 1 (Mørken-bounded knot removal — recommended near-term)

Capture original knot multiplicities of `a` and `b` before Bezier extraction. After `bezier_pieces_to_nurbs`, for each interior breakpoint compute the target multiplicity via Mørken Eq. (1). Replace `knot_remove_redundant` with a `knot_remove_to_target` that knows exactly how many removals to attempt per breakpoint (`current_mult − μ_target`). Never let A5.8 peel past the target regardless of tolerance.

- Cost: ~30-50 LOC in `algebra.rs`, plus regression tests.
- Pros: minimal change, fixes the bug, ships in hours.
- Cons: still goes through the Bezier round-trip (basis-conversion conditioning at high degree).

### Fix 2 (target-aware recompose — medium-term cleanup)

Replace `bezier_pieces_to_nurbs`'s blanket full-multiplicity output with a variant that takes per-breakpoint target multiplicities directly. Solve for B-spline coefficients on the Mørken knot vector by inverting the Bezier-extraction operator (banded linear system per breakpoint, well-conditioned in the lower→higher direction). Eliminates the post-pass entirely.

- Cost: ~1-2 days.
- Pros: natural smoothness enforced by construction.
- Cons: more invasive; reshapes the multiply pipeline.

### Fix 3 (direct Mørken via Patrizi-Sestini Algorithm 4 — long-term)

Implement Patrizi-Sestini's improved Mørken: construct the knot vector from Eq. (1), compute each B-spline coefficient via the Oslo Algorithm with the de-Boor-like inner kernel, with the distinct-knot-combination factorization. No Bezier round-trip.

- Cost: several days.
- Pros: numerically stable at high degree (>30); handles rational NURBS cleanly (relevant for future PA work).
- Cons: significantly more complex; only worth it if Fix 2 limits us.

## Numerical-stability note (deferred)

Farouki 2012 §6.5: basis conversions between Bernstein and any monomial basis (including our Pascal-shifted) are ill-conditioned for higher degrees. Condition number ≈ 2^d / √d. At MAX_DEGREE = 20 → product degree 40, condition number is ~10⁹-10¹⁰. Tolerable in f64 but high.

For the multiplication step itself, Bernstein × Bernstein has a closed-form coefficient formula and stays in the Bernstein basis (Farouki / Patrizi-Sestini both recommend this). Our current `BezierPiece` is in Pascal-shifted monomial. Reasonable hybrid: keep Pascal-shifted for evaluation, convert to Bernstein once for the multiply step. Defer until precision issues actually surface — not the cause of the current bug.

## Open-source state of the art

No widely-used OSS NURBS library ships a tested NURBS multiplication routine:

- **geomdl (NURBS-Python)** — no `multiply` operation.
- **OpenCASCADE** — knot insertion / removal / Bezier conversion only; no curve × curve product.
- **OpenNURBS (Rhino)** — no curve × curve product in public API.
- **tinynurbs (C++)** — NURBS basics only; no algebra.
- **nurbs-toolbox (MATLAB)** — no multiplication.

This means **our implementation is genuinely novel as an OSS artifact**. It also means we have no oracle to cross-check against — validation is on us, from first principles. The proptests must verify directly against Mørken (assert (a) result knot multiplicities match Eq. (1) exactly, (b) pointwise eval matches `eval(a,u) * eval(b,u)` on a dense grid).

The Patrizi-Sestini paper notes a Matlab reference implementation was used for their numerical experiments — worth contacting the authors if we want a reference oracle for Fix 3 validation.

## Recommendation

**Near-term (now):** Fix 1. Replace `knot_remove_redundant` with Mørken-bounded version. Add structural-multiplicity property test as the regression net (so this bug class can't reappear). Document the C⁰-pass-from-callers note in `nurbs-algebra-design.md`.

**Medium-term (post-Layer-3 MVP):** Reassess. If Fix 1's tolerance-bounded approach hits precision issues at high degree, do Fix 2.

**Long-term (only if needed):** Fix 3 if rational NURBS multiplication becomes load-bearing (depends on tanh PA's polynomial approximation path and whether weight-bearing NURBS enter the multiply pipeline).

## Sources

- **Patrizi & Sestini 2026** — *An algorithmic approach to direct spline products: procedures and computational aspects.* arXiv:2601.17432. Single most important reference. Eq. (1) is the knot-vector formula; Algorithm 4 is the recommended direct procedure for Fix 3.
- **Mørken 1991** — *Some identities for products and degree raising of splines.* Constructive Approximation 7, 195-208. Theorem 3.1 (the canonical formula).
- **Chen, Riesenfeld & Cohen 2007** — *Sliding windows algorithm for B-spline multiplication.* Blossoming-based variant; equivalent to Mørken (Ueda 1994).
- **Cohen, Lyche & Riesenfeld 1980 / Lyche & Mørken 1986** — Oslo Algorithm. The knot insertion primitive that direct-Mørken builds on.
- **Farouki 2012** — *The Bernstein polynomial basis: a centennial retrospective.* §6.5 on conditioning of basis conversions.
- **Piegl & Tiller, *The NURBS Book*, 2nd ed., 1997** — §5.6.3 (NURBS multiplication via Bezier — the algorithm we currently implement) and §5.4 / Algorithm A5.8 (knot removal). Predates the Mørken-aware critique.

## Pipeline-context note

The `multiply` callers in our planner pipeline include shaper-aware TOPP-RA (composing v(s)² with derivative magnitudes) and axis dynamic limits. TOPP-RA's v(s) is continuous (C⁰) but generally not C¹ at constraint-switching points. The algebra primitive must handle inputs with C⁰ continuity at interior knots correctly per Mørken Eq. (1). The proptest case `a` (degree 3 with knots at 0.1, 0.55) and `b` (degree 1 with kink at 0.1) is representative of TOPP-RA-velocity × geometric-NURBS-derivative compositions.

Note: `convolve` does not currently use the public `multiply` function — its inner integration uses `poly_multiply` (straight polynomial coefficient convolution within a single piece). The smooth-shaper bake in Layer 3 is therefore unaffected by the `multiply` bug. However, `convolve` shares the same `bezier_pieces_to_nurbs` + `knot_remove_redundant` post-pass and may have an analogous bug — this is the subject of a follow-up sanity check after Fix 1 lands.
