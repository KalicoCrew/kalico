# kalico-verifier smoke tests

**Date:** 2026-04-27
**Purpose:** Validate that `.claude/agents/kalico-verifier.md` behaves per spec on a known-incorrect claim and a known-correct claim. Re-runnable.

## Smoke test A — known-incorrect claim

### Claim
The convolution of a degree-`p` non-rational B-spline curve with a degree-`q` polynomial kernel is a non-rational B-spline of degree `p+q` whose knot vector is identical to the knot vector of the input.

### Why we expect INCORRECT
Convolution of B-splines does not preserve the input knot vector. The convolved curve is a B-spline of degree `p+q`, but its knot vector is generally richer than the input's — at minimum, knot multiplicities adjust to support the higher degree, and the convolution introduces additional structure depending on the kernel's support. Standard reference: Piegl & Tiller, *The NURBS Book*, ch. 5 (NURBS multiplication and related operations); convolution-with-polynomial-kernel is a closely related operation. A correct verifier should either produce a concrete counterexample (e.g., a small worked case showing the output knot vector differs) or cite a primary source pinning the actual knot-structure result.

### Why it matters (briefing context)
Layer 0 of the grand plan claims convolution-with-polynomial-kernel is one of the algebraic operations that makes smooth-shaper pre-bake possible. If kalico builds on a wrong knot-preservation assumption, the smooth-shaper application in Layer 3 will produce malformed output. (See `CLAUDE.md` Layer 0 → "NURBS algebraic operations" and Layer 3 → "Smooth-shaper application".)

### Pointers to send
- `CLAUDE.md` (already auto-loaded for any agent in this repo)
- `docs/research/firmware-survey.md` — likely irrelevant for this claim, but lets the agent demonstrate it actually reads existing research before reaching for the web.

### Expected result
- Verdict: `INCORRECT`.
- `Adversarial findings`: at least one concrete attack — a counterexample, a primary-source citation contradicting the knot-preservation claim, or a derivation showing the output knot structure.
- `Sources consulted`: at least one primary source (Piegl & Tiller, or peer-reviewed paper on B-spline convolution).
- `Research artifact`: a new or appended doc under `docs/research/` covering B-spline / NURBS convolution knot structure (web research expected, so artifact required).

### Result

**Run:** 2026-04-27
**Verdict returned:** INCORRECT
**Pass / fail:** PASS — verdict matches expectation; adversarial findings include the cardinal-B-spline counterexample (`B_{n+1} = B_0 * B_n` adds a knot per convolution), a support argument (kernel support shifts the right boundary), Minkowski-sum breakpoint structure, and an internal-consistency check against the grand plan's own Layer 3 description ("piece count grows by O(input pieces × kernel pieces)") which directly contradicts the input claim. Verifier also surfaced a secondary issue: degree may be `p+q+1` rather than `p+q` depending on the integration convention of the kernel.
**Research artifact written:** `docs/research/bspline-polynomial-convolution.md` (6.6 KB, 60 lines).

**Verifier report (verbatim):**

```
## Claim restated

The convolution `(C * w)(t) = ∫ C(τ) w(t-τ) dτ` of a degree-`p` non-rational B-spline curve `C` with a degree-`q` polynomial kernel `w` is itself a non-rational B-spline of degree `p+q` whose knot vector is identical, element-for-element, to the knot vector of `C`.

## Verification approach

Read existing research (firmware-survey.md — discarded as irrelevant). Identified the claim as two-part: degree behavior (low risk) and knot-vector preservation (high risk, attack here). Probed against the textbook cardinal-B-spline construction `B_{n+1} = B_0 * B_n`, against a parameter-domain support argument, and against the Minkowski-sum breakpoint structure standard in piecewise-polynomial convolution. Cross-checked against kalico's own grand-plan Layer 3 statement that output piece count grows multiplicatively with kernel pieces — directly contradicting "identical knot vector". Web-searched for confirming/contradicting primary sources (Chebfun B-spline convolution example, Wikipedia B-spline article, de Boor lecture notes, Lee/Kim convolution-curve paper).

## Adversarial findings

- **Cardinal-B-spline counterexample:** `B_{n+1}(t) = (B_0 * B_n)(t)` adds a knot at every convolution step. Knots `{0,…,n+1}` become `{0,…,n+2}`. Direct refutation on the canonical example.
- **Support counterexample:** if `C` is supported on `[u_min, u_max]` and `w` on `[0, T_w]` with `T_w > 0`, then `C * w` is supported on `[u_min, u_max + T_w]`. The right boundary knot must move; "identical knot vector" is impossible whenever the kernel has nonzero support, which is always for a nontrivial polynomial kernel.
- **Minkowski-sum breakpoints:** output breakpoints are `{ξ_i + η_j}` for input breakpoints `{ξ_i}` and kernel breakpoints `{η_j}`. New interior knots are introduced at each `ξ_i + T_w`. Piece count grows additively for single-piece kernels and multiplicatively for piecewise-polynomial (smooth-shaper) kernels.
- **Internal consistency check:** kalico's own Layer 3 description ("output piece count grows by O(input pieces × kernel pieces)") contradicts the claim under test. Two parts of the same plan disagree.
- **Degree side-issue:** `p+q` may be off by one. Convolving polynomial-on-an-interval with polynomial-on-an-interval yields degree `p+q+1` (the integration adds one). `p+q` would be correct only if the kernel is interpreted as a generalized function not subject to integration of itself. This is a smaller defect than the knot-vector error and depends on the precise kernel definition the implementer has in mind.
- **Failed-to-break:** I could not find any standard convolution convention (over kalico's parameter regime — open trajectory, compactly-supported smooth shaper) under which knot-vector preservation holds. Circular convolution on a periodic parameter domain with matched period would preserve support, but is irrelevant to the kalico use case.

## Sources consulted

- Existing research: `docs/research/firmware-survey.md` (read; not relevant to B-spline algebra).
- Web sources:
  - https://www.chebfun.org/examples/approx/BSplineConv.html — 2026-04-27 — demonstrates `B_{n+1} = B_0 * B_n`, support and order both grow with each convolution.
  - https://en.wikipedia.org/wiki/B-spline — 2026-04-27 (via search excerpts) — convolutional definition; knot-count identity ` #knots = #control points + degree + 1`.
  - https://pages.cs.wisc.edu/~deboor/887/lec1new.pdf — 2026-04-27 (search excerpt) — de Boor lecture notes on cardinal B-splines as convolution operators.
  - https://dl.acm.org/doi/10.1145/1090122.1090142 — 2026-04-27 (search excerpt) — convolution-curve / Minkowski-sum breakpoint correspondence.

## Unchecked assumptions

- Exact output degree (`p+q` vs `p+q+1`) depends on convolution convention; not pinned down because the knot-vector error is fatal regardless.
- Exact output piece count vs `{ξ_i + η_j}` collision behavior depends on whether breakpoint sums coincide (uniform breakpoints) or are distinct; irrelevant to the verdict.
- Did not investigate non-standard conventions (circular/periodic). They don't apply to kalico's open trajectory.
- Did not read `rust/` source — math-only scope.
- The claim assumes a single "polynomial kernel" but kalico's actual smooth-shaper kernel is likely piecewise-polynomial (impulse train smoothed); the verification covers both since the knot-growth argument applies to either.

## Verdict

**INCORRECT.** Confidence: very high. The knot-vector-preservation half of the claim is wrong under all standard interpretations of B-spline / polynomial convolution. The textbook cardinal-B-spline construction is an immediate refutation, and the Minkowski-sum breakpoint structure formalizes why: convolution shifts and replicates breakpoints, it doesn't leave them in place. Kalico's Layer 3 description in the grand plan agrees ("output piece count grows by O(input pieces × kernel pieces)"), so this is an internal contradiction that must be resolved before Layer 0 / Layer 3 implementation. Implementing the Layer 0 convolution primitive as "same knot vector, raise degree" would silently produce a degree-elevated copy of the input — not the convolution — and the smooth-shaper pre-bake would silently fail to apply any shaping.

The degree half (`p + q`) is approximately correct but possibly off by one; pin down the kernel-integration convention before implementing.

## Research artifact

`/Users/daniladergachev/Developer/kalico/docs/research/bspline-polynomial-convolution.md`
```

---

## Smoke test B — known-correct claim

### Claim
At any point along a `C¹`-smooth NURBS path with finite curvature `κ`, the centripetal-acceleration constraint `v² · κ ≤ a_max` (where `v` is tangential speed) is a valid upper bound on permissible speed under a constant `a_max` lateral-acceleration budget.

### Why we expect VERIFIED with non-empty Adversarial findings
The relation `a_centripetal = v² · κ` is correct for a particle following a smooth curve at tangential speed `v`, and bounding it by `a_max` is the standard centripetal constraint used throughout motion planning (Sonny Jeon junction deviation, TOPP-RA centripetal constraint, etc.). A competent adversarial check should still surface real caveats: the constraint assumes `κ < ∞` (fails at cusps and at `C⁰` junctions where curvature is unbounded or undefined), assumes `v` is the *tangential* speed (and not e.g. an axis-component speed), is a *necessary* condition not a *sufficient* one in the multi-axis case (per-axis acceleration limits can be tighter), and ignores tangential acceleration (which couples to `a_max` if `a_max` is a single isotropic budget). At least the curvature-finiteness caveat must appear under `Unchecked assumptions` for the verifier to be doing its job.

### Why it matters (briefing context)
This is the core relation underpinning Layer 2's "junction velocity from curvature continuity" bullet (`CLAUDE.md` Layer 2). If the constraint has a regime where it silently fails, every junction-velocity calculation downstream is suspect.

### Pointers to send
- `CLAUDE.md` Layer 2 description (auto-loaded).
- `docs/research/firmware-survey.md` (the planner survey; junction-deviation discussion likely relevant).

### Expected result
- Verdict: `VERIFIED`.
- `Adversarial findings`: non-empty — at least one attempted attack the verifier ran (cusps, `C⁰` junctions, tangential-vs-axis-component confusion, or interaction with tangential-acceleration budget).
- `Unchecked assumptions`: at least the curvature-finiteness / cusp caveat.
- `Sources consulted`: existing research likely sufficient; web research optional.
- `Research artifact`: present iff web research occurred; otherwise the literal "No new research artifact (verified from existing knowledge)." line.

### Result

**Run:** 2026-04-27
**Verdict returned:** VERIFIED
**Pass / fail:** PASS — verdict matches expectation; nine distinct adversarial probes attempted (Frenet-Serret decomposition / total-vs-lateral, per-axis vs. magnitude limits, `C¹`-jump curvature, `C⁰`-corner JD-as-virtual-κ, NURBS cusps, 3D torsion, time-varying `v`, shaper interaction, literature search). `Unchecked assumptions` calls out one-sided-κ evaluation at `C¹`-non-`C²` boundaries, virtual-κ construction for `C⁰` corners, and `a_max` semantic hazards.
**Research artifact written:** none (verifier returned the literal "No new research artifact (verified from existing knowledge)." line — correct, since no web research was performed).

**Behavioral note (does not affect verdict):** Verifier prefixed its formal report with its working notes (Workflow steps 3–5 prose). The spec says working notes should stay private; the formal seven-section report follows the working notes correctly. Not a verdict failure but a minor prompt-adherence drift — candidate for a Task 7 round if you want it tightened. Recorded here for visibility; not blocking.

**Verifier report (formal sections only, verbatim):**

```
## Claim restated

At any point on a `C¹`-smooth NURBS path with finite, well-defined curvature `κ`, the inequality `v² · κ ≤ a_max` (where `v` is tangential speed and `a_max` is the budget for the *lateral / centripetal* component of acceleration) is a valid pointwise upper bound on permissible tangential speed. Equivalently, `v ≤ sqrt(a_max / κ)` whenever `κ > 0`.

## Verification approach

Decomposed acceleration in the Frenet-Serret frame: `a = v̇·T̂ + v²κ·N̂` (in 3D the binormal component is zero). Verified that `v²κ` is exactly the magnitude of the centripetal (normal) component, so requiring it `≤ a_max` directly enforces a lateral-budget limit. Cross-checked against the firmware survey's account of Sonny Jeon junction deviation (`v²_max = a·R`, `R = δ·sin(θ/2)/(1−sin(θ/2))`) and the Verscheure-2009 / Pham-2018 TOPP convex-program formulation — all use the same algebra. Adversarial probes targeted: dimensional/component mismatches (total vs. lateral), per-axis vs. magnitude limits, `C¹`-non-`C²` curvature jumps, `C⁰` (tangent-discontinuous) corners, NURBS cusps from high-multiplicity knots, 3D torsion, time-varying speed, and literature contradictions.

## Adversarial findings

- **Total-vs-lateral acceleration confusion (does not break claim, but documentation hazard).** `|a|² = v̇² + (v²κ)²`. The bound `v²κ ≤ a_max` covers *only the lateral/normal component*, not the magnitude of total acceleration. The claim is correctly scoped to "lateral-acceleration budget", so it stands; but if downstream code accidentally feeds the global axis-acceleration limit as `a_max` here while also enforcing a separate global accel cap on `v̇`, it will be conservative (safe) — never a violation. Worth a comment in the implementation.
- **`C¹`-non-`C²` knot points produce one-sided curvatures.** At a `C¹` NURBS junction (knot multiplicity = degree − 1), the second derivative jumps and `κ` has different left/right limits. The formula remains pointwise valid on each side, but Layer 2 must evaluate `max(κ_left, κ_right)` to bound `v` correctly at the junction. The claim does not break; it requires the implementation to take both sides.
- **`C⁰` corners (G1↔G1 junctions) are explicitly outside the claim's stated `C¹` hypothesis.** The "JD is the degenerate case of curvature continuity" framing in CLAUDE.md is correct in algebraic form but operates on a *virtual* circle radius derived from the deviation tolerance `δ`, not the path's intrinsic curvature (which is a Dirac at the corner). Implementation must construct `κ_virtual` rather than reading `κ` off the geometry at `C⁰` corners. Not a flaw in the claim, but a non-trivial step in applying it to JD.
- **Per-axis vs. isotropic `a_max`.** On Cartesian or CoreXY hardware, lateral-acceleration limits are per-axis. The scalar bound is conservative when `a_max` is taken as `min(a_x_max, a_y_max)` — never violates. A direction-aware variant (project `N̂` onto axes) would be tighter but more expensive; the conservative scalar form remains a valid upper bound.
- **Cusps from high-multiplicity knots.** Already excluded by the "finite curvature" wording — at a cusp `κ → ∞`, the hypothesis fails and the bound vacuously requires `v = 0`, which is the physically correct answer.
- **3D paths (helical G2/G3, future fitter output).** Frenet-Serret gives zero binormal acceleration; the lateral-acceleration vector has magnitude exactly `v²κ`. Bound holds in 3D.
- **Literature search for contradictions.** Verscheure 2009, Pham 2018 (TOPP-RA), Tajima & Sencer 2016, Sonny Jeon 2011, Bobrow/Dubowsky/Gibson 1985, Shin & McKay 1985 — all use this exact constraint. No contradicting result found.

## Sources consulted

- Existing research: `/Users/daniladergachev/Developer/kalico/docs/research/firmware-survey.md` (lines 51, 99–101: confirms the algebra and the canonical TOPP/JD literature).
- Web sources: none (no gaps required lookup; all sub-claims resolve from standard differential geometry and the firmware survey's existing citations).

## Unchecked assumptions

- Layer 2 implementation correctly evaluates **both one-sided curvatures** at every `C¹`-non-`C²` boundary and uses the larger value to bound junction velocity.
- Layer 2 implementation distinguishes **intrinsic-`κ` evaluation** (smooth segments) from **virtual-`κ` construction** (`C⁰` JD corners) — the unifying-formula framing in CLAUDE.md is algebraically right but the data source for `κ` differs by case.
- `a_max` in the calling convention is the **lateral / centripetal acceleration budget**, not the total acceleration magnitude or per-axis cap. If the convention drifts, the bound becomes either conservative (safe) or invalid (unsafe) depending on direction of confusion. Recommend the variable name include the word "lateral" or "centripetal" in the implementation.
- "Finite curvature" excludes cusps; assumed the implementation handles `κ = ∞` (or `1/κ = 0`) by clamping `v = 0` at that point, which is what TOPP-RA naturally does.
- No interaction with shaper-aware constraints checked here — that's a separate Layer 2 ↔ Layer 3 feedback claim.

## Verdict

**VERIFIED**

High confidence. The inequality `v² · κ ≤ a_max` is the textbook expression of the centripetal-acceleration constraint applied as a pointwise speed bound, and is unambiguously correct under the stated `C¹`-smoothness and lateral-acceleration-budget hypotheses. Every adversarial probe either failed to land or revealed an implementation hazard rather than a mathematical flaw. The Layer 2 unifying-principle framing in CLAUDE.md is algebraically sound; the practical risks are (a) ensuring both one-sided curvatures are considered at `C¹`-only boundaries, (b) constructing the virtual `κ` correctly for `C⁰` JD corners, and (c) keeping `a_max`'s semantics (lateral, not total) clear in the calling code. None of these risks invalidate the claim.

## Research artifact

No new research artifact (verified from existing knowledge).
```
