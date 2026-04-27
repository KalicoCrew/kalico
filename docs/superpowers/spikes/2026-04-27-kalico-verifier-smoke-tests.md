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
<filled in by Task 6>
