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
<filled in by Task 5>

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
