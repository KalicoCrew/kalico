---
topic: B-spline / polynomial convolution — degree and knot-vector behavior
created: 2026-04-27
last_updated: 2026-04-27
verified_claims:
  - 2026-04-27 INCORRECT — "Convolution of degree-p B-spline with degree-q polynomial kernel produces a degree-(p+q) B-spline with the SAME knot vector as the input." Knot vector grows; support expands by the kernel's support; breakpoints become the Minkowski sum of input and kernel breakpoint sets.
sources:
  - https://www.chebfun.org/examples/approx/BSplineConv.html
  - https://en.wikipedia.org/wiki/B-spline
  - https://pages.cs.wisc.edu/~deboor/887/lec1new.pdf
  - https://dl.acm.org/doi/10.1145/1090122.1090142
---

# B-spline / polynomial convolution — degree and knot-vector behavior

## Summary

Convolution of a piecewise-polynomial B-spline curve with a polynomial (or piecewise-polynomial) kernel of compact support produces a piecewise-polynomial B-spline whose degree increases by approximately the kernel's degree (`p + q` or `p + q + 1` depending on convention) and whose knot vector is **strictly larger** than the input's: the output's breakpoint set is the Minkowski sum of the input's breakpoint set and the kernel's breakpoint set, and the parameter support widens by the kernel's support. Any architectural assumption that the input knot vector is preserved through convolution is wrong, including for the smooth-shaper pre-bake operation in kalico Layer 3.

## Verified claim — 2026-04-27

**Claim under test:** "The convolution of a degree-`p` non-rational B-spline curve with a degree-`q` polynomial kernel is a non-rational B-spline of degree `p+q` whose knot vector is identical to the knot vector of the input."

**Verdict:** INCORRECT (knot-vector part). The degree part is approximately correct (off-by-one depending on convolution convention).

### Verification

Three independent attacks all land:

1. **Cardinal B-spline construction.** `B_{n+1}(t) = (B_0 * B_n)(t)` where `B_0` is the box on `[0,1]` (a degree-0 polynomial kernel). `B_n` has knots `{0,…,n+1}`; `B_{n+1}` has knots `{0,…,n+2}`. A new knot appears at every convolution step. This is the textbook construction of the cardinal B-spline series and refutes "identical knot vector" on the smallest possible example.

2. **Support argument.** If `C(t)` is supported on `[u_min, u_max]` and `w(t)` on `[0, T_w]` with `T_w > 0`, then `(C * w)(t)` is supported on `[u_min, u_max + T_w]`. The right-hand boundary knot necessarily moves; the output knot vector cannot equal the input's.

3. **Minkowski-sum breakpoint structure.** If `C` has breakpoints `{ξ_i}` and `w` has breakpoints `{η_j}`, then `C * w` is piecewise polynomial with breakpoints in `{ξ_i + η_j}`. Even for a single-piece polynomial kernel (`{η_0=0, η_1=T_w}`) every input breakpoint duplicates and shifts by `T_w`, roughly doubling the knot count. For multi-piece kernels (the realistic smooth-shaper case), the breakpoint count grows multiplicatively. This matches the kalico grand plan's own statement in Layer 3 that "output piece count grows by O(input pieces × kernel pieces)" — which is itself inconsistent with the claim under test.

### What's actually true

- **Degree:** output degree is `p + q + 1` if the kernel is treated as a polynomial-on-an-interval that gets integrated, or `p + q` if the kernel is treated as a generalized-function-against-which-you-evaluate. Pin this down by inspecting the actual smooth-shaper kernel definition before implementing.
- **Knot vector:** output knots are the Minkowski sum `{ξ_i + η_j}`, with multiplicities determined by the smoothness drop at each breakpoint (the convolution raises smoothness by the kernel's regularity, so multiplicities can drop relative to a naive Minkowski-sum count).
- **Support:** widens by kernel support `[0, T_w]` on the right; in 3D-printer smooth-shaper terms, the trajectory's effective duration is extended by the shaper's impulse-train length.

### Architectural implication for kalico

The Layer 0 NURBS-convolution primitive must produce a **new, larger knot vector**. The Layer 3 smooth-shaper pre-bake must be designed for an output that is no longer aligned segment-for-segment with the input — segment boundaries get smeared by the kernel's support, and the per-segment NURBS that goes into convolution comes out as a NURBS that overlaps neighboring segments by the kernel-support width. This is what makes the 2-3 segment MCU buffer in Layer 4 ("segment buffer holding 2-3 adjacent segments for shaper-boundary handling") necessary; it's not about evaluation aesthetics, it's a mathematical consequence of the convolution-knot-set growing.

If the implementation team interpreted the claim literally and built convolution as "same knot vector, raise degree, recompute control points" (i.e., a disguised degree-elevation), the resulting curve will be a degree-elevated copy of the input — provably not the convolution — and the "smooth-shaper pre-bake" will silently produce unshaped output.

### Sources

- https://www.chebfun.org/examples/approx/BSplineConv.html — retrieved 2026-04-27 — demonstrates `B_{n+1} = B_0 * B_n` and the per-step support/order growth.
- https://en.wikipedia.org/wiki/B-spline — retrieved 2026-04-27 (search-result excerpt) — convolutional definition of B-splines; knot count = control points + degree + 1 identity.
- https://pages.cs.wisc.edu/~deboor/887/lec1new.pdf — retrieved 2026-04-27 (search-result excerpt) — de Boor lecture notes on cardinal B-splines and convolution operators.
- https://dl.acm.org/doi/10.1145/1090122.1090142 — retrieved 2026-04-27 (search-result excerpt) — convolution of curves and Minkowski-sum breakpoint structure.

### Caveats / unchecked assumptions

- Exact resulting degree (`p+q` vs `p+q+1`) depends on whether the kernel is treated as a polynomial-on-an-interval that gets integrated or as a distribution. Kalico's smooth-shaper kernel definition was not inspected; the kernel form should be pinned down before implementing the Layer 0 primitive.
- Exact output piece count depends on whether breakpoint sums `{ξ_i + η_j}` collide (e.g., uniform breakpoints) or are all distinct. For the claim under test (knot-vector preservation), this distinction does not matter.
- A non-standard convolution convention (e.g., circular convolution on a periodic parameter domain with matched period) was not investigated. For kalico's open trajectory + finite-support smooth-shaper use case, no such convention applies.
- Source code in `rust/` was not read (math-verification scope only).
