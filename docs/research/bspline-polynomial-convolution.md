---
topic: B-spline / polynomial convolution — degree and knot-vector behavior
created: 2026-04-27
last_updated: 2026-05-26
verified_claims:
  - 2026-04-27 INCORRECT — "Convolution of degree-p B-spline with degree-q polynomial kernel produces a degree-(p+q) B-spline with the SAME knot vector as the input." Knot vector grows; support expands by the kernel's support; breakpoints become the Minkowski sum of input and kernel breakpoint sets.
  - 2026-05-26 VERIFIED — "Convolution of degree-3 B-spline with degree-4 polynomial kernel of compact support gives degree 3+4+1=8 piecewise polynomial on Minkowski-sum knot vector; output control points are a linear function of input control points computable without piece-by-piece expansion (via Chui-Stockler 1994 divided-difference/blossom recurrence for non-uniform knots)."
sources:
  - https://www.chebfun.org/examples/approx/BSplineConv.html
  - https://en.wikipedia.org/wiki/B-spline
  - https://pages.cs.wisc.edu/~deboor/887/lec1new.pdf
  - https://dl.acm.org/doi/10.1145/1090122.1090142
  - https://www.sciencedirect.com/science/article/pii/0377042794901821
  - https://www.cs.jhu.edu/~misha/Notes/BSplines.html
  - https://dercuano.github.io/notes/spline-convolution.html
---

# B-spline / polynomial convolution — degree and knot-vector behavior

## Summary

Convolution of a piecewise-polynomial B-spline curve with a polynomial (or piecewise-polynomial) kernel of compact support produces a piecewise-polynomial B-spline whose degree is exactly `p + q + 1` (where p = input degree, q = kernel degree; the +1 comes from the integration inherent in convolution) and whose knot vector is **strictly larger** than the input's: the output's breakpoint set is the Minkowski sum of the input's breakpoint set and the kernel's breakpoint set, and the parameter support widens by the kernel's support. The output control points are a linear function of the input control points, computable without piece-by-piece polynomial expansion — for non-uniform knots via the Chui-Stockler (1994) divided-difference/blossom recurrence. Any architectural assumption that the input knot vector is preserved through convolution is wrong, including for the smooth-shaper pre-bake operation in kalico Layer 3.

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

## Verified claim — 2026-05-26

**Claim under test:** "Given a cubic B-spline f(t) = sum c_i N_{i,3}(t) on knot vector T, and a polynomial kernel w(t) of degree 4 with compact support [-h, h], the convolution g(u) = integral f(s) w(u-s) ds can be computed as a B-spline of degree 3+4+1=8 on a refined knot vector (Minkowski sum of input knots and kernel breakpoints), where the output control points are a discrete linear combination of the input control points weighted by integrals of the kernel against the B-spline basis functions."

**Verdict:** VERIFIED, with caveats on non-uniform-knot implementation complexity and degree-8 downstream implications.

### Verification

Seven adversarial probes:

1. **Degree formula: p+q+1 vs p+q.** The standard result for convolution of compactly-supported piecewise polynomials: if f has degree p and g has degree q, then f*g has degree p+q+1. The +1 comes from integration (convolution = integral of a product). Confirmed by the cardinal B-spline recursive construction: B_0 (degree 0) * B_0 (degree 0) = B_1 (degree 1) = 0+0+1. B_1 * B_0 = B_2 (degree 2) = 1+0+1. General: B_n * B_0 = B_{n+1}, consistent with n+0+1 = n+1. The "p+q" convention that appears in some references uses B-spline *order* (= degree + 1): order_out = order_f + order_w - 1, giving degree_out = (p+1) + (q+1) - 2 = p+q. This is the same formula in different notation. In the claim's degree notation, 3+4+1 = 8 is **correct**.

2. **Knot vector = Minkowski sum.** Confirmed by the 2026-04-27 verification and independently by the breakpoint-structure argument: on any interval where neither f's nor w's piecewise structure changes, the integrand is a single bivariate polynomial in (s,u), and the integral is a polynomial in u. A new breakpoint in g(u) occurs when either (a) the active piece of f changes (at u = xi_i + h or u = xi_i - h, where xi_i is an f-breakpoint and +/-h are the kernel boundary) or (b) the kernel boundary crosses an f-breakpoint. Both cases produce breakpoints at {xi_i + eta_j} where eta_j in {-h, h}. For multi-piece kernels with internal breakpoints, the set is {xi_i + eta_j} for all kernel breakpoints eta_j. This is the Minkowski sum. **Correct.**

3. **Control points computable from c_i without piece-by-piece expansion.** Chui and Stockler (1994, "On convolutions of B-splines", J. Comput. Appl. Math.) prove that the convolution of a B-spline expansion f = sum c_i N_{i,p}(t) with a compactly-supported piecewise polynomial w(t) can be expressed as g = sum d_j N_{j, p+q+1}(u) where the d_j are computable from the c_i via a stable divided-difference/blossom recurrence. The computation is linear in the c_i: d = M * c where M depends on the knot vectors and kernel but not on c. **Correct in principle.** However, for non-uniform knots, M is not a Toeplitz matrix (not a discrete convolution in the DSP sense). It is banded (O(bandwidth) nonzeros per row due to compact support), so the computation is O(n * bandwidth). Implementation requires the Chui-Stockler machinery, which is nontrivial.

4. **Numerical stability at 69 seconds, ~90 control points.** The Chui-Stockler recurrence is described as stable. The matrix M is banded and well-conditioned when the B-spline basis is well-conditioned (which it is, by de Boor's stability results for B-spline bases). With ~90 input control points and ~180-650 output control points (depending on kernel piece count), this is a modest computation. Degree-8 polynomial evaluation in f32 accumulates relative error O(8 * eps_f32) ~ 5e-7, giving position error ~0.15 um at 300 mm scale. **Stable.**

5. **Output knot count explosion for multi-piece kernels.** For a kernel with K pieces, the output has O(N_input * K) breakpoints. For smooth_mzv with ~5-7 pieces and N=94 input knots, that's ~500-650 output breakpoints / control points. This is the price of exact representation. Not a stability issue, but an architectural consideration for MCU transmission and evaluation.

6. **Degree-8 evaluation cost on MCU.** Horner evaluation of degree 8 in f32: 8 FMA with 3-cycle dependency chain = 24 cycles per axis. At 4 axes * 40 kHz = 160k evals/s, total = 3.84M cycles/s ~ 0.8% of one 480 MHz H723 core. Acceptable, but notably higher than the degree-3 or degree-4 alternatives (~12-15 cycles). Including velocity (degree 7) and acceleration (degree 6) derivatives roughly triples the budget.

7. **Discrete sample-convolve-refit alternative.** Simpler to implement, numerically stable, but introduces (a) aliasing error at the sampling step (negligible for ~90 control points at even moderate sampling rates like 1 kHz), and (b) refit error at the B-spline fitting step (the exact result is degree 8; fitting as degree 3 introduces approximation error bounded by the degree-reduction theory — Jackson-theorem-style bounds apply, with the convolved function being smoother than the input so convergence is better). If the downstream consumer requires degree 3 anyway, the discrete approach's refit error is comparable to the error any degree-reduction step would introduce. **Simpler and practically equivalent** when the output must be degree-reduced.

### Sources

- https://www.chebfun.org/examples/approx/BSplineConv.html — retrieved 2026-05-26 — cardinal B-spline convolution construction B_{n+1} = B_0 * B_n.
- https://www.cs.jhu.edu/~misha/Notes/BSplines.html — retrieved 2026-05-26 — B-spline recursive convolution definition and degree properties.
- https://www.sciencedirect.com/science/article/pii/0377042794901821 — Chui and Stockler 1994, "On convolutions of B-splines" — non-uniform-knot coefficient computation via divided differences and blossoms, stable recurrence.
- https://dercuano.github.io/notes/spline-convolution.html — retrieved 2026-05-26 — discrete convolution of B-spline coefficients (uniform-knot case).

### Caveats / unchecked assumptions

- The Chui-Stockler paper was not fetched in full (paywalled); the "stable recurrence" characterization is from the abstract and secondary descriptions. The exact recurrence formula was not independently verified.
- The claim assumes a single-piece polynomial kernel (no internal breakpoints). Multi-piece kernels (the realistic smooth-shaper case) multiply the output knot count; the degree formula is unchanged.
- Whether kalico's actual kernel polynomials vanish at their support boundary (affecting multiplicity of boundary-derived knots) was not checked.
- The comparison with the discrete approach assumes the downstream consumer can accept either degree-8 or degree-3 output; if exact degree-8 is required, the discrete approach is not equivalent.
- Source code in `rust/` was not read (math-verification scope only).
