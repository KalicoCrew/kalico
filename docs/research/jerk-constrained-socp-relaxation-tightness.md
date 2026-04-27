---
topic: Jerk-constrained TOPP SOCP relaxation tightness (Consolini-Locatelli 2024 Conjecture 4.1)
created: 2026-04-27
last_updated: 2026-04-27
verified_claims:
  - 2026-04-27 VERIFIED — The path-jerk constraint |b''|·√b ≤ 2J is genuinely non-convex (hyperbolic in (b, b'')); no single-SOCP convex reformulation enforces it tightly. The Consolini-Locatelli 2024 SOCP relaxation is conjecturally exact (Conjecture 4.1) but empirically loose on the R=20mm rational-quadratic 90° arc fixture. Lee 2024 SLP outer iteration with first-order Taylor cuts on 1/√b is the correct literature-grounded fix.
sources:
  - Consolini & Locatelli, "Is time-optimal speed planning under jerk constraints a convex problem?" Automatica 2024, arXiv:2310.07583
  - Lee, Bylard, Sun, Sentis, "On the Performance of Jerk-Constrained Time-Optimal Trajectory Planning for Industrial Manipulators," ICRA 2024, arXiv:2404.07889
  - Verscheure et al., "Time-Optimal Path Tracking for Robots," IEEE T-AC 2009
  - Pham 2018 IEEE T-RO (TOPP-RA, arXiv:1707.07239); Pham 2017 ICRA (TOPP3, arXiv:1609.05307)
---

# Jerk-constrained TOPP SOCP relaxation tightness

## Summary

The Consolini-Locatelli 2024 SOCP relaxation for time-optimal speed planning under jerk constraints is **not provably tight**. The authors state Conjecture 4.1 ("the convex relaxation is exact") but explicitly note "we have not been able to derive [a formal proof]." The kalico R=20mm rational-quadratic 90° arc fixture (TOPP prototype Step 4) is an empirical counterexample: at grid 184 the path-jerk constraint is violated by ratio 2.43×, with the SOC chain in the implementation's block (h) going slack while block (f)'s path-jerk row binds. The structural flaw is that block (f) and the chain both encode lower bounds on the same auxiliary t, and two lower bounds A≤t and B≤t do not imply A≤B. No conic substitution (`w = 1/√b`, power cones, Pham-line, Verscheure-line) closes the gap in a single SOCP — direct Hessian analysis shows the desired feasible region is non-convex. The remedy endorsed by spec §11 and confirmed here is Lee 2024 SLP outer iteration with first-order Taylor cuts on 1/√b at violator grid points.

## Verified claim — 2026-04-27

> The block-(f) path-jerk constraint `|b''|·√b ≤ 2J` in the Consolini-Locatelli SOCP relaxation used by `rust/temporal/src/topp/constraints.rs` is genuinely hyperbolic in (b, b'') and CANNOT be enforced tightly by any single-SOCP convex reformulation. The diagnosed slackness on the R=20mm 90° rational-quadratic arc at grid 184 (ratio 2.4318) is therefore a fundamental gap in the relaxation, not a coding bug. The recommended fix is SLP outer iteration per Lee 2024 (arXiv:2404.07889 §III–§IV).

### Verification

Six adversarial attacks were run against the claim. All landed on the side of the claim being correct:

1. **Hyperbolicity / `w = 1/√b` substitution.** The desired tightening direction requires `b·w² ≤ 1` on the positive orthant. Hessian of `f(b,w) = b·w²` is `[[0, 2w], [2w, 2b]]` with determinant `−4w² < 0` — indefinite, sublevel set non-convex. Counterexample: `(4, 0.5)` and `(0.25, 2)` both satisfy `bw²=1`, but the midpoint `(2.125, 1.25)` gives `bw²=3.32 > 1`. The opposite direction `b·w² ≥ 1` IS conic (3-d power cone, α=1/3), but it's the wrong direction — it relaxes rather than tightens.
2. **Lee 2024 convergence rigor.** Convergence is empirical only. Stopping criterion: `‖x_{1:N} − x̄_{1:N}‖ < ε`. Iteration count not reported. One named failure mode: "the solution can frequently sacrifice motion time or smoothness excessively depending on the input path." This is a real implementation risk but does not refute the recommendation; the alternative single-SOCP is empirically broken on the diagnostic fixture.
3. **Pham/Verscheure dismissal.** Verscheure 2009's `b = ṡ²` substitution is degree-1 in `s̈` and stops at the accel envelope. Pham 2018 TOPP-RA core is accel-only by construction. Pham 2017 TOPP3 extends to jerk but uses a numerical reachability sweep, not a single SOCP — itself iterative, no single-SOCP closure.
4. **Numerical reproduction.** `|Δ²b|·√b/(2Jh²) = 88.05·137.62 / (2·1e5·0.157869²) = 2.4309` ≈ reported 2.4318 (4 sig figs). Algebra is consistent with derivation `|b''|·√b ≤ 2J ⇔ |Δ²b|·√b ≤ 2Jh²`.
5. **Cap-fix interaction.** Block (c)'s row-skip predicate `(v_max,axis/|c'_axis|)² > B_MAX_CENT_CAP` depends only on geometry (path tangent and user-set v_max/cap), not on solved `b` values. SLP cuts on `b` cannot un-make those skips.
6. **Spec §11 alignment.** The SLP-fallback bullet (line 517) is sibling to (not nested under) the per-axis Cartesian jerk bullet (line 515). It explicitly applies "if Consolini-Locatelli SOCP fails on fixture 5 or any future representative input." The diagnostic fixture qualifies.

The Consolini-Locatelli 2024 paper itself acknowledges in Section 7.2 that exactness fails when constraints vary along the curve. The kalico fixture exhibits effectively-position-dependent binding of the centripetal limit (the high-curvature region of the rational-quadratic arc dominates), which interacts with the constant-J path-jerk envelope in exactly the way the paper warns is outside its proven-empirical regime.

### Sources
- Consolini & Locatelli 2024 (arXiv:2310.07583) — fetched HTML mirror, retrieved 2026-04-27. Conjecture 4.1 verbatim: "The convex relaxation (13) is exact, i.e., its optimal value is equal to the optimal value of (10), and given an optimal solution (w⋆,t⋆) for (13), w⋆ is feasible and optimal for (10)." Followed by: "However, in spite of many attempts to give a formal proof of this conjecture, up to now we have not been able to derive it."
- Lee, Bylard, Sun, Sentis 2024 (arXiv:2404.07889) — fetched HTML v1, retrieved 2026-04-27. SLP scheme: "linearizing the nonlinear term h_k(x_{k:k+2}) and leveraging its convexity to ensure the solution always satisfies the original constraints"; convergence empirical, no formal proof.
- 2025 follow-up arXiv:2503.09424 (re-uploaded as 2510.24286) — abstract checked; extends Consolini-Locatelli to vehicle travel-time-and-energy objective but does NOT strengthen the relaxation tightness for jerk; retrieval 2026-04-27.

### Caveats / unchecked assumptions
- Verification did not run the kalico solver to independently reproduce the 2.43× violation; the diagnosis JSON's numerical values were taken at face value (algebra cross-checked).
- Lee 2024 §III–§IV was read via the arXiv HTML mirror, which the WebFetch summarized; the exact algorithm pseudocode (Algorithm 1) was not transcribed line-for-line. The cut-construction sign in the proposed fix is consistent with 1/√b being convex (tangent below the curve = conservative), but a pen-and-paper re-derivation against the paper's notation is recommended at implementation time.
- The verification did not survey post-2024 literature exhaustively; web search at retrieval date returned no paper claiming to close the single-SOCP gap. New 2026 results, if any, are not covered.
- Per-axis Cartesian jerk (the `3·c''·v·a` cross-term mentioned in the source-file comment at L237–243) is a separate non-convexity, also Step-9 territory, and was not in scope for this verification — though the same SLP machinery would naturally extend.
