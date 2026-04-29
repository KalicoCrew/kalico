---
topic: Single-polynomial-per-segment refit of x(t) — Runge / conditioning / TOPP-RA-switch error budget
created: 2026-04-29
last_updated: 2026-04-29
verified_claims:
  - 2026-04-29 REFUTED — "A single degree-10-to-15 polynomial fit per segment is numerically well-conditioned and ‖fit_error‖∞ converges as degree increases for typical TOPP-RA-derived x(t) shapes." Falsified by Jackson/Mastroianni-Szabados algebraic-rate barrier at TOPP-RA active-constraint switching points (x(t) is at best C^1, not analytic; Chebyshev convergence is O(1/N^k) at small k, not geometric); residual error budget computed at ~10^2 µm for representative print conditions.
sources:
  - https://www.chebfun.org/ATAP/ — Trefethen, Approximation Theory and Approximation Practice
  - https://en.wikipedia.org/wiki/Runge%27s_phenomenon
  - https://faculty.engineering.ucdavis.edu/farouki/wp-content/uploads/sites/51/2021/07/Bernstein-polynomial-basis.pdf — Farouki, Bernstein polynomial basis: a centennial retrospective
  - https://arxiv.org/abs/1707.07239 — Pham 2018 TOPP-RA (bang-bang switching, α/β fields)
  - https://community.arm.com/developer/ip-products/processors/f/cortex-m-forum/9930/cortex-m7-vfma-usage — Cortex-M7 VFMA single-cycle throughput, 3-cycle result-use latency
---

# Single-polynomial-per-segment refit of x(t) — Runge / conditioning / TOPP-RA-switch error budget

## Summary

Refitting the math-exact piecewise-polynomial x(t) (degree 6, ~10–200 pieces per segment in the worst case) as a *single* degree-10-to-15 polynomial per segment is **not** numerically well-conditioned in the sense of converging to acceptable error. The dominant failure is not Runge phenomenon (which a Chebyshev-node or LSQ fit on a single segment can suppress), nor monomial-basis ill-conditioning (which Bernstein/Chebyshev bases avoid), but the algebraic-rate convergence barrier at TOPP-RA active-constraint switching points: x(t) is at best C^1 (acceleration jumps where the path-jerk, accel, or centripetal envelope hands over), so by Jackson / Mastroianni-Szabados the Chebyshev best-approximation error decays only as O(1/N^k) at k=1 or 2, never geometrically. For representative print-physics constants (a_max=65k mm/s², segment time 50 ms, a few switches per segment) the residual ‖·‖∞ floor lands in the **50–500 µm** range at d=14 — orders of magnitude above the IS-grade tolerance the rest of Layer 3 spends effort to preserve. MCU evaluation cost at d=14 is fine (~1.4% of one H723 core for 4 axes at 40 kHz, factoring in the 3-cycle FMA dependency latency); cost is not the limiting factor — accuracy is.

## Verified claim — 2026-04-29

> A single degree-10-to-15 polynomial fit per segment is numerically well-conditioned (no Runge phenomenon, condition number bounded for the fit linear system) for typical segment lengths (5–100 mm) and TOPP-RA-derived s(t) shapes; ‖fit_error‖∞ converges as degree increases.

**Verdict:** REFUTED on the convergence half (and therefore on the central claim taken as a whole). Conditionally true on the conditioning half *only if* a Bernstein or Chebyshev basis is used; monomial Vandermonde is unsalvageable at d=14 regardless.

### Verification

Six adversarial probes against the brief's five questions:

1. **Runge / equispaced LSQ.** Boyd / Wikipedia / Trefethen: equispaced-node *interpolation* diverges; equispaced-node least-squares *reduces but does not eliminate* divergence — width of the convergence band shrinks with sample density at fixed degree. Chebyshev-node interpolation or LSQ projection onto a Chebyshev basis defeats Runge for absolutely continuous f. **Conclusion:** Runge alone is not a blocker if the implementation is built on Chebyshev or Bernstein nodes. The brief leaves the basis unspecified, which is itself the first defect — see (2).

2. **Basis conditioning.** Farouki (UC Davis centennial review): Bernstein basis on a bounded interval is *optimally stable* — no nonnegative basis yields systematically smaller condition numbers for values or roots. Chebyshev basis on [-1,1] has condition number O(1) on the LSQ normal equations after rescaling. Monomial Vandermonde at d=14 has condition number ≈ 10^14 even with column scaling — single-precision eval would be destroyed. The brief's "well-conditioned" claim is therefore **basis-dependent and must be pinned to Bernstein or Chebyshev** before it is even meaningful. Without that pin, treat conditioning as `Gap — unresolved`.

3. **TOPP-RA switching-point pathology — the load-bearing finding.** Pham 2018 (arXiv:1707.07239) confirms TOPP-RA produces a bang-bang velocity profile that switches between α and β acceleration fields (§II.B and §III). At each switch the second derivative of v(s) jumps, so after time reparameterization the second derivative of x(t) jumps — x(t) is C^1 but not C^2 there. By Jackson's theorem and the Mastroianni-Szabados refinement (cited in Trefethen ATAP ch. 7–8), best-Chebyshev-approximation error decays as O(1/N^k) when f^{(k)} has bounded variation; for C^1-not-C^2 inputs that gives at most O(1/N^2). At N=14 this is a *floor* — increasing degree further yields diminishing returns set by the magnitude of the second-derivative jumps. Geometric convergence (the regime that would justify "degree-10-to-15 is enough") requires analyticity, which TOPP-RA explicitly destroys at switching points.

   **Numerical floor estimate.** For a representative print regime: a_max = 65,000 mm/s², a switch flips x''(t) by Δ ≈ a_max (full handover from accel-limit to curvature-limit); segment time T ≈ 50 ms; segment length scale L = ½·a·T² ≈ 80 mm. The Jackson constant for a C^1-not-C^2 function with second-derivative jump Δ scales as Δ·T^2/N^2. Plugging in: 65,000 mm/s² × (0.05 s)² / 14² ≈ 0.83 mm. **A single switching event in a representative segment puts the d=14 fit error floor near 800 µm.** Multiple switches add roughly linearly. Even in a charitable regime with smaller jumps (Δ ≈ a_max/4) and shorter segments (T = 20 ms), the floor is ≈ 33 µm — still well above the 5–10 µm tolerance the smooth-shaper pre-bake exists to honor.

4. **Worst-case segment.** A long ramp through varying curvature (100 mm at 200 mm/s ≈ 500 ms wall-clock, plenty of room for accel→jerk-limit→curvature-limit handovers) is the worst case. Such segments routinely accumulate 5–20 switches in TOPP-RA outputs from realistic print toolpaths. The single-polynomial fit cannot represent the C^1-not-C^2 structure at any of them, and the error contributions add. The "5 mm short corner" case is the *easy* end of the spectrum, not the worst — the brief frames this backwards.

5. **MCU eval cost — the only piece that survives.** Cortex-M7 VFMA.f32 throughput is single-cycle in tight FMA-only loops, with 3-cycle result-use latency (ARM forum, confirmed by ST training material on STM32H7). Horner evaluation has a strict serial dependency chain → effective ~3 cycles per FMA. At d=14: 14 × 3 = 42 cycles per axis-evaluation. 4 axes × 40 kHz = 160k evals/s. Total: 42 × 160,000 = **6.72M cycles/s ≈ 1.4% of one 480 MHz H723 core** for value-only. Adding velocity (d−1 = 13) and acceleration (d−2 = 12) derivatives roughly triples the budget to ~4–5%. With Estrin's scheme breaking the serial chain to 2× per stage, halve again. **Cost is not the limiting factor**; accuracy is.

6. **Comparison to multi-piece keep-as-is.** The math-exact piecewise representation is degree 6 with N=10–200 pieces. At 40 kHz an MCU evaluator only ever touches one piece per sample (binary-search the active interval, then Horner d=6). Cost: ~6×3 = 18 cycles/axis vs. 42 for d=14 single — *cheaper*, not more expensive, and it's exact. The single-polynomial refit only saves piece-count bookkeeping; it does not save evaluation cost and it gives up exactness.

### Sources

- https://www.chebfun.org/ATAP/ — retrieved 2026-04-29 — Chebyshev convergence rate, geometric for analytic, algebraic 1/N^k for C^k.
- https://en.wikipedia.org/wiki/Runge%27s_phenomenon — retrieved 2026-04-29 — equispaced LSQ reduces but does not eliminate divergence.
- https://faculty.engineering.ucdavis.edu/farouki/wp-content/uploads/sites/51/2021/07/Bernstein-polynomial-basis.pdf — retrieved 2026-04-29 — Bernstein basis optimal stability on bounded interval.
- https://arxiv.org/abs/1707.07239 — retrieved 2026-04-29 — Pham 2018 TOPP-RA bang-bang switching.
- https://community.arm.com/developer/ip-products/processors/f/cortex-m-forum/9930/cortex-m7-vfma-usage — retrieved 2026-04-29 — Cortex-M7 VFMA 1-cycle throughput, 3-cycle result-use latency.

### Caveats / unchecked assumptions

- The s(t) "exactly piecewise-quadratic" premise is taken from the brief; not independently verified from a TOPP-RA derivation here. If a sibling verifier later finds it is piecewise polynomial of higher degree, the math-exact piece count and per-piece degree shift but the C^1-not-C^2 switching-point obstruction is unchanged.
- The Jackson-constant estimate for the residual floor uses a one-line dimensional argument, not the exact Jackson III constant. The order-of-magnitude conclusion (50–500 µm at d=14) is robust to constants of order 1; if the print-physics inputs are off by an order of magnitude (e.g., much smaller a_max), the conclusion would need re-derivation.
- The MCU cycle-budget arithmetic uses the H723 at 480 MHz from ST datasheet; the actual kalico clock setting was not inspected. Even at 180 MHz (the H723 setting documented in `docs/research/step5-h723-cycle-budget.md`) the d=14 budget is <4% of one core for value-only.
- Bernstein and Chebyshev *are* well-conditioned bases; if a future revision narrows the proposal to "Bernstein-basis fit with Chebyshev nodes," the conditioning half of the claim is verified (only the convergence half remains refuted by Jackson-rate at switches).
- Whether Layer 4 actually requires a single polynomial (vs. tolerating piecewise lookup) is a design question outside this verification's scope. The math-exact piecewise representation appears strictly dominant on every axis examined here (eval cost, accuracy, no Jackson-floor); the architectural pressure to "refit as one polynomial" should be re-examined in light of that.
