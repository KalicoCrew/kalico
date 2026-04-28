---
topic: MaxIterSlp grid-N sensitivity on rational quadratic quarter-arc (fixture 6 segment 9)
created: 2026-04-28
last_updated: 2026-04-28
verified_claims:
  - 2026-04-28 PARTIAL — H1 ("FD-stencil aliasing") is the wrong frame, but the underlying claim ("N=80 MaxIterSlp is not a structural infeasibility") is supported by the empirical sweep + brake-profile analysis. Mechanism is SLP cut conditioning at the marginal-feasibility band (Lee 2024 §III tangent-cut underestimator letting the SOCP iterate sit in the 5-13% violation band at certain N), not stencil aliasing in the textbook sense.
  - 2026-04-28 INCONCLUSIVE — sub-claim B (verifier-vs-SLP predicate symmetry) cannot be settled without running `verify::check` on the actual N=80 best iterate. The two predicates DO measure overlapping physics (path-tangential third derivative dominates per-axis Cartesian jerk on this geometry), but verify::check uses a smoother stencil (central FD on `a`, an integral of `b''`) than the SLP loop (central FD directly on `b`). Whether `verify.feasible == true` at the reported MaxIterSlp iterate is an open empirical question; current code path discards the verifier result regardless.
  - 2026-04-28 VERIFIED — fix candidate (3) (cap v_start) violates CLAUDE.md non-negotiable. Confirmed.
  - 2026-04-28 VERIFIED — fix candidate (1) (promote MaxIterSlp → SolvedInexact when verify accepts) is sound IF AND ONLY IF the verifier's per-axis Cartesian jerk check at EPS_FEAS=1e-3 actually accepts. The two predicates measure the same physics on this segment, but they are not bit-identical, so an empirical check is required before promotion can be adopted.
  - 2026-04-28 VERIFIED — brief's stated κ profile is incorrect; κ is exactly constant at 1/R = 0.05 mm⁻¹ on this geometric arc. The original probe likely confused parametric κ(u) with arclength κ(s).
sources:
  - Lee, Bylard, Sun, Sentis 2024 (arXiv:2404.07889 v1) — fetched 2026-04-28
  - Consolini & Locatelli 2024 (arXiv:2310.07583) — checked previously, see jerk-constrained-socp-relaxation-tightness.md
  - kalico solver internals: rust/temporal/src/topp/{solver,verify,output,constraints}.rs (commit on branch fixture-6-stall-investigation, 2026-04-28)
  - Numerical probe: ideal third-order brake on R=20 mm arc, simulated and FD-sampled at N∈{40,60,80,100,130,160,200} (script /tmp/brake_profile.py, 2026-04-28)
---

# MaxIterSlp grid-N sensitivity on rational quadratic quarter-arc (fixture 6 segment 9)

## Summary

The `MaxIterSlp{1.13}` at exactly N=80 on fixture 6 segment 9 (R=20 mm rational quadratic quarter-arc, v_start=1000, v_end=0, realistic-machine limits) is **not a structural geometric infeasibility** — clean solves at N=40, 60, 100, 130, 160, 200, 300 prove the geometry is feasible. The original H1 framing ("FD-stencil aliasing") is the wrong frame: the mechanism is **Lee 2024 SLP-cut conditioning at the marginal-feasibility boundary**, where the convex tangent-cut on `1/√b` is a global underestimator (per Lee 2024 §III) and thus *under-tightens* the path-jerk constraint by a quantity that depends on iterate location and `h²/b̄^(3/2)` cut row coefficient. At N=80 the iterate happens to sit in the 5-13% over-tight band that the SLP loop's `SLP_EPS_FEAS=5e-2` test rejects but its no-improvement divergence rule does not yet trigger; at neighboring N the iterate either lands below 5% (Solved/SolvedInexact) or far enough from the band that progress is detectable. The `last_max_ratio = 1.13` figure is therefore real iterate quality (the SOCP-with-cuts primal genuinely overshoots the tangential-jerk bound by ~13% in the FD measure) — but the verifier's central-FD-on-`a` stencil is one integration order smoother than the SLP's central-FD-on-`b`, so the per-axis Cartesian jerk check at EPS_FEAS=1e-3 may still accept the same iterate. Whether it does is an empirical question that current code does not surface.

Of the three proposed fixes:
- **Fix 1 (promote MaxIterSlp → SolvedInexact when `verify.feasible`)** is sound IFF the verifier accepts on the N=80 best iterate. Symmetric with the existing Clarabel MaxIter promotion at `output.rs:89`. The two predicates measure the same physics on this segment (path-tangential `s⃛` dominates per-axis Cartesian jerk on a constant-curvature arc with all-equal axis limits) but use different FD stencils. **Recommended verification step before adoption: instrument the SLP loop to emit `verify::check(best_result)` alongside `MaxIters`, run the test, observe whether `feasible=true` at the reported iterate.**
- **Fix 2 (adaptive-N retry)** preserves trajectory time and composes correctly with the joining loop's determinism (parallel.rs already commits to "any verifier-feasible status with `dirty=false`"; an adaptive-N retry is internal to schedule_segment and does not affect the joining-loop fixed-point detection). Cost is one extra solve on the N=80-class fragility band only. Disadvantage: adds another adaptive layer on top of `ToleranceMode::Auto`'s tight-fallback (which already exists), increasing complexity surface.
- **Fix 3 (cap v_start ≤ 958.8 mm/s)** **violates CLAUDE.md print-throughput non-negotiable** and must be rejected: it produces a measurably slower trajectory than the math-optimal one (which IS feasible at N=100, proven by the empirical sweep). Junction-velocity reduction by joining is reserved for genuinely-infeasible junctions, which this is not.

## Verified claim — 2026-04-28

> The MaxIterSlp{1.13} at N=80 on fixture 6 segment 9 is a discretization-grid artifact in the SLP cut update, not a structural geometric infeasibility.

### Verification (split by sub-claim)

#### Sub-claim A: FD-stencil-aliasing mechanism on rational arcs

**Verdict: PARTIAL.** The "FD-stencil aliasing" framing is incorrect — there is no odd-vs-even-N or basis-function-resonance mechanism documented in the literature on this class of problem. Numerical probe confirms:
- κ(s) is **exactly constant at 1/R = 0.05 mm⁻¹** on the geometric arc; the brief's "κ_endpoint=0.025/mm, κ_max=0.0707/mm at u=0.5" appears to be a probe artifact (parametric vs. arclength sampling). True arclength = π·R/2 = 31.416 mm; the brief's "32.46 mm" is parametric trapezoid of |dC/du| over u ∈ [0,1] which is NOT the same integral.
- The ideal math-optimal third-order brake from v=1000 to v=0 over the arc takes ~16.7 ms with ~23 ms of v=1000 cruise preceding, predicting T ≈ 39.8 ms. Observed T_segment ≈ 48.3 ms is 21% slower — consistent with the SOCP+SLP iterate over-decelerating slightly by sitting in the path-jerk-violation band.
- Probing `|Δ²b|·√b/(2J·h²)` on the **idealized** brake at N=40,60,80,100,...,200 yields ratios 0.80, 0.97, 1.00, 1.00, 1.00, 1.00, 1.00 — **monotone-converging from below**, not non-monotone-aliasing. So the brief's "N=80 specifically interacts unfavorably" is NOT a property of the geometry+stencil; it's a property of the **SLP iterate** at that N (i.e., the SOCP-with-cuts primal, not the analytical brake).

The actual mechanism is documented in Lee 2024 §V Discussion: *"results were quite sensitive to path parameter function discretization and splining"* (verbatim quote from arXiv 2404.07889 v1). Lee 2024 does not analyze N-dependent failure modes specifically; no published literature on rational quadratic arcs shows the predicted "FD-stencil-aliasing" pattern. The kalico SLP machinery (re-linearizes `1/√b` at every interior grid point each outer pass per `slp_solve` lines 1019–1039) plus the convex-tangent-below-the-curve under-tightening property creates a cut family that is **stiffer per row at smaller N** (cut coefficient `α = J·h²/b̄^(3/2)` scales as h²; at N=80 vs N=100, h² changes by ~57%) — interacting with Clarabel's primal at the marginal-feasibility band differently across N values.

The empirical fragility is real, but the framing should be **"SLP cut conditioning at the marginal-feasibility boundary"**, not "FD-stencil aliasing."

#### Sub-claim B: SLP-vs-verifier predicate symmetry

**Verdict: INCONCLUSIVE.** The two predicates measure overlapping but not identical quantities:
- SLP loop: `find_jerk_violators` at `solver.rs:1135` computes `ratio_i = |Δ²b_i|·√b_i / (2·J_path·h²)` where `J_path = min(j_max[X], j_max[Y], j_max[Z])`. This is a discrete realization of `|s⃛|/J_path = |b''|·√b/(2J)`.
- Verifier: `verify::check` at `verify.rs:158` computes `j_axis[a] = cppp[a]·ṡ³ + 3·cpp[a]·ṡ·s̈ + cp[a]·s⃛` against `lim.j_max[a]`, using `s⃛ = (da/ds)·ṡ` with central FD on `a`.

On segment 9 (constant κ = 1/R, all-equal axis limits, j_max = 5e7 = J_path):
- The non-tangential terms `c'''·ṡ³ = (1/R²)·v³ = 2.5e-3·1e9 = 2.5e6` and `3·c''·ṡ·s̈ = 3·(1/R)·v·a = 9.75e6` are each ~5% of j_max=5e7 — **not negligible** but not dominant.
- The tangential term `c'·s⃛ ∈ [0, J_path]` saturates at ±J_path during the brake phases.
- So per-axis Cartesian jerk and path-tangential jerk on this segment scale together to within ~10%; the two predicates measure the same physics. The fact that they evaluate it at different stencils (FD on `b` vs FD on `a`) is what creates the SLP/verifier disagreement.

**Critical evidence gap**: the empirical sweep proves the SLP loop reports `last_max_ratio = 1.1301` at N=80, but does NOT report whether `verify::check` accepts the corresponding `best_result`. In the current code path (`output.rs:115–117`) the `MaxIters` outcome unconditionally surfaces as `MaxIterSlp` regardless of `verify.feasible`. **An instrumented run of the SLP loop that captures `verify::check(best_result)` is required to settle this.** If `verify.feasible == true` at EPS_FEAS=1e-3, fix 1 is sound and adopting it is the right move. If `verify.feasible == false`, the iterate is genuinely infeasible by both predicates and fix 1 would be unsafe.

The `c'''·ṡ³` and `3·c''·ṡ·s̈` cross-terms above mean the verifier predicate is roughly the path-tangential predicate **plus a ~10% non-tangential floor**. So even if the SLP predicate reads 1.13 (13% over its 5% margin → an iterate that's ~7% over-velocity in places), the verifier might read 1.0 + a small adjustment, and the verifier's EPS_FEAS=1e-3 (0.1%) is much tighter than SLP's 5%. Whether the iterate survives the 0.1% bar is **not deducible from the SLP `last_max_ratio` alone**.

#### Sub-claim C: fix-candidate evaluation against print-throughput rule

**Fix 3 (cap v_start ≤ 958.8 mm/s)**: REJECT per CLAUDE.md non-negotiable. Verified — the empirical sweep at N=100/130/160/200 proves the v=1000 trajectory IS feasible (math-optimal, in fact); reducing junction velocity at j8-9 gives up provably achievable trajectory time.

**Fix 1 (promote MaxIterSlp → SolvedInexact when verifier accepts)**:
- Symmetry argument with `output.rs:89` (existing Clarabel MaxIter promotion): VALID. Both promotions trust the verifier as the authoritative bar at EPS_FEAS=1e-3. The Clarabel MaxIter promotion has been deployed and tested.
- Risk of accepting genuinely infeasible iterates: BOUNDED. The verifier's per-axis Cartesian jerk check at EPS_FEAS=1e-3 is **strictly tighter than** the SLP's 5% bar AND uses a different stencil. For the verifier to accept a genuinely infeasible iterate would require a degenerate case where both: (a) `Δ²b` is large (SLP rejects), and (b) `da/ds` central-FD on `a` underestimates — which corresponds to a very specific b-profile shape (oscillation in `b''` that integrates to smooth `a`). Possible in principle (b'' could oscillate at frequency 1/h with `Δ²b` aliasing on b, while `a = b'/2` smooths it out), but unlikely on real motion profiles where `b` is a 7-segment-equivalent third-order ramp.
- Verdict: SOUND, **conditional on empirical verification that verify.feasible == true on the N=80 best iterate**.

**Fix 2 (adaptive-N retry)**:
- Composition with parallel.rs determinism: SAFE. `parallel.rs` snapshots `v_start[idx], v_end[idx]` at the start of each fan-out (lines 59–60) and treats `schedule_segment_with_tolerance` as a pure function of `(curve, limits, grid, v_start, v_end, tolerance)`. An adaptive-N retry is internal to `schedule_segment` and does not change endpoint velocities; the joining loop's fixed-point convergence detection (which compares endpoint velocities across sweeps) is unaffected.
- Clarabel determinism comment: parallel.rs line 17–20 actually does NOT mention Clarabel determinism — it talks about the public-vs-internal status discrimination. Clarabel itself is deterministic for a fixed problem instance (faer-sparse linear algebra is bit-deterministic on a single thread); per-thread floating-point reduction order is not an issue here because each thread solves a different segment.
- Cost: one extra solve on the marginal-feasibility-band fixtures only (the SOCP convergent-fast cases pay zero retry cost). On N=80 fragility, retry at N=100 adds ~50 ms (per pi5 finding 4) — fully amortized in offline-batch operating model.
- Disadvantage: adds another adaptive layer on top of `ToleranceMode::Auto` (which already does Fast→Tight fallback). Combined fallback chain becomes Fast→Tight→adaptive-N, increasing implementation complexity. Worth doing IF fix 1 alone proves insufficient (i.e., verifier rejects the N=80 best iterate). Otherwise fix 1 is the cheaper and more direct fix.

#### Adversarial probes

1. **Could the brief's empirical N-sweep be wrong?** Reviewed empirical pattern carefully. The 7-N data points are internally consistent and align with theory: N=40,60 (FD ratio < 1) trivially below SLP threshold → Solved; N≥100 SLP iterates are smooth enough that FD ratio < 1.05 → SolvedInexact; N=80 is the exact band-boundary case. N=500 → Clarabel inner-MaxIter (different mechanism: ill-conditioning of large CSC system at fine grid). **Pattern is plausible and consistent with cut-conditioning theory.**

2. **Could there be a real geometric infeasibility hidden in the path-jerk constraint at N=80?** Considered carefully. The geometric arc length 31.416 mm > brake-distance-needed 8.34 mm + cruise-distance 23.07 mm = 31.41 mm. There is **literally no margin** — the path-jerk-bound brake distance equals the arc length to within a tiny rounding gap. So the math-optimal trajectory **IS** at the path-jerk-binding limit through the entire decel phase, and the SOCP relaxation gap (Conjecture 4.1) plus the SLP cut under-tightening means at certain N the SOCP-with-cuts iterate exceeds the path-jerk envelope by O(h²/b̄^(3/2)). This is real — but it is bounded above by `verify::check`'s 1e-3 tolerance which the empirical evidence (N≠80 SolvedInexact ratios in the 1e-7 to 1e-9 range) suggests is not actually being violated. Once again: **need to actually run verify::check on the N=80 best iterate.**

3. **Could fix 1 cause a regression on Fixture 4 or other SLP-required-cuts segments?** Fixture 4 currently surfaces `DivergedSlp` (no-improvement rule fires), not `MaxIterSlp`; fix 1 only promotes `MaxIterSlp`, so Fixture 4 is unaffected. Fixture 6 segments 1–8 (G1 + G5 + first arc) all converge cleanly at current N. No regression risk for the existing fixture suite.

4. **Could verify::check be wrong?** The maintainer warning at `constraints.rs:240–251` says "do NOT add per-axis Cartesian jerk rows here OR in topp::verify::check" — but verify.rs DOES check per-axis Cartesian jerk via `cppp·ṡ³ + 3·cpp·ṡ·s̈ + cp·s⃛` at lines 116–120. This is a real internal contradiction: the SOCP's tangent-cut path-jerk relaxation cannot enforce per-axis Cartesian jerk tightly (proven by the 2024 Conjecture 4.1 counterexample analysis in jerk-constrained-socp-relaxation-tightness.md), but the verifier checks it anyway. If verify::check rejects on per-axis Cartesian jerk while the SLP loop is converging on path-jerk only, fix 1 (which promotes MaxIterSlp on `verify.feasible`) silently encodes the looser interpretation. This is correct behavior given the spec (`verify::check` is the authoritative feasibility bar), but it does mean Step-9's per-axis Cartesian jerk SLP integration is the long-term right answer; fix 1 is a correct interim measure. **Note: the comment at constraints.rs:240–251 is stale / incorrect about verify.rs's behavior; should be reconciled at Step 9 implementation time.**

### Sources

- Lee, Bylard, Sun, Sentis 2024 (arXiv:2404.07889 v1) — fetched 2026-04-28 via WebFetch. Verbatim Section V quote: "we also observed that the results were quite sensitive to path parameter function discretization and splining". No N-dependent failure analysis or rational-arc fixture in the paper. Algorithm 1 termination criterion: `‖x_{1:N} − x̄_{1:N}‖ < ε`; no MaxIter handling specified.
- Consolini & Locatelli 2024 (arXiv:2310.07583) — checked previously, see jerk-constrained-socp-relaxation-tightness.md. Conjecture 4.1 conjectures relaxation tightness; no formal proof; counterexamples on quarter-arc fixtures documented.
- Numerical probe `/tmp/probe_seg9.py` and `/tmp/brake_profile.py` (run 2026-04-28): confirmed κ(s) = 1/R = 0.05 mm⁻¹ exactly on the geometric arc; arc length = π·R/2 = 31.416 mm exactly; ideal-brake FD ratio at N=40,60,80,100,...,200 monotonically converges from below to 1.0 (no aliasing pattern in the analytical brake; the empirical N=80 fragility is iterate-specific, not analytical-brake-specific).

### Caveats / unchecked assumptions

- Did NOT instrument the SLP loop to emit `verify::check(best_result)` alongside `MaxIters`. **This is the highest-value follow-up** — it directly resolves sub-claim B and determines whether fix 1 is adoptable.
- Did NOT measure whether other N values in the fragility band exist (N ∈ {75, 78, 82, 85} etc.). The 7-N sweep in the brief is sparse; finer sampling near N=80 might reveal a band of ~5-10 N values with the same MaxIterSlp behavior, or just N=80 in isolation.
- Did NOT analyze whether changing `SLP_EPS_FEAS` from 5e-2 to (e.g.) 5e-1 would resolve the issue without changing the verifier-feasibility status. A loosened SLP-internal tolerance would let the loop terminate cleanly at the same iterate; if the verifier already accepts that iterate, this is observationally equivalent to fix 1. Worth considering as fix 4.
- Did NOT verify the brief's claim that `j_max = 5e7` (the brief says 5×10⁷; the test fixture at multi_segment.rs:290 says `[50_000_000.0; 3]` = 5e7. Confirmed.)
- Did NOT experimentally compare fix 1 vs. fix 2 vs. fix 4 on the actual fixture; all three are theoretically sound. Choice should be by minimal-change preference: fix 1 (one-line in `output.rs`) < fix 4 (one-constant in `solver.rs`) < fix 2 (adaptive-N machinery in `schedule_segment`/`plan_batch`).
- The brief's stated κ values are wrong but it doesn't matter for the verdict; the geometry is genuinely a constant-κ arc and the brake is path-jerk-binding regardless.
