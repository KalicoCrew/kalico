---
topic: Condensed smooth-chain SOCP for the multi-segment joining layer (junction condensation exactness, non-uniform stencils, corner-sweep monotonicity, infeasibility modes)
created: 2026-06-07
last_updated: 2026-06-07
verified_claims:
  - 2026-06-07 VERIFIED — Sharing one b_m and one a_m at a smooth (tangent-continuous) junction is convex and is the physically correct (not relaxation-loss) model, because a finite jerk bound forces the tangential acceleration s̈=b'/2 to be continuous in time across any junction, including curvature-discontinuous (G1-not-G2) ones. The per-segment one-sided decoupled accels were the LOOSER model.
  - 2026-06-07 VERIFIED-WITH-CAVEATS — Non-uniform 3-point stencils: a_m = b'_m/2 with b'_m = c_{-1}b_{m-1}+c_0 b_m+c_{+1}b_{m+1}, coefficients given; the junction second-difference for the jerk row is the Taylor-exact non-uniform 2nd-derivative stencil, which is O(h) (first order) when h_L≠h_R and O(h²) only at h_L=h_R. Conditioning spread grows ~max(h_L,h_R)/min(h_L,h_R).
  - 2026-06-07 VERIFIED-WITH-CAVEATS — Corner bidirectional min-sweep converges to the chain-decomposed optimum in the continuous limit because, with corner accel FREE, T(v_entry,v_exit) is monotone non-increasing and feasible boundary velocities form an interval [0,v_reach]. At finite N the discrete transition map is "not monotonic over the whole controllable set" (Pham TOPP-RA) — an O(grid) caveat the per-segment sweep already carries; chains shrink the coupling surface, never enlarge it.
  - 2026-06-07 VERIFIED-WITH-CAVEATS — Smooth-junction interior infeasibility cannot occur because junction b is a free variable; the only residual infeasibility is a corner cap pinned above what the adjacent chain can reach, covered exactly by the corner-endpoint bisection fallback.
sources:
  - Pham & Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis" (TOPP-RA), IEEE T-RO 2018, arXiv:1707.07239
  - Lee, Bylard, Sun, Sentis, "On the Performance of Jerk-Constrained Time-Optimal Trajectory Planning for Industrial Manipulators," ICRA 2024, arXiv:2404.07889
  - Consolini & Locatelli, "Is time-optimal speed planning under jerk constraints a convex problem?" Automatica 2024, arXiv:2310.07583
  - LeVeque, "Finite Difference Methods for ODEs and PDEs"; Gordon College CPS343 finite-difference notes (non-uniform 3-point stencil order)
---

# Condensed smooth-chain SOCP for the multi-segment joining layer

## Summary

The proposed "condensed smooth-chain SOCP" redesign is mathematically sound. (1) Sharing one b and one accel variable at a smooth junction is convex and physically EXACT, not a relaxation gap: the finite jerk bound makes tangential acceleration Lipschitz-in-time, hence continuous across any junction — so the single shared a_m is the correct model and the old per-segment one-sided decoupled accels were the looser relaxation. (2) The non-uniform stencils are derivable in closed form; the junction second-difference for the jerk row is Taylor-exact for b'' but only O(h) accurate when h_L≠h_R (O(h²) at h_L=h_R), with conditioning spread ~max/min spacing ratio. (3) The corner min-sweep retains global optimality in the continuous limit because corner accel is free; at finite N it inherits the same TOPP-RA non-monotonicity caveat the per-segment sweep already has, on a strictly smaller coupling surface. (4) Free junction b removes interior reachability failures; the only residual infeasibility is a corner cap above an adjacent chain's reach, covered by the existing corner-endpoint bisection. The most important IMPLEMENTATION hazard (item 5) is that the SOC time-chain block (h) and the Σt objective currently hardcode a single scalar h and MUST become point-local h_i across a spacing change, or the objective mis-costs traversal time.

## Verified claims — 2026-06-07

### Continuous identities (foundation for all four claims)

Symbolically verified (sympy, /tmp/verify_identity_clean.py) with an explicit smooth s(t):
- a = s̈ = b'(s)/2 (matches constraints.rs block (b)).
- path jerk s⃛ = √b · b''(s) / 2, exact (residual 0). Hence the uniform jerk row |b_{i+1}−2b_i+b_{i−1}|·√b_i ≤ 2J·h² is exactly |s⃛| ≤ J under b'' ≈ Δ²b/h². Matches stencil.rs::s_dddot_at = √b · b''/2.

### CLAIM 1 — condensation exactness & convexity: VERIFIED

Convexity: every junction row is either linear (boundary equalities, accel-linkage finite differences, per-axis velocity/accel rows, centripetal, e2 envelope) or a member of the existing SOC family (jerk chain). Emitting both sides' geometry rows at the shared point adds linear rows; sharing variables removes columns. No new nonlinearity ⇒ still a convex SOCP. (i) holds.

Exactness (ii) — the load-bearing finding. The question is whether forcing a SINGLE a_m (one b' value) where the per-segment formulation allowed two one-sided accels cuts off feasible profiles. Worked feasibility example (/tmp/verify_claim1.py): at a curvature-sign-flip junction (c'_L=c'_R=1, c''_L=+2, c''_R=−2), block (d) gives left a-interval and right a-interval that become DISJOINT as b_m rises; per-segment could pick inconsistent one-sided accels, chain cannot. So the chain is strictly MORE restrictive at the junction.

But this restriction is physically correct, not a loss:
- At a smooth junction the path is traversed at ONE instant with ONE dv/dt; two distinct one-sided tangential accels are unphysical for a no-stop junction.
- Decisive regularity argument: a finite jerk bound J<∞ makes s⃛=d(s̈)/dt bounded ⇒ s̈(t) is Lipschitz ⇒ s̈ CONTINUOUS in time across any junction, INCLUDING curvature-discontinuous (G1-not-G2) ones. Since s̈ = b'/2 and ṡ>0, b'(s) is continuous at the junction in the true continuous optimum. The single shared a_m models exactly this continuity; the per-segment decoupled one-sided accels were the LOOSER relaxation that admitted unphysical tangential-accel jumps.
- This is independently corroborated by Lee 2024 (arXiv:2404.07889) discussion: spline-induced jerk discontinuities between segments "can be mitigated by constraining acceleration changes in transitions between spline segments" — i.e. the jerk-feasible model wants accel continuity at junctions.

Discretization order of the residual effect: the discrete non-uniform centered stencil for a_m equals the true b'/2 to O(h) when h_L≠h_R (O(h²) at h_L=h_R). This is the SAME order as today's per-segment one-sided endpoint diffs (also O(h)). So condensation never degrades junction accuracy below today's per-segment endpoint accuracy; at h_L=h_R it improves it. No relaxation gap from sharing b; the only "cost" is an O(h) modeling of the (continuous) accel at the junction, identical in order to the existing endpoint stencils.

### CLAIM 2 — non-uniform stencils: VERIFIED-WITH-CAVEATS

Grid: s_{m−1}=s_m−h_L, s_m, s_{m+1}=s_m+h_R.

First-derivative (for a_m = b'_m/2), coefficients:
- c_{−1} = −h_R / (h_L(h_L+h_R))
- c_0    = (h_R−h_L) / (h_L h_R)
- c_{+1} = h_L / (h_R(h_L+h_R))
a_m = ½·(c_{−1} b_{m−1} + c_0 b_m + c_{+1} b_{m+1}). Numerically verified: coefficient sum = 0, weighted-offset sum = 1 (consistency). O(h²) at h_L=h_R, O(h) otherwise.

Second-difference for the junction jerk row, Taylor-exact 2nd-derivative coefficients:
- d_{−1} = 2 / (h_L(h_L+h_R))
- d_0    = −2 / (h_L h_R)
- d_{+1} = 2 / (h_R(h_L+h_R))
b''_m ≈ d_{−1}b_{m−1} + d_0 b_m + d_{+1}b_{m+1}.

The jerk envelope at the junction must read |b''_m|·√b_m ≤ 2J (the continuous bound |s⃛|≤J), i.e.
|d_{−1}b_{m−1} + d_0 b_m + d_{+1}b_{m+1}|·√b_m ≤ 2J.
NOTE: the uniform row's RHS "2J·h²" is the uniform form of "2J / (coefficient scale)". With the non-uniform d-coefficients above, do NOT reuse the uniform 1/(2hJ) scaling — fold the d-coefficients directly and keep RHS = 2J on the |b''|√b form, or equivalently RHS = 2J with the b'' second difference already carrying the non-uniform 1/(h·h) weights. This makes the bound mean the SAME scalar J on both sides regardless of h_L,h_R. (The uniform code packs the factor as hj=2·h·J and divides Δ²b by hj; the non-uniform analogue divides the non-uniform second difference's numerator structure by 2J after multiplying by √b — implement on the b'' form to avoid sign/scale slips.)

Truncation order: numerically confirmed (/tmp/verify_nonuniform_order.py) leading error = (h_R−h_L)/3 · b''' + O(h²). So O(h) when h_L≠h_R, O(h²) at h_L=h_R. Literature: standard non-uniform 3-point 2nd-derivative is first-order on a general mesh (LeVeque; Gordon CPS343 notes); central first-derivative likewise O(h) with |h_+−h_−|·‖f''‖ error term.

Conditioning hazard: coefficient magnitude spread ≈ max(h_L,h_R)/min(h_L,h_R) (verified: ratio 1→spread 2, 10→11, 100→101). solver.rs already documents QDLDL stalling on ~40000:1 in-row spread and applies row-∞-norm scaling for axis-jerk cuts. The junction jerk/accel rows need the SAME row-scaling guard when h_L/h_R is far from 1. Recommend bounding the per-segment grid spacing ratio at chain build (e.g. re-grid so h_L/h_R ≲ ~4) to keep both truncation error and conditioning controlled; an unbounded ratio degrades the junction row to first-order AND ill-conditions it simultaneously.

### CLAIM 3 — corner-sweep exactness / monotonicity: VERIFIED-WITH-CAVEATS

Continuous-limit result: with corner ACCEL FREE (design), higher v_entry strictly dominates lower via the brake-down-then-replay argument — from a higher v_entry one can jerk-brake to the lower-entry state over distance d>0, during which the trajectory is pointwise faster, then replay. Since T=∫ds/v is monotone-decreasing in pointwise v, T(v_entry,v_exit) is non-increasing in each boundary velocity. The only way a higher v_entry fails is by breaching a downstream cap before it can bleed off — that is INFEASIBILITY, not a time inversion. Hence the feasible boundary-velocity set is an interval [0,v_reach] and the two boundary velocities decouple (raising one cannot shrink the other's feasible interval below its own cap). The bidirectional min-sweep over corner velocities therefore converges to the chain-decomposed global optimum in the continuous limit. This matches the classical accel-only TOPP property and EXTENDS to jerk specifically because entry accel is free (so the (v,a) entry state collapses to a v-only interval).

Caveat 1 (finite N): Pham TOPP-RA (arXiv:1707.07239) states the maximal transition function "can be made monotone by increasing the number of discretization points" and is "not monotonic over the whole controllable set" at finite N. So the sweep fixed point is the discrete optimum to O(grid). This caveat ALREADY applies to today's per-segment sweep; chains do not introduce it. Chains REDUCE the number of sweep couplings (only tangent-discontinuous corners couple; smooth junctions are internalized), strictly shrinking the surface where finite-N non-monotonicity can act.

Caveat 2 (load-bearing assumption): monotonicity relies on corner entry accel being FREE. If a future change PINS corner accel, the boundary state becomes (v,a), reachable sets need not nest in v alone, and monotonicity can fail — CLAIM 3 would require re-verification. The "accel free at corners" design choice is essential, not incidental.

### CLAIM 4 — interior infeasibility removal: VERIFIED-WITH-CAVEATS

Inside a smooth chain, junction b values are FREE optimization variables, so there is no separate per-junction reachability test that can fail — the SOCP is feasible iff the pinned data (batch-edge b_0, a_0, terminal v with e2 envelope; corner velocity pins) admit any profile. The per-junction "unreachable from neighbor" failure mode that today's bisection ladder (parallel.rs::solve_with_boundary_fallback) handles cannot occur for smooth junctions because that mode is an artifact of solving segments separately with pinned junction velocities.

Residual infeasibility modes the design must still handle:
1. Corner cap pinned above what the adjacent chain can reach. A corner pins a scalar velocity (JD/centripetal/v_max cap). If that cap exceeds the velocity the adjacent chain can actually attain given its own batch/corner pins and limits, the chain SOCP is infeasible at that endpoint. The corner-endpoint bisection fallback (the unpinned-endpoint branch of solve_with_boundary_fallback) covers exactly this: it scales the unpinned corner velocity down until feasible, then the sweep propagates the achieved value. Confirmed this is necessary and sufficient for corners.
2. Batch-edge pins themselves infeasible (e.g. v_start above the centripetal MVC at grid 0). Already handled pre-solve by BuildOutcome::Boundary (constraints.rs L148–157). Unchanged by condensation.
3. Both corner endpoints of a single-chain batch pinned and jointly infeasible (e.g. a very short chain between two high JD caps it cannot bridge under jerk). With both endpoints pinned the bisection cannot move either; today's code returns the infeasible solve as-is (both-pinned branch). Condensation does not remove this; it is the genuine "this batch slice cannot be traversed at the pinned boundary speeds" case and must surface as a loud failure, consistent with the project's fail-loudly rule.

### ITEM 5 — other mathematical hazards in the condensation design

5a (HIGH PRIORITY — correctness): SOC time-chain block (h) and the Σt objective currently hardcode ONE scalar h (constraints.rs uses grid.s[1]−grid.s[0], sqrt_h, and 2·h constants). The chain spans intervals of different lengths. The time surrogate t_i ≥ h/√b_i is the time to cross ONE interval of length h around point i; with mixed spacing it MUST use the point-local interval length h_i (RHS h_i², and the objective weight). If left as a single h, the objective mis-costs traversal time by (h_local/h_global)² on the minority-spacing side (e.g. 0.062× at h_L/h_R=1/4) — the optimizer would slow down there "for free," yielding a wrong, non-time-optimal profile. The fix is trivial mathematically (t_i b_i ≥ h_i²) but requires the chain builder to carry per-point h_i, not a scalar. This is the single most important implementation correctness item.

5b (conditioning only): SolverScale σ = max-over-chain v_max is pure nondimensionalization and EXACT for a multi-Limits chain regardless of which segment σ is drawn from. Only conditioning degrades if the per-segment v_max span across the chain is large; max-over-chain keeps the dominant b near V_TARGET² and is sound.

5c (coverage): the (e2) rest-boundary reachable-envelope rows must fire at any endpoint with v=0, not only the global batch edge. A corner that pins v=0 (full stop / near-reversal) is a chain endpoint at rest and needs e2 rows to close the b→0 jerk-impulse hole (where block (f)'s √b weight vanishes). Confirm the chain builder gates e2 on "endpoint v==0" (as build() already does via endpoints.v_start/v_end == 0.0), NOT on "is global batch edge." If e2 were gated on batch-edge identity, zero-velocity corners would silently lose their envelope rows and the jerk-impulse hole reopens at internal stops.

5d (SLP on larger problems): the path-jerk and per-axis-jerk SLP outer loops (solver.rs) operate per-problem; a chain is a single larger SOCP with N_chain = Σ N_seg points. The full-grid cut placement (N−2 path-jerk rows, replaced not accumulated) and the L∞ trust region scale linearly in N, so per-iteration cost grows but the convergence MECHANISM is unchanged. Two watch-items: (i) the row-∞-norm scaling for axis-jerk cuts (cp·√b/h² ~ O(N²)) gets larger on a bigger chain — already mitigated by per-row scaling, but verify on representative chain sizes; (ii) the no-improvement divergence window (SLP_NO_IMPROVEMENT_WINDOW=10) and warmup (8) are iteration-count heuristics tuned on per-segment sizes; on much larger chains the iterate may need a wider window before the divergence rule fires. These are tuning risks, not correctness defects.

## Sources
- Pham & Pham, TOPP-RA, arXiv:1707.07239 — searched + abstract/figure text retrieved 2026-06-07. Controllable/reachable sets are intervals in squared-velocity for SECOND-ORDER (acceleration) dynamics; maximal transition function "can be made monotone by increasing the number of discretization points," "not monotonic over the whole controllable set" at finite N. PDF binary did not render in WebFetch; relied on indexed text excerpts.
- Lee, Bylard, Sun, Sentis, ICRA 2024, arXiv:2404.07889 — HTML v1 fetched 2026-06-07. Global (not junction-decomposed) formulation over N segments; spline jerk discontinuities "can be mitigated by constraining acceleration changes in transitions between spline segments"; SLP timing 7.5±5.8 ms vs TOPP-RA 0.27 ms; no trust-region detail in their formulation.
- Consolini & Locatelli, Automatica 2024, arXiv:2310.07583 — via existing repo doc jerk-constrained-socp-relaxation-tightness.md (relaxation is conjecturally but not provably exact; SLP fix endorsed).
- Non-uniform 3-point FD order: Gordon CPS343 finite-difference notes and LeVeque FD text — searched 2026-06-07; confirm central first-derivative O(h) on nonuniform mesh (|h_+−h_−|‖f''‖ term) and 3-point second-derivative O(h) (first order) off-uniform. Numerically reproduced independently (/tmp/verify_nonuniform_order.py): leading error (h_R−h_L)/3·f'''.

## Caveats / unchecked assumptions
- The continuous-optimum regularity argument for CLAIM 1 (jerk bound ⇒ s̈ continuous) is exact for the IDEAL continuous problem. The discrete SOCP approximates it to O(h) at the junction; no attempt was made to bound the global trajectory-time difference between the chain SOCP and a hypothetical fully-resolved per-segment SOCP — only the per-junction modeling order (O(h), same as existing endpoints).
- CLAIM 3 monotonicity is argued from the continuous value-function structure and the TOPP-RA finite-N caveat; a fully rigorous proof that the specific kalico discrete SOCP transition map is monotone for all representative inputs was NOT constructed. The verdict is "holds in the continuous limit; finite-N caveat identical to the existing per-segment sweep."
- Item 5a (per-point h_i in block (h)/objective) was diagnosed from the source structure (single scalar h in constraints.rs); it was not confirmed how the redesign intends to build block (h) for chains. If the redesign already carries per-point h_i this is moot — but it must be explicitly checked, as the current code would be wrong if reused verbatim.
- Item 5c (e2 at zero-velocity corners) depends on how the chain builder passes endpoint velocities; verified the EXISTING build() gates on v==0, but the chain-level plumbing was not inspected.
- The bisection-at-corner fallback was confirmed sufficient for the corner-cap-above-reach mode; the both-corners-pinned-infeasible mode (5/Claim4 item 3) must surface loudly and was not traced through the chain-level error path.
- TOPP-RA PDF and the FD lecture PDF did not render in WebFetch (binary streams); their claims were taken from search-index excerpts cross-checked against independent numerical reproduction, not full-text reading.
