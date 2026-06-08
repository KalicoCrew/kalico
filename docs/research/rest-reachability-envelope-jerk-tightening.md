---
topic: From-rest double-S reachability envelope as a jerk-tightening row in the CL-2024 SOCP
created: 2026-06-07
last_updated: 2026-06-07
verified_claims:
  - 2026-06-07 REFUTED (as stated) — C1: env(s) (Biagiotti-Melchiorri double-S from-rest, assuming a(0)=0) is NOT an exact necessary condition under the formulation's actual boundary contract, which leaves path acceleration s̈(0) FREE (only b_0 = v_start² is pinned). With a(0) free in [0,a_max], the maximal reachable v(s) is the accel envelope √(2·a_max·s), strictly ABOVE the a(0)=0 double-S envelope by up to ~46% near s=0. The fix is only valid if an additional a(0)=0 contract is imposed AND made consistent across segment junctions.
  - 2026-06-07 CONFIRMED — C2: IF a(0)=0 is contracted, capping b by env(s)² near both rest endpoints recovers T≈0.347 on fixture_1 (vs double-S 0.350, trapezoid 0.300), a 0.85% underestimate, well within the test's 2%/8% tolerance.
  - 2026-06-07 CONFIRMED — C3: the rows b_i ≤ env(s_i)² are linear in b with constant RHS; they preserve convexity and cannot cause infeasibility against b_0=0 (env(0)=0 ⇒ row at i=0 is 0≤0) or against block (e) centripetal caps (intersection of two upper half-spaces on b is always non-empty, contains b=0).
  - 2026-06-07 CONFIRMED-WITH-CONDITIONS — C4: on a curve, centripetal load reduces available tangential accel, so a tangential-only env using the TRUE tangential accel cap is an upper bound (necessary). BUT a_path = min_axis(a_max) is NOT always the true tangential cap — on a diagonal the tangential budget is a_max·√2 > min_axis, so min-axis a_path makes env over-conservative and can cut the true optimum, violating the print-throughput rule. The conservative direction holds only for the curvature trade-off, not for the axis-projection choice.
  - 2026-06-07 CONFIRMED — C5: for v_start>0 with a(0) free, starting at a(0)=a_max gives v(s)=√(v0²+2·a_max·s) (accel envelope); jerk adds zero tightening. Restoring jerk tightening requires an accel-continuity contract (a_start fixed to the prior segment's a_end), which the current joining loop does not propagate.
sources:
  - Lee, Bylard, Sun, Sentis 2024 (arXiv:2404.07889 v1) — TOPP jerk-constrained; boundary state (s₀,ṡ₀)=(0,0), no s̈(0) constraint
  - Berscheid & Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States" (Ruckig), RSS 2021, arXiv:2105.04830
  - Biagiotti & Melchiorri, "Trajectory Planning for Automatic Machines and Robots", Springer 2008 (double-S profile)
  - Existing: docs/research/jerk-constrained-socp-relaxation-tightness.md, docs/research/maxiterslp-grid-sensitivity-fixture-6.md
---

# From-rest double-S reachability envelope as a jerk-tightening row in the CL-2024 SOCP

## Summary

The proposed fix adds linear rows `b_i ≤ env(s_i)²` to the Consolini-Locatelli 2024 SOCP, where `env(s)` is the Biagiotti-Melchiorri double-S maximum velocity reachable from rest at arc-length `s`, to close the known hole where the discrete SOCP's sqrt(b) jerk weight vanishes at b≈0 and lets the trajectory step immediately to full path-acceleration (spurious accel-limited trapezoid, T=0.300 instead of jerk-limited double-S T=0.350 on fixture_1). The fix's correctness hinges entirely on **C1's a(0) question**: `env(s)` is only a necessary upper bound if the trajectory's initial path acceleration is pinned to zero. The kalico SOCP (`constraints.rs`) leaves `a(0)` FREE — block (a) pins only `b_0 = v_start²`; block (b) merely *defines* `a_i` by finite difference; block (d) only bounds `|a_i| ≤ a_max`. This exactly mirrors Lee 2024's boundary state `(s,ṡ)=(0,0)` with no acceleration constraint. With `a(0)` free, the true maximal reachable velocity is the **accel** envelope `√(2·a_max·s)`, which lies strictly above the double-S `env(s)` by up to 46% near s=0 — so the proposed rows would forbid feasible (faster) trajectories, violating the print-throughput non-negotiable. The fix is therefore **REFUTED as stated** and is only sound under an *added* contract that pins `a(0)=0` at rest endpoints and enforces acceleration continuity across junctions (which the joining loop does not currently do).

## Verified claim — 2026-06-07

> [The five claims C1-C5 from the brief; see verified_claims frontmatter for per-claim verbatim verdicts.]

### Verification

**C1 — REFUTED as stated (the crux).**
The double-S `env(s)` assumes `a(0)=0` (jerk-up phase: v = ½J t², s = J t³/6, giving v(s) = ½J(6s/J)^(2/3) for s ≤ s₁ = a_max³/(6J²)). Numerically on fixture_1 scalars (a_max=5000, J=1e5): env(0.5mm)=48.3, env(1mm)=76.6, env(2mm)=121.6 mm/s.

The formulation does NOT pin a(0). Grep of `constraints.rs` for `off_a` at endpoints finds only block-(b) finite-difference *definition* rows (a_0 = (b_1−b_0)/2h) and the block-(d) `|a_i|≤a_max` bound. There is no `a_0 = 0` equality. Lee 2024 (arXiv:2404.07889 §II) — the paper the SLP loop is built on — imposes only `(s,ṡ)=(0,0)`, confirmed by WebFetch: "satisfies an initial state (s₀,ṡ₀)=(0,0)… does not explicitly specify boundary conditions for path acceleration (s̈)."

Physically, a body at rest CAN begin accelerating with nonzero a(0): a step in acceleration requires only finite jerk applied over finite time, and the *instantaneous* a(0) is a free initial condition. Ruckig (Berscheid & Kröger 2021, arXiv:2105.04830) is built precisely on arbitrary initial/target acceleration as an independent boundary degree of freedom: "Initial and target states are represented by position, velocity, and acceleration."

With a(0) free in [0, a_max], the velocity-optimal start is a(0)=a_max immediately, giving v(s)=√(2·a_max·s) — the accel envelope. This strictly dominates env(s) everywhere (ratio 1.46 at s=0.5mm, 1.15 at s=s₁=2.083mm, →1 as s→∞). So b_i ≤ env(s_i)² would cut feasible trajectories. REFUTED unless a(0)=0 is separately contracted.

The resolution the brief gestures at — "the machine was previously at rest with zero force, so a(0)=0" — is a *modeling choice*, not a kinematic necessity, and it is self-consistent only if (i) the rest endpoint also pins a(end)=0, and (ii) interior junctions propagate acceleration continuity. The joining loop (`joining.rs`) propagates ONLY velocity (`v_junction.min(v_end).min(v_start)`); no acceleration coupling exists. So even adopting a(0)=0 at true rest endpoints, the contract is incomplete for the multi-segment case.

**C2 — CONFIRMED (conditional on a(0)=0).**
With env enforced, the time-optimal rest-to-rest profile velocity is min(env(s), env(L−s), v_max). Fine-quadrature time integral ∫ds/v = 0.34702 s vs closed-form double-S 0.350 (0.85% under) and vs trapezoid 0.29973. The 0.85% gap is the standard MVC-following slack (min() permits a velocity kink the true profile must smooth), making it an underestimate — safely inside the fixture_1 test bar (rel_err ≤ 0.08; even a 2% target is met). The discrete SOCP objective Σt_k ≈ ∫ds/v shares this surrogate, so recovery to within ~1% of 0.350 is expected.

**C3 — CONFIRMED.**
`b_i ≤ env(s_i)²` is `−b_i + env(s_i)² ≥ 0`: a single nonneg-cone row, linear in the decision variable b with constant RHS. Linear inequalities preserve convexity trivially. Infeasibility: at i=0, env(0)=0 and the boundary equality forces b_0 = v_start² = 0 for a rest start, so the row reads 0 ≤ 0 — feasible, not violated. Against block (e): both are upper bounds on b_i; their intersection is `b_i ≤ min(env², b_max_cent)`, always satisfiable (contains b_i=0). No interaction can empty the feasible set when v_start=0.

**C4 — CONFIRMED-WITH-CONDITIONS.**
Curvature direction (the brief's concern): correct. On a curve the per-axis budget is split a_axis = c''·b + c'·a_tan; centripetal term c''·b consumes budget, leaving less for tangential, so the true reachable v is ≤ the all-budget-tangential env. env stays an upper bound → necessary. CONFIRMED in that direction.
BUT the axis-projection choice is a second, independent direction the brief glosses. a_path = min_axis(a_max) is NOT always the true tangential cap. On a 45° diagonal the tangential accel cap is a_max·√2 ≈ 7071 > min_axis 5000, so env built with a_path=5000 sits BELOW true reachable v and would cut the optimum. The "conservative ⇒ never cuts optimum" claim holds for the curvature trade-off but FAILS for min-axis projection on oblique paths. The envelope's a_path must be the true per-segment tangential projection (direction-dependent), not the scalar min-axis, to remain a valid necessary upper bound everywhere.

**C5 — CONFIRMED.**
v_start>0, a(0) free: start at a(0)=a_max gives v(s)=√(v0²+2·a_max·s), the pure accel envelope; jerk contributes zero tightening because acceleration may jump to a_max at t=0. To restore jerk tightening, a_start must be fixed (to the prior segment's a_end), i.e. an acceleration-continuity contract across junctions — which the joining loop does not implement.

### Sources
- Lee, Bylard, Sun, Sentis 2024 (arXiv:2404.07889 v1) — fetched https://arxiv.org/html/2404.07889v1, 2026-06-07: boundary state (s₀,ṡ₀)=(0,0); no s̈ endpoint constraint.
- Berscheid & Kröger (Ruckig) 2021 (arXiv:2105.04830) — RSS abstract + search, retrieved 2026-06-07: arbitrary initial AND target acceleration as independent boundary DOF; first Type-V OTG for non-zero target acceleration.
- Biagiotti & Melchiorri 2008 — double-S profile (from kalico fixture closed form, prototype.rs; algebra re-derived and matched to T=0.350).
- Numerical: /tmp/envelope_check.py, /tmp/env_recover.py, /tmp/c1_crux.py, /tmp/c4_c5.py (run 2026-06-07).

### Caveats / unchecked assumptions
- Did NOT run the kalico solver with the env rows added; C2's 0.347 is a continuous time-integral surrogate, not a Clarabel solve of the modified SOCP. The discrete optimum may differ by O(h²) but is bounded by the surrogate.
- The block-(b) endpoint finite-difference stencil for a_0 uses a one-sided width-1 difference (a_0 = (b_1−b_0)/2h). Even if env rows are added, whether the *discrete* a_0 it implies can be forced to zero by an added equality (and whether that over-constrains the N=200 grid) was not solver-tested.
- Did NOT survey whether any printer-motion contract in the broader kalico spec already mandates a(0)=0 at true rest (vs. the multi-segment interior). The verdict treats a(0) as free per the code as written.
- The C4 axis-projection issue assumes the proposed env uses scalar min-axis a_path (as the brief states). If the implementation instead uses the true per-segment tangential projection, the C4 second-direction objection dissolves; the curvature direction remains valid either way.
