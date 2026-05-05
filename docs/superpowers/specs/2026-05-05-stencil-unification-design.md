# Stencil unification — design

**Date:** 2026-05-05
**Author:** Phase 4 homing-stall investigation (continuation of `2026-05-05-mvp-global-scalar-jerk-design.md`, which it supersedes for the homing-unblock thread).
**Status:** Design ready; implementation pending.
**Verifier sign-off:** kalico-verifier VERIFIED with three text corrections (incorporated below). Codex review: 5 blocking findings (incorporated). See §11.

## 1. Summary

Replace the temporal crate's mixed finite-difference stencils for path-third-derivative `s‴` with a uniform **width-1 b-FD stencil** across verifier and per-axis Cartesian-jerk SLP. The path-jerk SOC chain (block (h)) and path-jerk SLP cuts already use width-1 b-FD; this change brings the verifier (`verify::da_ds_at`), the per-axis jerk diagnostic (`solver::max_axis_ratio`), and the per-axis SLP cut linearization (`solver::append_axis_jerk_cut_to_clarabel`) into agreement with that established stencil. The lockstep partner is the cut-algebra rewrite — interior cuts now touch variables `(b_{i-1}, b_i, b_{i+1}, a_i)` instead of `(b_i, a_{i-1}, a_i, a_{i+1})`. Boundary indices i ∈ {0, N-1} get one-sided forward / backward FD with O(h)·b''' truncation; interior i ∈ [1, N-2] uses central FD with O(h²)·b'''' truncation.

This unblocks Phase 4 G28 X homing (currently `StalledOnInfeasibleSegment` because the verifier's wider stencil over-estimates jerk by ~1.2% on the 300 mm pure-X collinear cubic, exceeding `EPS_FEAS=2e-3`) and resolves the boundary-adjacent O(1)·b''/8 bias the prior session's verifier-only attempt couldn't close.

## 2. Motivation

The diagnostic matrix in `rust/trajectory/tests/homing_diagnostic.rs` (committed to the working tree this session) showed the failure mode is **not** in the bridge config layer (the prior MVP plan's premise), nor in Stage 1 SLP, nor in β-medium. The matrix:

```
V1 topp-only 300mm  -> StalledOnInfeasibleSegment  DivergedSlp { last_max_ratio: 1.0120, outer_iters: 6 }
V2 topp-only 30mm   -> StalledOnInfeasibleSegment  DivergedSlp { last_max_ratio: 1.0023, outer_iters: 7 }
V3 topp-only 100mm  -> StalledOnInfeasibleSegment  DivergedSlp { last_max_ratio: 1.0045, outer_iters: 6 }
V4 topp-only 200mm  -> StalledOnInfeasibleSegment  DivergedSlp { last_max_ratio: 1.0075, outer_iters: 10 }
```

`last_max_ratio` scales with `h²` (h capped at 1.5 mm by `max_n=200` on the 300 mm fixture). Probe with verifier `EPS_FEAS = 2e-2` confirmed: the homing test passes in 1.48 s (down from 58 s failure) when the over-estimate's effective threshold is widened. The trajectory at the SLP-converged iterate IS feasible by Stage-1's width-1 measure; the verifier and Stage-2 SLP's width-2 measure rejects it.

The fix is to align all `s‴` measurements at the more accurate stencil. Per the math (verified independently — see §3), width-1 b-FD has truncation `h²·b''''/24` while width-2 substituted from a-FD has `h²·b''''/6` — exactly **4× worse**.

## 3. Math foundations

### 3.1 Chain rule

With `b(s) = ṡ²`, the path-third-derivative `s‴` derives as:

```
ṡ        = √b
s̈       = ½·b'(s)
s‴      = ½·b''(s)·√b
```

Both finite-difference families estimate `b''(s)`. The verifier's `da_ds_at` does so via central-FD on `a` (which itself is `½·b'`), introducing the wider-stencil substitution; the path-jerk SOC chain does so via direct width-1 second-difference on `b`.

### 3.2 Width-1 b-FD strict interior

For i ∈ [1, N-2] on a uniform grid with spacing h:

```
(b[i-1] − 2·b[i] + b[i+1]) / h² = b''(s_i) + (h²/12)·b''''(s_i) + O(h⁴)
```

Therefore:

```
s‴_width1[i] = √b_i · (b[i-1] − 2·b[i] + b[i+1]) / (2h²)
             = √b_i · b''(s_i) / 2 + √b_i · h² · b''''(s_i) / 24 + O(h⁴)
```

Leading truncation coefficient on `√b·h²·b''''` is **1/24**.

### 3.3 Width-2 substituted (current code)

Interior central-FD on `a = ½·b'` substituted with the constraint linkage `a_i = (b[i+1] − b[i-1])/(4h)`:

```
(a[i+1] − a[i-1]) / (2h) = (b[i+2] − 2·b[i] + b[i-2]) / (8h²)
                         = ½·b''(s_i) + (h²/6)·b''''(s_i) + O(h⁴)
```

Multiplying by `√b_i`:

```
s‴_width2[i] = √b_i · b''(s_i) / 2 + √b_i · h² · b''''(s_i) / 6 + O(h⁴)
```

Leading truncation coefficient on `√b·h²·b''''` is **1/6** — exactly **4×** the width-1 coefficient.

### 3.4 Boundary one-sided stencils

At i=0 (forward FD):

```
(b[0] − 2·b[1] + b[2]) / h² = b''(s_0) + h·b'''(s_0) + (7h²/12)·b''''(s_0) + O(h³)
```

Leading truncation is **O(h)·b'''(s_0)** — asymptotically worse than central O(h²) but acceptable at typical grid spacings. Symmetric expression at i=N-1 (backward FD).

### 3.5 Boundary-adjacent resolution under Option B (the critical insight)

Under the current width-2 substituted stencil, indices i=1 and i=N-2 use central-FD on `a` that reaches into endpoint one-sided FDs on `a`. The kalico-verifier confirmed the resulting expression at i=1:

```
(a[2] − a[0]) / (2h) = (b[3] − 3·b[1] + 2·b[0]) / (8h²)
                     = (3/8)·b''(s_1) + (h/8)·b'''(s_1) + (3h²/32)·b''''(s_1) + O(h³)
```

The estimator targets `½·b''`. Leading error = `((3/8) − ½)·b'' = −b''/8` — an **O(1) bias on b''** (not on b''' as one might naively expect). The bias survives on constant-acceleration plateaus (homing regime) where b'' is generically non-zero.

Under Option B, i ∈ {1, N-2} uses the **standard central width-1 b-FD stencil** — they are interior indices. The O(1) bias vanishes by construction; only i=0 and i=N-1 need one-sided stencils.

### 3.6 Hidden boundary-index correction (kalico-verifier bonus finding)

The current code's verifier at i=0 substitutes:

```
da_ds_at(i=0) = (a[1] − a[0]) / h
              = [(b[2] − b[0])/(4h) − (b[1] − b[0])/(2h)] / h
              = (b[0] − 2·b[1] + b[2]) / (4h²)
```

Note the divisor of **4h²**, not 2h². Times `√b_0`:

```
current s‴_verify[0] = √b_0 · (b[0] − 2·b[1] + b[2]) / (4h²)
```

The estimator targets `½·b''`. Expanding the numerator (forward-FD second-difference):

```
(b[0] − 2·b[1] + b[2]) / h² = b'' + h·b''' + (7h²/12)·b'''' + ...
```

Divided by 4 (i.e. the current `/(4h²)`): leading term = `b''/4 + ...`. Estimator targets `b''/2`. Therefore current code at i=0 has **O(1) bias of `−b''/4`** — exactly half of the b''/8 bias at i=1, but same order. Option B's `(b[0] − 2·b[1] + b[2]) / (2h²)` (with divisor of 2h², not 4h²) directly estimates `b''` (then × ½·√b for s‴), with no O(1) bias — only the textbook O(h)·b''' one-sided truncation.

This is a **meaningful additional improvement** at boundary indices. Currently 4 grid points carry O(1) bias on `b''`; Option B reduces that to zero.

### 3.7 Process commitment on curved-arc fixture non-regression

The existing `rust/temporal/tests/conditioning.rs::rational_quadratic_arc_n200_solves_with_centripetal_cruise` fixture binds at the midpoint via `Centripetal`, where `b''≈0` on the constant-curvature cruise. The verifier-ratio direction depends on `sign(b'''')` at the relevant grid indices; on this fixture, b'''' is small near the binding zone and the change of stencil produces a small ratio shift that doesn't flip Centripetal-binding into Jerk-binding. Whether SLP's cut-shape change perturbs the iteration trajectory cannot be guaranteed first-principles. This must be a **process commitment**: the curved-arc test runs as part of the change's gate, and any regression triggers investigation before merge. Not a first-principles non-regression claim.

## 4. Architecture

### 4.1 New shared stencil module

Create `rust/temporal/src/topp/stencil.rs`, declared as `pub(crate) mod stencil;` in `rust/temporal/src/topp/mod.rs` (matching the existing `pub(crate) mod output;` / `pub(crate) mod solver;` / `pub(crate) mod verify;` visibility). Defines a single helper:

```rust
/// Path-third-derivative `s‴` at grid index `i` via width-1 b-FD.
///
/// Caller-provided invariants: `n ≥ 3` (required for boundary stencils);
/// `h > 0`; `b.len() == n`. The helper applies `.max(0.0)` to `b[i]`
/// defensively before `sqrt` to keep numerically-borderline iterates
/// (where Clarabel may produce slightly-negative `b[i]` due to
/// solver-residual rounding) from producing `NaN`. The b-FD second-
/// difference itself accepts any `b` values; nothing in the stencil
/// arithmetic requires non-negativity beyond the `√b` factor.
///
/// # Stencil dispatch
///
/// - `i = 0`: forward FD `(b[0] − 2·b[1] + b[2]) / h²`, O(h)·b''' truncation.
/// - `i ∈ [1, n-2]`: central FD `(b[i-1] − 2·b[i] + b[i+1]) / h²`, O(h²)·b'''' truncation.
/// - `i = n-1`: backward FD `(b[n-3] − 2·b[n-2] + b[n-1]) / h²`, O(h)·b''' truncation.
///
/// Returns `s‴_i = √b_i · b''(s_i) / 2`.
pub(crate) fn s_dddot_at(b: &[f64], i: usize, h: f64) -> f64 {
    debug_assert!(b.len() >= 3, "stencil requires n >= 3");
    debug_assert!(h > 0.0);
    let n = b.len();
    let s_dot = b[i].max(0.0).sqrt();
    let b_dd = if i == 0 {
        (b[0] - 2.0*b[1] + b[2]) / (h*h)
    } else if i == n - 1 {
        (b[n-3] - 2.0*b[n-2] + b[n-1]) / (h*h)
    } else {
        (b[i-1] - 2.0*b[i] + b[i+1]) / (h*h)
    };
    s_dot * b_dd / 2.0
}

/// Stencil dispatch tag, mirroring `s_dddot_at`'s branches. Used by the SLP
/// cut linearization to select the correct coefficient formulas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SDddotStencil {
    StartBoundary,  // i = 0, forward FD
    Interior,       // i ∈ [1, n-2], central FD
    EndBoundary,    // i = n-1, backward FD
}

pub(crate) fn stencil_for(n: usize, i: usize) -> SDddotStencil {
    if i == 0 { SDddotStencil::StartBoundary }
    else if i == n - 1 { SDddotStencil::EndBoundary }
    else { SDddotStencil::Interior }
}
```

The existing `solver::AxisJerkStencil` enum is renamed/replaced by `SDddotStencil` (or kept as a thin re-export — implementation choice). The semantics are identical.

### 4.2 Verifier change

`rust/temporal/src/topp/verify.rs`:

- Remove `da_ds_at` (line 87).
- In `verify::check`, replace the `s_dddot = da_ds_at(result, &grid.s, i) * s_dot` line (~234) with `s_dddot = stencil::s_dddot_at(&result.b, i, h)`. The function's per-axis Cartesian jerk identity (`c'''·ṡ³ + 3·c''·ṡ·s̈ + c'·s‴`) is unchanged; only the s‴ source is.
- Add `h: f64` to `verify::check`'s signature, computed from the grid (or carried via `ArclengthGrid` if more convenient — implementation choice).
- Update module docstring at `verify.rs:1-30`: remove the "width-1 SOC vs width-2 verifier mismatch" paragraph since the mismatch no longer exists; reference this spec for the unification rationale.
- `EPS_FEAS = 2e-3` stays. Rationale of "covers the 0.2% jerk-stencil mismatch" is now stale; replace the comment with "0.2% feasibility margin per spec §6.2; uniform width-1 b-FD across SOCP/SLP/verifier per `2026-05-05-stencil-unification-design.md`."

### 4.3 Solver-side per-axis Cartesian jerk machinery

`rust/temporal/src/topp/solver.rs`:

- Remove `da_ds_along` (line 1558). Solver code now calls `stencil::s_dddot_at` directly.
- `max_axis_ratio` (line 1457): replace `da_ds_along(...) * s_dot` with `stencil::s_dddot_at(&result.b, i, h)`. Per-axis jerk identity unchanged. Add `h: f64` parameter.
- `build_axis_jerk_cuts` (line 1500): replace `da_ds_along(...) * s_dot` with `stencil::s_dddot_at(...)`. The dispatch into `AxisJerkStencil { Interior, StartBoundary, EndBoundary }` semantics is preserved (3 variants); change is in the variables each variant references and the cut payload. Add `h` parameter.
- `AxisJerkCut` struct (around `solver.rs:113`): rename `a_bars: [f64; 3]` to `b_bars: [f64; 3]` carrying `(b̄_{i-1}, b̄_i, b̄_{i+1})` for Interior, `(b̄_0, b̄_1, b̄_2)` for StartBoundary, `(b̄_{n-3}, b̄_{n-2}, b̄_{n-1})` for EndBoundary. Add `a_bar_i: f64` carrying the iterate's `ā_i` value (single index, not three).
- `append_axis_jerk_cut_to_clarabel` (line 444): full re-derivation per §5 below.

### 4.4 Constraint-bundle interaction

`rust/temporal/src/topp/constraints.rs` is **unchanged**. The SOCP variable layout (`b_i` at `0..N`, `a_i` at `N..2N`, interior `t/x1/x2`), the block-(b) acceleration linkage (`a_i = ½·b'(s_i)` equality rows), and block (h) (path-jerk SOC chain) all stay as-is. Option B operates entirely within `verify.rs`, the new `stencil.rs`, and the per-axis-cut portions of `solver.rs`.

The MAINTAINER WARNING at `constraints.rs:236-247` is still accurate and gets a small append: a paragraph noting that Option B unifies the verifier and per-axis SLP stencils with block (h)'s existing width-1 b-FD, reducing the system's stencil count from 2 to 1.

### 4.5 Comment / documentation sweep

Per Codex's blocking finding #3, also update:

- `rust/temporal/src/topp/verify.rs:1-30` (module docstring re: stencil mismatch).
- `rust/temporal/src/topp/solver.rs:809` (comment "tolerances match verify::check's EPS_FEAS=1e-3" — actual value is 2e-3, stale).
- `rust/temporal/src/topp/solver.rs:1213` and surrounding (Step-9 SLP comment block referencing the prior stencil-disagreement story).
- The block comment at `solver.rs:373-442` (cut-algebra documentation): replace with the new derivation per §5.

## 5. Cut algebra derivation (full)

### 5.1 General setup

The per-axis Cartesian jerk identity (unchanged):

```
j_axis(i, axis) = c'''_axis · ṡ³ + 3·c''_axis · ṡ · s̈ + c'_axis · s‴
```

Substituting `ṡ = √b`, `s̈ = a` (the SOCP variable), and `s‴` via the new width-1 b-FD stencil:

**Interior** (i ∈ [1, n-2]):
```
j_axis = c'''·b^(3/2) + 3·c''·a_i·√b_i + c'·(b_{i-1} − 2·b_i + b_{i+1})·√b_i / (2h²)
```

**StartBoundary** (i = 0):
```
j_axis = c'''·b_0^(3/2) + 3·c''·a_0·√b_0 + c'·(b_0 − 2·b_1 + b_2)·√b_0 / (2h²)
```

**EndBoundary** (i = n-1):
```
j_axis = c'''·b_{n-1}^(3/2) + 3·c''·a_{n-1}·√b_{n-1} + c'·(b_{n-3} − 2·b_{n-2} + b_{n-1})·√b_{n-1} / (2h²)
```

Let `S_i = √b̄_i` (iterate value) and `S3_i = b̄_i^{3/2} = S_i^3`. Define stencil-specific second-differences:

```
D₂_int = b̄_{i-1} − 2·b̄_i + b̄_{i+1}      (Interior at i)
D₂_fwd = b̄_0 − 2·b̄_1 + b̄_2              (StartBoundary)
D₂_bwd = b̄_{n-3} − 2·b̄_{n-2} + b̄_{n-1}  (EndBoundary)
```

### 5.2 Linearization recipe

For each stencil variant, compute `α` coefficients as partial derivatives of the per-axis jerk identity at the iterate, plus the constant term `K`:

```
K = j_axis(iterate) − Σ_v α_v · iterate_value(v)
```

…where the sum is over all variables the cut touches. The two Nonneg cone rows are:

```
positive side:  +(J_lim_inflated − f_lin) ≥ 0   →  rhs = J_lim − K,  row = [−α_v]
negative side:  +(J_lim_inflated + f_lin) ≥ 0   →  rhs = J_lim + K,  row = [+α_v]
```

This gives `|f_lin(b, a) − K| ≤ J_lim_inflated · 1` ... no wait — let me restate cleanly. The cut bounds `|j_axis_lin(b, a)| ≤ J_lim_inflated`. The linearized identity is `j_axis_lin = Σ α_v · v + K`. So:

```
positive side:  J_lim_inflated − (Σ α_v·v + K) ≥ 0  →  row = [−α_{v_1}, ...],  rhs = J_lim_inflated − K
negative side:  J_lim_inflated + (Σ α_v·v + K) ≥ 0  →  row = [+α_{v_1}, ...],  rhs = J_lim_inflated + K
```

### 5.3 Interior coefficients (i ∈ [1, n-2])

Variables: `b_{i-1}, b_i, b_{i+1}, a_i`. Iterate: `(b̄_{i-1}, b̄_i, b̄_{i+1}, ā_i)`.

```
α_{b,i-1} = c'·S_i / (2h²)
α_{b,i+1} = c'·S_i / (2h²)
α_{a,i}   = 3·c''·S_i

α_{b,i}   = (3/2)·c'''·S_i
          + 3·c''·ā_i / (2·S_i)
          − c'·S_i / h²
          + c'·D₂_int / (4h² · S_i)

K_int     = j_axis(iterate)
          − [ α_{b,i-1}·b̄_{i-1} + α_{b,i}·b̄_i + α_{b,i+1}·b̄_{i+1} + α_{a,i}·ā_i ]
          = −(1/2)·c'''·S3_i
            − (3/2)·c''·ā_i·S_i
            − c'·D₂_int·S_i / (4h²)
```

(K simplification follows by direct expansion; `step9_cut_identity.rs` numerically pins this at known iterates.)

### 5.4 StartBoundary coefficients (i = 0)

Variables: `b_0, b_1, b_2, a_0`. Iterate: `(b̄_0, b̄_1, b̄_2, ā_0)`.

```
α_{b,0} = (3/2)·c'''·S_0
        + 3·c''·ā_0 / (2·S_0)
        + c'·S_0 / (2h²)
        + c'·D₂_fwd / (4h² · S_0)
α_{b,1} = −c'·S_0 / h²
α_{b,2} = c'·S_0 / (2h²)
α_{a,0} = 3·c''·S_0

K_fwd   = −(1/2)·c'''·S_0³
        − (3/2)·c''·ā_0·S_0
        − c'·D₂_fwd·S_0 / (4h²)
```

Same closed form as `K_int` (substituting `S_0` for `S_i`, `D₂_fwd` for `D₂_int`). Derivation parallel: expand `α_{b,0}·b̄_0` (using `b̄_0·S_0 = S_0³`, `b̄_0/S_0 = S_0`), combine with `α_{b,1}·b̄_1 = −c'·S_0·b̄_1/h²`, `α_{b,2}·b̄_2 = c'·S_0·b̄_2/(2h²)`, `α_{a,0}·ā_0 = 3·c''·S_0·ā_0`. After substituting `D₂_fwd = b̄_0 − 2·b̄_1 + b̄_2`, the residual `c'·S_0/h²` terms collapse to `−c'·D₂_fwd·S_0/(4h²)`.

### 5.5 EndBoundary coefficients (i = n-1)

By symmetry with StartBoundary, with the backward stencil `D₂_bwd`:

```
α_{b,n-3} = c'·S_{n-1} / (2h²)
α_{b,n-2} = −c'·S_{n-1} / h²
α_{b,n-1} = (3/2)·c'''·S_{n-1}
          + 3·c''·ā_{n-1} / (2·S_{n-1})
          + c'·S_{n-1} / (2h²)
          + c'·D₂_bwd / (4h² · S_{n-1})
α_{a,n-1} = 3·c''·S_{n-1}

K_bwd   = −(1/2)·c'''·S_{n-1}³
        − (3/2)·c''·ā_{n-1}·S_{n-1}
        − c'·D₂_bwd·S_{n-1} / (4h²)
```

Same closed form as `K_int` and `K_fwd` (substituting `S_{n-1}` and `D₂_bwd`). Derivation symmetric to §5.4.

### 5.6 SLP_B_FLOOR

The existing `SLP_B_FLOOR` guards against `1/√b̄ → ∞` when the iterate's b̄ is near zero. Same guard applies under Option B (still divides by `S_i` in the α formulas above). Implementation: `let S = b̄.max(SLP_B_FLOOR).sqrt()`. Unchanged.

## 6. Test plan

### 6.1 Unit pin: `s_dddot_at` against analytic ground truth

New test file `rust/temporal/src/topp/stencil.rs` (or `stencil_tests.rs`) with `#[cfg(test)] mod tests`. Pin `s_dddot_at` against a closed-form `b(s)` whose `b''(s)` is known analytically:

- Quadratic: `b(s) = α·s² + β·s + γ`. Then `b''(s) = 2α` everywhere; `b''''(s) = 0`. Width-1 stencil should produce **exactly** `2α` at all interior i (truncation term `b''''/12 = 0`). Therefore `s‴_i = √b_i · α`. Pin to within machine epsilon.
- Cubic: `b(s) = α·s³ + β·s² + ...`. Then `b''(s) = 6α·s + 2β`; `b''''(s) = 0` again. Width-1 still exact.
- Quartic: `b(s) = α·s⁴ + ...`. Then `b''''(s) = 24α`; truncation term `√b·h²·24α/24 = √b·h²·α` (non-zero). Pin the magnitude with tolerance `±h²·α·max(√b)`.
- Boundary stencils tested at i=0 and i=n-1 against the same analytic forms; tolerance widens at boundary because of O(h)·b''' leading error.

Three regime-specific cases per the brainstorm critique. Note: `s_dddot_at` (verifier helper) and the cut linearization use **different** floors. The verifier helper applies `.max(0.0)` defensively (cheap rounding-error guard); the cut linearization at `solver.rs:461` applies `b_bar.max(SLP_B_FLOOR)` where `SLP_B_FLOOR = 1.0` (avoids `1/√b̄ → ∞` blowup in the linearization coefficients). The two are tested separately:

- **`s_dddot_at` near-zero edge**: i=1 with b[1] = 0.0 exactly. Verify the helper's `.max(0.0)` guard makes `s_dot = 0.0`, so `s_dddot_at` returns `0.0` regardless of the b-FD numerator. No NaN/Inf.
- **Cut linearization near-floor edge** (in `step9_cut_identity`, §6.2): `b̄_i = 0.5` (just below `SLP_B_FLOOR = 1.0`). The cut helper must use `S = √(max(0.5, 1.0)) = 1.0` for its row coefficients, not `√0.5 ≈ 0.707`. Pin a coefficient that depends linearly on S and verify it equals the floored-value-derived expectation.
- **Constant b̄**: b uniform at e.g. 100.0 throughout. b'' = 0 everywhere; `s_dddot_at` must return 0 at all interior i. Boundary i=0 and i=n-1 also return 0 (forward/backward second-difference of a constant is 0).
- **Sharp-corner approximation**: b with a kink (piecewise linear in s). Width-1 picks up the kink at the junction grid point; magnitude pinned but order-of-magnitude check rather than exact (kinks are non-smooth).

### 6.2 Cut identity: `step9_cut_identity` (rewritten from scratch)

`rust/temporal/tests/step9_cut_identity.rs` is **rewritten from scratch**, not patched. The existing test wires the old (a-FD width-2) coefficient helpers (`da_ds_at` over `a` at line 73; `interior_cut_coeffs` / `boundary_start_cut_coeffs` / `boundary_end_cut_coeffs` at lines 108/156/193; row-sum loop at line 320) and the variable touch set `(b_i, a_{i-1}, a_i, a_{i+1})` for interior. Under Option B, every coefficient helper changes, the row-sum loop indexes a different variable set `(b_{i-1}, b_i, b_{i+1}, a_i)` for interior (and analogous for boundaries), and the direct `j_actual` computation moves from a-FD `da_ds_at` to b-FD `s_dddot_at`. The fixture strategy also changes — from the existing test's full-fixture-scan pattern to a smaller synthetic-iterate selected-index sweep (motivated by per-stencil coverage rather than fixture-shape representativeness, since the cut-identity check is a pure algebraic equality and doesn't need fixture diversity).

New test structure:

1. Pick a synthetic `(b̄, ā)` iterate on a small grid (say n=10).
2. For each grid index i in {0, 1, 5, 8, 9} (covering Start, post-Start interior, mid interior, pre-End interior, End):
   - Compute `j_axis(iterate)` directly via the chain rule + new stencil.
   - Compute `Σ α·iterate + K` from the cut linearization.
   - Assert equality within machine epsilon.
3. Repeat for several `(c', c'', c''')` triples covering: collinear (`c''=c'''=0`), curved (`c''≠0, c'''=0`), pathological (`c'≠0, c''≠0, c'''≠0`).
4. Repeat with one iterate at `b̄_i = 0.5` (below `SLP_B_FLOOR = 1.0`) to verify the cut helper's `b_bar.max(SLP_B_FLOOR)` floor is applied correctly: row coefficients computed with `S = √1.0 = 1.0`, NOT `S = √0.5 ≈ 0.707`. Identity must still hold at the floored value.

### 6.3 Architectural correctness gate: `homing_300mm_pure_x`

`rust/trajectory/tests/homing_300mm_pure_x.rs` (currently in working tree, currently failing) flips to passing without modification of test logic. The test's docstring is updated to reflect its new role — pinning Stage-2 SLP convergence on the homing fixture under uniform width-1 b-FD stencil unification, rather than the prior MVP plan's uniform-`j_max` premise.

### 6.4 Diagnostic regression: `homing_diagnostic`

`rust/trajectory/tests/homing_diagnostic.rs` (currently `#[ignore]`) gets re-enabled (remove `#[ignore]`) and converted from a print-only diagnostic to a hard regression: all 8+ variants must produce `JoiningStatus::Converged` from `temporal::multi::plan_batch` and `Ok` from `trajectory::shape_batch`. The length-scan (V1/V2/V3/V4 at 30/100/200/300 mm) currently shows `last_max_ratio` scaling with h² under width-2; after Option B all variants should land at the trajectory layer's success path.

### 6.5 Mid-print junction with non-zero endpoint velocity

NEW test `rust/temporal/tests/midprint_junction_non_zero_endpoints.rs` (or as a function in an existing test file). Single-segment fixture with `v_start = 30.0`, `v_end = 50.0`, exercising Option B's O(h)·b''' boundary truncation in a regime where `√b_endpoint > 0`. Asserts the segment converges and the verifier's `worst_violation` stays within `EPS_FEAS = 2e-3`.

### 6.6 Curved-arc fixture (regression baseline)

`rust/temporal/tests/conditioning.rs::rational_quadratic_arc_n200_solves_with_centripetal_cruise` must remain green. Per the kalico-verifier's correction, this is a **process commitment** rather than a first-principles non-regression claim. If the test breaks under Option B, investigate whether the failure is a real regression (curved-fixture worst-violation no longer landing at `Centripetal`) or a numerical drift; do not merge until resolved.

### 6.7 Full workspace tests

`cargo test -p temporal -p trajectory -p motion-bridge` must be green. Run with both `--release` and default profiles to catch optimization-sensitive numerical drift.

## 7. Known asymmetries (deliberate)

- **Boundary jerk-enforcement.** Under Option B, the verifier and per-axis SLP cuts evaluate jerk at i=0 and i=n-1 (the boundary indices). The path-jerk SOC chain in block (h) does NOT enforce path-jerk at endpoints (its rows iterate only over interior `1..n-1`; endpoints are pinned by block (a)). This asymmetry is intentional: the SOC chain doesn't need to bind at pinned variables, but the verifier's job — diagnostically check that the chosen trajectory satisfies physical limits — is meaningful at endpoints regardless. Per-axis SLP cuts at boundary indices act on the same diagnostic surface. The Codex-flagged concern about "asymmetric enforcement" is acknowledged and accepted as the right behavior.

- **Boundary trust-region.** The trust region in `solver.rs:639` skips boundary `b` variables (they're pinned). Per-axis cuts at i=0 touch `b_0, b_1, b_2, a_0` — three of those are inside the trust region (b_1, b_2, a_0), one is fixed (b_0). The cut still constrains the iterate but with fewer effective degrees of freedom than interior cuts. SLP convergence under this reduction is empirical; the curved-arc fixture serves as the marginal test.

- **`n < 3` precondition.** The new stencil requires at least 3 grid points. The bridge's adaptive grid sets `min_n: 20` so this is satisfied in practice. We add an explicit `debug_assert!(b.len() >= 3)` inside `s_dddot_at` and a public-facing assertion at the entry to `verify::check` and `solver::max_axis_ratio` to surface the precondition cleanly. Code paths that today support `n=2` (e.g. constraints tests at `constraints.rs:939`) are NOT consumers of these helpers and remain unaffected.

## 8. Acceptance criteria

The change is complete when:

1. New `rust/temporal/src/topp/stencil.rs` exists with `s_dddot_at`, `SDddotStencil`, `stencil_for`, declared in `topp/mod.rs`.
2. `verify::da_ds_at` is removed; `verify::check` calls `stencil::s_dddot_at` for `s‴`. EPS_FEAS unchanged at 2e-3.
3. `solver::da_ds_along` is removed; `max_axis_ratio` and `build_axis_jerk_cuts` call `stencil::s_dddot_at`.
4. `solver::AxisJerkCut` carries `(b̄_{i-1}, b̄_i, b̄_{i+1})` (or boundary-stencil equivalents) plus `ā_i`.
5. `solver::append_axis_jerk_cut_to_clarabel` re-derived per §5; block comment at `solver.rs:373-442` updated to document the new algebra.
6. Stale doc/comment sweep done (see §4.5).
7. New unit tests in §6.1 green.
8. `step9_cut_identity` rewritten and green.
9. `homing_300mm_pure_x` (currently failing) green.
10. `homing_diagnostic` matrix re-enabled and asserts all variants converge.
11. New `midprint_junction_non_zero_endpoints` test green.
12. `rational_quadratic_arc_n200_solves_with_centripetal_cruise` still green.
13. Full `cargo test -p temporal -p trajectory -p motion-bridge` green in both `--release` and default profiles.
14. `docs/superpowers/plan-changes-log.md` records the change.
15. `constraints.rs:236-247` MAINTAINER WARNING gets the appended paragraph (see §4.4).

## 9. Out of scope

- **Z-jerk bridge config bug.** `rust/motion-bridge/src/config.rs::PlannerLimits::to_temporal_limits` defaults `j_max[Z] = 2 × max_z_accel = 200`, which silently bottlenecks `J_path = min(j_max)` for non-Z moves. Real bug, real fix is the prior MVP spec's bridge config layer change. Recommend filing as a separate small task post-stencil-unification — homing passes regardless once stencil unification lands, but the Z default is still wrong on its own merits.
- **5-point O(h²) boundary stencils** (Option C from brainstorming). Boundary stencil error stays O(h)·b''' under Option B. If real prints surface boundary-driven regressions, revisit then.
- **Stage-2 SLP convergence-test loosening** (`SLP9_EPS_FEAS` raise). Not needed if Option B works as the math predicts. Skip indefinitely.
- **β-medium loop changes**. Untouched. β consumes the trajectory layer's outputs unchanged.
- **Per-axis Cartesian jerk SOCP relaxation in block (h)** (the maintainer warning's deferred work). Distinct architectural change; out of this spec's scope.

### 9.1 Triage risk from deferred items

Two of the deferred items (Z-jerk bridge config bug; `SLP9_EPS_FEAS` value) are not architecturally entangled with stencil unification, but they sit in the same constraint-binding pipeline. If Option B regresses something — particularly on a fixture where the stencil change shifts which constraint binds — failure-attribution becomes harder because either deferred item could be a confounder:

- **Z-jerk dominance in `J_path`.** `j_path = min(j_max[X], j_max[Y], j_max[Z])` at `constraints.rs:248`. The bridge config still emits `j_max[Z] = 2 × max_z_accel = 200`, so `J_path` for non-Z-bearing moves is artificially clamped. A regression on, say, the curved-arc fixture under Option B might be (a) genuine stencil-induced Stage-2 SLP behavior change, (b) the J_path clamp interacting with the new stencil's tighter ratios, or (c) both. If we suspect (b), we'd want the bridge config fix landed before triaging — but it's not in this spec.
- **`SLP9_EPS_FEAS = 1e-3` gating per-axis cut activation.** `build_axis_jerk_cuts` skips ratios ≤ 1.0 + SLP9_EPS_FEAS at `solver.rs:1524`. Under Option B, ratios scale-down by ~4× from the current width-2 baseline (per §3.2 vs §3.3); if the new width-1 ratio at the homing fixture's iterate sits below SLP9_EPS_FEAS at iter 0, Stage 2 short-circuits and Stage 1's iterate (already converged at width-1 SOC chain, which IS the new measurement) becomes the final answer. That's the expected happy path. But if some fixture's iter-0 width-1 ratio happens to land in the (1e-3, current-width-2-ratio) window, Option B might newly activate Stage 2 on fixtures that previously short-circuited via verifier rejection. Behavior change is bounded but worth monitoring during the curved-arc-fixture green-gate.

Practical implication: when a regression surfaces during implementation, check both items as confounders before attributing the regression to the stencil unification itself. Long-term, both deferred items should land soon-after to remove these confounders from the system.

## 10. Future work

If real-world prints surface stencil-related issues on segments with `v_start ≠ 0` AND large `b'''(s)` near the segment boundary, derive 5-point one-sided stencils with O(h²) leading error at i ∈ {0, n-1}. The cut algebra extends naturally (5-variable rows at boundary) but adds notable complexity. Defer until evidence demands it.

If the curved-arc fixture's worst-violation grid ever lands at strict interior under Option B with significant ratio, revisit whether the per-axis cut needs the regime-specific analysis from `solver.rs:439-442`'s "non-adjacent oscillation" note (extending the active-set neighborhood from ±0 to ±2). Not anticipated under Option B but documenting the trigger condition.

## 11. Review trail

- **kalico-verifier (this session)**: VERIFIED the math claims at the order/sign/scaling level. Three text corrections applied: (a) Claim 5's leading bias is on b'' with coefficient −1/8, not on b'''; the order claim O(1) is correct, the symbol was wrong. (b) Step 9's per-axis cut never had a tangent-below tightness guarantee; phrasing strengthened to acknowledge empirical-only convergence. (c) Curved-arc fixture non-regression is a process commitment. Bonus finding: current code at i=0 has hidden O(1)·b''/4 bias (factor-of-2 error in substitution); Option B fixes this too. Incorporated into §3.5 / §3.6 / §3.7.
- **Codex review (this session)**: 5 blocking findings, all addressed. (a) `n < 3` precondition explicit; §7. (b) Full cut algebra derivation in spec body; §5. (c) `mod stencil;` declaration + stale doc/comment sweep; §4.5. (d) Homing test docstring update (test logic unchanged); §6.3. (e) Typo `b(s_0)` → `b''(s_0)` (caught upstream of writing this spec, so no longer surfaces).
- **Self-review (Claude, this session)**: Two additions Codex didn't catch. (f) Boundary jerk-enforcement asymmetry deliberate-not-bug; §7. (g) Three regime-specific cases for cut-identity test rather than one (near-zero b̄, constant b̄, sharp corner); §6.1.
- **Codex second-pass review (this session)**: Five concerns, all verified and addressed. (h) §4.1 b≥0 guard contradiction (caller-vs-helper) — resolved by accepting the helper-applies-`.max(0.0)` pattern (matches existing `max_axis_ratio` and `verify::check`). (i) §6.1 near-zero test wording wrong (used `S_i.max(...)` instead of `b̄.max(...)`, used `b[1]=1e-6` near a floor that's actually 1.0) — separated into two distinct tests for the verifier helper (`.max(0.0)`) vs cut linearization (`SLP_B_FLOOR=1.0`). (j) §5.4/§5.5 missing closed-form `K_fwd` and `K_bwd` — added; both share the same closed form as `K_int` with stencil-specific `S` and `D₂`, derivation independently re-verified by Codex. (k) §6.2 step9_cut_identity rewrite scope under-specified (implied mechanical update) — sharpened to "rewritten from scratch" with explicit acknowledgment of fixture-strategy change and full coefficient-helper re-derivation. (l) §9 deferred items create triage risk — added §9.1 noting Z-jerk J_path clamp and SLP9_EPS_FEAS as confounders during regression triage.

## 12. References

- `docs/research/stall-homing-move.md` — original stencil-agreement analysis (Taylor expansions, leading coefficients, "Which Stencil Should Be Authoritative?").
- `docs/superpowers/specs/2026-04-27-layer-2-topp-prototype-design.md` §11 — Lee 2024 SLP adoption rationale; this spec's per-axis cut machinery is its descendant.
- `docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md` — superseded MVP spec; its motivation and Codex-cross-check transcript informed this work.
- `rust/temporal/src/topp/constraints.rs:236-247` — MAINTAINER WARNING about per-axis Cartesian jerk in the SOCP; gets an appended paragraph per §4.4.
- `rust/trajectory/tests/homing_diagnostic.rs` — this session's diagnostic matrix that pinned the failure to stencil disagreement.
- `rust/trajectory/tests/homing_300mm_pure_x.rs` — currently-failing regression; flips to passing under Option B.
