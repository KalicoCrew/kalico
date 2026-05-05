## Root Cause

The reported bridge error is emitted from the temporal joining loop at `rust/temporal/src/multi/joining.rs:45-48`. At that point `bidirectional_junction_sweep` has no velocity changes left to propagate (`dirty_count == 0`), but at least one `SegmentState` is still `dirty=true`, so `join_until_converged` returns `JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count: 1 }`.

The segment remains dirty because `rust/temporal/src/multi/parallel.rs:83-101` only clears `dirty` when `profile.status` is one of `Solved`, `SolvedInexact`, or `SolvedSlp`. Any `DivergedSlp`, `MaxIterSlp`, `MaxIter`, or `Infeasible` profile is stored, but deliberately left dirty. With a single homing segment there are no junctions, so the next sweep immediately takes the early-stall branch above.

The root cause is **(c) SLP non-convergence on jerk relaxation**, specifically a verifier/SLP knife edge on the jerk constraint. The TOPP-RA constraint builder uses the scalar path jerk limit `J_path = min(j_max[X], j_max[Y], j_max[Z])` in `rust/temporal/src/topp/constraints.rs:233-248`. The SOC chain bounds the width-1 second-difference stencil on `b = s_dot^2`; the verifier computes `s_triple_dot` from finite differences of `a`, which expands to a width-2 stencil. On pure-axis start/stop moves with `v_start = v_end = 0`, the SLP can plateau just outside its internal ratio threshold even when the verifier-level trajectory is acceptable. Pre-fix this surfaced as `SolveStatus::DivergedSlp`, which `fan_out_solves` treats as non-success, leaving the only segment dirty.

This is not root cause (a): the beta-medium loop already has an acceleration floor in `rust/trajectory/src/beta.rs` (`BETA_ACCEL_MIN_RATIO`) and the exact 30 mm X repro on current `sota-motion` converges in one beta iteration. It is not (b): the 50 Hz smooth-MZV support is about 19.125 ms, and the pad/trim path in `rust/trajectory/src/pad.rs` extends batch boundaries with constant-position pieces rather than making kernel width >= segment duration infeasible. It is not (d): the underlying single-segment SOCP is feasible; the failure mode is the SLP/verifier status mapping at the jerk boundary.

Important current-branch note: this checkout already contains `485ec4d93 fix(temporal): pure-axis moves stalled SLP at verifier knife-edge`. A scratch probe of the exact supplied 30 mm X homing segment on current `87225220f` returned:

```text
single status: Solved
batch joining: Converged
batch profile status: Solved
shape: Ok((Converged, None, 1))
```

So the inline test below is a regression test for the reported failure mode on the pre-fix code path. On the current checkout it should pass if written to assert convergence; to reproduce the historical stall, run it against a revision before `485ec4d93` or remove that commit's status-mapping/tolerance changes.

## Unit Test (full Rust code)

Place this as `rust/trajectory/tests/stall_homing_move.rs` when validating the historical failure. It is intentionally shown inline only; I did not add it to the workspace.

```rust
use geometry::segment::EMode;
use nurbs::VectorNurbs;
use temporal::multi::{BatchInput, GridStrategy, JoiningStatus, SegmentInput};
use temporal::{GridConfig, GridScheme, Limits, SolveStatus, ToleranceMode};
use trajectory::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

fn collinear_cubic(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    let cp1 = [
        start[0] + (end[0] - start[0]) / 3.0,
        start[1] + (end[1] - start[1]) / 3.0,
        start[2] + (end[2] - start[2]) / 3.0,
    ];
    let cp2 = [
        start[0] + 2.0 * (end[0] - start[0]) / 3.0,
        start[1] + 2.0 * (end[1] - start[1]) / 3.0,
        start[2] + 2.0 * (end[2] - start[2]) / 3.0,
    ];

    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![start, cp1, cp2, end],
        None,
    )
    .unwrap()
}

fn homing_limits() -> Limits {
    Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 200.0],
        5.0_f64.powi(2) / (3000.0 * 0.5),
    )
}

#[test]
fn homing_x_segment_does_not_remain_dirty_after_topp_solve() {
    let curve = collinear_cubic([-30.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let limits = homing_limits();

    let direct = temporal::schedule_segment_with_tolerance(
        &curve,
        &limits,
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 60,
        },
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .unwrap();

    assert!(
        matches!(
            direct.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "single-segment TOPP-RA solve returned non-success status: {:?}",
        direct.status
    );

    let segment = SegmentInput {
        curve: &curve,
        limits,
        trailing_junction_chord_tolerance_mm: 0.05,
    };
    let batch = temporal::multi::plan_batch(BatchInput {
        segments: &[segment],
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
    })
    .unwrap();

    assert!(
        matches!(batch.joining_status, JoiningStatus::Converged),
        "temporal joining left the segment dirty: {:?}; profile status = {:?}",
        batch.joining_status,
        batch.profiles[0].status
    );
}

#[test]
fn bridge_shape_batch_homing_x_repro_converges() {
    let curve = collinear_cubic([-30.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let segment = SegmentInput {
        curve: &curve,
        limits: homing_limits(),
        trailing_junction_chord_tolerance_mm: 0.05,
    };
    let shape_segment = ShapeSegmentInput {
        temporal: segment,
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 50.0,
    };

    let result = trajectory::shape_batch(&ShapeBatchInput {
        segments: &[shape_segment],
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
            y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
    });

    match result {
        Ok(output) => assert!(
            matches!(output.temporal_status, JoiningStatus::Converged),
            "shape_batch returned non-converged temporal status: {:?}",
            output.temporal_status
        ),
        Err(ShapeError::TemporalJoining(status)) => {
            panic!("reproduced planner-side stall: {status:?}");
        }
        Err(err) => panic!("unexpected shape_batch error: {err:?}"),
    }
}
```

For a test that intentionally asserts the pre-fix bug, change the second assertion to expect:

```rust
assert!(matches!(
    batch.joining_status,
    JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count: 1 }
));
```

## Proposed Fix Diff Sketch

Do not apply this on the current branch without checking whether it is already present. This is the minimal fix shape used by `485ec4d93`: accept verifier-feasible `Diverged` SLP outcomes as `SolvedInexact`, and widen the verifier tolerance from 0.1% to 0.2% to cover the width-1/width-2 jerk-stencil mismatch.

```diff
diff --git a/rust/temporal/src/topp/output.rs b/rust/temporal/src/topp/output.rs
@@
         (
             SlpOutcome::Converged { outer_iters },
             SolveStatus::Solved | SolveStatus::SolvedInexact { .. },
         ) if outer_iters > 0 => SolveStatus::SolvedSlp { outer_iters },
+        (
+            SlpOutcome::Diverged {
+                last_max_ratio: _,
+                outer_iters: _,
+            },
+            _,
+        ) if verify.feasible => SolveStatus::SolvedInexact {
+            residual: verify.worst_violation,
+        },
         (
             SlpOutcome::Diverged {
                 last_max_ratio,
                 outer_iters,
diff --git a/rust/temporal/src/topp/verify.rs b/rust/temporal/src/topp/verify.rs
@@
-/// 0.1% feasibility margin per spec §6.2.
-pub(crate) const EPS_FEAS: f64 = 1e-3;
+/// 0.2% feasibility margin. Covers the width-1 SOC jerk stencil vs width-2
+/// verifier jerk stencil mismatch at adaptive-grid knife edges.
+pub(crate) const EPS_FEAS: f64 = 2e-3;
```

## Rationale

`StalledOnInfeasibleSegment` is a secondary symptom, not the algorithm that made the segment infeasible. The joining loop is doing the right thing by refusing to clear a non-success profile: if `DivergedSlp` is truly infeasible, silently accepting it would hide a bad trajectory.

The minimal fix is therefore at the TOPP-RA status boundary. The authoritative feasibility check is `verify::check`; if the SLP reports `Diverged` because its internal residual plateaued but the verifier says the assembled profile is within tolerance, the public status should be `SolvedInexact`, matching the existing `MaxIters` promotion logic. The tolerance increase is scoped to the known discretization mismatch in jerk verification and does not relax velocity, acceleration, or the SOCP construction itself.

Ranking of candidate causes:

1. **SLP non-convergence on jerk relaxation**: most likely and matches the dirty-state mechanism (`DivergedSlp` is non-success in `fan_out_solves`).
2. **β-medium derate**: unlikely for the supplied X move on current branch; the exact shape probe converges with no beta warning.
3. **smooth-MZV kernel pad width**: unlikely; pad-and-trim supports kernel overlap beyond segment duration via constant boundary extension.
4. **SOCP single-segment edge case**: unlikely; direct `schedule_segment_with_tolerance(..., v_start=0, v_end=0)` returns a feasible profile on current branch.

## 2026-05-05 Follow-Up: Captured Descending-X SmoothZv Input

### Exact Inputs Used

I added the Rust regression test at `rust/trajectory/tests/stall_homing_move.rs` so the test exercises the same public entry point as `motion-bridge::planner::run_pipeline`: `trajectory::shape_batch`.

The test uses the captured values verbatim:

```text
buffer.len = 1
window_capacity = 32
fit_tolerance_mm = 0.005
beta_max_iters = 10
beta_convergence_ratio = 0.05
worker_threads = 3

shaper:
  x = RequiredShaper::SmoothZv { frequency_hz: 50.0 }
  y = RequiredShaper::SmoothZv { frequency_hz: 50.0 }
  z = AxisShaper::Passthrough
e_limits = ELimits { v_max: 50.0, a_max: 5000.0 }

limits:
  v_max = [300.0, 300.0, 15.0]
  a_max = [3000.0, 3000.0, 100.0]
  j_max = [6000.0, 6000.0, 200.0]
  a_centripetal_max = 5.0^2 / (3000.0 * 0.5) = 0.016666666666666666

segment:
  feedrate_mm_s = 50.0
  e_mode = Travel
  extrusion_per_xy_mm = 0.0
  degree = 3
  knots = [0,0,0,0, 1,1,1,1]
  control_points = [
      [30.0, 0.0, 0.0],
      [20.0, 0.0, 0.0],
      [10.0, 0.0, 0.0],
      [ 0.0, 0.0, 0.0],
  ]
  trailing_junction_chord_tolerance_mm = 0.05

grid_strategy = Adaptive { min_n: 20, max_n: 200, target_grid_spacing_mm: 0.5 }
```

### Does The Rust Regression Reproduce?

No. On `sota-motion` HEAD `87225220f`, the exact Rust regression does not return `StalledOnInfeasibleSegment` and does not return an error containing that text.

Evidence:

```text
$ cargo test -p trajectory --test stall_homing_move -- --nocapture
running 1 test
test captured_descending_x_smooth_zv_homing_move_does_not_stall ... ok

$ cargo test -p trajectory
test result: ok. 58 passed
test result: ok. 6 passed
test result: ok. 1 passed
```

The test result means the captured `ShapeBatchInput` reaches `ShapeBatchOutput { temporal_status: JoiningStatus::Converged, segments.len() == 1, ... }`.

### Root Cause Or Divergence Hypothesis

Because the exact public `shape_batch` input does not reproduce in Rust, the live failure is not explained by SmoothZv kernel padding or by a temporal/trajectory deterministic infeasibility for the captured curve.

Also, the prime-suspect premise is backwards for this codebase: `rust/trajectory/src/kernel.rs:7-9` and `rust/trajectory/src/kernel.rs:47-49` define SmoothZv support as `T_sm = 0.8025 / f` and SmoothMzv support as `T_sm = 0.95625 / f`. At 50 Hz:

```text
SmoothZv T_sm  = 0.8025  / 50 = 0.01605 s
SmoothMzv T_sm = 0.95625 / 50 = 0.019125 s
```

So SmoothZv is narrower than SmoothMzv here. If SmoothMzv converges, SmoothZv should not uniquely fail because of larger kernel support. The shaper path pads and trims at `rust/trajectory/src/beta.rs:427-455` via `pad_segment_axis` and `shape_axis`; for a single segment, `rust/trajectory/src/pad.rs` extends batch boundaries with constant-position pieces rather than requiring neighboring motion to cover the kernel.

The remaining divergence is between the live bridge process and the tested `shape_batch` input. Plausible stateful causes, ranked:

1. **Stale or different loaded Rust extension**: the live Python bridge may not be using the same compiled code as the Rust workspace test. This would explain why the same textual input converges in `cargo test` but stalls in the simulator. The process should log the Rust crate/version/git hash at `init_planner`.

2. **Single-slot planner error replay**: `PlannerHandle::check_error` returns and clears a previously stored planner error at `rust/motion-bridge/src/planner.rs:113-117` before enqueuing later moves. If the observed Python exception is read on a later call, it may describe an earlier `run_pipeline` input, not the currently printed buffer. The run loop stores errors at `rust/motion-bridge/src/planner.rs:320-325`.

3. **Incomplete captured state around message barriers**: `run_loop` buffers messages at `rust/motion-bridge/src/planner.rs:222-257`, applies `UpdateLimits` / `UpdateShaper` after shaping at `rust/motion-bridge/src/planner.rs:338-345`, and builds the public `ShapeBatchInput` at `rust/motion-bridge/src/planner.rs:399-432`. The capture includes the main segment/config fields but not whether a pending flush, dwell, config update, prior error, or shutdown barrier was present in the same loop turn.

4. **Bridge commanded-position divergence before classification**: `submit_homing_move_inner` reads bridge-local `commanded_pos` at `rust/motion-bridge/src/bridge.rs:1595-1605`; regular moves advance it at `rust/motion-bridge/src/bridge.rs:1274-1278`; `set_position` overwrites it at `rust/motion-bridge/src/bridge.rs:1429-1431`. On the Python side, `MotionToolhead.set_position` mirrors into the bridge at `klippy/motion_toolhead.py:244-247`, while `drip_move` submits homing at `klippy/motion_toolhead.py:299-303` without updating Python `commanded_pos` there. If prior `SET_KINEMATIC_POSITION`, `G1`, or homing reconciliation traffic diverges between Python and Rust state, a later captured exception may be associated with a different classified move than expected. The exact captured control points argue against this for this specific dump, but it is still the state boundary to instrument.

### Proposed Minimal Fix Location

No temporal or trajectory fix is justified by the current Rust reproduction result. The minimal next step is instrumentation, not a planner-side workaround:

1. In `rust/motion-bridge/src/planner.rs`, immediately before `run_pipeline(&buffer, &config)`, emit one structured event containing: planner error slot state before shaping, loop turn id, pending barrier kind, full config, every segment's degree/knots/control points/e-mode/feedrate, and a stable hash of the assembled `ShapeBatchInput`.

2. Immediately after `trajectory::shape_batch(&input)` inside `run_pipeline`, emit either `Ok { temporal_status, beta_warning, shaped_count, durations }` or `Err { ShapeError, per-profile status if available }`. The key missing field is the actual `ShapeError` returned by the same compiled extension, not only the input dump.

3. In `rust/motion-bridge/src/bridge.rs`, log `commanded_pos` before and after `submit_move`, `submit_homing_move_inner`, and `set_position`, with a monotonically increasing bridge command id. This ties Python `SET_KINEMATIC_POSITION` / `G1` / homing traffic to the exact segment later seen by the planner.

4. In the Python bridge/toolhead layer, include the same command id in `motion_toolhead.py::move`, `set_position`, and `drip_move` logs so the Rust position state can be compared against Klippy's `commanded_pos`.

If future instrumentation proves that the same in-process `ShapeBatchInput` fails inside `trajectory::shape_batch`, the fix belongs in `rust/trajectory/` around the beta iteration or shaper pad/trim path, not in `motion-bridge`. With the current Rust evidence, however, the failure is more likely stale-code or state-correlation drift than a deterministic SmoothZv TOPP-RA/shaper infeasibility.

## Stencil-Agreement Analysis

### Code Facts

The scalar path-jerk construction in `rust/temporal/src/topp/constraints.rs` uses the width-1 second-difference stencil on `b(s) = (ds/dt)^2`. The implementation derives `s''' = 0.5 * b''(s) * sqrt(b)` and encodes the interior-grid relation with `Delta2 b_i = b[i-1] - 2*b[i] + b[i+1]` at `rust/temporal/src/topp/constraints.rs:501-531`. The actual row coefficients are emitted at `rust/temporal/src/topp/constraints.rs:533-563`, with `hj = 2.0 * h * j_path` at line 534 and rows:

```text
t_i - (b[i-1] - 2*b[i] + b[i+1]) / (2*h*J_path) >= 0
t_i + (b[i-1] - 2*b[i] + b[i+1]) / (2*h*J_path) >= 0
```

The SOC chain then enforces the time-surrogate relation `t_i >= h / sqrt(b_i)` through three standard SOC blocks, documented and emitted at `rust/temporal/src/topp/constraints.rs:592-680`. The pure SOCP block is a relaxation because `t_i` is lower-bounded, not upper-bounded. The SLP loop is what tests and tightens the original path-jerk product. Its violator scan at `rust/temporal/src/topp/solver.rs:1133-1153` computes:

```text
ratio_i = abs(b[i-1] - 2*b[i] + b[i+1]) * sqrt(b[i]) / (2*J_path*h^2)
```

So the solver-side authoritative scalar path-jerk sample is:

```text
s3_soc[i] = sqrt(b[i]) * (b[i-1] - 2*b[i] + b[i+1]) / (2*h^2)
```

The SLP cuts use the same width-1 expression. `append_path_jerk_cut_to_clarabel` documents the linearized rows at `rust/temporal/src/topp/solver.rs:320-327` and emits the `b[i-1], b[i], b[i+1]` coefficients at `rust/temporal/src/topp/solver.rs:332-371`.

The verifier uses a different estimator. `verify::da_ds_at` computes `da/ds` by finite differences on `result.a`: forward at `i=0`, backward at `i=N-1`, and central `(a[i+1] - a[i-1]) / (s[i+1] - s[i-1])` for interior nodes at `rust/temporal/src/topp/verify.rs:86-115`. `verify::check` then computes `s_dddot = da_ds_at(...) * sqrt(b_i)` at `rust/temporal/src/topp/verify.rs:227-235`, feeds it into the Cartesian jerk identity at `rust/temporal/src/topp/verify.rs:139-150`, and gates feasibility with `worst_violation <= EPS_FEAS` at `rust/temporal/src/topp/verify.rs:275-280`. `EPS_FEAS` is currently `2e-3` at `rust/temporal/src/topp/verify.rs:55-57`; commit `485ec4d93` changed only this tolerance in `rust/temporal/src/topp/verify.rs`.

The acceleration linkage that defines `result.a` is also in the constraint bundle: endpoint one-sided differences and interior `a_i = (b[i+1] - b[i-1]) / (4*h)` at `rust/temporal/src/topp/constraints.rs:301-368`. Substituting that linkage into the verifier's interior central difference gives:

```text
a[i+1] = (b[i+2] - b[i]) / (4*h)
a[i-1] = (b[i] - b[i-2]) / (4*h)
da/ds |_verify,i = (a[i+1] - a[i-1]) / (2*h)
                 = (b[i+2] - 2*b[i] + b[i-2]) / (8*h^2)
s3_verify[i]     = sqrt(b[i]) * (b[i+2] - 2*b[i] + b[i-2]) / (8*h^2)
```

For collinear cubic Beziers, `c'(s)` is the active-axis unit tangent and `c''(s) = c'''(s) = 0`, so the verifier's Cartesian jerk reduces exactly to `c' * s3_verify` through `rust/temporal/src/topp/verify.rs:146-150`. The continuous physical constraint being compared is therefore the same scalar `abs(s''') <= min(j_max)`, but the solver and verifier discretize it differently.

### Taylor Expansions

Let `b_i = b(s_i)` on a uniform arclength grid `s_i = s0 + i*h`, and expand `b(s_i + k*h)` about `s_i`. Dropped terms are the next even derivatives in the same centered-difference series.

For the SOC/SLP width-1 stencil:

```text
b[i-1] - 2*b[i] + b[i+1]
  = h^2*b''(s_i) + (h^4/12)*b''''(s_i) + (h^6/360)*b''''''(s_i) + O(h^8)

s3_soc[i]
  = sqrt(b_i) * (b[i-1] - 2*b[i] + b[i+1]) / (2*h^2)
  = sqrt(b_i) * (0.5*b''(s_i)
                 + (h^2/24)*b''''(s_i)
                 + (h^4/720)*b''''''(s_i)
                 + O(h^6))
```

For the verifier width-2 stencil, using the code's own acceleration linkage first and then the central difference on `a`:

```text
b[i-2] - 2*b[i] + b[i+2]
  = 4*h^2*b''(s_i) + (4/3)*h^4*b''''(s_i) + (8/45)*h^6*b''''''(s_i) + O(h^8)

s3_verify[i]
  = sqrt(b_i) * (b[i+2] - 2*b[i] + b[i-2]) / (8*h^2)
  = sqrt(b_i) * (0.5*b''(s_i)
                 + (h^2/6)*b''''(s_i)
                 + (h^4/45)*b''''''(s_i)
                 + O(h^6))
```

Both estimate the same continuous quantity:

```text
s'''(t_i) = 0.5 * b''(s_i) * sqrt(b_i)
```

But their leading truncation terms differ:

```text
s3_verify[i] - s3_soc[i]
  = sqrt(b_i) * ((h^2/8)*b''''(s_i)
                 + (h^4/48)*b''''''(s_i)
                 + O(h^6))
```

So the background claim is confirmed with one clarification: the mismatch is `O(h^2) * sqrt(b_i) * |b''''|`, where `b''''` is the derivative of `b'''` with respect to path arclength `s`. If the shorthand `b'''prime` means `d(b''')/ds`, then the gap is `O(h^2) * |b'''prime|` up to the local `sqrt(b_i)` factor in the time-domain jerk. At bang-bang acceleration switch points, the smooth Taylor model is locally invalid; in practice the same conclusion becomes worse, not better, because the discrete stencils straddle a low-regularity kink differently.

### Which Stencil Should Be Authoritative?

For this specific straight-line failure, the width-1 SOC/SLP stencil should be authoritative. It is not merely "the one the solver used"; it has the smaller leading truncation-error coefficient for the same continuous `0.5*b''*sqrt(b)` quantity. The SOC/SLP coefficient is `1/24` on `sqrt(b)*h^2*b''''`, while the verifier coefficient is `1/6`, four times larger. Therefore the verifier is the lower-accuracy estimator on the scalar path-jerk constraint for collinear cubic segments.

That means the observed `worst_violation ~= 0.019` at `h ~= 1.5 mm` can be a false negative from stencil disagreement even when the solver has driven its own path-jerk residual to its target. Raising `EPS_FEAS` from `1e-3` to `2e-3` in commit `485ec4d93` does not solve the underlying problem because the mismatch scales with `h^2*sqrt(b)*b''''` and can exceed any fixed small tolerance on coarser adaptive grids or sharper acceleration switches.

Changing the SOC/SLP side to the verifier stencil is possible but less attractive. A verifier-matching width-2 path-jerk row would use:

```text
D2_wide[i] = b[i-2] - 2*b[i] + b[i+2]
abs(D2_wide[i]) * sqrt(b[i]) <= 8*J_path*h^2
```

With the existing `t_i >= h/sqrt(b_i)` chain, the linear rows would be:

```text
t_i - D2_wide[i] / (8*h*J_path) >= 0
t_i + D2_wide[i] / (8*h*J_path) >= 0
```

Those rows remain linear, and the `t_i,b_i` relation remains the same SOC chain, so each inner problem remains convex. However, this direction knowingly replaces the more accurate width-1 second-difference estimator with the less accurate width-2 estimator, loses two additional boundary-adjacent check locations unless special boundary stencils are added, and requires changing the SLP violator scan and `append_path_jerk_cut_to_clarabel` to use `b[i-2], b[i], b[i+2]`. That is a planning-quality regression without physical justification.

The better minimal direction is to change the verifier to use the solver's width-1 scalar path-jerk stencil for `s_dddot` where the verifier is checking the same scalar-tangential constraint. Code-level sketch:

```text
rust/temporal/src/topp/verify.rs:
  - replace or supplement da_ds_at(result, s, i) with s_dddot_from_b_stencil(result, s, i)
  - interior: sqrt(b_i) * (b[i-1] - 2*b[i] + b[i+1]) / (2*h*h)
  - endpoints: do not pretend the centered width-1 stencil exists at i=0 or i=N-1;
               either preserve the current endpoint policy explicitly or add a separately
               derived one-sided second-derivative stencil with its own error analysis
  - at rust/temporal/src/topp/verify.rs:231-235, use that value for s_dddot
```

There is one important Step-9 concern: `append_axis_jerk_cut_to_clarabel` currently says its per-axis jerk SLP cuts mirror `verify::da_ds_at` exactly at `rust/temporal/src/topp/solver.rs:131-142` and `rust/temporal/src/topp/solver.rs:373-443`. If `verify::check` is changed globally from `a`-FD to `b`-FD, the Step-9 per-axis cut linearization must be updated in the same change so the verifier and axis-jerk SLP still agree at the iterate. For the immediate straight-line path-jerk bug, a narrowly-scoped alternative is to make verifier dispatch use the width-1 `b` stencil when `c''` and `c'''` are zero at the grid point, leaving the existing axis-jerk SLP path untouched for curved geometry. That is smaller but introduces geometry-dependent verifier behavior, so the cleaner long-term fix is one unified `s_dddot` estimator plus matching Step-9 cut algebra.

### Regression Test Sketch

This test is for after the stencil fix. It belongs in an integration-test file such as `rust/trajectory/tests/stencil_agreement.rs` because `smooth_zv@50Hz` is a Layer-3 shaper input; `temporal::multi::plan_batch` itself has no shaper configuration. Mark it ignored until the source fix lands.

```rust
use geometry::segment::EMode;
use nurbs::VectorNurbs;
use temporal::Limits;
use temporal::multi::{GridStrategy, JoiningStatus, SegmentInput};
use trajectory::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

fn pure_x_300mm_collinear_cubic() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [200.0, 0.0, 0.0],
            [300.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap()
}

fn default_motion_limits() -> Limits {
    // Match the captured/default bridge limits used by the stall investigation.
    Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 200.0],
        5.0_f64.powi(2) / (3000.0 * 0.5),
    )
}

#[test]
#[ignore = "documents current verifier/SOC stencil mismatch; enable after s''' stencil unification"]
fn pure_x_collinear_cubic_smooth_zv_converges_after_stencil_fix() {
    let curve = pure_x_300mm_collinear_cubic();
    let segment = ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: default_motion_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 50.0,
    };

    let result = trajectory::shape_batch(&ShapeBatchInput {
        segments: &[segment],
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 1.5,
        },
        worker_threads: 3,
        shaper: ShaperConfig {
            x: RequiredShaper::SmoothZv { frequency_hz: 50.0 },
            y: RequiredShaper::SmoothZv { frequency_hz: 50.0 },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
    });

    match result {
        Ok(output) => {
            assert!(
                matches!(output.temporal_status, JoiningStatus::Converged),
                "expected JoiningStatus::Converged, got {:?}",
                output.temporal_status
            );
            assert_eq!(output.segments.len(), 1);
            assert!(
                output.beta_warning.is_none(),
                "unexpected beta warning: {:?}",
                output.beta_warning
            );
        }
        Err(ShapeError::TemporalJoining(status)) => {
            panic!(
                "current pre-fix failure is expected to look like \
                 StalledOnInfeasibleSegment {{ last_dirty_count: 1 }}, got {status:?}"
            );
        }
        Err(err) => panic!("unexpected shape_batch error: {err:?}"),
    }
}
```

If the test is placed directly under `rust/temporal/tests/`, remove the shaper fields and call `temporal::multi::plan_batch` with the same curve, limits, and adaptive grid. That variant should assert `output.joining_status == JoiningStatus::Converged` and then assert each `output.profiles[k].status` is `Solved`, `SolvedInexact`, or `SolvedSlp`. The public API does not expose `VerifyReport`, so `verify.feasible = true` is observed indirectly: `rust/temporal/src/topp/output.rs:69-80` maps a solved inner result with verifier failure to `SolveStatus::Infeasible`, and `rust/trajectory/src/beta.rs:319-330` rejects any per-profile non-success status.
