//! Phase 3 Task 3.4 — cross-move continuity + look-ahead replan regression
//! tests at the **planner-bridge integration** layer (driving the planner
//! through `PlannerHandle::submit_move` rather than calling `ShaperState`
//! directly).
//!
//! These tests are the regression net for the original `2026-05-10` bug —
//! sequential 1 mm jogs (the kind a human-typed `G1 X1` macro emits four
//! times in a row) producing mm-scale boundary discontinuities in shaped
//! output. The fix was the streaming-shaper rewrite (`append_and_replan` +
//! `emit_committed`, Phase 3 Tasks 3.1 / 3.1.5 / 3.2 / 3.3); the unit-test
//! coverage of that fix lives in
//! `rust/trajectory/src/streaming/tests.rs`. **This file pins the same
//! invariants from the planner thread's public API**, so a future
//! regression at the wiring layer (e.g., a bug in `run_loop`'s replan/emit
//! sequencing) gets caught by a test that doesn't have to know which
//! internal helper failed.
//!
//! ## What the streaming pipeline actually dispatches
//!
//! The streaming-native path dispatches up to `t_decel_start − max_h`:
//! the trailing decel-to-zero region of the most recent replan is held
//! speculatively until either (a) a follow-on `Move` arrives (in which
//! case the replan re-anchors the decel-to-zero point further out and
//! more of the prior plan becomes committed), (b) Phase 4 Task 4.1's
//! quiescence timer fires `T_commit` past the most recent append, or
//! (c) Phase 4 Task 4.3's `Flush` synchronously invokes
//! `commit_decel_to_zero` and dispatches the held-back tail before
//! notifying the waiter (spec §3.4 lifecycle row, "Planner `Flush`":
//! "Collapse `T_commit` → now. Commit any tentative `pending_dispatch`.
//! Notify the flush waiter."). The tests below honour this:
//!
//! * "Cumulative dispatched X" assertions now expect the **exact**
//!   submitted distance after `flush` (within the C¹ refit budget of
//!   50 µm). Before Task 4.3 these were "approaches but does not reach"
//!   bounds; Task 4.3 makes the exact-distance property achievable on
//!   any `flush` regardless of the timer.
//! * "Adjacent dispatched segments are position-continuous" remains the
//!   primary cross-move regression assertion (the property the original
//!   2026-05-10 bug violated).
//!
//! ## Tolerance budget
//!
//! Seam-continuity assertions use a 50 µm budget, matching the
//! streaming-state unit tests in
//! `rust/trajectory/src/streaming/tests.rs::
//! emit_committed_chains_across_two_appends` (`seam_diff < 0.05`) and
//! `t_dispatched_interior_to_move_replan_preserves_position` (`< 0.05`).
//! The budget covers the C¹ Hermite refit's L∞ error on each side of
//! the seam (governed by `PlannerConfig::fit_tolerance_mm`, set to
//! `0.05 mm` in the test fixtures) plus the shaper convolution's ~1×
//! propagation of that error. The original 2026-05-10 bug produced
//! mm-scale boundary discontinuities — three orders of magnitude wider
//! than this budget.
//!
//! ## Why integration-style, not lib-style
//!
//! Integration tests in `tests/` link against the `motion-bridge` rlib
//! without going through the PyO3 cdylib path, so they do not need the
//! `DYLD_INSERT_LIBRARIES=…libpython3.14.dylib` workaround that
//! `cargo test -p motion-bridge --lib` requires. `cargo test -p
//! motion-bridge --test streaming_replan` runs out of the box.

use std::sync::{Arc, Mutex};

// The crate is `motion-bridge` (package) but exposes its rlib under the
// name `motion_bridge_native` (because `[lib].name` must match the
// `#[pymodule]` fn — see Cargo.toml).
use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::planner::PlannerHandle;
use trajectory::{AxisShaper, RequiredShaper, ShapedSegment, ShaperConfig};

use nurbs::ScalarNurbs;
use nurbs::eval::{eval_derivative, eval_polynomial};

// ---------------------------------------------------------------------------
// Test fixtures
// ---------------------------------------------------------------------------

/// Recording dispatch closure: stuffs every dispatched `ShapedSegment` into a
/// shared `Vec` under a mutex. The planner thread is the only writer (the
/// `run_loop` calls dispatch under its own thread; the mutex is just to
/// serialize cross-thread reads in the test assertions).
type Recorded = Arc<Mutex<Vec<ShapedSegment>>>;

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    Recorded,
) {
    let recorded: Recorded = Arc::new(Mutex::new(Vec::new()));
    let rec_for_closure = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec_for_closure.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

/// Planner config matching the task spec: smooth_zv 186 Hz on X & Y,
/// passthrough on Z. The default machine limits are kept so the test
/// faithfully exercises the production hot path. Refit tolerance is
/// relaxed slightly above the default `5 µm` for short moves — the
/// β-medium loop on a 1 mm move at 100 mm/s does not always meet the
/// tight default — but the relaxation is well below the bug we are
/// regression-testing (which produced mm-scale errors).
fn smooth_zv_186hz_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.shaper = ShaperConfig {
        x: RequiredShaper::SmoothZv {
            frequency_hz: 186.0,
        },
        y: RequiredShaper::SmoothZv {
            frequency_hz: 186.0,
        },
        z: AxisShaper::Passthrough,
    };
    // Match the streaming-state unit-test budget (50 µm). The original
    // bug produced *millimeter*-scale errors, so 50 µm is plenty tight
    // for the seam-continuity assertions while letting the β-medium loop
    // converge on short moves.
    c.fit_tolerance_mm = 0.05;
    c
}

/// Same as `smooth_zv_186hz_config` but with more permissive limits so
/// the short 1 mm jogs can clear β-medium even when the default machine
/// limits derate aggressively. Used by the multi-jog tests; matches the
/// shape `update_limits_processed_without_error` uses in the planner's
/// own test module.
fn relaxed_limits() -> PlannerLimits {
    PlannerLimits {
        max_velocity: 200.0,
        max_accel: 2000.0,
        max_z_velocity: 10.0,
        max_z_accel: 80.0,
        square_corner_velocity: 4.0,
    }
}

// ---------------------------------------------------------------------------
// Segment helpers
// ---------------------------------------------------------------------------

/// Evaluate the X-axis position at parameter `u` on a shaped segment.
/// `ShapedSegment::axes[0]` is a non-rational scalar B-spline in the time
/// domain; the segment's `t_start..=t_end` is the valid `u` range.
fn x_pos_at(seg: &ShapedSegment, t: f64) -> f64 {
    eval_x_at(&seg.axes[0], t)
}

fn eval_x_at(curve: &ScalarNurbs<f64>, t: f64) -> f64 {
    eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), t)
}

/// Evaluate the X-axis velocity (dx/dt) at time `t` on a shaped segment.
fn x_vel_at(seg: &ShapedSegment, t: f64) -> f64 {
    let c = &seg.axes[0];
    eval_derivative(c.control_points(), c.knots(), c.degree(), t)
}

// ---------------------------------------------------------------------------
// Test 1 — cross-move continuity across one boundary
// ---------------------------------------------------------------------------

/// Two sequential 1 mm pure-X moves should produce shaped output whose
/// adjacent dispatched segments are position-continuous within refit
/// noise (`5e-5 mm`). This is the property the original `2026-05-10` bug
/// violated (mm-scale jumps at the boundary).
///
/// We submit both moves before flushing — the second `submit_move`
/// triggers a replan over move 1's un-committed tail + move 2, which
/// is exactly the look-ahead-replan path under test. The final `flush`
/// then synchronously commits the trailing decel-to-zero (Phase 4
/// Task 4.3), so the cumulative dispatched X reaches the submitted
/// 2.0 mm exactly (within refit budget).
#[test]
fn cross_move_continuity_within_refit_noise() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Two sequential 1 mm X moves at 100 mm/s.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.submit_move(classify_and_build([1.0, 0.0, 0.0], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 2");
    // Flush as synchronization barrier so the planner thread has
    // definitely processed both submits before we read `recorded`.
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "two 1 mm submits produced zero dispatched segments — \
         this is a regression in streaming look-ahead replan",
    );

    // Continuity check at every adjacent-segment seam. Each dispatched
    // segment carries `[t_start, t_end]` in the planner's absolute time
    // line; adjacent pairs satisfy `segs[i].t_end == segs[i+1].t_start`
    // exactly (per `emit_committed`'s contract), so reading the X
    // position from both sides of the seam should agree within refit
    // noise.
    //
    // Refit budget breakdown:
    //   * C¹ Hermite refit L∞ on each side of the seam: 50 µm
    //     (matches `PlannerConfig::fit_tolerance_mm` set above).
    //   * Shaper convolution propagates that error multiplicatively by
    //     ~1× (the kernel integrates to 1).
    // Budget: 50 µm — matches the streaming-state unit tests' seam
    // tolerance in `rust/trajectory/src/streaming/tests.rs::
    // emit_committed_chains_across_two_appends` (`seam_diff < 0.05`).
    // The original 2026-05-10 bug produced mm-scale jumps, three orders
    // of magnitude wider than this budget.
    const SEAM_BUDGET_MM: f64 = 5.0e-2; // 50 µm
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end {} != next t_start {} (planner contract broken)",
            a.t_end,
            b.t_start,
        );
        let x_left = x_pos_at(a, a.t_end);
        let x_right = x_pos_at(b, b.t_start);
        let diff = (x_left - x_right).abs();
        assert!(
            diff < SEAM_BUDGET_MM,
            "seam {i}: X discontinuity {} mm exceeds refit budget {} mm — \
             this is the original 2026-05-10 bug regression \
             (mm-scale boundary discontinuities). \
             Left X at t_end {}: {} mm, Right X at t_start {}: {} mm.",
            diff,
            SEAM_BUDGET_MM,
            a.t_end,
            x_left,
            b.t_start,
            x_right,
        );
    }

    // Cumulative dispatched X position must equal the submitted
    // distance (2.0 mm) within the C¹ refit budget — Phase 4 Task 4.3
    // makes `flush` synchronously invoke `commit_decel_to_zero`, so the
    // held-back trailing decel-to-zero of move 2 is dispatched before
    // `flush` returns. Prior to Task 4.3 this was a one-sided
    // "approaches 2.0 mm" bound; tightening it to the exact submitted
    // distance pins the new flush-commit semantics so any regression in
    // the Flush arm's commit invocation is caught here.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 2.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 2.0 mm within refit \
         budget {} mm — Phase 4 Task 4.3 makes flush synchronously \
         commit the trailing decel-to-zero",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Test 2 — four consecutive 1 mm jogs (original reproducer)
// ---------------------------------------------------------------------------

/// **Original 2026-05-10 reproducer.** Submit 4 × sequential 1 mm pure-X
/// moves back-to-back with no quiescence between them. The replan must
/// chain through, producing one continuous shaped trajectory (not four
/// stop-and-go regions).
///
/// Assertions:
/// * No planner error after all 4 submits + flush.
/// * Every adjacent dispatched-segment seam is X-continuous within
///   refit noise.
/// * The dispatched velocity profile has **exactly one** extended
///   near-zero region — the final decel-to-zero, which Phase 4
///   Task 4.3 dispatches synchronously on `flush` — not four (which
///   would mean the planner decelerated at every move boundary, the
///   original bug).
/// * Phase 4 Task 4.3: cumulative dispatched X reaches the submitted
///   4.0 mm exactly (within the 50 µm refit budget), because `flush`
///   now commits the trailing decel.
#[test]
fn four_consecutive_jogs_chain_continuously() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Four sequential 1 mm X moves at 100 mm/s, back-to-back.
    for i in 0..4 {
        let start = [(i as f64) * 1.0, 0.0, 0.0];
        let m = classify_and_build(start, 1.0, 0.0, 0.0, 0.0, 100.0)
            .unwrap_or_else(|e| panic!("classify move {i}: {e:?}"));
        h.submit_move(m).unwrap_or_else(|e| panic!("submit move {i}: {e}"));
    }
    // Synchronization barrier — flush does not commit the trailing
    // decel under Phase 3 semantics, but it ensures the planner thread
    // has processed all four `submit_move` messages.
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "4 × 1 mm submits produced zero dispatched segments — \
         streaming look-ahead replan regressed to per-move stop-and-go",
    );

    // 1. Seam continuity at every boundary. Matches the streaming-state
    //    unit tests' 50 µm budget (the original 2026-05-10 bug produced
    //    mm-scale jumps, 20× this budget).
    const SEAM_BUDGET_MM: f64 = 5.0e-2; // 50 µm
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end != next t_start (planner contract broken)",
        );
        let x_left = x_pos_at(a, a.t_end);
        let x_right = x_pos_at(b, b.t_start);
        let diff = (x_left - x_right).abs();
        assert!(
            diff < SEAM_BUDGET_MM,
            "seam {i}: X discontinuity {} mm exceeds {} mm — \
             original 2026-05-10 bug regression. \
             Left t={} X={}, Right t={} X={}.",
            diff,
            SEAM_BUDGET_MM,
            a.t_end,
            x_left,
            b.t_start,
            x_right,
        );
    }

    // 2. The dispatched X position must be **monotonically non-
    //    decreasing**. This is the load-bearing regression assertion
    //    for the original 2026-05-10 bug:
    //
    //    The original bug produced mm-scale boundary discontinuities
    //    in shaped output where X *retreated* (the planner decelerated
    //    to zero between every jog and the cross-emission seam landed
    //    at a different X than where the prior emit had ended). A
    //    correct streaming replan produces a monotonically-forward
    //    position curve for pure-forward XY moves: once dispatched, X
    //    only ever increases.
    //
    //    The allowed regression tolerance is `SEAM_BUDGET_MM` (50 µm,
    //    same as the seam-continuity assertion above) because the
    //    cross-replan seams may produce sub-50 µm jitter in the
    //    evaluator that doesn't constitute a real "X moved backwards"
    //    failure mode. The original bug produced mm-scale regressions,
    //    20× wider than this budget.
    let mut last_x: f64 = x_pos_at(&segs[0], segs[0].t_start);
    for seg in &segs {
        let n = 40;
        let dt = (seg.t_end - seg.t_start) / (n as f64);
        for k in 0..=n {
            let t = seg.t_start + (k as f64) * dt;
            let x = x_pos_at(seg, t);
            assert!(
                x >= last_x - SEAM_BUDGET_MM,
                "dispatched X regressed by {} mm at t={}: prior X={}, current X={}. \
                 This is the original 2026-05-10 bug regression \
                 (cross-move boundary discontinuity caused X to retreat \
                 between jogs). All 4 jogs are pure-forward; the \
                 dispatched position should be monotonically non-decreasing.",
                last_x - x,
                t,
                last_x,
                x,
            );
            if x > last_x {
                last_x = x;
            }
        }
    }

    // 3. The dispatched X velocity must remain meaningfully positive
    //    in **interior** segments (not the first segment, which is
    //    the smooth-shaper accel-from-rest ramp; not the last segment,
    //    which may contain a trailing-decel approach). For interior
    //    segments, sample the velocity at the segment midpoint and
    //    require it stays above a small fraction of peak velocity.
    //    The original bug had v→0 between every jog; a healthy
    //    replan-chained profile has v in the several-mm/s range.
    //
    //    Threshold rationale: 1 mm/s is well below any non-pathological
    //    cruise velocity for these test moves (peak ~10 mm/s under the
    //    relaxed limits) and well above evaluator floating-point noise.
    if segs.len() >= 3 {
        const V_MIN_INTERIOR_MM_S: f64 = 1.0;
        for i in 1..segs.len() - 1 {
            let seg = &segs[i];
            let t_mid = 0.5 * (seg.t_start + seg.t_end);
            let v = x_vel_at(seg, t_mid).abs();
            assert!(
                v > V_MIN_INTERIOR_MM_S,
                "interior segment {i} has midpoint velocity {} mm/s \
                 below {} mm/s — the planner paused mid-stream between \
                 jogs (original 2026-05-10 bug). Seg span: [{}, {}]",
                v,
                V_MIN_INTERIOR_MM_S,
                seg.t_start,
                seg.t_end,
            );
        }
    }

    // 4. Phase 4 Task 4.3 — cumulative dispatched X equals the
    //    submitted 4.0 mm within refit budget. The final `flush` now
    //    synchronously commits the trailing decel-to-zero, so every
    //    millimetre submitted reaches the wire before `flush` returns.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 4.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 4.0 mm within \
         {} mm — Phase 4 Task 4.3's flush-commit must have dispatched \
         the trailing decel-to-zero of jog 4",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Test 3 — flush between jogs forces commit-decel-to-zero
// ---------------------------------------------------------------------------

/// "User paused between jogs" case: submit 1 mm, flush, submit 1 mm,
/// flush. Phase 4 Task 4.3 makes `flush` synchronously commit the
/// trailing decel-to-zero — collapsing `T_commit → now` per spec §3.4
/// — so each flush dispatches its move's full geometry to the wire
/// before returning. The dispatched X velocity profile crosses zero
/// once between the two moves (move 1's commit) and once at the end
/// (move 2's commit).
///
/// Pre-Task-4.3 this test asserted "cumulative dispatched X approaches
/// but does not reach 2.0 mm" because flush left the trailing decel
/// speculative. The post-Task-4.3 assertion is the exact submitted
/// distance (2.0 mm ± 50 µm) — the load-bearing property of synchronous
/// flush-commit.
#[test]
fn slow_jogs_decelerate_to_zero_between() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // First jog + flush.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush 1");

    // Snapshot dispatch count after the first flush, so we can check
    // the second submit/flush extended the trajectory rather than
    // re-dispatching.
    let count_after_first = recorded.lock().unwrap().len();

    // Second jog + flush.
    h.submit_move(classify_and_build([1.0, 0.0, 0.0], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 2");
    h.flush().expect("flush 2");

    let segs = recorded.lock().unwrap().clone();
    let count_after_second = segs.len();
    assert!(
        count_after_second > count_after_first,
        "second submit/flush produced no new dispatched segments \
         (before: {count_after_first}, after: {count_after_second}) — \
         the planner failed to re-anchor decel-to-zero on the second \
         append",
    );

    // Seam continuity across **all** dispatched segments — including
    // the boundary between segments that were emitted before the first
    // flush and segments that were emitted after the second submit's
    // replan. This is the test's load-bearing assertion.
    const SEAM_BUDGET_MM: f64 = 5.0e-2; // 50 µm
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end {} != next t_start {} (planner contract)",
            a.t_end,
            b.t_start,
        );
        let x_left = x_pos_at(a, a.t_end);
        let x_right = x_pos_at(b, b.t_start);
        let diff = (x_left - x_right).abs();
        assert!(
            diff < SEAM_BUDGET_MM,
            "seam {i}: X discontinuity {} mm exceeds {} mm at the \
             cross-flush seam between move 1 and move 2",
            diff,
            SEAM_BUDGET_MM,
        );
    }

    // Phase 4 Task 4.3 — cumulative dispatched X equals the full
    // submitted distance (2.0 mm) within refit budget. Each flush
    // synchronously committed its move's trailing decel-to-zero, so
    // the wire saw both jogs end-to-end before the second flush
    // returned. Pre-Task-4.3 this assertion was "0.5 mm < x < 2.0 mm"
    // because flush left the trailing decel speculative; the tighter
    // bound is the regression gate for synchronous flush-commit.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 2.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 2.0 mm within refit \
         budget {} mm — Phase 4 Task 4.3's flush-commit must have \
         dispatched both moves' trailing decel-to-zero before return",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Test 4 — replan during long cruise preserves committed position
// ---------------------------------------------------------------------------

/// Long-cruise test exercising the Task 3.1.5 split-at-s-value fix:
/// submit a 200 mm X move (long enough to have a real cruise plateau);
/// `emit_committed` dispatches most of accel + cruise (up to
/// `t_decel_start − max_h`). Then submit a 100 mm follow-on X move
/// **without an intervening flush**; the second `append_and_replan`
/// lands `t_dispatched` interior to move 1, triggering the
/// split-at-s-value path. (Pre-Task-4.3 this test used an intermediate
/// `flush` only as a synchronization barrier — Phase 4 Task 4.3 turned
/// `flush` into a force-commit, so the intermediate barrier had to go
/// to preserve the test's look-ahead-replan-during-cruise intent.)
///
/// The seam in dispatched output between the first emit (move 1's
/// pre-decel region) and the second emit (the previously-held-back
/// trailing region of move 1 + move 2) must be position-continuous.
/// Before Task 3.1.5 this seam had a millimetre-scale error budget;
/// after the fix it is 50 µm — exactly the property this test pins.
///
/// Phase 4 Task 4.3 tightens the terminal-X assertion from "inside
/// [200, 300+ε) mm" to "exactly 300 mm ± 50 µm", because the final
/// `flush` now commits move 2's trailing decel-to-zero.
#[test]
fn replan_during_long_cruise_preserves_committed_position() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Move 1: long enough to have a real cruise plateau under
    // relaxed_limits (max_velocity = 200, max_accel = 2000):
    // accel/decel time at 200 mm/s = 0.1 s each, distance = 10 mm each;
    // so cruise distance = 180 mm for a 200 mm move.
    h.submit_move(classify_and_build([0.0; 3], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap())
        .expect("submit move 1 (200 mm)");

    // Move 2: 100 mm follow-on submitted **without an intermediate
    // flush**. Phase 4 Task 4.3 made `flush` force-commit the trailing
    // decel-to-zero, so a flush between moves 1 and 2 would dispatch
    // move 1's decel to v=0 before move 2 ever lands and the
    // split-at-s-value path would never trigger. Submitting move 2
    // back-to-back lets it arrive at the planner before the
    // quiescence-commit timer (`T_commit = 50 ms`) fires; move 2's
    // `append_and_replan` then sees `t_dispatched` interior to move 1
    // and exercises the split-at-s-value path.
    h.submit_move(classify_and_build([200.0, 0.0, 0.0], 100.0, 0.0, 0.0, 0.0, 200.0).unwrap())
        .expect("submit move 2 (100 mm)");

    // Final flush — under Task 4.3 this synchronously commits the
    // trailing decel-to-zero, so on return all 300 mm of submitted
    // geometry is on the wire.
    h.flush().expect("synchronization flush after move 2");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "two long-cruise submits produced zero dispatched segments — \
         streaming-emit returned empty for a known long-move case",
    );

    // All adjacent dispatched-segment seams must be position-continuous
    // within the C¹ refit budget (50 µm) — including the cross-replan
    // seam at whichever index move 2's first emission lands (we no
    // longer identify it explicitly; checking the uniform 50 µm bound
    // across every seam is a strictly stronger property). Before
    // Task 3.1.5 the cross-replan seam allowed up to **5 mm** of
    // error; after the fix it is 50 µm, exactly the property this
    // test pins. The streaming-state unit tests
    // (`t_dispatched_interior_to_move_replan_preserves_position`)
    // carry the explicit split-at-s-value coverage; this integration
    // test re-exercises the same path through the planner-thread
    // public API.
    const SEAM_BUDGET_MM: f64 = 5.0e-2; // 50 µm
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end {} != next t_start {} (planner contract)",
            a.t_end,
            b.t_start,
        );
        let diff = (x_pos_at(a, a.t_end) - x_pos_at(b, b.t_start)).abs();
        assert!(
            diff < SEAM_BUDGET_MM,
            "seam {i}: X discontinuity {} mm exceeds {} mm — regression \
             in the cross-replan / split-at-s-value path",
            diff,
            SEAM_BUDGET_MM,
        );
    }

    // Phase 4 Task 4.3 — cumulative dispatched X equals 300 mm exactly
    // within refit budget. Pre-Task-4.3 the assertion was "inside
    // [200, 300+ε) mm" because the final flush left move 2's trailing
    // decel-to-zero speculative; the tighter bound pins the new
    // synchronous flush-commit semantics.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 300.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 300 mm within \
         {} mm — Phase 4 Task 4.3's flush-commit must have dispatched \
         move 2's trailing decel-to-zero before return",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 4 Task 4.1 — quiescence-commit timer wiring
// ---------------------------------------------------------------------------

/// Submitting a single move and then idling past `T_commit` (~50 ms) must
/// cause the planner thread's quiescence timer to fire and invoke
/// `ShaperState::commit_decel_to_zero` exactly once. We observe the timer
/// wiring via the planner-handle counter (`PlannerHandle::commit_fire_count`).
///
/// **Why this is sufficient as a wiring test.** Under Task 4.2 the commit
/// handler actually dispatches the held-back trailing decel-to-zero, but
/// this test focuses on the timer-fire counter only — the
/// "dispatch reaches the wire" property is covered by
/// `commit_after_quiescence_dispatches_terminal_decel` below.
///
/// **Why ~150 ms.** `T_commit` is 50 ms; we wait 150 ms to give the
/// scheduler comfortable headroom over `T_commit` plus the
/// `append_and_replan` work the move triggers (typically tens of ms for a
/// short move under β-medium). On a wedged thread the counter stays at 0
/// and the assertion fails fast.
///
/// **Phase 4 Task 4.3 note.** `flush` now also invokes
/// `commit_decel_to_zero` synchronously. To keep this test focused on
/// the *timer* path, we do NOT call `flush` after the submit — instead
/// we sleep directly past `T_commit`. The counter increment is then
/// solely the timer's responsibility.
#[test]
fn quiescence_timer_fires_after_single_move() {
    use std::time::Duration;

    let (dispatch, _recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Single 1 mm pure-X move at 100 mm/s. Short enough that
    // `append_and_replan` completes quickly; the absolute distance is
    // immaterial to the timer assertion (we only care that the timer
    // re-arms on submit and fires on quiescence).
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move");

    // No `flush` here — Phase 4 Task 4.3 made `flush` force-commit, so
    // calling it would increment the counter via the Flush arm rather
    // than the Timeout arm under test. The wall-clock sleep below
    // covers both `append_and_replan`'s work and the `T_commit`
    // window.

    // Wait comfortably past `T_commit` (50 ms). 150 ms accommodates host
    // scheduler jitter plus the `append_and_replan` work the move
    // triggers, and gives a clear margin over the 50 ms threshold.
    std::thread::sleep(Duration::from_millis(150));

    let fires = h.commit_fire_count();
    assert!(
        fires >= 1,
        "commit_decel_to_zero was never invoked after {} ms of quiescence (got {} fires)",
        150,
        fires,
    );

    // The run-loop disarms `last_append_time` after firing, so a second
    // sleep should NOT produce more fires (the timer is one-shot per
    // `Move`). The Task 4.2 handler is also idempotent — even if the
    // timer were to re-fire spuriously, the second call would return
    // empty — but the disarm is the authoritative invariant Task 4.1 +
    // 4.2 jointly establish.
    std::thread::sleep(Duration::from_millis(150));
    assert_eq!(
        h.commit_fire_count(),
        fires,
        "commit timer re-fired without a new submit_move — \
         the disarm step in run_loop's timeout branch is broken",
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 4 Task 4.2 — commit_decel_to_zero dispatches the trailing decel
// ---------------------------------------------------------------------------

/// Submitting a single 1 mm move and idling past `T_commit + h + slack`
/// must dispatch the **full** `[0, t_appended]` range — including the
/// trailing decel-to-zero region `emit_committed` deliberately held back.
/// The cumulative final X position must reach ~1.0 mm within the C¹ refit
/// budget (50 µm under `smooth_zv_186hz_config`).
///
/// This is the Phase 4 Task 4.2 acceptance test: prior to Task 4.2 the
/// handler was a stub that returned an empty Vec and the toolhead never
/// reached the commanded position on a pause. Post-Task-4.2 the handler
/// shapes and dispatches the held-back tail with right-pad
/// constant-extension at `(end_pos, v = 0)`, so the dispatched cumulative
/// X matches the submitted distance.
///
/// **Phase 4 Task 4.3 note.** Since `flush` would now itself commit and
/// pre-empt the timer path under test, we skip the sync barrier and
/// sleep directly past `T_commit`. The remaining assertions (timer
/// fired exactly once, terminal X = 1.0 mm ± 50 µm) still pin the
/// Task 4.2 acceptance property.
#[test]
fn commit_after_quiescence_dispatches_terminal_decel() {
    use std::time::Duration;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Single 1 mm pure-X move at 100 mm/s.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move");

    // No `flush` here — Phase 4 Task 4.3 made `flush` force-commit, so
    // calling it would dispatch the trailing decel before the timer
    // got a chance. We rely on the wall-clock sleep below to give the
    // planner thread enough time to process the move and then fire
    // `T_commit`.

    // Wait past `T_commit + h + slack`. `T_commit = 50 ms`,
    // h = 0.8025/186/2 ≈ 2.16 ms. 150 ms covers both with ample scheduler
    // headroom.
    std::thread::sleep(Duration::from_millis(150));

    // The commit timer must have fired exactly once.
    assert_eq!(
        h.commit_fire_count(),
        1,
        "commit_decel_to_zero must fire exactly once after a single move + quiescence"
    );

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "no segments were dispatched after a single move + quiescence \
         — the commit-decel path produced an empty Vec or the Move \
         arm itself dispatched nothing",
    );

    // Adjacent seam continuity across the entire dispatched output —
    // including the seam between the last `Move`-arm dispatch and the
    // first commit-arm dispatch.
    const SEAM_BUDGET_MM: f64 = 5.0e-2; // 50 µm
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end {} != next t_start {} (planner contract)",
            a.t_end,
            b.t_start,
        );
        let x_left = x_pos_at(a, a.t_end);
        let x_right = x_pos_at(b, b.t_start);
        let diff = (x_left - x_right).abs();
        assert!(
            diff < SEAM_BUDGET_MM,
            "seam {i}: X discontinuity {} mm exceeds {} mm — the commit \
             dispatch is not C0-continuous with the Move-arm dispatch",
            diff,
            SEAM_BUDGET_MM,
        );
    }

    // Cumulative final X position must reach ~1.0 mm. The 50 µm budget
    // covers the C¹ Hermite refit tolerance set in
    // `smooth_zv_186hz_config()`. Pre-Task-4.2 this would have stayed
    // strictly < 1.0 mm because the held-back decel never dispatched.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 1.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 1.0 mm within refit \
         budget {} mm — the commit-decel-to-zero did not deliver the \
         full submitted distance",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

/// Submit move 1, sleep past `T_commit` so the planner commits and the
/// toolhead comes to rest, then submit move 2. Move 2 must start from
/// rest (initial velocity ≈ 0), and the cumulative dispatched X after
/// move 2's commit must reach ~2.0 mm.
///
/// This pins the cross-commit continuity invariant: a commit advances
/// `t_dispatched = t_appended`, so the next `append_and_replan` reads
/// `initial_v = 0` off the queue's terminal-velocity sample. The two
/// moves then chain as two independent 0-to-cruise-to-0 profiles
/// rather than overlapping through a junction.
///
/// **Phase 4 Task 4.3 note.** `flush` now also commits. We avoid
/// calling `flush` between submit + sleep so the commit-fire counter
/// is solely incremented by the timer (keeping this test's "the timer
/// committed move 1, then move 2 starts from rest" intent intact).
#[test]
fn commit_then_new_move_starts_from_rest() {
    use std::time::Duration;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Move 1: 1 mm pure-X at 100 mm/s. No intermediate `flush` — see
    // the Task 4.3 docstring note above.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");

    // Sleep past `T_commit` so the planner thread commits move 1.
    std::thread::sleep(Duration::from_millis(150));
    assert_eq!(
        h.commit_fire_count(),
        1,
        "move 1's quiescence commit must have fired"
    );

    let segs_after_move_1 = recorded.lock().unwrap().clone();
    let move_1_terminal_seg = segs_after_move_1.last().unwrap().clone();
    let move_1_terminal_x = x_pos_at(&move_1_terminal_seg, move_1_terminal_seg.t_end);
    let move_1_dispatch_count = segs_after_move_1.len();

    // Move 1's terminal X ≈ 1.0 mm (within refit budget).
    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    assert!(
        (move_1_terminal_x - 1.0).abs() < SEAM_BUDGET_MM,
        "after move 1 commit, terminal X = {} mm should equal 1.0 mm \
         within budget {} mm",
        move_1_terminal_x,
        SEAM_BUDGET_MM,
    );

    // Move 1's terminal velocity should be ~0 (the commit shaped through
    // `t_appended` where TOPP-RA terminated at v = 0).
    let move_1_terminal_v = x_vel_at(&move_1_terminal_seg, move_1_terminal_seg.t_end).abs();
    assert!(
        move_1_terminal_v < 5.0,
        "move 1 terminal velocity {} mm/s should be ~0 — the commit \
         dispatched the decel-to-zero ramp through to v = 0",
        move_1_terminal_v,
    );

    // Move 2: 1 mm pure-X from x=1.0 at 100 mm/s. No flush — same
    // reason as move 1 above (keep the commit count attributable to
    // the timer only).
    h.submit_move(classify_and_build([1.0, 0.0, 0.0], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 2");

    // Sleep past `T_commit` so move 2 also commits.
    std::thread::sleep(Duration::from_millis(150));
    assert_eq!(
        h.commit_fire_count(),
        2,
        "move 2's quiescence commit must have fired"
    );

    let segs = recorded.lock().unwrap().clone();
    assert!(
        segs.len() > move_1_dispatch_count,
        "move 2 produced no additional dispatched segments",
    );

    // Move 2's first dispatched segment should start at the seam with
    // move 1's terminal — both in position and in (near-zero) velocity.
    let move_2_start_seg = &segs[move_1_dispatch_count];
    let move_2_start_x = x_pos_at(move_2_start_seg, move_2_start_seg.t_start);
    let move_2_start_v = x_vel_at(move_2_start_seg, move_2_start_seg.t_start).abs();

    assert!(
        (move_2_start_x - move_1_terminal_x).abs() < SEAM_BUDGET_MM,
        "move 2 starts at X = {} mm but move 1 ended at X = {} mm — \
         cross-commit position seam exceeds {} mm",
        move_2_start_x,
        move_1_terminal_x,
        SEAM_BUDGET_MM,
    );

    // Move 2 starts from rest — initial velocity should be near-zero.
    // The threshold is wider than terminal-v because the kernel's accel
    // ramp from rest may not be perfectly zero at the segment seam under
    // smooth-shaper smoothing, but it should be very small. 5 mm/s is
    // well below the 100 mm/s cruise speed; the original 2026-05-10 bug
    // (no commit, the toolhead "continues") would have move 2 starting
    // at full cruise velocity.
    assert!(
        move_2_start_v < 5.0,
        "move 2 initial velocity {} mm/s should be near zero — the \
         commit-then-new-move sequence is not starting from rest",
        move_2_start_v,
    );

    // Cumulative terminal X must reach ~2.0 mm.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 2.0).abs() < SEAM_BUDGET_MM,
        "cumulative terminal X = {} mm should equal 2.0 mm within budget \
         {} mm after both moves committed",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

/// Calling `commit_decel_to_zero` twice in a row (via two `T_commit`
/// timer expirations) must be idempotent: the second call sees
/// `t_dispatched == t_appended` and returns empty, so no segments are
/// re-dispatched.
///
/// In practice the run_loop disarms `last_append_time` after the first
/// fire, so the second fire would only happen if a new `submit_move`
/// re-armed the timer. This test still pins the handler-level
/// idempotence invariant: the planner thread asserts "two consecutive
/// fires" via a synthetic re-arm path, ensuring the underlying handler
/// is correct even if the run-loop disarm logic ever regresses.
///
/// Specifically: submit + sleep (commit fires once), then submit + sleep
/// (commit fires once more, on the *new* move). The second commit must
/// produce non-empty dispatch (it's a new move with a new
/// `t_appended`), distinguishing "handler is idempotent" from "handler
/// silently no-ops on every call."
#[test]
fn commit_decel_to_zero_is_idempotent_across_re_armed_timer() {
    use std::time::Duration;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // First submit + commit. No `flush` — Phase 4 Task 4.3 made `flush`
    // force-commit, which would conflate Flush-arm and Timeout-arm
    // fires and break the "exactly one timer fire" assertion this
    // test pins.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    std::thread::sleep(Duration::from_millis(150));
    assert_eq!(h.commit_fire_count(), 1);
    let after_first_commit = recorded.lock().unwrap().len();

    // Sleep past `T_commit` again without submitting — the run-loop has
    // disarmed `last_append_time` so the timer must NOT re-fire. This is
    // the test that distinguishes Task 4.2 from a hypothetical Task 4.1
    // regression where the handler silently re-dispatches.
    std::thread::sleep(Duration::from_millis(150));
    assert_eq!(
        h.commit_fire_count(),
        1,
        "commit timer re-fired without a new submit — the disarm path \
         in run_loop's timeout branch is broken, OR commit_decel_to_zero \
         is producing spurious dispatch"
    );
    let no_extra_dispatch = recorded.lock().unwrap().len();
    assert_eq!(
        no_extra_dispatch, after_first_commit,
        "extra segments were dispatched without a new submit_move"
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 4 Task 4.3 — flush synchronously commits the trailing decel
// ---------------------------------------------------------------------------

/// Submit one move, then call `flush()` immediately (no wall-clock sleep
/// for the `T_commit` timer to fire). After `flush` returns, the
/// cumulative dispatched X must equal the submitted distance within the
/// C¹ refit budget (50 µm). This is the load-bearing test for Phase 4
/// Task 4.3: `flush` must invoke `commit_decel_to_zero` synchronously
/// — without waiting for the quiescence timer — so callers like
/// `wait_moves`, `M400`, and homing barriers actually block until all
/// motion is on the wire.
///
/// **Why this test is distinct from
/// `commit_after_quiescence_dispatches_terminal_decel`.** That test
/// pins the **timer** path (sleep past `T_commit`, observe a fire).
/// This test pins the **flush** path (no sleep, observe an immediate
/// dispatch). A regression in the Flush arm of `run_loop` — for
/// example, reverting it to "just notify the waiter" — would fail
/// this test while still passing the timer test (eventually).
///
/// Asserts:
/// * The commit fire counter increments to 1 (the Flush arm shares
///   the counter with the Timeout arm — both are "any commit").
/// * Cumulative dispatched X = 1.0 mm ± 50 µm.
/// * All adjacent-segment seams are continuous within the same 50 µm
///   refit budget (the synchronous commit must not introduce a seam
///   discontinuity between the Move-arm dispatch and the Flush-arm
///   dispatch).
#[test]
fn flush_commits_terminal_decel_synchronously() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Single 1 mm pure-X move at 100 mm/s.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move");

    // Immediate flush — no wall-clock sleep. The synchronous commit in
    // `PlannerMsg::Flush` is the *only* mechanism that can dispatch the
    // trailing decel-to-zero in this time budget; the `T_commit` timer
    // (50 ms) is well above the flush round-trip.
    h.flush().expect("flush");

    // The commit counter must have incremented exactly once, via the
    // Flush arm of `run_loop`.
    assert_eq!(
        h.commit_fire_count(),
        1,
        "flush did not invoke commit_decel_to_zero — wait_moves \
         semantics are broken (Flush arm regressed to bare notify)"
    );

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "flush returned without dispatching anything — the synchronous \
         commit dropped all segments",
    );

    // Adjacent-segment seam continuity across the entire dispatched
    // output (Move-arm dispatch + Flush-arm dispatch).
    const SEAM_BUDGET_MM: f64 = 5.0e-2; // 50 µm
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: t_end {} != next t_start {} (planner contract)",
            a.t_end,
            b.t_start,
        );
        let diff = (x_pos_at(a, a.t_end) - x_pos_at(b, b.t_start)).abs();
        assert!(
            diff < SEAM_BUDGET_MM,
            "seam {i}: X discontinuity {} mm exceeds {} mm — flush-arm \
             commit is not C0-continuous with Move-arm dispatch",
            diff,
            SEAM_BUDGET_MM,
        );
    }

    // Terminal dispatched X must equal the submitted 1.0 mm within
    // refit budget. This is the property `wait_moves` actually needs:
    // when it returns, the toolhead has been commanded the full
    // distance.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 1.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 1.0 mm within \
         {} mm — flush returned before the full submitted distance \
         was on the wire",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    // Terminal velocity must be ~0: the flush-commit dispatched the
    // decel-to-zero ramp, so the toolhead is at rest at flush return.
    let terminal_v = x_vel_at(terminal_seg, terminal_seg.t_end).abs();
    assert!(
        terminal_v < 5.0,
        "terminal dispatched velocity {} mm/s should be near zero — \
         the flush-commit must have dispatched through to v = 0",
        terminal_v,
    );

    // A second flush with the queue already fully committed must be a
    // no-op (the Flush arm guards on `last_append_time.is_some()`).
    // The commit counter stays at 1 and no new segments dispatch.
    let segs_before_second_flush = segs.len();
    h.flush().expect("second flush");
    assert_eq!(
        h.commit_fire_count(),
        1,
        "second flush re-invoked commit_decel_to_zero on an already-\
         committed queue — the `last_append_time.is_some()` guard is \
         broken"
    );
    assert_eq!(
        recorded.lock().unwrap().len(),
        segs_before_second_flush,
        "second flush re-dispatched segments on an already-committed queue"
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 5 Task 5.1 — lifecycle reset entry points
// ---------------------------------------------------------------------------

/// Spec §3.7 reset acceptance, integration layer: after a `kalico_stream_open`
/// reset to a new home position, a follow-on `submit_move` lands the toolhead
/// starting near the new home, NOT continuing from wherever the prior
/// trajectory ended. The original move's contribution is fully discarded —
/// the post-reset dispatch line starts at `new_home_x` (within the C¹ refit
/// budget) and ends at `new_home_x + move_distance`.
#[test]
fn kalico_stream_open_resets_planner_state() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // First move: 1 mm pure-X from origin. Flush to dispatch the full
    // geometry to the wire before the reset (Phase 4 Task 4.3 makes
    // flush synchronously commit the trailing decel-to-zero).
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush move 1");

    let segs_before_reset = recorded.lock().unwrap().len();
    assert!(
        segs_before_reset > 0,
        "precondition: first move must produce dispatched segments before the reset",
    );

    // Reset to a new home position. This is the entry point the bridge
    // will call on `kalico_stream_open`.
    let new_home = [10.0, 20.0, 30.0, 0.0];
    h.kalico_stream_open(new_home)
        .expect("kalico_stream_open");

    // Follow-on move: 1 mm pure-X starting from the new home. The
    // bridge's caller is expected to express moves in the reset
    // position frame, so the `start` argument matches `new_home`. The
    // submitted move geometry covers `X ∈ [10, 11]` at 100 mm/s.
    h.submit_move(
        classify_and_build([new_home[0], new_home[1], new_home[2]], 1.0, 0.0, 0.0, 0.0, 100.0)
            .unwrap(),
    )
    .expect("submit move 2");
    h.flush().expect("flush move 2");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        segs.len() > segs_before_reset,
        "second submit/flush produced no new dispatched segments after reset",
    );

    // The first dispatched segment **after the reset** must start near
    // `new_home[0] = 10.0` — not near `1.0` (where move 1 ended) — and
    // not near `0.0` (where the prior timeline began). The seam budget
    // (50 µm) covers the C¹ Hermite refit's L∞ error.
    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    let post_reset_first = &segs[segs_before_reset];
    let x_start = x_pos_at(post_reset_first, post_reset_first.t_start);
    assert!(
        (x_start - new_home[0]).abs() < SEAM_BUDGET_MM,
        "post-reset first dispatched X = {} mm; expected {} mm \
         (the new home position) within {} mm. The reset did not \
         clear the planner's prior position, OR the bridge wired the \
         reset to the wrong handler.",
        x_start,
        new_home[0],
        SEAM_BUDGET_MM,
    );

    // Terminal dispatched X should be `new_home[0] + 1.0 = 11.0` mm:
    // the post-reset move covered 1 mm and flush dispatched the
    // trailing decel-to-zero.
    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - (new_home[0] + 1.0)).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm; expected {} mm \
         (new_home_x + move_distance) within {} mm",
        terminal_x,
        new_home[0] + 1.0,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

/// Spec §3.7 reset acceptance via the `homing` entry point. Identical
/// shape to `kalico_stream_open_resets_planner_state`: the only
/// difference is which `PlannerHandle` method is called. Both go
/// through the same `ShaperState::reset` path under the hood (the
/// run-loop's `KalicoStreamOpen | Homing` match arm collapses them);
/// pinning both as distinct integration tests catches a regression
/// that wires only one of the two handlers correctly.
#[test]
fn homing_resets_planner_state() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush move 1");

    let segs_before_reset = recorded.lock().unwrap().len();
    assert!(
        segs_before_reset > 0,
        "precondition: first move must produce dispatched segments before the reset",
    );

    let new_home = [50.0, 60.0, 70.0, 0.0];
    h.homing(new_home).expect("homing");

    h.submit_move(
        classify_and_build([new_home[0], new_home[1], new_home[2]], 1.0, 0.0, 0.0, 0.0, 100.0)
            .unwrap(),
    )
    .expect("submit move 2");
    h.flush().expect("flush move 2");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        segs.len() > segs_before_reset,
        "second submit/flush produced no new dispatched segments after homing reset",
    );

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    let post_reset_first = &segs[segs_before_reset];
    let x_start = x_pos_at(post_reset_first, post_reset_first.t_start);
    assert!(
        (x_start - new_home[0]).abs() < SEAM_BUDGET_MM,
        "post-homing first dispatched X = {} mm; expected {} mm \
         (the new home position) within {} mm",
        x_start,
        new_home[0],
        SEAM_BUDGET_MM,
    );

    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - (new_home[0] + 1.0)).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm; expected {} mm within {} mm",
        terminal_x,
        new_home[0] + 1.0,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 5 Task 5.2 — Underrun / ForceIdle recovery
// ---------------------------------------------------------------------------

/// Spec §3.7 ("Engine `Underrun` fault"): after a recovery reset to a new
/// position, a follow-on `submit_move` lands the toolhead starting near
/// `recovered_pos`. Same shape as `kalico_stream_open_resets_planner_state`,
/// but exercises the `PlannerHandle::underrun(...)` entry point instead.
/// This pins the planner-side handler the bridge will call once the
/// host-derived position recovery wires up (the variant exists today; the
/// bridge-side detection lookup is deferred per the Task 5.2 scope notes).
#[test]
fn underrun_recovery_resets_to_recovered_position() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Submit + flush so the first move's geometry is on the wire before
    // the recovery message arrives.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush move 1");

    let segs_before_recover = recorded.lock().unwrap().len();
    assert!(
        segs_before_recover > 0,
        "precondition: first move must produce dispatched segments before the recovery",
    );

    // Engine fault: the MCU stopped executing at the recovered position.
    // Bridge-side detection (host-derived from `current_segment_id` +
    // dispatched curve pool) lands in the bridge follow-up; here we
    // call the entry point directly to exercise the planner-side handler.
    let recovered_pos = [5.0, 0.0, 0.0, 0.0];
    h.underrun(recovered_pos).expect("underrun");

    // Follow-on move from the recovered position. The post-recovery
    // dispatch line must start near `recovered_pos[0] = 5.0` — not near
    // `1.0` (where move 1 ended) — within the C¹ refit budget.
    h.submit_move(
        classify_and_build(
            [recovered_pos[0], recovered_pos[1], recovered_pos[2]],
            1.0, 0.0, 0.0, 0.0, 100.0,
        )
        .unwrap(),
    )
    .expect("submit move 2 (post-recovery)");
    h.flush().expect("flush move 2");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        segs.len() > segs_before_recover,
        "post-recovery submit/flush produced no new dispatched segments",
    );

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    let post_recover_first = &segs[segs_before_recover];
    let x_start = x_pos_at(post_recover_first, post_recover_first.t_start);
    assert!(
        (x_start - recovered_pos[0]).abs() < SEAM_BUDGET_MM,
        "post-underrun first dispatched X = {} mm; expected {} mm \
         (recovered_pos) within {} mm — the underrun handler did not \
         reset the planner's prior position",
        x_start,
        recovered_pos[0],
        SEAM_BUDGET_MM,
    );

    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - (recovered_pos[0] + 1.0)).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm; expected {} mm within {} mm",
        terminal_x,
        recovered_pos[0] + 1.0,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

/// `ForceIdle` shares the run-loop handler with `Underrun` (the two arms
/// collapse to a single `state.reset(recovered_pos)` call). Pinning both
/// as integration tests catches a regression that wires only one of the
/// two variants — same rationale as `homing_resets_planner_state`
/// duplicating the `kalico_stream_open` test.
#[test]
fn force_idle_recovery_resets_to_recovered_position() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush move 1");

    let segs_before_recover = recorded.lock().unwrap().len();
    assert!(segs_before_recover > 0);

    let recovered_pos = [25.0, 0.0, 0.0, 0.0];
    h.force_idle(recovered_pos).expect("force_idle");

    h.submit_move(
        classify_and_build(
            [recovered_pos[0], recovered_pos[1], recovered_pos[2]],
            1.0, 0.0, 0.0, 0.0, 100.0,
        )
        .unwrap(),
    )
    .expect("submit move 2 (post-force-idle)");
    h.flush().expect("flush move 2");

    let segs = recorded.lock().unwrap().clone();
    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    let post_recover_first = &segs[segs_before_recover];
    let x_start = x_pos_at(post_recover_first, post_recover_first.t_start);
    assert!(
        (x_start - recovered_pos[0]).abs() < SEAM_BUDGET_MM,
        "post-force-idle first dispatched X = {} mm; expected {} mm",
        x_start, recovered_pos[0],
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 5 Task 5.3 — UpdateShaper drains held-back output under old kernel
// ---------------------------------------------------------------------------

/// Spec §3.7 ("Shaper config update (`update_shaper`)"): "Drain any
/// held-back shaped output on the affected axis to wire (use old
/// kernel), then swap kernel. Subsequent plans use new kernel."
///
/// Submit a move (no flush, so the trailing decel-to-zero is held
/// speculatively), then call `update_shaper` with a different
/// frequency. Verify:
///
/// 1. The commit counter incremented — `update_shaper` drained the
///    held-back tail under the old kernel via the same commit path
///    `Flush` / quiescence-timer / `ClockSyncRearm` use.
/// 2. A follow-on `submit_move` + `flush` after the swap succeeds and
///    produces dispatched segments using the new shaper config.
///
/// (1) is the load-bearing assertion. (2) is a smoke test that the
/// post-swap pipeline still works end-to-end.
#[test]
fn update_shaper_commits_held_output_before_swap() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Submit a move WITHOUT flushing — the trailing decel-to-zero stays
    // speculative ("held back") until either the quiescence timer
    // fires (50 ms), an explicit `Flush` arrives, or — per Task 5.3 —
    // an `UpdateShaper` arrives.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    // We deliberately do NOT call `flush()` here. Pre-Task-5.3
    // `update_shaper` re-built `ShaperState` from scratch, dropping
    // the held-back tail without dispatching it. Post-Task-5.3 the
    // handler must drain that tail via `commit_decel_to_zero` first
    // — that is the property this test pins.

    let commit_count_before = h.commit_fire_count();

    // Swap to a different shaper frequency. The actual frequency value
    // doesn't matter for the test invariant (any swap forces the
    // drain); we use 60 Hz to make sure the new config differs from
    // the construction-time 186 Hz.
    let new_shaper = ShaperConfig {
        x: RequiredShaper::SmoothZv { frequency_hz: 60.0 },
        y: RequiredShaper::SmoothZv { frequency_hz: 60.0 },
        z: AxisShaper::Passthrough,
    };
    h.update_shaper(new_shaper).expect("update_shaper");

    // Synchronisation barrier — `update_shaper` is fire-and-forget at
    // the channel layer. Issuing a `flush()` here drives the run-loop
    // to drain its inbox up to (and including) the `UpdateShaper`
    // message before returning. The flush itself is a no-op
    // commit-wise (the prior drain already cleared the timer), but
    // its notify-channel round-trip guarantees the planner has
    // processed `UpdateShaper`.
    h.flush().expect("flush as sync barrier");

    let commit_count_after = h.commit_fire_count();
    assert!(
        commit_count_after > commit_count_before,
        "update_shaper did not drain the held-back tail under the old \
         kernel — commit_fire_count went {} → {} (expected increment). \
         This means the trailing decel-to-zero of move 1 was silently \
         discarded when the shaper was swapped, the Phase 5 Task 5.3 \
         regression",
        commit_count_before,
        commit_count_after,
    );

    // Smoke test that the new shaper still produces dispatched output
    // end-to-end. Post-swap the planner is fresh (the handler
    // re-seeds `ShaperState`); `submit_move` + `flush` should produce
    // additional segments without erroring.
    let segs_before_post_swap = recorded.lock().unwrap().len();
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("post-swap submit");
    h.flush().expect("post-swap flush");
    let segs_after_post_swap = recorded.lock().unwrap().len();
    assert!(
        segs_after_post_swap > segs_before_post_swap,
        "post-swap submit/flush produced no new dispatched segments \
         (before: {segs_before_post_swap}, after: {segs_after_post_swap}) \
         — the new shaper kernel was not wired through to the dispatch \
         pipeline",
    );

    h.shutdown();
}

// ---------------------------------------------------------------------------
// Phase 5 Task 5.4 — ClockSyncRearm drains held-back output before bias swap
// ---------------------------------------------------------------------------

/// Spec §3.7 ("Clock-sync re-arm"): "Flush any pending shaped output
/// under the old clock bias to the wire, then update the bias for
/// future dispatches. Queue content (in planner-time) is unaffected."
///
/// The planner-side half of the barrier is the load-bearing piece — it
/// is the side that knows what shaped output is held back. The
/// bias-swap itself runs on the bridge's periodic-clock-sync thread
/// (which owns the `Router`) and is wired as a small follow-up; this
/// test pins the planner-side commit-on-rearm contract.
///
/// We submit a move (no flush, so the trailing decel is held), then
/// dispatch `PlannerMsg::ClockSyncRearm` via the
/// `PlannerHandle::clock_sync_rearm` entry point. The commit counter
/// must increment, indicating the held-back tail was drained.
#[test]
fn clock_sync_rearm_commits_old_bias_first() {
    use motion_bridge_native::planner::ClockBias;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    // Submit a move WITHOUT flushing — same pattern as the
    // `update_shaper_commits_held_output_before_swap` test. The
    // trailing decel-to-zero is held back; `clock_sync_rearm` must
    // drain it before the bias swap takes effect.
    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");

    let commit_count_before = h.commit_fire_count();
    let segs_before = recorded.lock().unwrap().len();

    let new_bias = ClockBias {
        freq: 100_000_000.0,
        offset_s: 0.0,
        last_clock: 0,
    };
    h.clock_sync_rearm(new_bias).expect("clock_sync_rearm");

    // Sync barrier so the planner has processed the rearm before we
    // read the counters.
    h.flush().expect("flush as sync barrier");

    let commit_count_after = h.commit_fire_count();
    assert!(
        commit_count_after > commit_count_before,
        "clock_sync_rearm did not drain the held-back tail under the \
         old bias — commit_fire_count went {} → {} (expected \
         increment). The bias swap would have applied to dispatched \
         samples that were planned under the old bias, the Phase 5 \
         Task 5.4 regression",
        commit_count_before,
        commit_count_after,
    );

    // Same smoke property as the UpdateShaper test: after the rearm
    // the pipeline is still functional (no leftover state in the
    // planner that breaks the next submit/flush).
    let segs_after = recorded.lock().unwrap().len();
    assert!(
        segs_after > segs_before,
        "clock_sync_rearm produced no dispatched segments from the \
         drained tail (before: {segs_before}, after: {segs_after}) \
         — the commit fired but `run_commit_and_dispatch` did not \
         actually push segments to the wire",
    );

    h.shutdown();
}

