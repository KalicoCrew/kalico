use std::sync::{Arc, Mutex};

// The crate is `motion-bridge` (package) but exposes its rlib under the
// name `motion_bridge_native` (because `[lib].name` must match the
// `#[pymodule]` fn — see Cargo.toml).
use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::planner::{DispatchError, PlannerHandle};
use trajectory::{AxisShaper, ShapedSegment, ShaperConfig};

use nurbs::ScalarNurbs;
use nurbs::eval::{eval_derivative, eval_polynomial};

type Recorded = Arc<Mutex<Vec<ShapedSegment>>>;

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Recorded,
) {
    let recorded: Recorded = Arc::new(Mutex::new(Vec::new()));
    let rec_for_closure = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec_for_closure.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

fn smooth_zv_186hz_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.shaper = ShaperConfig {
        x: AxisShaper::SmoothZv {
            frequency_hz: 186.0,
        },
        y: AxisShaper::SmoothZv {
            frequency_hz: 186.0,
        },
        z: AxisShaper::Passthrough,
    };
    c.fit_tolerance_mm = 0.05;
    c
}

fn relaxed_limits() -> PlannerLimits {
    PlannerLimits {
        max_velocity: 200.0,
        max_accel: 2000.0,
        max_z_velocity: 10.0,
        max_z_accel: 80.0,
        square_corner_velocity: 4.0,
    }
}

fn wait_for_commits(h: &PlannerHandle, target: u32) {
    let start = std::time::Instant::now();
    while h.commit_fire_count() < target {
        assert!(
            start.elapsed() < std::time::Duration::from_secs(5),
            "commit fired only {} of {target} times within 5s",
            h.commit_fire_count()
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

fn x_pos_at(seg: &ShapedSegment, t: f64) -> f64 {
    eval_x_at(&seg.axes[0], t)
}

fn eval_x_at(curve: &ScalarNurbs<f64>, t: f64) -> f64 {
    eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), t)
}

fn x_vel_at(seg: &ShapedSegment, t: f64) -> f64 {
    let c = &seg.axes[0];
    eval_derivative(c.control_points(), c.knots(), c.degree(), t)
}

#[test]
fn cross_move_continuity_within_refit_noise() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.submit_move(classify_and_build([1.0, 0.0, 0.0], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 2");
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "two 1 mm submits produced zero dispatched segments — \
         this is a regression in streaming look-ahead replan",
    );

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
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

#[test]
fn four_consecutive_jogs_chain_continuously() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    for i in 0..4 {
        let start = [(i as f64) * 1.0, 0.0, 0.0];
        let m = classify_and_build(start, 1.0, 0.0, 0.0, 0.0, 100.0)
            .unwrap_or_else(|e| panic!("classify move {i}: {e:?}"));
        h.submit_move(m)
            .unwrap_or_else(|e| panic!("submit move {i}: {e}"));
    }
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "4 × 1 mm submits produced zero dispatched segments — \
         streaming look-ahead replan regressed to per-move stop-and-go",
    );

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
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

#[test]
fn slow_jogs_decelerate_to_zero_between() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush 1");

    let count_after_first = recorded.lock().unwrap().len();

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

    // Task 6 — time-based flush: `flush` sleeps until `sync_instant +
    // t_appended + LEAD`, so the second move arrives after real wall-clock
    // time has advanced past move 1's planner-time end. The placement rule
    // (spec §A) then inserts a rest-hold (advance_idle) bridging the gap,
    // meaning move 2's segments start at a higher planner-time than move 1's
    // segments end. Temporal contiguity therefore holds **within** each
    // move's segment batch but NOT across the cross-flush boundary.
    //
    // Position continuity at the cross-flush boundary still holds: move 1
    // ends at X=1.0 mm and move 2 starts from X=1.0 mm (the rest-hold is
    // zero velocity, same position). All other adjacent seams remain both
    // temporally and positionally contiguous.
    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    let cross_flush = count_after_first.saturating_sub(1);
    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        if i != cross_flush {
            assert!(
                (a.t_end - b.t_start).abs() < 1e-9,
                "seam {i}: t_end {} != next t_start {} (planner contract)",
                a.t_end,
                b.t_start,
            );
        } else {
            assert!(
                b.t_start >= a.t_end - 1e-9,
                "seam {i} (cross-flush): move 2 starts ({}) before move 1 ends ({})",
                b.t_start,
                a.t_end,
            );
        }
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

    let terminal_seg = segs.last().unwrap();
    let terminal_x = x_pos_at(terminal_seg, terminal_seg.t_end);
    assert!(
        (terminal_x - 2.0).abs() < SEAM_BUDGET_MM,
        "terminal dispatched X = {} mm should equal 2.0 mm within refit \
         budget {} mm — both moves' trailing decel-to-zero must be on the wire",
        terminal_x,
        SEAM_BUDGET_MM,
    );

    h.shutdown();
}

#[test]
fn replan_during_long_cruise_preserves_committed_position() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap())
        .expect("submit move 1 (200 mm)");
    h.submit_move(classify_and_build([200.0, 0.0, 0.0], 100.0, 0.0, 0.0, 0.0, 200.0).unwrap())
        .expect("submit move 2 (100 mm)");
    h.flush().expect("synchronization flush after move 2");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "two long-cruise submits produced zero dispatched segments — \
         streaming-emit returned empty for a known long-move case",
    );

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
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

#[test]
fn quiescence_timer_fires_after_single_move() {
    use std::time::Duration;

    let (dispatch, _recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move");

    wait_for_commits(&h, 1);
    let fires = h.commit_fire_count();

    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        h.commit_fire_count(),
        fires,
        "commit timer re-fired without a new submit_move — \
         the t_dispatched < t_appended guard in run_loop's timeout branch is broken",
    );

    h.shutdown();
}

#[test]
fn commit_after_quiescence_dispatches_terminal_decel() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move");

    wait_for_commits(&h, 1);

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

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
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

#[test]
fn commit_then_new_move_starts_from_rest() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");

    wait_for_commits(&h, 1);
    assert_eq!(
        h.commit_fire_count(),
        1,
        "move 1's decel-commit deadline must have fired"
    );

    let segs_after_move_1 = recorded.lock().unwrap().clone();
    let move_1_terminal_seg = segs_after_move_1.last().unwrap().clone();
    let move_1_terminal_x = x_pos_at(&move_1_terminal_seg, move_1_terminal_seg.t_end);
    let move_1_dispatch_count = segs_after_move_1.len();

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
    assert!(
        (move_1_terminal_x - 1.0).abs() < SEAM_BUDGET_MM,
        "after move 1 commit, terminal X = {} mm should equal 1.0 mm \
         within budget {} mm",
        move_1_terminal_x,
        SEAM_BUDGET_MM,
    );

    let move_1_terminal_v = x_vel_at(&move_1_terminal_seg, move_1_terminal_seg.t_end).abs();
    assert!(
        move_1_terminal_v < 5.0,
        "move 1 terminal velocity {} mm/s should be ~0 — the commit \
         dispatched the decel-to-zero ramp through to v = 0",
        move_1_terminal_v,
    );

    h.submit_move(classify_and_build([1.0, 0.0, 0.0], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 2");

    wait_for_commits(&h, 2);
    assert_eq!(
        h.commit_fire_count(),
        2,
        "move 2's decel-commit deadline must have fired"
    );

    let segs = recorded.lock().unwrap().clone();
    assert!(
        segs.len() > move_1_dispatch_count,
        "move 2 produced no additional dispatched segments",
    );

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

    assert!(
        move_2_start_v < 5.0,
        "move 2 initial velocity {} mm/s should be near zero — the \
         commit-then-new-move sequence is not starting from rest",
        move_2_start_v,
    );

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

#[test]
fn commit_decel_to_zero_is_idempotent_across_re_armed_timer() {
    use std::time::Duration;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    wait_for_commits(&h, 1);
    assert_eq!(h.commit_fire_count(), 1);
    let after_first_commit = recorded.lock().unwrap().len();

    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        h.commit_fire_count(),
        1,
        "commit deadline re-fired without a new submit — the \
         t_dispatched < t_appended guard in run_loop's timeout branch is broken, \
         OR commit_decel_to_zero is producing spurious dispatch"
    );
    let no_extra_dispatch = recorded.lock().unwrap().len();
    assert_eq!(
        no_extra_dispatch, after_first_commit,
        "extra segments were dispatched without a new submit_move"
    );

    h.shutdown();
}

#[test]
fn flush_commits_terminal_decel_synchronously() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move");
    h.flush().expect("flush");

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

    const SEAM_BUDGET_MM: f64 = 5.0e-2;
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

    let terminal_v = x_vel_at(terminal_seg, terminal_seg.t_end).abs();
    assert!(
        terminal_v < 5.0,
        "terminal dispatched velocity {} mm/s should be near zero — \
         the flush-commit must have dispatched through to v = 0",
        terminal_v,
    );

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

#[test]
fn kalico_stream_open_resets_planner_state() {
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

    let new_home = [10.0, 20.0, 30.0, 0.0];
    h.kalico_stream_open(new_home).expect("kalico_stream_open");

    h.submit_move(
        classify_and_build(
            [new_home[0], new_home[1], new_home[2]],
            1.0,
            0.0,
            0.0,
            0.0,
            100.0,
        )
        .unwrap(),
    )
    .expect("submit move 2");
    h.flush().expect("flush move 2");

    let segs = recorded.lock().unwrap().clone();
    assert!(
        segs.len() > segs_before_reset,
        "second submit/flush produced no new dispatched segments after reset",
    );

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


#[test]
fn underrun_recovery_resets_to_recovered_position() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");
    h.flush().expect("flush move 1");

    let segs_before_recover = recorded.lock().unwrap().len();
    assert!(
        segs_before_recover > 0,
        "precondition: first move must produce dispatched segments before the recovery",
    );

    let recovered_pos = [5.0, 0.0, 0.0, 0.0];
    h.underrun(recovered_pos).expect("underrun");

    h.submit_move(
        classify_and_build(
            [recovered_pos[0], recovered_pos[1], recovered_pos[2]],
            1.0,
            0.0,
            0.0,
            0.0,
            100.0,
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
            1.0,
            0.0,
            0.0,
            0.0,
            100.0,
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
        x_start,
        recovered_pos[0],
    );

    h.shutdown();
}

#[test]
fn update_shaper_commits_held_output_before_swap() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit move 1");

    let commit_count_before = h.commit_fire_count();

    let new_shaper = ShaperConfig {
        x: AxisShaper::SmoothZv { frequency_hz: 60.0 },
        y: AxisShaper::SmoothZv { frequency_hz: 60.0 },
        z: AxisShaper::Passthrough,
    };
    h.update_shaper(new_shaper).expect("update_shaper");
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

#[test]
fn clock_sync_rearm_commits_old_bias_first() {
    use motion_bridge_native::planner::ClockBias;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

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

#[test]
fn submit_move_advances_last_move_time_synchronously() {
    let (dispatch, _recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    let t0 = h.last_move_time();
    assert!(
        t0.abs() < 1e-12,
        "fresh planner should start with last_move_time = 0.0, got {t0}",
    );

    let m = classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    let expected_nominal = m.nominal_duration();
    assert!(
        (expected_nominal - 0.01).abs() < 1e-12,
        "test setup: nominal_duration should be 0.01 s for 1 mm @ 100 mm/s, got {expected_nominal}",
    );

    h.submit_move(m).expect("submit move");

    let t1 = h.last_move_time();
    let advance = t1 - t0;
    assert!(
        (advance - expected_nominal).abs() < 1e-9,
        "last_move_time did not advance synchronously by nominal_duration: \
         t0 = {t0}, t1 = {t1}, advance = {advance}, expected ≈ {expected_nominal}. \
         If this fails to ~0 the caller-side advance regressed; if this is the \
         shaped duration (~0.05+ s on a 1 mm move) the atomic is being clobbered \
         by the planner thread's rectification before submit_move returns.",
    );

    h.shutdown();
}

#[test]
fn rectification_corrects_actual_duration() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    let m = classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    let nominal = m.nominal_duration();

    h.submit_move(m).expect("submit move");
    h.flush().expect("flush as sync barrier");

    let post_flush = h.last_move_time();

    assert!(
        post_flush >= 2.0 * nominal,
        "rectification did not correct nominal → actual: \
         atomic post-flush = {post_flush} s, nominal = {nominal} s. \
         Expected post_flush >= 2 × nominal (the move has accel-from-zero \
         and decel-to-zero ramps that physically can't fit in the cruise \
         estimate). If post_flush ≈ nominal the rectification CAS did \
         not fire.",
    );

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "rectification test: no dispatched segments — \
         flush did not commit the trailing decel-to-zero",
    );
    let dispatched_terminus = segs.last().unwrap().t_end;
    assert!(
        post_flush + 1e-6 >= dispatched_terminus,
        "atomic post-flush ({post_flush} s) below dispatched terminus \
         ({dispatched_terminus} s) — the rectified t_appended is the \
         upper bound for shaped-time output; if the atomic is smaller \
         the rectification under-corrected",
    );

    h.shutdown();
}

#[test]
fn inline_event_scheduling_uses_queued_time() {
    let (dispatch, _recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    let m1 = classify_and_build([0.0; 3], 1.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    let nominal1 = m1.nominal_duration();
    h.submit_move(m1).expect("submit move 1");
    let after_m1 = h.last_move_time();

    let m2 = classify_and_build([1.0, 0.0, 0.0], 2.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    let nominal2 = m2.nominal_duration();
    h.submit_move(m2).expect("submit move 2");
    let after_m2 = h.last_move_time();

    let m3 = classify_and_build([3.0, 0.0, 0.0], 3.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    let nominal3 = m3.nominal_duration();
    h.submit_move(m3).expect("submit move 3");
    let after_m3 = h.last_move_time();

    assert!(
        after_m1 < after_m2,
        "after_m1 ({after_m1}) >= after_m2 ({after_m2}); submit_move 2 did not advance the atomic",
    );
    assert!(
        after_m2 < after_m3,
        "after_m2 ({after_m2}) >= after_m3 ({after_m3}); submit_move 3 did not advance the atomic",
    );

    const TIME_EPS_S: f64 = 1e-6;
    assert!(
        after_m1 + TIME_EPS_S >= nominal1,
        "after_m1 ({after_m1}) below nominal1 ({nominal1})",
    );
    assert!(
        after_m2 + TIME_EPS_S >= nominal1 + nominal2,
        "after_m2 ({after_m2}) below cumulative nominal {} (n1 + n2)",
        nominal1 + nominal2,
    );
    assert!(
        after_m3 + TIME_EPS_S >= nominal1 + nominal2 + nominal3,
        "after_m3 ({after_m3}) below cumulative nominal {} (n1 + n2 + n3)",
        nominal1 + nominal2 + nominal3,
    );

    h.shutdown();
}

#[test]
fn wait_moves_blocks_until_dispatch_catches_up() {
    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(relaxed_limits()).expect("update_limits");

    let starts = [[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]];
    let mut cumulative_nominal = 0.0;
    for start in starts.iter() {
        let m = classify_and_build(*start, 1.0, 0.0, 0.0, 0.0, 100.0).unwrap();
        cumulative_nominal += m.nominal_duration();
        h.submit_move(m).expect("submit move");
    }

    let lmt_after_submits = h.last_move_time();
    assert!(
        lmt_after_submits + 1e-6 >= cumulative_nominal,
        "pre-flush queued time {lmt_after_submits} s below cumulative \
         nominal {cumulative_nominal} s — caller-side advance regressed \
         (Phase 6 Task 7.1).",
    );

    h.flush().expect("flush as wait_moves barrier");

    let lmt_post_flush = h.last_move_time();
    assert!(
        lmt_post_flush + 1e-9 >= lmt_after_submits,
        "atomic regressed across flush: pre-flush {lmt_after_submits} > \
         post-flush {lmt_post_flush}. The atomic must be monotonic.",
    );

    let segs = recorded.lock().unwrap().clone();
    assert!(
        !segs.is_empty(),
        "wait_moves returned without dispatching anything — the \
         synchronous Flush commit dropped all segments",
    );

    let dispatched_start = segs.first().unwrap().t_start;
    let dispatched_terminus = segs.last().unwrap().t_end;
    assert!(
        dispatched_start.abs() < 1e-9,
        "first dispatched segment t_start = {dispatched_start} — \
         expected ~0 (the planner's time-zero).",
    );

    assert!(
        dispatched_terminus + 1e-6 >= lmt_after_submits,
        "dispatched_terminus ({dispatched_terminus} s) below pre-flush \
         queued time ({lmt_after_submits} s); difference = {} s. \
         `wait_moves` returned before dispatch caught up to the queued \
         timeline. This is the M400 contract violation Phase 6 Task 7.3 \
         pins.",
        lmt_after_submits - dispatched_terminus,
    );

    for i in 0..segs.len().saturating_sub(1) {
        let a = &segs[i];
        let b = &segs[i + 1];
        assert!(
            (a.t_end - b.t_start).abs() < 1e-9,
            "seam {i}: a.t_end = {} != b.t_start = {} — dispatched \
             window has a time hole",
            a.t_end,
            b.t_start,
        );
    }

    let segs_count_before_second = segs.len();
    let lmt_before_second = h.last_move_time();
    h.flush().expect("second flush");
    let segs_after_second = recorded.lock().unwrap().clone();
    assert_eq!(
        segs_after_second.len(),
        segs_count_before_second,
        "second flush re-dispatched segments on an already-committed queue",
    );
    let lmt_after_second = h.last_move_time();
    assert!(
        (lmt_after_second - lmt_before_second).abs() < 1e-9,
        "second flush advanced the atomic from {lmt_before_second} to \
         {lmt_after_second}; expected no-op on already-committed queue",
    );

    h.shutdown();
}
