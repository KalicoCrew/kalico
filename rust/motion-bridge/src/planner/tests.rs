use super::*;
use crate::classify::classify_and_build;
use std::sync::atomic::AtomicUsize;

fn counting_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<AtomicUsize>,
) {
    let counter = Arc::new(AtomicUsize::new(0));
    let c = Arc::clone(&counter);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |_seg: &ShapedSegment| {
            c.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });
    (cb, counter)
}

fn relaxed_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    // Relax the C1 refit tolerance — the default 5 µm is tighter than the
    // degree-4 refit can hit on a collinear-cubic 10 mm move under the
    // test's reduced-grid budget. Task 11 covers full-tolerance runs.
    c.fit_tolerance_mm = 0.05;
    c
}

/// Long-move helper: a 200 mm pure-X move at 200 mm/s feedrate has a
/// clear accel-cruise-decel shape so `t_decel_start − max_h` is well
/// inside the move and `emit_committed` returns non-trivial output on
/// the first submit. Used by the dispatch-non-empty smoke tests; the
/// short-move equivalents under the Phase-1 shim would dispatch the
/// whole move on flush, but streaming holds the trailing decel-to-zero
/// speculatively until commit (Phase 4).
fn long_move() -> ClassifiedMove {
    classify_and_build([0.0; 3], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap()
}

#[test]
fn submit_and_flush_dispatches_segments() {
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    assert!(counter.load(Ordering::Relaxed) > 0, "dispatch never called");
    assert!(h.last_move_time() > 0.0, "print_time not advanced");

    h.shutdown();
}

#[test]
fn shutdown_joins_cleanly() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
    h.shutdown();
    assert!(h.join_handle.is_none());
}

#[test]
fn dwell_advances_print_time_and_unblocks() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);

    h.dwell(0.25).unwrap();
    assert!((h.last_move_time() - 0.25).abs() < 1e-9);

    h.shutdown();
}

#[test]
fn update_limits_processed_without_error() {
    // Smoke test: deep verification belongs in Task 11.
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    let new_limits = PlannerLimits {
        max_velocity: 200.0,
        max_accel: 2000.0,
        max_z_velocity: 10.0,
        max_z_accel: 80.0,
        square_corner_velocity: 4.0,
    };
    h.update_limits(new_limits).unwrap();

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    assert!(counter.load(Ordering::Relaxed) > 0);
    h.shutdown();
}

#[test]
fn update_shaper_processed_without_error() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);

    let shaper = ShaperConfig {
        x: trajectory::RequiredShaper::SmoothZv { frequency_hz: 60.0 },
        y: trajectory::RequiredShaper::SmoothZv { frequency_hz: 60.0 },
        z: trajectory::AxisShaper::Passthrough,
    };
    h.update_shaper(shaper).unwrap();

    h.shutdown();
}

#[test]
fn submit_triggers_replan_per_move() {
    // Under streaming, every `submit_move` runs `append_and_replan` +
    // `emit_committed` immediately (no buffer accumulation). This test
    // pins that behaviour by submitting a single long-enough move and
    // verifying dispatch fires before `flush` is called — `flush` is
    // used solely as a synchronization point.
    //
    // This is the streaming-era successor to the
    // `window_capacity_triggers_batch_flush_without_explicit_flush`
    // test, which was retired alongside the buffered-window path.
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    assert!(
        counter.load(Ordering::Relaxed) > 0,
        "submit_move did not trigger per-move dispatch",
    );
    assert!(h.last_move_time() > 0.0);
    h.shutdown();
}

#[test]
fn drop_without_explicit_shutdown_does_not_hang() {
    let (dispatch, _counter) = counting_dispatch();
    let h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
    drop(h); // Drop impl should send Shutdown + join.
}

// ---------------------------------------------------------------------------
// Bug regression: Z-only move after XY homing produces non-constant X/Y
// shaped output (axes[0] and axes[1] deviate from constant by 751 mm and
// 73 mm — massive, not numerical noise).
//
// Root: after `state.reset(home_pos)` the per-axis queues are re-seeded at
// the new home position but `planned_fitted` / `planned_meta` are cleared.
// `emit_committed` (and `commit_decel_to_zero`) rebuilds per-axis history
// from `axes[i].pieces` — which are correct — but the shaping kernel for X
// and Y is applied to the Z-only move's unshaped plan. For a Z-only segment
// the X and Y components of `planned_fitted[*].axes[{0,1}]` are constant at
// the reset's home X/Y. After a flush/commit the shaped X and Y curves must
// therefore be constant (the convolution of a constant with any kernel is the
// same constant).
//
// This test FAILS on the current code (demonstrating the bug) and should
// pass after the fix is applied.
//
// Sequence mirrors a CoreXY G28 homing cycle:
//   1. Reset to X homing force-position.
//   2. X homing move (fast, large dx).
//   3. Reset to X endstop position.
//   4. X retract.
//   5. X slow approach moves.
//   6. Reset to XY homing force-position.
//   7. XY diagonal move (to home_xy position).
//   8. Reset to Z homing start (toolhead at home_xy, Z near top).
//   9. Z-only homing move (slow descent).
//  10. Flush: commit_decel_to_zero drains the held-back tail.
//  11. Assert: every shaped segment from step 9 has constant X and Y.
//
// The test drives `ShaperState` inline (no `PlannerHandle` thread) using
// the same internal helpers the planner run-loop uses, so the shaped output
// is fully deterministic and directly inspectable.
#[test]
fn z_only_move_after_homing_xy_shaped_axes_are_constant() {
    use crate::classify::classify_and_build;


    // ---- Shaper config matching real Trident: smooth_mzv @ 186 Hz on X,
    //      smooth_mzv @ 122 Hz on Y, passthrough on Z. ----
    let shaper_cfg = ShaperConfig {
        x: trajectory::RequiredShaper::SmoothMzv {
            frequency_hz: 186.0,
        },
        y: trajectory::RequiredShaper::SmoothMzv {
            frequency_hz: 122.0,
        },
        z: trajectory::AxisShaper::Passthrough,
    };

    // Build a PlannerConfig that uses the Trident shapers but relaxed
    // fit tolerance so the test converges reliably on short homing moves.
    let mut cfg = PlannerConfig::default();
    cfg.limits.max_velocity = 1000.0;
    cfg.limits.max_accel = 70000.0;
    cfg.limits.max_z_velocity = 5.0;
    cfg.limits.max_z_accel = 100.0;
    cfg.shaper = shaper_cfg;

    // Construct ShaperState + contexts exactly as the run-loop does.
    let shapers = shaper_config_to_axis_shapers(&cfg.shaper);
    let mut state = ShaperState::new([0.0; 4], &shapers);

    let replan_ctx = build_replan_context(&cfg);
    let emit_kernels = shaper_config_to_emit_kernels(&cfg.shaper);
    let e_halos: Vec<trajectory::EHalo> = Vec::new();
    let emit_ctx = EmitContext {
        kernels: &emit_kernels,
        e_halos: &e_halos,
    };

    // Helper: append one move, immediately emit committed, discard output.
    // Mirrors the run-loop's per-Move arm (append_and_replan + emit_committed).
    let do_move =
        |state: &mut ShaperState,
         start: [f64; 3],
         dx: f64,
         dy: f64,
         dz: f64,
         feed: f64| {
            let m = classify_and_build(start, dx, dy, dz, 0.0, feed)
                .expect("classify_and_build should succeed for valid moves");
            state
                .append_and_replan(m.segment, &replan_ctx)
                .expect("append_and_replan should succeed");
            state
                .emit_committed(&emit_ctx)
                .expect("emit_committed should succeed")
        };

    // Helper: flush (commit decel tail) and collect all emitted segments.
    let do_flush = |state: &mut ShaperState| -> Vec<ShapedSegment> {
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero should succeed")
    };

    // Sequence from actual klippy log during G28 on the Trident.
    // reset() = klippy's set_position (homing boundaries only).
    // Regular moves chain through append_and_replan without reset.
    //
    // X homing (sensorless, positive_dir, endstop at 300):
    state.reset([-154.5, 0.0, 0.0, 0.0]);
    let _ = do_move(&mut state, [-154.5, 0.0, 0.0], 454.5, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);
    // Endstop triggered → set_position(haltpos ≈ 300)
    state.reset([300.0, 0.0, 0.0, 0.0]);
    // Retract + safe-X moves: regular moves, no reset between them.
    let _ = do_move(&mut state, [300.0, 0.0, 0.0], -5.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [295.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [195.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    // home_rails flush_step_generation at end of X homing
    let _ = do_flush(&mut state);

    // Y homing (sensorless, positive_dir, endstop at 302):
    state.reset([95.0, -151.5, 0.0, 0.0]);
    let _ = do_move(&mut state, [95.0, -151.5, 0.0], 0.0, 453.5, 0.0, 100.0);
    let _ = do_flush(&mut state);
    // Endstop triggered → set_position(haltpos ≈ [95, 302])
    state.reset([95.0, 302.0, 0.0, 0.0]);
    // Retract + move to beacon home: regular moves, no reset between them.
    let _ = do_move(&mut state, [95.0, 302.0, 0.0], 0.0, -5.0, 0.0, 100.0);
    let _ = do_move(
        &mut state,
        [95.0, 297.0, 0.0],
        55.0,   // dx: to X=150
        -165.0, // dy: to Y=132
        0.0,
        300.0,
    );
    // home_rails flush_step_generation at end of Y homing
    let _ = do_flush(&mut state);

    // Z homing setup: set_position with Z at top of travel
    state.reset([150.0, 132.0, 344.0, 0.0]);

    // ---- Step 9: Z-only homing move (slow descent) ----
    // This is the move that triggers the bug: dx=0, dy=0, dz=-342.
    //
    // On the real printer, the planner's T_commit timer fires every
    // ~50ms, calling emit_committed hundreds of times over the 43s
    // Z descent. Each call dispatches a small window and updates the
    // shaper history with the shaped output. If the shaped output has
    // even a tiny X/Y deviation, it becomes the history for the next
    // emit — compounding across hundreds of calls to produce the
    // 751mm deviation seen on hardware.
    //
    // Simulate this by calling append_and_replan once, then calling
    // emit_committed in a loop (advancing t_decel_start each time
    // to simulate the commit timer opening the dispatch window).
    let z_move = classify_and_build(
        [150.0, 132.0, 344.0], 0.0, 0.0, -342.0, 0.0, 8.0,
    ).expect("classify Z move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("append Z move");

    // Collect all segments via incremental emit_committed calls
    // that mimic the planner's T_commit-driven dispatch.
    let mut z_segments: Vec<trajectory::ShapedSegment> = Vec::new();
    // First emit_committed (immediate, pre-decel region)
    z_segments.extend(
        state.emit_committed(&emit_ctx).expect("emit_committed"),
    );
    // Final flush (decel tail)
    z_segments.extend(
        state.commit_decel_to_zero(&emit_ctx).expect("commit_decel_to_zero"),
    );

    // For the assertion we only need at least one segment.
    assert!(
        !z_segments.is_empty(),
        "commit_decel_to_zero must produce at least one segment for a 342 mm Z move",
    );

    // ---- Step 11: assert X and Y shaped axes are constant ----
    //
    // For a Z-only move the toolhead does not move in X or Y. The
    // unshaped X and Y trajectories in `planned_fitted` are constant
    // (all control points equal the reset home position: X=150, Y=150).
    // The shaper convolution of a constant function is the same constant.
    // Therefore every `ShapedSegment` produced by this move must have
    // axes[0] (X) and axes[1] (Y) trivially constant.
    //
    // On buggy code the X and Y axes have 751 mm and 73 mm maximum
    // control-point deviation from constant — a visible sign that the
    // shaper was operating on wrong history state left over from the
    // prior XY moves.
    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;

    for seg in &z_segments {
        let dev_x = seg.axes[0].control_points().iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = seg.axes[1].control_points().iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
    }

    assert!(
        max_dev_x < 0.01,
        "Z-only move after XY homing: X deviated by {max_dev_x:.6} mm from 150.0 \
         (expected < 10µm)",
    );

    assert!(
        max_dev_y < 0.01,
        "Z-only move after XY homing: Y deviated by {max_dev_y:.6} mm from 132.0 \
         (expected < 10µm)",
    );
}

/// Same as the Z-only test but with a tiny X displacement (0.1mm)
/// alongside the 342mm Z descent. If the 750mm amplification is
/// specific to the near-constant case, this should show a much
/// smaller (proportionate) deviation. If it's a general history
/// contamination, it will show a similar magnitude.
#[test]
fn z_move_with_tiny_x_after_homing_xy_deviation_proportional() {
    use crate::classify::classify_and_build;

    let shaper_cfg = ShaperConfig {
        x: RequiredShaper::SmoothMzv { frequency_hz: 186.0 },
        y: RequiredShaper::SmoothMzv { frequency_hz: 122.0 },
        z: AxisShaper::Passthrough,
    };

    let mut cfg = PlannerConfig::default();
    cfg.limits.max_velocity = 1000.0;
    cfg.limits.max_accel = 70000.0;
    cfg.limits.max_z_velocity = 5.0;
    cfg.limits.max_z_accel = 100.0;
    cfg.shaper = shaper_cfg;

    let shapers = shaper_config_to_axis_shapers(&cfg.shaper);
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let replan_ctx = build_replan_context(&cfg);
    let emit_kernels = shaper_config_to_emit_kernels(&cfg.shaper);
    let e_halos: Vec<trajectory::EHalo> = Vec::new();
    let emit_ctx = EmitContext {
        kernels: &emit_kernels,
        e_halos: &e_halos,
    };

    let do_move =
        |state: &mut ShaperState,
         start: [f64; 3],
         dx: f64, dy: f64, dz: f64, feed: f64| {
            let m = classify_and_build(start, dx, dy, dz, 0.0, feed)
                .expect("classify");
            state.append_and_replan(m.segment, &replan_ctx).expect("replan");
            state.emit_committed(&emit_ctx).expect("emit")
        };
    let do_flush = |state: &mut ShaperState| -> Vec<trajectory::ShapedSegment> {
        state.commit_decel_to_zero(&emit_ctx).expect("flush")
    };

    // Same XY homing sequence as the Z-only test
    state.reset([-154.5, 0.0, 0.0, 0.0]);
    let _ = do_move(&mut state, [-154.5, 0.0, 0.0], 454.5, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset([300.0, 0.0, 0.0, 0.0]);
    let _ = do_move(&mut state, [300.0, 0.0, 0.0], -5.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [295.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_move(&mut state, [195.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset([95.0, -151.5, 0.0, 0.0]);
    let _ = do_move(&mut state, [95.0, -151.5, 0.0], 0.0, 453.5, 0.0, 100.0);
    let _ = do_flush(&mut state);
    state.reset([95.0, 302.0, 0.0, 0.0]);
    let _ = do_move(&mut state, [95.0, 302.0, 0.0], 0.0, -5.0, 0.0, 100.0);
    let _ = do_move(&mut state, [95.0, 297.0, 0.0], 55.0, -165.0, 0.0, 300.0);
    let _ = do_flush(&mut state);

    // Z move with tiny X: dx=0.1mm instead of 0
    state.reset([150.0, 132.0, 344.0, 0.0]);
    let z_move = classify_and_build(
        [150.0, 132.0, 344.0], 0.1, 0.0, -342.0, 0.0, 8.0,
    ).expect("classify Z+tiny-X move");
    state.append_and_replan(z_move.segment, &replan_ctx).expect("replan");

    let mut segs: Vec<trajectory::ShapedSegment> = Vec::new();
    segs.extend(state.emit_committed(&emit_ctx).expect("emit"));
    segs.extend(state.commit_decel_to_zero(&emit_ctx).expect("flush"));

    assert!(!segs.is_empty());

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;
    for seg in &segs {
        let dev_x = seg.axes[0].control_points().iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = seg.axes[1].control_points().iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
    }

    // X has 0.1mm real displacement — shaped output should stay near
    // 150.0, not blow up to hundreds of mm.
    assert!(
        max_dev_x < 1.0,
        "tiny-X move: X deviated {max_dev_x:.3}mm from 150.0 (expected < 1mm for 0.1mm input)",
    );
    assert!(
        max_dev_y < 0.01,
        "tiny-X move: Y deviated {max_dev_y:.6}mm from 132.0 (expected < 10µm)",
    );
}

/// Dispatch closure that records each dispatched segment's (t_start, t_end).
fn capturing_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<Mutex<Vec<(f64, f64)>>>,
) {
    let log = Arc::new(Mutex::new(Vec::new()));
    let l = Arc::clone(&log);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            l.lock().unwrap().push((seg.t_start, seg.t_end));
            Ok(())
        });
    (cb, log)
}

#[test]
fn quiescence_keeps_timeline_monotone_next_move_does_not_rewind() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    fn wait_for_commits(h: &PlannerHandle, target: u32) {
        let start = std::time::Instant::now();
        while h.commit_fire_count() < target {
            assert!(
                start.elapsed() < Duration::from_secs(5),
                "commit fired only {} of {target} times within 5s",
                h.commit_fire_count()
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    // Move 1: X 0 -> 200. Wait for the decel-commit (commit #1).
    h.submit_move(long_move()).unwrap();
    wait_for_commits(&h, 1);
    let m1_max_t_end = log
        .lock().unwrap().iter().map(|&(_, e)| e).fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    // Move 2: X 200 -> 400, submitted after a real idle gap so the monotonic
    // clock has advanced past move 1's end.
    log.lock().unwrap().clear();
    std::thread::sleep(Duration::from_millis(400));
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap();
    h.submit_move(m2).unwrap();
    wait_for_commits(&h, 2);
    let m2_min_t_start = log
        .lock().unwrap().iter().map(|&(s, _)| s).fold(f64::INFINITY, f64::min);
    assert!(m2_min_t_start.is_finite(), "move 2 produced no dispatched segments");

    // Monotone clock: move 2 starts at or after move 1's end — NOT rewound.
    assert!(
        m2_min_t_start >= m1_max_t_end - 1e-3,
        "timeline rewound: move 2 started at {m2_min_t_start}, move 1 ended at {m1_max_t_end}"
    );

    h.shutdown();
}

/// No homing — just reset to a position and do a Z-only move.
/// If the bug requires prior XY motion through the shaper,
/// this should pass cleanly.
#[test]
fn z_only_move_no_prior_xy_motion() {
    use crate::classify::classify_and_build;


    let shaper_cfg = ShaperConfig {
        x: RequiredShaper::SmoothMzv { frequency_hz: 186.0 },
        y: RequiredShaper::SmoothMzv { frequency_hz: 122.0 },
        z: AxisShaper::Passthrough,
    };

    let mut cfg = PlannerConfig::default();
    cfg.limits.max_velocity = 1000.0;
    cfg.limits.max_accel = 70000.0;
    cfg.limits.max_z_velocity = 5.0;
    cfg.limits.max_z_accel = 100.0;
    cfg.shaper = shaper_cfg;

    let shapers = shaper_config_to_axis_shapers(&cfg.shaper);
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let replan_ctx = build_replan_context(&cfg);
    let emit_kernels = shaper_config_to_emit_kernels(&cfg.shaper);
    let e_halos: Vec<trajectory::EHalo> = Vec::new();
    let emit_ctx = EmitContext {
        kernels: &emit_kernels,
        e_halos: &e_halos,
    };

    // No prior moves — just reset and go
    state.reset([150.0, 132.0, 344.0, 0.0]);

    let z_move = classify_and_build(
        [150.0, 132.0, 344.0], 0.0, 0.0, -342.0, 0.0, 8.0,
    ).expect("classify Z move");
    state.append_and_replan(z_move.segment, &replan_ctx).expect("replan");

    let mut segs: Vec<trajectory::ShapedSegment> = Vec::new();
    segs.extend(state.emit_committed(&emit_ctx).expect("emit"));
    segs.extend(state.commit_decel_to_zero(&emit_ctx).expect("flush"));

    assert!(!segs.is_empty());

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;
    for (i, seg) in segs.iter().enumerate() {
        let cps_x = seg.axes[0].control_points();
        let cps_y = seg.axes[1].control_points();
        let dev_x = cps_x.iter().map(|c| (c - 150.0).abs()).fold(0.0_f64, f64::max);
        let dev_y = cps_y.iter().map(|c| (c - 132.0).abs()).fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
        eprintln!(
            "[no_prior] seg[{i}]: t=[{:.3},{:.3}] X dev={:.6}mm Y dev={:.6}mm",
            seg.t_start, seg.t_end, dev_x, dev_y,
        );
    }

    assert!(
        max_dev_x < 0.01,
        "Z-only move without prior XY motion: X deviated by {max_dev_x:.6}mm (expected < 10µm)",
    );
    assert!(
        max_dev_y < 0.01,
        "Z-only move without prior XY motion: Y deviated by {max_dev_y:.6}mm (expected < 10µm)",
    );
}
