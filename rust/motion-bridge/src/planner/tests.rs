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
    c.fit_tolerance_mm = 0.05;
    c
}

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
    drop(h);
}

#[test]
fn z_only_move_after_homing_xy_shaped_axes_are_constant() {
    use crate::classify::classify_and_build;

    let shaper_cfg = ShaperConfig {
        x: trajectory::RequiredShaper::SmoothMzv {
            frequency_hz: 186.0,
        },
        y: trajectory::RequiredShaper::SmoothMzv {
            frequency_hz: 122.0,
        },
        z: trajectory::AxisShaper::Passthrough,
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
        |state: &mut ShaperState, start: [f64; 3], dx: f64, dy: f64, dz: f64, feed: f64| {
            let m = classify_and_build(start, dx, dy, dz, 0.0, feed)
                .expect("classify_and_build should succeed for valid moves");
            state
                .append_and_replan(m.segment, &replan_ctx)
                .expect("append_and_replan should succeed");
            state
                .emit_committed(&emit_ctx)
                .expect("emit_committed should succeed")
        };

    let do_flush = |state: &mut ShaperState| -> Vec<ShapedSegment> {
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero should succeed")
    };

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

    state.reset([150.0, 132.0, 344.0, 0.0]);

    let z_move = classify_and_build([150.0, 132.0, 344.0], 0.0, 0.0, -342.0, 0.0, 8.0)
        .expect("classify Z move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("append Z move");

    let mut z_segments: Vec<trajectory::ShapedSegment> = Vec::new();
    z_segments.extend(state.emit_committed(&emit_ctx).expect("emit_committed"));
    z_segments.extend(
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero"),
    );

    assert!(
        !z_segments.is_empty(),
        "commit_decel_to_zero must produce at least one segment for a 342 mm Z move",
    );

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;

    for seg in &z_segments {
        let dev_x = seg.axes[0]
            .control_points()
            .iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = seg.axes[1]
            .control_points()
            .iter()
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

#[test]
fn z_move_with_tiny_x_after_homing_xy_deviation_proportional() {
    use crate::classify::classify_and_build;

    let shaper_cfg = ShaperConfig {
        x: RequiredShaper::SmoothMzv {
            frequency_hz: 186.0,
        },
        y: RequiredShaper::SmoothMzv {
            frequency_hz: 122.0,
        },
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
        |state: &mut ShaperState, start: [f64; 3], dx: f64, dy: f64, dz: f64, feed: f64| {
            let m = classify_and_build(start, dx, dy, dz, 0.0, feed).expect("classify");
            state
                .append_and_replan(m.segment, &replan_ctx)
                .expect("replan");
            state.emit_committed(&emit_ctx).expect("emit")
        };
    let do_flush = |state: &mut ShaperState| -> Vec<trajectory::ShapedSegment> {
        state.commit_decel_to_zero(&emit_ctx).expect("flush")
    };

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

    state.reset([150.0, 132.0, 344.0, 0.0]);
    let z_move = classify_and_build([150.0, 132.0, 344.0], 0.1, 0.0, -342.0, 0.0, 8.0)
        .expect("classify Z+tiny-X move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("replan");

    let mut segs: Vec<trajectory::ShapedSegment> = Vec::new();
    segs.extend(state.emit_committed(&emit_ctx).expect("emit"));
    segs.extend(state.commit_decel_to_zero(&emit_ctx).expect("flush"));

    assert!(!segs.is_empty());

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;
    for seg in &segs {
        let dev_x = seg.axes[0]
            .control_points()
            .iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = seg.axes[1]
            .control_points()
            .iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
        max_dev_x = max_dev_x.max(dev_x);
        max_dev_y = max_dev_y.max(dev_y);
    }

    assert!(
        max_dev_x < 1.0,
        "tiny-X move: X deviated {max_dev_x:.3}mm from 150.0 (expected < 1mm for 0.1mm input)",
    );
    assert!(
        max_dev_y < 0.01,
        "tiny-X move: Y deviated {max_dev_y:.6}mm from 132.0 (expected < 10µm)",
    );
}

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

    h.submit_move(long_move()).unwrap();
    wait_for_commits(&h, 1);
    let m1_max_t_end = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(_, e)| e)
        .fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    log.lock().unwrap().clear();
    std::thread::sleep(Duration::from_millis(400));
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap();
    h.submit_move(m2).unwrap();
    wait_for_commits(&h, 2);
    let m2_min_t_start = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(s, _)| s)
        .fold(f64::INFINITY, f64::min);
    assert!(
        m2_min_t_start.is_finite(),
        "move 2 produced no dispatched segments"
    );

    assert!(
        m2_min_t_start >= m1_max_t_end - 1e-3,
        "timeline rewound: move 2 started at {m2_min_t_start}, move 1 ended at {m1_max_t_end}"
    );

    h.shutdown();
}

#[test]
fn z_only_move_no_prior_xy_motion() {
    use crate::classify::classify_and_build;

    let shaper_cfg = ShaperConfig {
        x: RequiredShaper::SmoothMzv {
            frequency_hz: 186.0,
        },
        y: RequiredShaper::SmoothMzv {
            frequency_hz: 122.0,
        },
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

    state.reset([150.0, 132.0, 344.0, 0.0]);

    let z_move = classify_and_build([150.0, 132.0, 344.0], 0.0, 0.0, -342.0, 0.0, 8.0)
        .expect("classify Z move");
    state
        .append_and_replan(z_move.segment, &replan_ctx)
        .expect("replan");

    let mut segs: Vec<trajectory::ShapedSegment> = Vec::new();
    segs.extend(state.emit_committed(&emit_ctx).expect("emit"));
    segs.extend(state.commit_decel_to_zero(&emit_ctx).expect("flush"));

    assert!(!segs.is_empty());

    let mut max_dev_x: f64 = 0.0;
    let mut max_dev_y: f64 = 0.0;
    for (i, seg) in segs.iter().enumerate() {
        let cps_x = seg.axes[0].control_points();
        let cps_y = seg.axes[1].control_points();
        let dev_x = cps_x
            .iter()
            .map(|c| (c - 150.0).abs())
            .fold(0.0_f64, f64::max);
        let dev_y = cps_y
            .iter()
            .map(|c| (c - 132.0).abs())
            .fold(0.0_f64, f64::max);
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

#[test]
#[ignore]
fn flush_blocks_until_motion_complete_by_clock() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);
    let t0 = std::time::Instant::now();
    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let elapsed = t0.elapsed().as_secs_f64();
    assert!(
        elapsed >= 0.25 * 0.9,
        "flush returned too early: {:.4}s",
        elapsed
    );
    h.shutdown();
}

#[test]
fn flush_then_move_dispatches_without_error() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let m1_max_t_end = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(_, e)| e)
        .fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    log.lock().unwrap().clear();
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap();
    h.submit_move(m2).unwrap();
    h.flush().unwrap();
    let m2_log = log.lock().unwrap().clone();

    assert!(!m2_log.is_empty(), "move 2 produced no dispatched segments");

    let m2_min_t_start = m2_log.iter().map(|&(s, _)| s).fold(f64::INFINITY, f64::min);

    assert!(
        m2_min_t_start >= m1_max_t_end - 1e-3,
        "timeline rewound across flush boundary: \
         move 2 t_start={m2_min_t_start:.6} < move 1 t_end={m1_max_t_end:.6}",
    );

    h.shutdown();
}

/// Build a dispatch closure that fails with `DispatchError::SegmentLate` on the
/// Nth segment (1-based) and succeeds for all others.  The `invocations` counter
/// is incremented for every call including the failing one.
fn failing_dispatch_on_nth(
    fail_on: usize,
) -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<AtomicU32>,
) {
    let invocations = Arc::new(AtomicU32::new(0));
    let inv = Arc::clone(&invocations);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            let n = inv.fetch_add(1, Ordering::SeqCst) as usize + 1;
            if n == fail_on {
                Err(DispatchError::SegmentLate {
                    gap_s: 0.5,
                    seg_t_start: seg.t_start,
                })
            } else {
                Ok(())
            }
        });
    (cb, invocations)
}

#[test]
fn dispatch_error_poisons_stream_subsequent_moves_not_dispatched() {
    // Fail on the very first segment.  After that, any further moves must not
    // invoke the dispatch closure at all.
    let (dispatch, invocations) = failing_dispatch_on_nth(1);
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    // First move — dispatch closure fires and returns the error.
    let _ = h.submit_move(long_move());

    // Spin until the error slot is populated; the dispatch closure increments the
    // counter before returning, but the planner stores the error only after the
    // closure returns, so spinning on the slot is the race-free wait.
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "dispatch error was never stored within 5s"
        );
        if h.error.lock().unwrap_or_else(|p| p.into_inner()).is_some() {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }

    // Record how many calls happened so far (at least 1, for the failing segment).
    let after_first = invocations.load(Ordering::SeqCst);
    assert!(
        after_first >= 1,
        "dispatch was never called for the first move"
    );

    // The error slot holds exactly the first error.  Drain it now; subsequent
    // submit_move calls would drain it themselves via check_error() and mask it.
    let first_err = h.check_error();
    assert!(
        matches!(
            first_err,
            Err(PlannerError::Dispatch(DispatchError::SegmentLate { .. }))
        ),
        "expected SegmentLate error, got: {first_err:?}"
    );

    // Submit more moves.  The planner is poisoned; none of them should reach dispatch.
    for _ in 0..3 {
        let _ = h.submit_move(long_move());
    }

    // Give the planner time to process the dropped moves.
    std::thread::sleep(Duration::from_millis(200));

    let after_poison = invocations.load(Ordering::SeqCst);
    assert_eq!(
        after_poison, after_first,
        "dispatch was called after stream was poisoned: \
         expected {after_first} invocations, got {after_poison}"
    );

    h.shutdown();
}

#[test]
fn stream_open_clears_poison_and_resumes_dispatch() {
    let (dispatch, invocations) = failing_dispatch_on_nth(1);
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    // Trigger poison.
    let _ = h.submit_move(long_move());

    // Spin until the error slot is populated before draining it.
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "dispatch error was never stored within 5s"
        );
        if h.error.lock().unwrap_or_else(|p| p.into_inner()).is_some() {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }

    let after_error = invocations.load(Ordering::SeqCst);
    assert!(after_error >= 1);

    // Drain the stored error before checking that dispatch resumes.
    let _ = h.check_error();

    // Reset the stream — this must clear the poison flag.
    h.kalico_stream_open([0.0; 4]).unwrap();

    // Give the planner a moment to process the KalicoStreamOpen message.
    std::thread::sleep(Duration::from_millis(50));

    let before_resume = invocations.load(Ordering::SeqCst);

    // Submit a new move after the reset; dispatch should be called again.
    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();

    let after_resume = invocations.load(Ordering::SeqCst);
    assert!(
        after_resume > before_resume,
        "dispatch was not called after stream reset: \
         before={before_resume} after={after_resume}"
    );

    h.shutdown();
}

#[test]
fn flush_during_poisoned_state_returns_promptly() {
    let (dispatch, _invocations) = failing_dispatch_on_nth(1);
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    // Trigger poison.
    let _ = h.submit_move(long_move());

    // Spin until the error slot is populated, confirming the planner has stored it.
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "dispatch error was never stored within 5s"
        );
        // Peek without taking: lock, check Some, unlock.
        if h.error.lock().unwrap_or_else(|p| p.into_inner()).is_some() {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }

    // Drain the error so flush() itself doesn't return an error from check_error.
    let _ = h.check_error();

    let t0 = std::time::Instant::now();
    // With a poisoned stream the Flush arm replies with None immediately.
    // flush() receives None and skips the sleep, so it returns in well under LEAD (0.25 s).
    let result = h.flush();
    let elapsed = t0.elapsed();

    // The result should be Ok because we already drained the stored error.
    assert!(
        result.is_ok(),
        "flush returned error after error was drained: {result:?}"
    );
    assert!(
        elapsed < Duration::from_millis(200),
        "flush blocked too long during poisoned state: {elapsed:?}"
    );

    h.shutdown();
}
