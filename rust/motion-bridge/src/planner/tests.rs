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
    Arc<std::sync::Mutex<Vec<(f64, f64)>>>,
) {
    let log = Arc::new(std::sync::Mutex::new(Vec::new()));
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

fn peak_speed_of_single_x_move(max_velocity: f64, max_accel: f64, feedrate: f64) -> f64 {
    let mut cfg = PlannerConfig::default();
    cfg.limits.max_velocity = max_velocity;
    cfg.limits.max_accel = max_accel;

    let shapers = shaper_config_to_axis_shapers(&cfg.shaper);
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let replan_ctx = build_replan_context(&cfg);
    let emit_kernels = shaper_config_to_emit_kernels(&cfg.shaper);
    let e_halos: Vec<trajectory::EHalo> = Vec::new();
    let emit_ctx = EmitContext {
        kernels: &emit_kernels,
        e_halos: &e_halos,
    };

    state.reset([0.0; 4]);
    let m = classify_and_build([0.0; 3], 600.0, 0.0, 0.0, 0.0, feedrate)
        .expect("classify_and_build should succeed");
    state
        .append_and_replan(m.segment, &replan_ctx)
        .expect("append_and_replan should succeed");

    let mut segs: Vec<ShapedSegment> = Vec::new();
    segs.extend(
        state
            .emit_committed(&emit_ctx)
            .expect("emit_committed should succeed"),
    );
    segs.extend(
        state
            .commit_decel_to_zero(&emit_ctx)
            .expect("commit_decel_to_zero should succeed"),
    );
    assert!(!segs.is_empty(), "move produced no shaped segments");

    let mut peak = 0.0_f64;
    for seg in &segs {
        let vel: Vec<nurbs::ScalarNurbs<f64>> =
            seg.axes.iter().map(nurbs::eval::derivative).collect();
        const SAMPLE_DT: f64 = 2e-4;
        let steps = ((seg.t_end - seg.t_start) / SAMPLE_DT).ceil().max(1.0) as usize;
        for i in 0..=steps {
            let t = seg.t_start + (seg.t_end - seg.t_start) * (i as f64) / (steps as f64);
            let speed = vel
                .iter()
                .map(|d| nurbs::eval::eval(d, t).powi(2))
                .sum::<f64>()
                .sqrt();
            peak = peak.max(speed);
        }
    }
    peak
}

#[test]
fn motion_at_velocity_limit_cruises_at_limit() {
    let peak = peak_speed_of_single_x_move(1000.0, 50_000.0, 1000.0);
    assert!(
        (peak - 1000.0).abs() < 15.0,
        "feedrate at machine limit (1000 mm/s): peak speed {peak:.1} mm/s, expected ≈ 1000",
    );
}

#[test]
fn motion_above_velocity_limit_clamps_to_limit() {
    let peak = peak_speed_of_single_x_move(1000.0, 50_000.0, 1100.0);
    assert!(
        (peak - 1000.0).abs() < 15.0,
        "feedrate above machine limit (1100 > 1000 mm/s): \
         peak speed {peak:.1} mm/s, expected clamp to ≈ 1000",
    );
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
