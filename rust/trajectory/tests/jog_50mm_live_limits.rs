use geometry::segment::EMode;
use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{AxisShaper, ELimits, ShapeBatchInput, ShapeSegmentInput, ShaperConfig};

fn x_50mm_collinear_cubic() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [50.0 / 3.0, 0.0, 0.0],
            [2.0 * 50.0 / 3.0, 0.0, 0.0],
            [50.0, 0.0, 0.0],
        ],
    )
    .unwrap()
}

fn live_limits() -> temporal::Limits {
    temporal::Limits::new(
        [1000.0, 1000.0, 5.0],
        [70000.0, 70000.0, 100.0],
        [140000.0, 140000.0, 200.0],
        5.0_f64.powi(2) / (70000.0 * 0.5),
    )
}

#[test]
fn jog_50mm_at_100mms_with_live_limits() {
    let curve = x_50mm_collinear_cubic();

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: live_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: AxisShaper::SmoothMzv {
                frequency_hz: 186.0,
            },
            y: AxisShaper::SmoothMzv {
                frequency_hz: 122.0,
            },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let result = trajectory::shape_batch(&input).expect("shape_batch failed");
    assert_eq!(result.segments.len(), 1);
    let seg = &result.segments[0];
    let duration = seg.t_end - seg.t_start;
    eprintln!(
        "[probe] 50mm @ 100mm/s collinear-X duration={:.6}s avg_v={:.3} mm/s beta_warning={:?}",
        duration,
        50.0 / duration,
        result.beta_warning,
    );
    eprintln!("[probe] temporal_status={:?}", result.temporal_status);
    // No assertion — pure probe. CARGO_LOG=1 cargo test prints duration.
    assert!(
        duration < 5.0,
        "trajectory exploded to {duration}s — probe needs adjustment"
    );
}

#[test]
fn jog_50mm_with_higher_scv() {
    let curve = x_50mm_collinear_cubic();
    let limits = temporal::Limits::new(
        [1000.0, 1000.0, 5.0],
        [70000.0, 70000.0, 100.0],
        [140000.0, 140000.0, 200.0],
        70.0_f64.powi(2) / (70000.0 * 0.5),
    );

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: AxisShaper::SmoothMzv {
                frequency_hz: 186.0,
            },
            y: AxisShaper::SmoothMzv {
                frequency_hz: 122.0,
            },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let result = trajectory::shape_batch(&input).expect("shape_batch failed");
    let seg = &result.segments[0];
    let duration = seg.t_end - seg.t_start;
    eprintln!(
        "[probe scv=70] duration={:.6}s avg_v={:.3} mm/s",
        duration,
        50.0 / duration,
    );
}

fn probe_with_feedrate(feedrate: f64, dist_mm: f64) -> f64 {
    let curve = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [dist_mm / 3.0, 0.0, 0.0],
            [2.0 * dist_mm / 3.0, 0.0, 0.0],
            [dist_mm, 0.0, 0.0],
        ],
    )
    .unwrap();
    let limits = live_limits();
    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: feedrate,
    }];
    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: AxisShaper::SmoothMzv {
                frequency_hz: 186.0,
            },
            y: AxisShaper::SmoothMzv {
                frequency_hz: 122.0,
            },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v: 0.0,
        terminal_v: 0.0,
    };
    let result = trajectory::shape_batch(&input).expect("shape_batch failed");
    let seg = &result.segments[0];
    seg.t_end - seg.t_start
}

#[test]
fn sweep_feedrate() {
    for fr in [10.0, 25.0, 50.0, 100.0, 200.0, 500.0, 1000.0] {
        let d = probe_with_feedrate(fr, 50.0);
        eprintln!(
            "[sweep feedrate={:>7.1}] duration={:.6}s avg_v={:.3} mm/s",
            fr,
            d,
            50.0 / d,
        );
    }
}

#[test]
fn jog_50mm_with_z_jmax_uncapped() {
    let curve = x_50mm_collinear_cubic();
    let limits = temporal::Limits::new(
        [1000.0, 1000.0, 5.0],
        [70000.0, 70000.0, 1000.0],
        [140000.0, 140000.0, 2000.0],
        5.0_f64.powi(2) / (70000.0 * 0.5),
    );

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: AxisShaper::SmoothMzv {
                frequency_hz: 186.0,
            },
            y: AxisShaper::SmoothMzv {
                frequency_hz: 122.0,
            },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let result = trajectory::shape_batch(&input).expect("shape_batch failed");
    let seg = &result.segments[0];
    let duration = seg.t_end - seg.t_start;
    eprintln!(
        "[probe Z_uncapped] duration={:.6}s avg_v={:.3} mm/s",
        duration,
        50.0 / duration,
    );
}

#[test]
fn sweep_distance() {
    for dist in [1.0, 5.0, 10.0, 25.0, 50.0, 100.0] {
        let d = probe_with_feedrate(100.0, dist);
        eprintln!(
            "[sweep dist={:>6.1}] duration={:.6}s avg_v={:.3} mm/s",
            dist,
            d,
            dist / d,
        );
    }
}

#[test]
fn jog_50mm_low_accel_baseline() {
    let curve = x_50mm_collinear_cubic();
    let limits = temporal::Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 200.0],
        5.0_f64.powi(2) / (3000.0 * 0.5),
    );

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: AxisShaper::SmoothMzv { frequency_hz: 50.0 },
            y: AxisShaper::SmoothMzv { frequency_hz: 50.0 },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let result = trajectory::shape_batch(&input).expect("shape_batch failed");
    let seg = &result.segments[0];
    let duration = seg.t_end - seg.t_start;
    eprintln!(
        "[probe sim-baseline] a_max=3000 duration={:.6}s avg_v={:.3} mm/s",
        duration,
        50.0 / duration,
    );
}
