use geometry::segment::EMode;
use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, JoiningStatus, SegmentInput};
use trajectory::{
    AxisShaper, ELimits, ShapeBatchInput, ShapeError, ShapeSegmentInput, ShaperConfig,
};

fn pure_x_300mm_collinear_cubic() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [-300.0, 0.0, 0.0],
            [-200.0, 0.0, 0.0],
            [-100.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
    )
    .unwrap()
}

fn sim_homing_limits() -> temporal::Limits {
    temporal::Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 6000.0],
        5.0_f64.powi(2) / (3000.0 * 0.5),
    )
}

#[test]
fn homing_300mm_pure_x_at_uniform_jerk_converges() {
    let curve = pure_x_300mm_collinear_cubic();

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: sim_homing_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 50.0,
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

    let result = trajectory::shape_batch(&input);

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
        Err(ShapeError::TemporalJoining(status, detail)) => {
            panic!(
                "regression: 300 mm pure-X at j_max=[6000;3] failed temporal joining: {status:?}{detail}"
            );
        }
        Err(err) => panic!("unexpected shape_batch error: {err:?}"),
    }
}
