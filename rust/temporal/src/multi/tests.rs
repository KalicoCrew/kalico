use super::*;
use crate::Limits;
use nurbs::VectorNurbs;

fn straight_50mm() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .unwrap()
}

fn textbook_limits() -> Limits {
    Limits {
        v_max: [500.0; 3],
        a_max: [5_000.0; 3],
        j_max: [100_000.0; 3],
        a_centripetal_max: 2_500.0,
    }
}

#[test]
fn plan_batch_single_segment_works() {
    let curve = straight_50mm();
    let segment = SegmentInput {
        curve: &curve,
        limits: textbook_limits(),
        trailing_junction_chord_tolerance_mm: 0.05,
    };
    let input = BatchInput {
        segments: &[segment],
        grid_strategy: GridStrategy::Adaptive {
            min_n: 10,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        initial_velocity: 0.0,
        terminal_velocity: 0.0,
    };
    let output = plan_batch(input).expect("should succeed");
    assert_eq!(output.profiles.len(), 1);

    // Single segment endpoints both 0.
    assert!(output.profiles[0].samples[0].v < 1e-3);
    assert!(output.profiles[0].samples.last().unwrap().v < 1e-3);
}

/// Step-0 plumbing contract: a non-zero `initial_velocity` reaches
/// TOPP-RA's boundary condition and the first sample of the first
/// (and only) segment's profile matches the requested starting speed
/// to within the joining `ε_velocity = 1 mm/s` tolerance.
#[test]
fn plan_batch_threads_nonzero_initial_velocity() {
    // 200 mm move: enough path length to feasibly start at 50 mm/s and
    // decelerate to 0.0 under the textbook 5 km/s² limit.
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
    )
    .unwrap();
    let segment = SegmentInput {
        curve: &curve,
        limits: textbook_limits(),
        trailing_junction_chord_tolerance_mm: 0.05,
    };
    let input = BatchInput {
        segments: &[segment],
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        initial_velocity: 50.0,
        terminal_velocity: 0.0,
    };
    let output = plan_batch(input).expect("nonzero initial_velocity should plan");
    assert_eq!(output.profiles.len(), 1);

    let v0 = output.profiles[0].samples[0].v;
    assert!(
        (v0 - 50.0).abs() < 1.0,
        "first-sample velocity {v0} must equal requested initial_velocity 50.0 mm/s \
         within the 1 mm/s joining tolerance",
    );
    // Terminal should be at rest.
    let v_last = output.profiles[0].samples.last().unwrap().v;
    assert!(
        v_last < 1.0,
        "terminal velocity {v_last} must be ≈ 0 mm/s under terminal_velocity = 0.0",
    );
    assert!(output.junctions.is_empty());
}
