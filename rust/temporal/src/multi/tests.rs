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

fn smooth_u_turn() -> (VectorNurbs<f64, 3>, VectorNurbs<f64, 3>) {
    let r = 5.0;
    let k = r * 4.0 * (std::f64::consts::SQRT_2 - 1.0) / 3.0;
    let left = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [k, 0.0, 0.0],
            [r, r - k, 0.0],
            [r, r, 0.0],
        ],
    )
    .unwrap();
    let right = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, r, 0.0],
            [r, r + k, 0.0],
            [k, 2.0 * r, 0.0],
            [0.0, 2.0 * r, 0.0],
        ],
    )
    .unwrap();
    (left, right)
}

#[test]
#[ignore = "RED until the condensed-chain solver lands (Task 11)"]
fn smooth_junction_has_no_accel_impulse() {
    let (left, right) = smooth_u_turn();
    let limits = textbook_limits();
    let segs = [
        SegmentInput {
            curve: &left,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        SegmentInput {
            curve: &right,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
    ];
    let out = plan_batch(BatchInput {
        segments: &segs,
        grid_strategy: GridStrategy::Fixed(32),
        worker_threads: 1,
        initial_velocity: 0.0,
        terminal_velocity: 0.0,
    })
    .expect("plan_batch");

    let a_end_left = out.profiles[0].samples.last().unwrap().a;
    let a_start_right = out.profiles[1].samples[0].a;

    // Pre-fix: independent FD endpoints, V-profile makes them differ by
    // O(a_max) — expect this assert to fail with step ≈ 1e3..1e4.
    // Post-fix: structural check only — slicing duplicates the single shared
    // junction variable into both profiles, so step == 0 by construction.
    // The PHYSICAL test is the contract-(b) jerk assertion below.
    let step = (a_end_left - a_start_right).abs();
    assert!(
        step < 1.0,
        "junction accel step {step:.1} mm/s² — boundary accels are decoupled"
    );

    // Contract (b): the junction-spanning discrete jerk obeys j_max. Build
    // the spanning second difference from the two slices (junction sample
    // duplicated, so left[n-2], junction, right[1]).
    let l = &out.profiles[0].samples;
    let r = &out.profiles[1].samples;
    let (bl, bj, br) = (l[l.len() - 2].b, l[l.len() - 1].b, r[1].b);
    let hl = l[l.len() - 1].s - l[l.len() - 2].s;
    let hr = r[1].s - r[0].s;
    let d = hl * hr * (hl + hr);
    let b_dd = (2.0 * hr * bl - 2.0 * (hl + hr) * bj + 2.0 * hl * br) / d;
    let jerk = bj.max(0.0).sqrt() * b_dd / 2.0;
    let j_path = limits.j_max[0].min(limits.j_max[1]).min(limits.j_max[2]);
    assert!(
        jerk.abs() <= j_path * 1.10,
        "junction-spanning jerk {jerk:.0} exceeds j_path {j_path:.0}"
    );
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
