//! Adaptive-tolerance regression tests. Spec §2.1 + Pi 5 investigation Finding 2.

use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};
use nurbs::VectorNurbs;
use temporal::{
    GridConfig, GridScheme, Limits, SolveStatus, ToleranceMode, schedule_segment_with_tolerance,
    topp::path::sample_arclength_grid,
};

fn textbook_limits() -> Limits {
    Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
}

/// Reconstruct the G5 cubic from `prototype.rs` `fixture_4` via the geometry pipeline.
///
/// G-code: `G5 X10 Y0 I3 J3 P-3 Q3 F1500`
/// Degree-3 non-rational NURBS: P0=(0,0,0), P1=(3,3,0), P2=(7,3,0), P3=(10,0,0).
/// κ is non-zero at both endpoints (symmetric control polygon).
fn build_g5_fixture_4_curve() -> VectorNurbs<f64, 3> {
    let src = "G5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
    let mut pipeline = GeometryPipeline::new(FitterParams::default());
    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        pipeline.process(src, &mut sink).collect()
    };
    items
        .into_iter()
        .find_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c.xyz),
            _ => None,
        })
        .expect("G5 reduction must emit exactly one Segment::Cubic")
}

/// Return the MVC-derived endpoint velocities at 50% of the centripetal cap.
///
/// κ is sampled at n=3 (endpoints + midpoint); `b_max_cent = a_cent_max / κ` at
/// each endpoint. `v = 0.5 * sqrt(b_max_cent)` puts endpoints well inside the MVC
/// while still exercising the per-axis SLP code path with non-trivial curvature.
fn fifty_pct_mvc_velocities(curve: &VectorNurbs<f64, 3>, limits: &Limits) -> (f64, f64) {
    let grid = sample_arclength_grid(curve, 3).expect("arclength grid");
    let kappa_start = grid.kappa[0];
    let kappa_end = *grid.kappa.last().expect("at least 2 points");
    let b_start = (limits.a_centripetal_max / kappa_start.max(1e-12)).min(1e8);
    let b_end = (limits.a_centripetal_max / kappa_end.max(1e-12)).min(1e8);
    (0.5 * b_start.sqrt(), 0.5 * b_end.sqrt())
}

#[test]
fn auto_succeeds_on_straight_line() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        None,
    )
    .unwrap();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 50,
    };
    let profile = schedule_segment_with_tolerance(
        &curve,
        &textbook_limits(),
        &grid,
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("Auto should succeed on straight line");
    assert!(matches!(
        profile.status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    ));
}

/// Demonstrates the actual Fast→Tight fallback recovery contract.
///
/// The Pi 5 investigation (Finding 2 SAFETY UPDATE) documented that the G5
/// cubic with non-zero endpoint κ — `prototype.rs` `fixture_4` geometry — produces
/// `DivergedSlp { last_max_ratio: 1.0320..., outer_iters: 7 }` under
/// `ToleranceMode::Fast` (1e-5) at N=200 with v ≈ 50% MVC, while
/// `ToleranceMode::Tight` (1e-8) resolves the same problem with `SolvedSlp`.
///
/// This test asserts all three legs of the contract explicitly:
///   1. `Fast` returns a *non-success* status on this fixture.
///   2. `Tight` returns a *success* status on the same fixture.
///   3. `Auto` returns a *success* status (fallback recovered the failure).
///
/// TODO: ignored after b87cd55f ("topp/path: fix 1-ulp overshoot in uniform-s
/// grid endpoint"). The 1-ulp endpoint fix changed the grid-endpoint sample
/// just enough that Fast now converges on this fixture in 2 outer iterations
/// instead of diverging — Pi 5 Finding 2's documented `DivergedSlp` regime no
/// longer applies here. The fallback chain still exists; this fixture just
/// stops exercising it. Find a new fixture (different curvature profile or
/// MVC fraction) that genuinely diverges under Fast, then re-enable.
#[ignore = "fixture no longer triggers Fast SLP divergence after b87cd55f endpoint fix; needs a new diverging case"]
#[test]
fn auto_falls_back_on_fixture_4_class() {
    let limits = textbook_limits();
    let curve = build_g5_fixture_4_curve();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        // N=200: the grid density at which the Fast SLP convergence breaks down
        // on this geometry (documented in the Pi 5 investigation as the worst
        // case: cubic@N=200 = 1596 ms at default 1e-8, 142 ms at 1e-5, but
        // DivergedSlp at 1e-5 for G5-with-endpoint-κ at 50% MVC).
        n: 200,
    };

    // Endpoint velocities at 50% of the centripetal MVC cap. This is the
    // regime where the SLP outer loop's convergence detection is fragile at
    // 1e-5: the relaxation tightness gap is small enough that noisier inner
    // solves at 1e-5 cause the "did we improve enough?" check to false-exit.
    let (v_start, v_end) = fifty_pct_mvc_velocities(&curve, &limits);

    // Leg 1: Fast (tol=1e-5) must return a non-success status.
    // Pi5 investigation reports DivergedSlp { last_max_ratio: 1.0320..., outer_iters: 7 }.
    let fast = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Fast,
    )
    .expect("Fast must not return ScheduleError");
    assert!(
        !matches!(
            fast.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "Fast must fail on this fixture (expected DivergedSlp/MaxIter/Infeasible); got {:?}",
        fast.status,
    );

    // Leg 2: Tight (tol=1e-8) must return a success status on the same fixture.
    let tight = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Tight,
    )
    .expect("Tight must not return ScheduleError");
    assert!(
        matches!(
            tight.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "Tight must succeed on this fixture; got {:?}",
        tight.status,
    );

    // Leg 3: Auto must return a success status (fallback recovered the Fast failure).
    let auto_profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Auto,
    )
    .expect("Auto must not return ScheduleError");
    assert!(
        matches!(
            auto_profile.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "Auto must succeed (fallback to Tight recovered the Fast failure); got {:?}",
        auto_profile.status,
    );

    // Sanity: Auto's trajectory time must be >= Tight's (Auto can only do as
    // well as Tight on a fallback case — it ran Fast first, which failed, so
    // Tight is the actual result returned).
    assert!(
        (auto_profile.total_time - tight.total_time).abs() < 1e-6,
        "Auto time ({}) should equal Tight time ({}) on a fallback case",
        auto_profile.total_time,
        tight.total_time,
    );

    eprintln!(
        "auto_falls_back_on_fixture_4_class: v_start={v_start:.4}, v_end={v_end:.4}, \
         Fast={:?}, Tight={:?}, Auto={:?}",
        fast.status, tight.status, auto_profile.status,
    );
}
