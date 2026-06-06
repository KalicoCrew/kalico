//! Offline repro of the bench-measured multi-second `append_and_replan` solve
//! on a 2-move X reversal window (Trident config: corexy, vmax 1000, amax 70000,
//! smooth_mzv X@186 / Y@122; macro recipe: 30mm jogs @ 25mm/s).
//!
//! Bench measurement (Pi, 2026-06-06): move 2 replan_us=4,160,141 with
//! solve_us=4,160,064, beta_iters=1, converged — the time is one temporal solve.
//!
//! Run: cargo run -p trajectory --release --example replan_repro

use std::time::Instant;

use geometry::segment::{CubicSegment, EMode, SourceRange};
use nurbs::VectorNurbs;
use trajectory::plan_velocity::{PlanShaper, SafetyMode};
use trajectory::streaming::{EmitContext, ReplanContext, ShaperState};
use trajectory::{AxisShaper, ELimits, RequiredShaper};

fn collinear_cubic(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    let lerp = |t: f64| {
        [
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        ]
    };
    VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![start, lerp(1.0 / 3.0), lerp(2.0 / 3.0), end],
    )
    .unwrap()
}

fn travel(start: [f64; 3], end: [f64; 3], feed: f64) -> CubicSegment {
    CubicSegment::try_new(
        collinear_cubic(start, end),
        EMode::Travel,
        0.0,
        None,
        feed,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .unwrap()
}

fn main() {
    let limits = temporal::Limits::new(
        [1000.0, 1000.0, 15.0],
        [70000.0, 70000.0, 100.0],
        [140000.0, 140000.0, 200.0],
        5.0_f64.powi(2) / (70000.0 * 0.5),
    );
    let ctx = ReplanContext {
        limits,
        kernels: [
            Some(PlanShaper::SmoothMzv {
                frequency_hz: 186.0,
            }),
            Some(PlanShaper::SmoothMzv {
                frequency_hz: 122.0,
            }),
            Some(PlanShaper::Passthrough),
            None,
        ],
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        junction_chord_tolerance_mm: 0.05,
        worker_threads: 3,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        fallback_initial_v: 0.0,
        safety_mode: SafetyMode::WorstCaseFuture,
    };

    let shapers = [
        Some(AxisShaper::SmoothMzv {
            frequency_hz: 186.0,
        }),
        Some(AxisShaper::SmoothMzv {
            frequency_hz: 122.0,
        }),
        Some(AxisShaper::Passthrough),
        None,
    ];
    // Emit kernels exactly as the planner builds them (planner.rs
    // shaper_config_to_emit_kernels): required X/Y kernels, passthrough Z/E.
    let emit_kernels = [
        Some(
            RequiredShaper::SmoothMzv {
                frequency_hz: 186.0,
            }
            .to_kernel(),
        ),
        Some(
            RequiredShaper::SmoothMzv {
                frequency_hz: 122.0,
            }
            .to_kernel(),
        ),
        AxisShaper::Passthrough.to_kernel(),
        None,
    ];
    let emit_ctx = EmitContext {
        kernels: &emit_kernels,
        e_halos: &[],
    };

    // Mirror the planner Move arm: append → emit (advances t_dispatched, so the
    // next append splits the in-flight move and enters the window at speed).
    // Loops to give a sampling profiler a steady workload (arg = iterations).
    let iters: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    for i in 0..iters {
        let mut state = ShaperState::new([150.0, 150.0, 10.0, 0.0], &shapers);
        let m1 = travel([150.0, 150.0, 10.0], [180.0, 150.0, 10.0], 25.0);
        let m2 = travel([180.0, 150.0, 10.0], [150.0, 150.0, 10.0], 25.0);

        for (name, seg) in [("m1 (+30mm)", m1), ("m2 (-30mm reversal)", m2)] {
            let t = Instant::now();
            let report = state.append_and_replan(seg, &ctx).expect("replan failed");
            let drained = state.emit_committed(&emit_ctx).expect("emit failed");
            println!(
                "[{i}] {name}: total={:.3?} solve_us={} split_us={} rebuild_us={} window={} beta_iters={} converged={} drained={}",
                t.elapsed(),
                report.solve_us,
                report.split_us,
                report.rebuild_us,
                report.window_segments,
                report.plan.beta_iterations,
                report.plan.beta_converged,
                drained.len(),
            );
        }
    }
}
