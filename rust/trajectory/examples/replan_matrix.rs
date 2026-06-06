//! Trigger isolation for the slow reversal-window solve: times `plan_velocity`
//! over (stub length × entry velocity) variants of the Trident macro recipe.
//!
//! Run: cargo run -p trajectory --release --example replan_matrix

use std::time::Instant;

use geometry::segment::{CubicSegment, EMode, SourceRange};
use nurbs::VectorNurbs;
use trajectory::plan_velocity::{plan_velocity, PlanInput, PlanSegment, PlanShaper, SafetyMode};
use trajectory::ELimits;

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

// Mirrors state.rs per_segment_limits for a pure-X collinear segment at feed f:
// inactive axes get the max active j_max; active axes get v capped by feed.
fn x_only_limits(base: &temporal::Limits, feed: f64) -> temporal::Limits {
    let j_active = base.j_max[0];
    temporal::Limits::new(
        [base.v_max[0].min(feed), base.v_max[1], base.v_max[2]],
        base.a_max,
        [j_active, j_active, j_active],
        base.a_centripetal_max,
    )
}

fn main() {
    let base = temporal::Limits::new(
        [1000.0, 1000.0, 15.0],
        [70000.0, 70000.0, 100.0],
        [140000.0, 140000.0, 200.0],
        5.0_f64.powi(2) / (70000.0 * 0.5),
    );
    let kernels = [
        Some(PlanShaper::SmoothMzv {
            frequency_hz: 186.0,
        }),
        Some(PlanShaper::SmoothMzv {
            frequency_hz: 122.0,
        }),
        Some(PlanShaper::Passthrough),
        None,
    ];

    let feed = 25.0;
    let stub_lens = [0.05_f64, 0.2, 0.5, 2.0, 10.0, 30.0];
    let entry_vs = [0.0_f64, 10.0, 20.0, 25.0];

    println!("stub_mm  entry_v  ->  total_ms  beta_iters  converged");
    for &stub in &stub_lens {
        for &v0 in &entry_vs {
            let seg1 = travel([180.0 - stub, 150.0, 10.0], [180.0, 150.0, 10.0], feed);
            let seg2 = travel([180.0, 150.0, 10.0], [150.0, 150.0, 10.0], feed);
            let segs = [seg1, seg2];
            let lims = x_only_limits(&base, feed);
            let plan_segments: Vec<PlanSegment<'_>> = segs
                .iter()
                .map(|m| PlanSegment {
                    temporal: temporal::multi::SegmentInput {
                        curve: &m.xyz,
                        limits: lims,
                        trailing_junction_chord_tolerance_mm: 0.05,
                    },
                    e_mode: m.e_mode,
                    extrusion_per_xy_mm: m.extrusion_per_xy_mm,
                    e_independent: m.e_independent.as_ref(),
                    feedrate_mm_s: m.feedrate_mm_s,
                })
                .collect();
            let input = PlanInput {
                segments: &plan_segments,
                grid_strategy: temporal::multi::GridStrategy::Adaptive {
                    min_n: 20,
                    max_n: 200,
                    target_grid_spacing_mm: 0.5,
                },
                worker_threads: 3,
                kernels,
                fit_tolerance_mm: 0.005,
                beta_max_iters: 10,
                beta_convergence_ratio: 0.05,
                e_limits: ELimits {
                    v_max: 50.0,
                    a_max: 5000.0,
                },
                initial_v: v0,
                terminal_v: 0.0,
                safety_mode: SafetyMode::WorstCaseFuture,
            };
            let t = Instant::now();
            match plan_velocity(&input) {
                Ok(out) => println!(
                    "{stub:7.2}  {v0:7.1}  -> {:9.1}  {:10}  {}",
                    t.elapsed().as_secs_f64() * 1e3,
                    out.stats.beta_iterations,
                    out.stats.beta_converged,
                ),
                Err(e) => println!("{stub:7.2}  {v0:7.1}  ->  ERR {e:?}"),
            }

            // Deep-dive the raw temporal batch for the same window: isolates
            // joining sweeps from per-solve SLP cost (no shaping / fit / beta).
            let batch_input = temporal::multi::BatchInput {
                segments: &plan_segments.iter().map(|p| p.temporal).collect::<Vec<_>>(),
                grid_strategy: input.grid_strategy,
                worker_threads: 3,
                initial_velocity: v0,
                terminal_velocity: 0.0,
            };
            let t2 = Instant::now();
            match temporal::multi::plan_batch(batch_input) {
                Ok(b) => println!(
                    "{:>32} batch_ms={:8.1} sweeps={} seg_times=[{}]",
                    "",
                    t2.elapsed().as_secs_f64() * 1e3,
                    b.joining_sweeps,
                    b.profiles
                        .iter()
                        .map(|p| format!("{:.4} ({:?})", p.total_time, p.status))
                        .collect::<Vec<_>>()
                        .join(", "),
                ),
                Err(e) => println!("{:>32} batch ERR {e:?}", ""),
            }
        }
    }
}
