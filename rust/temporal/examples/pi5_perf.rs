//! Pi-5 performance harness for `schedule_segment`.
//!
//! Measures per-call wall-clock for representative single-segment SOCP solves.
//! Used to settle the Step-4.5 (A)-vs-(B) joining-vs-solving architecture choice:
//! whether re-solving the SOCP on every joining iteration is feasible at MVP
//! throughput on the actual target host.
//!
//! Usage: `pi5_perf <fixture> <iters> [grid_n]`
//!   fixture  ∈ { straight, arc, cubic }
//!   `iters`    > 0
//!   `grid_n`   default 200
//!
//! Emits one line per iteration: `<idx> <wallclock_ns>`. Final line: `summary
//! min=<ns> p50=<ns> p95=<ns> p99=<ns> p999=<ns> max=<ns> mean=<ns> stdev=<ns>
//! n=<iters> total_s=<f>`.
//!
//! Host computes percentiles from the per-iteration stream (so we can re-bucket
//! without re-running the bench).

use std::time::Instant;

use nurbs::VectorNurbs;
use temporal::{GridConfig, GridScheme, Limits, schedule_segment};

fn textbook_limits() -> Limits {
    Limits::new(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        2_500.0,
    )
}

/// Spec §5.1 fixture 1: degree-1 NURBS from (0,0,0) to (100,0,0).
fn straight() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        None,
    )
    .unwrap()
}

/// Spec §5.1 fixture 3 equivalent: quarter-circle arc, R=20mm, 90° sweep,
/// rational quadratic NURBS. Hand-rolled (geometry crate's reduce path is
/// equivalent but heavier to set up).
fn arc() -> VectorNurbs<f64, 3> {
    let w = std::f64::consts::FRAC_1_SQRT_2; // sqrt(2)/2
    VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 20.0, 0.0]],
        Some(vec![1.0, w, 1.0]),
    )
    .unwrap()
}

/// G5-style cubic NURBS with non-zero curvature throughout. Single piece,
/// degree-3, 4 CPs, clamped knot vector. Designed to exercise the SLP outer
/// loop on jerk-bounded scheduling of varying curvature.
fn cubic() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [30.0, 50.0, 0.0],
            [70.0, 50.0, 0.0],
            [100.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let fixture = args.next().expect("fixture: straight|arc|cubic");
    let iters: usize = args
        .next()
        .expect("iters: integer > 0")
        .parse()
        .expect("iters must parse");
    let grid_n: usize = args
        .next()
        .map_or(200, |s| s.parse().expect("grid_n must parse"));

    let curve = match fixture.as_str() {
        "straight" => straight(),
        "arc" => arc(),
        "cubic" => cubic(),
        other => panic!("unknown fixture: {other}"),
    };
    let limits = textbook_limits();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: grid_n,
    };

    // Warm-up: 5 iterations untimed, to amortize first-solve allocator cost
    // and let the kernel's CPU governor reach a steady state.
    for _ in 0..5 {
        let _ = schedule_segment(&curve, &limits, &grid, 0.0, 0.0).expect("warm-up");
    }

    let mut samples = Vec::with_capacity(iters);
    let start = Instant::now();
    for i in 0..iters {
        let t0 = Instant::now();
        let profile = schedule_segment(&curve, &limits, &grid, 0.0, 0.0).expect("schedule_segment");
        let ns = t0.elapsed().as_nanos() as u64;
        // Black-box prevent the compiler from optimizing the call away.
        std::hint::black_box(&profile);
        samples.push(ns);
        println!("{i} {ns}");
    }
    let total = start.elapsed().as_secs_f64();

    let mut sorted = samples.clone();
    sorted.sort_unstable();
    let n = sorted.len();
    let pct = |p: f64| -> u64 {
        // p is always in [0,1]; floor is non-negative. Suppress sign-loss lint.
        #[allow(clippy::cast_sign_loss)]
        let idx = ((p * (n as f64)).floor() as usize).min(n - 1);
        sorted[idx]
    };
    let min = sorted[0];
    let max = sorted[n - 1];
    let mean: f64 = samples.iter().map(|&x| x as f64).sum::<f64>() / n as f64;
    let var: f64 = samples
        .iter()
        .map(|&x| (x as f64 - mean).powi(2))
        .sum::<f64>()
        / n as f64;
    let stdev = var.sqrt();

    eprintln!(
        "summary fixture={fixture} grid_n={grid_n} min={min} p50={} p95={} p99={} p999={} max={max} mean={mean:.0} stdev={stdev:.0} n={n} total_s={total:.3}",
        pct(0.50),
        pct(0.95),
        pct(0.99),
        pct(0.999),
    );
}
