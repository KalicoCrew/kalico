//! Throughput benchmark: how many independent `schedule_segment` calls per
//! second can we sustain across N OS threads on the Pi 5?
//!
//! Models the per-segment-parallel batch-planning regime: after joining has
//! decided every segment's `(v_start, v_end)`, the per-segment SOCPs are
//! embarrassingly parallel and can fan out across cores.
//!
//! Usage: `pi5_parallel <fixture> <total_iters> <threads> [grid_n]`
//!   fixture     ∈ { straight, arc, cubic }
//!   `total_iters` > 0     (split evenly across threads)
//!   `threads`     1..=4   (Pi 5 has 4 A76 cores)
//!   `grid_n`      default 100

use std::sync::Arc;
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

fn straight() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        None,
    )
    .unwrap()
}

fn arc() -> VectorNurbs<f64, 3> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 20.0, 0.0]],
        Some(vec![1.0, w, 1.0]),
    )
    .unwrap()
}

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
    let total_iters: usize = args.next().expect("total_iters").parse().expect("parse");
    let threads: usize = args.next().expect("threads").parse().expect("parse");
    let grid_n: usize = args.next().map_or(100, |s| s.parse().expect("parse"));

    let curve = Arc::new(match fixture.as_str() {
        "straight" => straight(),
        "arc" => arc(),
        "cubic" => cubic(),
        other => panic!("unknown fixture: {other}"),
    });
    let limits = Arc::new(textbook_limits());
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: grid_n,
    };

    // Warm-up across all threads.
    for _ in 0..5 {
        let _ = schedule_segment(&curve, &limits, &grid, 0.0, 0.0).expect("warm-up");
    }

    let per_thread = total_iters.div_ceil(threads);
    let actual_iters = per_thread * threads;

    let start = Instant::now();
    let mut handles = Vec::with_capacity(threads);
    for _ in 0..threads {
        let curve = Arc::clone(&curve);
        let limits = Arc::clone(&limits);
        handles.push(std::thread::spawn(move || {
            for _ in 0..per_thread {
                let profile = schedule_segment(&curve, &limits, &grid, 0.0, 0.0).expect("solve");
                std::hint::black_box(&profile);
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }
    let total = start.elapsed().as_secs_f64();

    let throughput = actual_iters as f64 / total;
    let per_iter_amortized_ms = (total / actual_iters as f64) * 1000.0;
    println!(
        "fixture={fixture} grid_n={grid_n} threads={threads} total_iters={actual_iters} \
         wall_s={total:.3} throughput_per_s={throughput:.1} amortized_per_iter_ms={per_iter_amortized_ms:.2}"
    );
}
