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
    )
    .unwrap()
}

fn arc() -> VectorNurbs<f64, 3> {
    let r = 20.0_f64;
    let k = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, 0.0, 0.0],
            [r, k * r, 0.0],
            [k * r, r, 0.0],
            [0.0, r, 0.0],
        ],
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

    for _ in 0..5 {
        let _ = schedule_segment(&curve, &limits, &grid, 0.0, 0.0).expect("warm-up");
    }

    let mut samples = Vec::with_capacity(iters);
    let start = Instant::now();
    for i in 0..iters {
        let t0 = Instant::now();
        let profile = schedule_segment(&curve, &limits, &grid, 0.0, 0.0).expect("schedule_segment");
        let ns = t0.elapsed().as_nanos() as u64;
        std::hint::black_box(&profile);
        samples.push(ns);
        println!("{i} {ns}");
    }
    let total = start.elapsed().as_secs_f64();

    let mut sorted = samples.clone();
    sorted.sort_unstable();
    let n = sorted.len();
    let pct = |p: f64| -> u64 {
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
