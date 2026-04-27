//! Stress N OS threads with a tight FP loop to occupy CPU cores while another
//! process runs the SOCP benchmark. Prints `started <n>` then runs forever
//! until SIGTERM/SIGINT. Use `pkill cpu_stress` to stop.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

fn main() {
    let n: usize = std::env::args()
        .nth(1)
        .expect("usage: cpu_stress <threads>")
        .parse()
        .expect("threads must parse");
    let stop = Arc::new(AtomicBool::new(false));
    let mut handles = Vec::with_capacity(n);
    for tid in 0..n {
        let stop = Arc::clone(&stop);
        handles.push(std::thread::spawn(move || {
            let mut x: f64 = (tid as f64) + 1.0;
            while !stop.load(Ordering::Relaxed) {
                for _ in 0..10_000 {
                    x = (x * 1.0000001 + 0.0000001).sin().abs() + 1.0;
                }
                std::hint::black_box(x);
            }
        }));
    }
    println!("started {n}");
    // Block forever — Ctrl-C / SIGTERM kills us.
    for h in handles {
        let _ = h.join();
    }
}
