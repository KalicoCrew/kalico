//! Usage: servo-excite --axis X --min 10 --max 210 \
//!   --accels 1000,3000,6000 --speeds 100,200,300 [--reps 4] [--out f.gcode]
#![allow(clippy::exit)]

use servo_ident::gcode_gen::{generate, Excitation};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

fn req(args: &[String], key: &str) -> String {
    arg(args, key).unwrap_or_else(|| {
        eprintln!("servo-excite: missing required {key}");
        std::process::exit(1);
    })
}

fn list(s: &str) -> Vec<f64> {
    s.split(',')
        .map(|v| {
            v.trim().parse().unwrap_or_else(|_| {
                eprintln!("servo-excite: bad number {v:?}");
                std::process::exit(1);
            })
        })
        .collect()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let min_mm = list(&req(&args, "--min"))
        .into_iter()
        .next()
        .unwrap_or_else(|| {
            eprintln!("servo-excite: --min produced no value");
            std::process::exit(1);
        });
    let max_mm = list(&req(&args, "--max"))
        .into_iter()
        .next()
        .unwrap_or_else(|| {
            eprintln!("servo-excite: --max produced no value");
            std::process::exit(1);
        });
    let e = Excitation {
        axis: req(&args, "--axis"),
        min_mm,
        max_mm,
        accels_mm_s2: list(&req(&args, "--accels")),
        speeds_mm_s: list(&req(&args, "--speeds")),
        reps: arg(&args, "--reps").map_or(4, |r| {
            r.parse().unwrap_or_else(|_| {
                eprintln!("servo-excite: bad --reps {r:?}");
                std::process::exit(1);
            })
        }),
    };
    let g = generate(&e).unwrap_or_else(|err| {
        eprintln!("servo-excite: {err:?}");
        std::process::exit(1);
    });
    match arg(&args, "--out") {
        Some(p) => std::fs::write(&p, g).unwrap_or_else(|e| {
            eprintln!("servo-excite: write {p}: {e}");
            std::process::exit(1);
        }),
        None => print!("{g}"),
    }
}
