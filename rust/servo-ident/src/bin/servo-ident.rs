//! Usage: servo-ident --capture run.csv --structure scalar|corexy \
//!   --axes x[,b] --out profile.toml \
//!   [--rated-torque-nm T --rotor-inertia-kgm2 J --rotation-distance-mm D]
#![allow(clippy::exit)]

use servo_ident::capture::parse_capture_csv;
use servo_ident::fit::{fit, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::profile_out::{c0006_recommendation, render_profile};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

fn req(args: &[String], key: &str) -> String {
    arg(args, key).unwrap_or_else(|| {
        eprintln!("servo-ident: missing required {key}");
        std::process::exit(1);
    })
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let structure = match req(&args, "--structure").as_str() {
        "scalar" => Structure::CartesianScalar,
        "corexy" => Structure::CoreXY,
        other => {
            eprintln!("servo-ident: unknown structure {other}");
            std::process::exit(1);
        }
    };
    let axes_arg = req(&args, "--axes");
    let axes: Vec<&str> = axes_arg.split(',').map(str::trim).collect();
    if axes.len() != structure.axis_count() {
        eprintln!(
            "servo-ident: {} axes given, structure needs {}",
            axes.len(),
            structure.axis_count()
        );
        std::process::exit(1);
    }

    let capture_path = req(&args, "--capture");
    let text = std::fs::read_to_string(&capture_path).unwrap_or_else(|e| {
        eprintln!("servo-ident: read {capture_path}: {e}");
        std::process::exit(1);
    });
    let cap = parse_capture_csv(&text, &axes).unwrap_or_else(|e| {
        eprintln!("servo-ident: capture invalid: {e:?}");
        std::process::exit(1);
    });
    let input = FitInput {
        structure,
        acc: cap.acc,
        vel: cap.vel,
        torque: cap.torque,
    };
    let r = fit(&input, &FitOptions::default()).unwrap_or_else(|e| {
        eprintln!("servo-ident: refusing to emit a profile: {e:?}");
        std::process::exit(2);
    });

    eprintln!(
        "fit: {} samples/motor, rms residual {:.2} (0.1% rated), condition {:.1e}",
        r.samples, r.rms_residual, r.condition
    );
    if let (Some(t), Some(j), Some(d)) = (
        arg(&args, "--rated-torque-nm").and_then(|v| v.parse::<f64>().ok()),
        arg(&args, "--rotor-inertia-kgm2").and_then(|v| v.parse::<f64>().ok()),
        arg(&args, "--rotation-distance-mm").and_then(|v| v.parse::<f64>().ok()),
    ) {
        let n = r.params.mass.len();
        let m_light = if n == 2 {
            r.params.mass[0][0] + r.params.mass[0][1]
        } else {
            r.params.mass[0][0]
        };
        eprintln!(
            "recommended C00.06 (light direction): {:.0}%",
            c0006_recommendation(m_light, t, d, j)
        );
        if n == 2 {
            let m_heavy = r.params.mass[0][0] - r.params.mass[0][1];
            eprintln!(
                "heavy-direction equivalent (reference only): {:.0}%",
                c0006_recommendation(m_heavy, t, d, j)
            );
        }
    }

    let rms = vec![r.rms_residual; axes.len()];
    let profile = render_profile(&r.params, &axes, &rms);
    let out = req(&args, "--out");
    std::fs::write(&out, profile).unwrap_or_else(|e| {
        eprintln!("servo-ident: write {out}: {e}");
        std::process::exit(1);
    });
    eprintln!("profile written to {out}");
}
