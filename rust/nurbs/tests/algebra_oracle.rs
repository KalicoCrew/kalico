//! Cross-check our algebra ops against a sympy-generated oracle corpus.
//! Corpus file: `tests/data/algebra_corpus.json` (regenerated via the
//! Python script in `tests/scripts/`).

#![cfg(feature = "host")]

use serde_json::Value;
use std::fs;
use std::path::PathBuf;

const TOLERANCE: f64 = 1e-9;

fn parse_curve(v: &Value) -> nurbs::ScalarNurbs<f64> {
    let degree = v["degree"].as_u64().unwrap() as u8;
    let knots: Vec<f64> = v["knots"]
        .as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect();
    let cps: Vec<f64> = v["control_points"]
        .as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect();
    let weights: Option<Vec<f64>> = if v["weights"].is_null() {
        None
    } else {
        Some(
            v["weights"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap())
                .collect(),
        )
    };
    nurbs::ScalarNurbs::try_new(degree, knots, cps, weights).unwrap()
}

fn parse_kernel(v: &Value) -> nurbs::algebra::PiecewisePolynomialKernel<f64> {
    let pieces: Vec<nurbs::BezierPiece<f64>> = v["pieces"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| {
            let u_start = p["u_start"].as_f64().unwrap();
            let u_end = p["u_end"].as_f64().unwrap();
            let coeffs: Vec<f64> = p["coeffs"]
                .as_array()
                .unwrap()
                .iter()
                .map(|c| c.as_f64().unwrap())
                .collect();
            nurbs::BezierPiece {
                u_start,
                u_end,
                coeffs,
            }
        })
        .collect();
    nurbs::algebra::PiecewisePolynomialKernel { pieces }
}

#[test]
fn algebra_oracle_matches_for_corpus() {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "tests",
        "data",
        "algebra_corpus.json",
    ]
    .iter()
    .collect();
    let raw = fs::read_to_string(&path).expect("corpus must exist");
    let v: Value = serde_json::from_str(&raw).expect("valid JSON");

    for fixture in v["fixtures"].as_array().unwrap() {
        let name = fixture["name"].as_str().unwrap();
        let op = fixture["operation"].as_str().unwrap();
        let result = match op {
            "multiply" => {
                let a = parse_curve(&fixture["a"]);
                let b = parse_curve(&fixture["b"]);
                nurbs::algebra::multiply(&a, &b)
                    .unwrap_or_else(|e| panic!("{name}: multiply failed: {e:?}"))
            }
            "convolve" => {
                let curve = parse_curve(&fixture["curve"]);
                let kernel = parse_kernel(&fixture["kernel"]);
                nurbs::algebra::convolve(&curve, &kernel)
                    .unwrap_or_else(|e| panic!("{name}: convolve failed: {e:?}"))
            }
            other => panic!("unknown operation: {other}"),
        };

        for sample in fixture["samples"].as_array().unwrap() {
            let u = sample["u"].as_f64().unwrap();
            let expected = sample["value"].as_f64().unwrap();
            let got = nurbs::eval::eval(&result.as_view(), u);
            let diff = (got - expected).abs();
            assert!(
                diff < TOLERANCE,
                "{name} u={u}: got {got} expected {expected} (diff {diff})"
            );
        }
    }
}
