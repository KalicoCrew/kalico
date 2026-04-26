//! Cross-check our eval against NURBS-Python (geomdl) on a fixed corpus.
//! Corpus file: `tests/data/geomdl_corpus.json` (regenerated via the
//! Python script in `tests/scripts/`).

#![cfg(feature = "host")]

use serde_json::Value;
use std::fs;
use std::path::PathBuf;

const TOLERANCE: f64 = 1e-9;

#[test]
fn oracle_matches_for_corpus_curves() {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "tests",
        "data",
        "geomdl_corpus.json",
    ]
    .iter()
    .collect();
    let raw = fs::read_to_string(&path).expect("corpus must exist");
    let v: Value = serde_json::from_str(&raw).expect("valid JSON");

    for curve_v in v["curves"].as_array().unwrap() {
        let name = curve_v["name"].as_str().unwrap();
        let degree = curve_v["degree"].as_u64().unwrap() as u8;
        let knots: Vec<f64> = curve_v["knots"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| x.as_f64().unwrap())
            .collect();
        let cps_3d: Vec<[f64; 3]> = curve_v["control_points"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| {
                let arr = p.as_array().unwrap();
                [
                    arr[0].as_f64().unwrap(),
                    arr[1].as_f64().unwrap(),
                    arr[2].as_f64().unwrap(),
                ]
            })
            .collect();
        let weights: Option<Vec<f64>> = if curve_v["weights"].is_null() {
            None
        } else {
            Some(
                curve_v["weights"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|x| x.as_f64().unwrap())
                    .collect(),
            )
        };

        let curve = nurbs::VectorNurbs::<f64, 3>::try_new(degree, knots, cps_3d, weights)
            .unwrap_or_else(|e| panic!("{name}: try_new failed: {e:?}"));

        for sample in curve_v["samples"].as_array().unwrap() {
            let u = sample["u"].as_f64().unwrap();
            let expected = sample["point"].as_array().unwrap();
            let result = nurbs::eval::vector_eval(&curve.as_view(), u);
            for axis in 0..3 {
                let exp = expected[axis].as_f64().unwrap();
                let diff = (result[axis] - exp).abs();
                assert!(
                    diff < TOLERANCE,
                    "{name} u={u} axis={axis}: got {} expected {} (diff {diff})",
                    result[axis],
                    exp,
                );
            }
        }
    }
}
