//! Helical G2/G3 → 3D rational quadratic `ArcSegment` with linear-Z control points.
//! Locks the full-3D commitment as a tested invariant.
//!
//! Gated behind `legacy-reference`: live pipeline rejects G2/G3 at reduce time
//! (Step-13 compat-layer territory).

#![cfg(feature = "legacy-reference")]

use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

fn run(text: &str) -> Vec<Item> {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    p.process(text, &mut sink).collect()
}

#[test]
fn helical_g2_quarter_with_z_progression() {
    // Quarter-circle in XY from (1,0,0) to (0,1,0), Z progresses 0 → 0.5.
    let items = run("G1 X1 Z0 F1500\nG2 X0 Y1 Z0.5 I-1 J0\n");
    let arc = items
        .iter()
        .find_map(|it| match it {
            Item::Segment(Segment::Arc(a)) => Some(a),
            _ => None,
        })
        .expect("expected ArcSegment");
    // Rational quadratic with 3 control points.
    assert_eq!(arc.xyz.degree(), 2);
    let cps = arc.xyz.control_points();
    assert_eq!(cps.len(), 3);
    // Weights: [1, cos(45°), 1] for a 90° arc. cos(π/4) ≈ 0.7071.
    let weights = arc.xyz.weights().expect("rational arc has weights");
    assert!((weights[0] - 1.0).abs() < 1e-12);
    assert!((weights[1] - (std::f64::consts::FRAC_1_SQRT_2)).abs() < 1e-9);
    assert!((weights[2] - 1.0).abs() < 1e-12);
    // Z linear: 0.0, 0.25, 0.5
    assert!((cps[0][2] - 0.0).abs() < 1e-12);
    assert!((cps[1][2] - 0.25).abs() < 1e-12);
    assert!((cps[2][2] - 0.5).abs() < 1e-12);
}
