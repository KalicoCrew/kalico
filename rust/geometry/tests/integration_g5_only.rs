use geometry::{
    FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent, split_segment_to_cap,
};
use nurbs::eval::vector_eval;

const BOUNDARY_TOL: f64 = 1e-12;

#[test]
fn synthetic_long_g5_reduces_splits_and_plans() {
    let g5_input = "G5 X50 Y0 I0 J20 P0 Q20 F1000\n";

    let mut pipeline = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    let items: Vec<_> = pipeline.process(g5_input, &mut sink).collect();

    let cubic = items
        .iter()
        .find_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
            _ => None,
        })
        .expect("expected at least one Segment::Cubic");

    let split = split_segment_to_cap(&cubic, 12.5).expect("split ok");
    assert!(
        split.len() >= 4,
        "50 mm cubic should split into ≥4 sub-segments at 12.5 mm cap, got {}",
        split.len()
    );

    for w in split.windows(2) {
        let lend = vector_eval(&w[0].xyz, 1.0);
        let rstart = vector_eval(&w[1].xyz, 0.0);
        for axis in 0..3 {
            let diff = (lend[axis] - rstart[axis]).abs();
            assert!(
                diff < BOUNDARY_TOL,
                "boundary continuity axis {axis}: diff={diff}, lend={lend:?} rstart={rstart:?}"
            );
        }
    }
}
