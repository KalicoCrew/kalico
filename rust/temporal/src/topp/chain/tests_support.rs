use nurbs::VectorNurbs;

use crate::Limits;
use crate::topp::chain::ChainGrid;

pub(crate) fn line(from: [f64; 3], to: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![from, to]).unwrap()
}

pub(crate) fn line_50mm() -> VectorNurbs<f64, 3> {
    line([0.0; 3], [50.0, 0.0, 0.0])
}

/// 40 mm + 60 mm collinear lines with different v_max per side
/// (300 / 150 mm/s); 11 + 13 grid points → junction at index 10,
/// h = 4 mm left, 5 mm right.
pub(crate) fn two_segment_chain_with_junction() -> ChainGrid {
    let ga =
        crate::topp::path::sample_arclength_grid(&line([0.0; 3], [40.0, 0.0, 0.0]), 11).unwrap();
    let gb =
        crate::topp::path::sample_arclength_grid(&line([40.0, 0.0, 0.0], [100.0, 0.0, 0.0]), 13)
            .unwrap();
    let lim = |v: f64| Limits {
        v_max: [v; 3],
        a_max: [5_000.0; 3],
        j_max: [100_000.0; 3],
        a_centripetal_max: 2_500.0,
    };
    ChainGrid::from_segment_grids(vec![ga, gb], vec![lim(300.0), lim(150.0)])
}
