use super::*;
use crate::multi::SegmentInput;
use crate::{GridConfig, GridScheme, Limits};
use nurbs::VectorNurbs;

fn straight() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
        None,
    )
    .unwrap()
}

fn limits() -> Limits {
    Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
}

#[test]
fn fan_out_processes_all_dirty() {
    let curves: Vec<_> = (0..4).map(|_| straight()).collect();
    let inputs: Vec<SegmentInput> = curves
        .iter()
        .map(|c| SegmentInput {
            curve: c,
            limits: limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        })
        .collect();
    let grids = vec![
        GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 20
        };
        4
    ];
    let mut states: Vec<_> = (0..4)
        .map(|_| SegmentState {
            v_start: 0.0,
            v_end: 0.0,
            profile: None,
            dirty: true,
        })
        .collect();
    fan_out_solves(&inputs, &mut states, &grids, 3).unwrap();
    for s in &states {
        assert!(s.profile.is_some());
        assert!(!s.dirty);
    }
}
