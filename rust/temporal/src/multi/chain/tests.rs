use super::*;
use crate::multi::junction::JunctionKind;
use crate::{BindingConstraint, GridSample, GridScheme, SolveStatus, TopProfile};

#[test]
fn partition_splits_only_at_corners() {
    let kinds = [JunctionKind::Smooth, JunctionKind::Corner, JunctionKind::Smooth];
    let chains = partition_chains(4, &kinds);
    assert_eq!(chains, vec![0..=1, 2..=3]);
}

#[test]
fn partition_all_smooth_is_one_chain() {
    let kinds = [JunctionKind::Smooth, JunctionKind::Smooth];
    assert_eq!(partition_chains(3, &kinds), vec![0..=2]);
}

#[test]
fn partition_all_corners_is_all_singletons() {
    let kinds = [JunctionKind::Corner, JunctionKind::Corner];
    assert_eq!(partition_chains(3, &kinds), vec![0..=0, 1..=1, 2..=2]);
}

#[test]
fn partition_single_segment_no_junctions() {
    assert_eq!(partition_chains(1, &[]), vec![0..=0]);
}

#[test]
fn slice_duplicates_junction_sample_and_splits_time() {
    // 2-segment chain profile: ranges (0,2) and (2,4), 5 samples, uniform v=10.
    let ranges = vec![(0usize, 2usize), (2, 4)];
    let samples: Vec<GridSample> = (0..5)
        .map(|i| GridSample {
            s: i as f64,
            v: 10.0,
            a: 0.0,
            b: 100.0,
            binding: BindingConstraint::None,
        })
        .collect();
    let chain_profile = TopProfile {
        samples,
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time: 0.4,
    };
    let per_segment = slice_chain_profile(&chain_profile, &ranges);
    assert_eq!(per_segment.len(), 2);
    assert_eq!(per_segment[0].samples.len(), 3);
    assert_eq!(per_segment[1].samples.len(), 3);
    // Junction sample duplicated into both, with per-segment s rebased to 0.
    assert_eq!(per_segment[0].samples[2].v, per_segment[1].samples[0].v);
    assert!((per_segment[1].samples[0].s - 0.0).abs() < 1e-12);
    assert!((per_segment[0].samples[2].s - 2.0).abs() < 1e-12); // last of seg0
    // 2 mm at 10 mm/s each → 0.2 s per segment.
    assert!((per_segment[0].total_time - 0.2).abs() < 1e-9);
    assert!((per_segment[1].total_time - 0.2).abs() < 1e-9);
}
