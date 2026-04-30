//! Stage 0: batch partitioning.
//!
//! Splits a `&[ShapeSegmentInput]` into contiguous runs of XY-motion segments
//! (`CoupledToXy` or `Travel`) separated by independent-E gaps (retraction, prime,
//! filament-change). Each E gap is pre-scheduled via `schedule_e_duration` so
//! that its duration is known before the beta-medium loop begins.
//!
//! The structural partition (runs + `e_gaps`) is returned immediately. Global
//! time offsets are computed later during the beta loop after TOPP-RA provides
//! per-run segment durations.

use geometry::segment::EMode;
use nurbs::eval::vector_eval;

/// Result of partitioning a segment buffer into XY-motion runs and E gaps.
#[derive(Debug)]
pub struct BatchPartition {
    /// Contiguous runs of XY-motion segments (`CoupledToXy` or `Travel`).
    pub runs: Vec<Run>,
    /// Independent E segments between (or before/after) runs, in input order.
    pub e_gaps: Vec<EGap>,
}

/// A contiguous run of XY-motion segments.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Run {
    /// Range of indices into the original input segments array.
    pub segment_range: std::ops::Range<usize>,
}

/// An independent E segment that sits between runs (or before the first run,
/// or after the last run).
#[derive(Debug, Clone, PartialEq)]
pub struct EGap {
    /// Index into the original input segments array.
    pub segment_index: usize,
    /// Pre-scheduled duration of this E-only segment (seconds).
    pub duration: f64,
    /// XYZ position where the machine is stationary during the E move. Derived
    /// from the preceding segment's endpoint; `[0, 0, 0]` if there is no
    /// preceding segment.
    pub xyz_position: [f64; 3],
}

/// Partition a segment buffer into XY-motion runs and E gaps.
///
/// Algorithm:
/// 1. Iterate input segments. Group consecutive `CoupledToXy` / `Travel`
///    segments into runs. `Independent` segments become E gaps.
/// 2. For each E gap: call `schedule_e_duration` to pre-compute its duration.
///    Derive the XYZ hold position from the preceding segment's geometry-curve
///    endpoint (parameter u = 1). If there is no preceding segment, use
///    `[0, 0, 0]`.
pub fn partition_batch(
    segments: &[crate::ShapeSegmentInput<'_>],
    e_limits: &crate::ELimits,
) -> BatchPartition {
    let mut runs = Vec::new();
    let mut e_gaps = Vec::new();
    let mut run_start: Option<usize> = None;

    for (i, seg) in segments.iter().enumerate() {
        match seg.e_mode {
            EMode::CoupledToXy | EMode::Travel => {
                // Extend or start a run.
                if run_start.is_none() {
                    run_start = Some(i);
                }
            }
            EMode::Independent => {
                // Close any open run.
                if let Some(start) = run_start.take() {
                    runs.push(Run {
                        segment_range: start..i,
                    });
                }

                // Schedule the E gap.
                let duration = match seg.e_independent {
                    Some(e_nurbs) => crate::e_independent::schedule_e_duration(
                        e_nurbs,
                        seg.feedrate_mm_s,
                        e_limits,
                    ),
                    None => 0.0, // Shouldn't happen per EMode invariants, but be safe.
                };

                let xyz_position = preceding_endpoint(segments, i);

                e_gaps.push(EGap {
                    segment_index: i,
                    duration,
                    xyz_position,
                });
            }
        }
    }

    // Close trailing run.
    if let Some(start) = run_start {
        runs.push(Run {
            segment_range: start..segments.len(),
        });
    }

    BatchPartition { runs, e_gaps }
}

/// Get the XYZ endpoint of the segment immediately before `index`, or
/// `[0, 0, 0]` if no preceding segment exists or the preceding segment is
/// also an E gap (in which case we walk backwards until we find an XY segment
/// or exhaust the list).
fn preceding_endpoint(segments: &[crate::ShapeSegmentInput<'_>], index: usize) -> [f64; 3] {
    // Walk backwards to find the most recent segment with a geometry curve.
    // All segments (including Independent) carry a `temporal.curve` reference.
    if index == 0 {
        return [0.0, 0.0, 0.0];
    }

    // The immediately preceding segment's curve endpoint is the hold position.
    // We use the preceding segment regardless of its EMode — the machine is at
    // that segment's endpoint when the E gap begins.
    let prev = &segments[index - 1];
    vector_eval(&prev.temporal.curve.as_view(), 1.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ELimits, ShapeSegmentInput};
    use geometry::segment::EMode;
    use nurbs::{ScalarNurbs, VectorNurbs};

    /// Build a simple cubic Bezier XYZ curve from `start` to `end` (straight line).
    fn make_xyz_curve(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
        // Cubic Bezier with collinear control points at 1/3 and 2/3 lerp.
        let cp1 = [
            start[0] + (end[0] - start[0]) / 3.0,
            start[1] + (end[1] - start[1]) / 3.0,
            start[2] + (end[2] - start[2]) / 3.0,
        ];
        let cp2 = [
            start[0] + 2.0 * (end[0] - start[0]) / 3.0,
            start[1] + 2.0 * (end[1] - start[1]) / 3.0,
            start[2] + 2.0 * (end[2] - start[2]) / 3.0,
        ];
        VectorNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![start, cp1, cp2, end],
            None,
        )
        .unwrap()
    }

    /// Build a linear E NURBS for retraction/prime.
    fn make_e_nurbs(e_start: f64, e_end: f64) -> ScalarNurbs<f64> {
        ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![e_start, e_end], None).unwrap()
    }

    fn default_limits() -> temporal::Limits {
        temporal::Limits::new(
            [500.0, 500.0, 500.0],
            [10_000.0, 10_000.0, 10_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            5_000.0,
        )
    }

    fn default_e_limits() -> ELimits {
        ELimits {
            v_max: 100.0,
            a_max: 5000.0,
        }
    }

    fn make_xy_segment(
        curve: &VectorNurbs<f64, 3>,
        e_mode: EMode,
        extrusion_per_xy_mm: f64,
    ) -> ShapeSegmentInput<'_> {
        ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode,
            extrusion_per_xy_mm,
            e_independent: None,
            feedrate_mm_s: 100.0,
        }
    }

    fn make_independent_segment<'a>(
        curve: &'a VectorNurbs<f64, 3>,
        e_nurbs: &'a ScalarNurbs<f64>,
    ) -> ShapeSegmentInput<'a> {
        ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Independent,
            extrusion_per_xy_mm: 0.0,
            e_independent: Some(e_nurbs),
            feedrate_mm_s: 50.0,
        }
    }

    #[test]
    fn partition_all_xy_single_run() {
        // All segments are CoupledToXy — should produce 1 run, 0 gaps.
        let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let c2 = make_xyz_curve([10.0, 0.0, 0.0], [20.0, 0.0, 0.0]);
        let c3 = make_xyz_curve([20.0, 0.0, 0.0], [30.0, 0.0, 0.0]);

        let segments = vec![
            make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
            make_xy_segment(&c2, EMode::CoupledToXy, 0.04),
            make_xy_segment(&c3, EMode::Travel, 0.0),
        ];
        let e_limits = default_e_limits();
        let result = partition_batch(&segments, &e_limits);

        assert_eq!(result.runs.len(), 1);
        assert_eq!(result.runs[0].segment_range, 0..3);
        assert_eq!(result.e_gaps.len(), 0);
    }

    #[test]
    fn partition_with_retraction() {
        // [CoupledToXy, Independent, CoupledToXy] -> 2 runs, 1 gap.
        let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 5.0, 0.0]);
        let c_hold = make_xyz_curve([10.0, 5.0, 0.0], [10.0, 5.0, 0.0]); // stationary
        let c2 = make_xyz_curve([10.0, 5.0, 0.0], [20.0, 5.0, 0.0]);
        let e_retract = make_e_nurbs(10.0, 5.0); // 5mm retraction

        let segments = vec![
            make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
            make_independent_segment(&c_hold, &e_retract),
            make_xy_segment(&c2, EMode::CoupledToXy, 0.04),
        ];
        let e_limits = default_e_limits();
        let result = partition_batch(&segments, &e_limits);

        assert_eq!(result.runs.len(), 2);
        assert_eq!(result.runs[0].segment_range, 0..1);
        assert_eq!(result.runs[1].segment_range, 2..3);
        assert_eq!(result.e_gaps.len(), 1);
        assert_eq!(result.e_gaps[0].segment_index, 1);
        assert!(result.e_gaps[0].duration > 0.0);
        // XYZ position should be the endpoint of segment 0: [10, 5, 0].
        let pos = result.e_gaps[0].xyz_position;
        assert!((pos[0] - 10.0).abs() < 1e-10);
        assert!((pos[1] - 5.0).abs() < 1e-10);
        assert!((pos[2] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn partition_leading_e() {
        // [Independent, CoupledToXy] -> 1 run, 1 gap (leading E before first run).
        let c_hold = make_xyz_curve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let e_prime = make_e_nurbs(5.0, 10.0); // 5mm prime

        let segments = vec![
            make_independent_segment(&c_hold, &e_prime),
            make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
        ];
        let e_limits = default_e_limits();
        let result = partition_batch(&segments, &e_limits);

        assert_eq!(result.runs.len(), 1);
        assert_eq!(result.runs[0].segment_range, 1..2);
        assert_eq!(result.e_gaps.len(), 1);
        assert_eq!(result.e_gaps[0].segment_index, 0);
        assert!(result.e_gaps[0].duration > 0.0);
        // No preceding segment — xyz_position should be [0, 0, 0].
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(result.e_gaps[0].xyz_position, [0.0, 0.0, 0.0]);
        }
    }

    #[test]
    fn partition_trailing_e() {
        // [CoupledToXy, Independent] -> 1 run, 1 gap (trailing E after last run).
        let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let c_hold = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let e_retract = make_e_nurbs(10.0, 5.0);

        let segments = vec![
            make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
            make_independent_segment(&c_hold, &e_retract),
        ];
        let e_limits = default_e_limits();
        let result = partition_batch(&segments, &e_limits);

        assert_eq!(result.runs.len(), 1);
        assert_eq!(result.runs[0].segment_range, 0..1);
        assert_eq!(result.e_gaps.len(), 1);
        assert_eq!(result.e_gaps[0].segment_index, 1);
        let pos = result.e_gaps[0].xyz_position;
        assert!((pos[0] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn partition_consecutive_e_gaps() {
        // [CoupledToXy, Independent, Independent, CoupledToXy]
        // -> 2 runs, 2 gaps (retraction then prime).
        let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let c_hold1 = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let c_hold2 = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let c2 = make_xyz_curve([10.0, 0.0, 0.0], [20.0, 0.0, 0.0]);
        let e_retract = make_e_nurbs(10.0, 5.0);
        let e_prime = make_e_nurbs(5.0, 10.0);

        let segments = vec![
            make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
            make_independent_segment(&c_hold1, &e_retract),
            make_independent_segment(&c_hold2, &e_prime),
            make_xy_segment(&c2, EMode::CoupledToXy, 0.04),
        ];
        let e_limits = default_e_limits();
        let result = partition_batch(&segments, &e_limits);

        assert_eq!(result.runs.len(), 2);
        assert_eq!(result.runs[0].segment_range, 0..1);
        assert_eq!(result.runs[1].segment_range, 3..4);
        assert_eq!(result.e_gaps.len(), 2);
        assert_eq!(result.e_gaps[0].segment_index, 1);
        assert_eq!(result.e_gaps[1].segment_index, 2);
    }

    #[test]
    fn partition_empty_input() {
        let result = partition_batch(&[], &default_e_limits());
        assert!(result.runs.is_empty());
        assert!(result.e_gaps.is_empty());
    }

    #[test]
    fn partition_all_independent() {
        // All E segments — no runs.
        let c_hold = make_xyz_curve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let e1 = make_e_nurbs(10.0, 5.0);
        let e2 = make_e_nurbs(5.0, 10.0);

        let segments = vec![
            make_independent_segment(&c_hold, &e1),
            make_independent_segment(&c_hold, &e2),
        ];
        let e_limits = default_e_limits();
        let result = partition_batch(&segments, &e_limits);

        assert!(result.runs.is_empty());
        assert_eq!(result.e_gaps.len(), 2);
    }

    #[test]
    fn e_gap_duration_matches_schedule() {
        // Verify the duration in the E gap matches what schedule_e_duration returns.
        let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let c_hold = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
        let e_retract = make_e_nurbs(10.0, 5.0);
        let e_limits = default_e_limits();

        let expected_dur = crate::e_independent::schedule_e_duration(&e_retract, 50.0, &e_limits);

        let segments = vec![
            make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
            make_independent_segment(&c_hold, &e_retract),
        ];
        let result = partition_batch(&segments, &e_limits);

        assert!(
            (result.e_gaps[0].duration - expected_dur).abs() < 1e-15,
            "expected {expected_dur}, got {}",
            result.e_gaps[0].duration
        );
    }
}
