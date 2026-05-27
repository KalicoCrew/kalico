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
mod tests;
