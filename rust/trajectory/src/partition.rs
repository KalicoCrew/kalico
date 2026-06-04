use geometry::segment::EMode;
use nurbs::eval::vector_eval;

#[derive(Debug)]
pub struct BatchPartition {
    pub runs: Vec<Run>,
    pub e_gaps: Vec<EGap>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Run {
    pub segment_range: std::ops::Range<usize>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EGap {
    pub segment_index: usize,
    pub duration: f64,
    pub xyz_position: [f64; 3],
}

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
                if run_start.is_none() {
                    run_start = Some(i);
                }
            }
            EMode::Independent => {
                if let Some(start) = run_start.take() {
                    runs.push(Run {
                        segment_range: start..i,
                    });
                }

                let duration = match seg.e_independent {
                    Some(e_nurbs) => crate::e_independent::schedule_e_duration(
                        e_nurbs,
                        seg.feedrate_mm_s,
                        e_limits,
                    ),
                    None => 0.0,
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

    if let Some(start) = run_start {
        runs.push(Run {
            segment_range: start..segments.len(),
        });
    }

    BatchPartition { runs, e_gaps }
}

fn preceding_endpoint(segments: &[crate::ShapeSegmentInput<'_>], index: usize) -> [f64; 3] {
    if index == 0 {
        return [0.0, 0.0, 0.0];
    }

    let prev = &segments[index - 1];
    vector_eval(&prev.temporal.curve.as_view(), 1.0)
}

#[cfg(test)]
mod tests;
