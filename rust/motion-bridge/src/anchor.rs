const CONTIGUITY_EPS: f64 = 1e-6;
const DEFAULT_LEAD_SECS: f64 = 0.25;

/// Returned when a segment arrives scheduled in the past: the planner failed to
/// stay ahead of the MCU playhead and silent re-anchoring would hide the defect.
#[derive(Debug, Clone, Copy)]
pub struct SegmentLate {
    /// The host-clock time at which the segment was scheduled to begin.
    pub scheduled_host: f64,
    /// The host-clock time at the moment of dispatch.
    pub host_now: f64,
    /// `host_now − scheduled_host` (always positive when `SegmentLate` is returned).
    pub gap_s: f64,
    /// The stream-relative start time of the late segment.
    pub seg_t_start: f64,
}

pub struct Anchor {
    t0: Option<f64>,
    last_t_end: f64,
    lead_secs: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
            lead_secs: DEFAULT_LEAD_SECS,
        }
    }

    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
    ) -> Result<(f64, bool), SegmentLate> {
        let reanchor = match self.t0 {
            // Case 1: stream start — silent re-anchor.
            None => true,
            Some(t0) => {
                let backward_jump = seg_t_start + CONTIGUITY_EPS < self.last_t_end;
                let starvation = t0 + seg_t_start < host_now;

                if starvation && !backward_jump {
                    // Case 3: segment scheduled in the past, not a deliberate reset.
                    let scheduled_host = t0 + seg_t_start;
                    let gap_s = host_now - scheduled_host;
                    return Err(SegmentLate {
                        scheduled_host,
                        host_now,
                        gap_s,
                        seg_t_start,
                    });
                }

                // Case 2: backward jump (deliberate timeline reset) — silent re-anchor,
                // even if the segment is also technically "late" by the old t0.
                backward_jump
            }
        };

        if reanchor {
            let condition = match self.t0 {
                None => "first",
                Some(_) => "backward-jump",
            };
            self.t0 = Some(host_now + self.lead_secs - seg_t_start);
            let t0 = self.t0.unwrap();
            tracing::info!(host_now, t0, seg_t_start, condition, "[anchor-decision]");
        }
        self.last_t_end = seg_t_end;
        Ok((self.t0.unwrap(), reanchor))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_segment_lands_lead_ahead() {
        let mut a = Anchor::new();
        let (t0, fresh) = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
        assert!(fresh);
        assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }

    #[test]
    fn contiguous_segment_keeps_t0() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
        let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 100.9).unwrap();
        assert!(!fresh);
        assert_eq!(t0_a, t0_b);
    }

    #[test]
    fn late_segment_returns_err_with_correct_gap() {
        let mut a = Anchor::new();
        // First segment anchors t0 = 100.0 + 0.25 - 0.0 = 100.25
        let _ = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
        // Next segment at stream t=1.0; scheduled host = 100.25 + 1.0 = 101.25
        // host_now = 104.0 → scheduled_host = 101.25, gap = 2.75
        let result = a.anchor_segment(1.0, 2.0, 104.0);
        let err = result.expect_err("starvation must return Err");
        assert!(err.gap_s > 0.0, "gap_s must be positive, got {}", err.gap_s);
        let expected_gap = 104.0 - (100.25 + 1.0);
        assert!(
            (err.gap_s - expected_gap).abs() < 1e-9,
            "gap_s={} expected={expected_gap}",
            err.gap_s
        );
        assert_eq!(err.seg_t_start, 1.0);
    }

    #[test]
    fn backward_jump_reanchors() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0).unwrap();
        let (t0_b, fresh) = a.anchor_segment(0.0, 1.0, 130.0).unwrap();
        assert!(fresh);
        assert_ne!(t0_a, t0_b);
        assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }

    #[test]
    fn backward_jump_while_late_reanchors_silently() {
        let mut a = Anchor::new();
        // Anchor at t0 = 100.25 for stream starting at 0.0, ending at 5.0.
        let _ = a.anchor_segment(0.0, 5.0, 100.0).unwrap();
        // Backward jump: seg_t_start=0.0 < last_t_end=5.0; also "late" under old t0.
        // Must re-anchor silently, not return Err.
        let result = a.anchor_segment(0.0, 1.0, 130.0);
        let (t0_new, fresh) = result.expect("backward jump must re-anchor, not error");
        assert!(fresh, "backward jump must be fresh");
        assert!(
            (t0_new - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
            "t0_new={t0_new}"
        );
    }
}
