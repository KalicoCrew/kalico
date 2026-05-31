//! Shared host-time anchor mapping planner time → host time. One `T0` per
//! stream, re-established only when the planner timeline jumps backward
//! (a reset). See spec §3.2.1.

const CONTIGUITY_EPS: f64 = 1e-6; // seconds; planner timestamps compare to each other
const DEFAULT_LEAD_SECS: f64 = 0.25;

pub struct Anchor {
    /// Host-time instant (seconds) that planner t = 0 maps to. `None` until
    /// the first segment establishes it.
    t0: Option<f64>,
    /// Previous segment's planner t_end (seconds).
    last_t_end: f64,
    lead_secs: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self { t0: None, last_t_end: 0.0, lead_secs: DEFAULT_LEAD_SECS }
    }

    /// Map a segment to host time. `host_now` is the shared host clock now
    /// (seconds). Returns `T0` such that piece host time = `T0 + u_start`.
    /// Re-anchors when `seg_t_start` is not contiguous with the previous
    /// segment's `t_end` (a backward jump = fresh stream).
    ///
    /// Returns `(t0, fresh_stream)`.
    pub fn anchor_segment(&mut self, seg_t_start: f64, seg_t_end: f64, host_now: f64) -> (f64, bool) {
        let fresh = match self.t0 {
            None => true,
            Some(_) => seg_t_start + CONTIGUITY_EPS < self.last_t_end, // backward jump
        };
        if fresh {
            self.t0 = Some(host_now + self.lead_secs - seg_t_start);
        }
        self.last_t_end = seg_t_end;
        (self.t0.unwrap(), fresh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_segment_lands_lead_ahead() {
        let mut a = Anchor::new();
        let (t0, fresh) = a.anchor_segment(0.0, 1.0, 100.0);
        assert!(fresh);
        // piece at u_start=0 → host time t0 + 0 = now + lead
        assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }

    #[test]
    fn contiguous_segment_keeps_t0() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
        // next segment starts where the last ended → same T0, host_now advanced
        let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 100.9);
        assert!(!fresh);
        assert_eq!(t0_a, t0_b);
    }

    #[test]
    fn backward_jump_reanchors() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0);
        // timeline reset to ~0 after a long idle; host_now jumped way forward
        let (t0_b, fresh) = a.anchor_segment(0.0, 1.0, 130.0);
        assert!(fresh);
        assert_ne!(t0_a, t0_b);
        assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }
}
