const CONTIGUITY_EPS: f64 = 1e-6;
const DEFAULT_LEAD_SECS: f64 = 0.25;

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
    ) -> (f64, bool) {
        let fresh = match self.t0 {
            None => true,
            Some(t0) => {
                (seg_t_start + CONTIGUITY_EPS < self.last_t_end) || (t0 + seg_t_start < host_now)
            }
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
        assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }

    #[test]
    fn contiguous_segment_keeps_t0() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
        let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 100.9);
        assert!(!fresh);
        assert_eq!(t0_a, t0_b);
    }

    #[test]
    fn idle_gap_reanchors() {
        let mut a = Anchor::new();
        let (_t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
        let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 104.0);
        assert!(fresh, "idle gap must trigger re-anchor");
        let expected_t0 = 104.0 + DEFAULT_LEAD_SECS - 1.0;
        assert!(
            (t0_b - expected_t0).abs() < 1e-9,
            "t0_b={t0_b} expected={expected_t0}"
        );
        assert!((t0_b + 1.0 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }

    #[test]
    fn backward_jump_reanchors() {
        let mut a = Anchor::new();
        let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0);
        let (t0_b, fresh) = a.anchor_segment(0.0, 1.0, 130.0);
        assert!(fresh);
        assert_ne!(t0_a, t0_b);
        assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    }
}
