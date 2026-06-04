//! Shared host-time anchor mapping planner time → host time. One `T0` per
//! stream, re-established when the planner timeline jumps backward (a reset)
//! OR when an idle/stall gap means the segment would otherwise map to a host
//! time already in the past (real time outran the planner timeline). In both
//! cases the resumed stream is re-anchored `lead_secs` ahead of `host_now`.
//! See spec §3.2.1.

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
        Self {
            t0: None,
            last_t_end: 0.0,
            lead_secs: DEFAULT_LEAD_SECS,
        }
    }

    /// Map a segment to host time. `host_now` is the shared host clock now
    /// (seconds). Returns `T0` such that piece host time = `T0 + u_start`.
    ///
    /// Re-anchors (returns `fresh == true`) in two cases:
    ///
    /// (a) **Backward planner jump**: `seg_t_start` is not contiguous with the
    ///     previous segment's `t_end` — the planner timeline reset.
    ///
    /// (b) **Idle/stall gap**: real (host) time has outrun the planner timeline,
    ///     so `t0 + seg_t_start < host_now`. Keeping the stale `T0` would push
    ///     a piece whose `start_time` is behind `host_now`, causing the endpoint
    ///     to cold-adopt a stale piece → `-308 PieceStartInPast`. Re-anchoring
    ///     places the resumed stream `lead_secs` ahead of now instead.
    ///
    /// During continuous streaming, pieces land `~lead_secs` ahead of real
    /// time, so condition (b) is false and T0 is preserved unchanged.
    ///
    /// Returns `(t0, fresh_stream)`.
    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
    ) -> (f64, bool) {
        let fresh = match self.t0 {
            None => true,
            Some(t0) => {
                // (a) backward planner jump (timeline reset)
                (seg_t_start + CONTIGUITY_EPS < self.last_t_end)
                // (b) idle/stall gap: real time has outrun the planner timeline;
                // this segment would map to a host time already in the past.
                || (t0 + seg_t_start < host_now)
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
    fn idle_gap_reanchors() {
        // Move 1 finishes; T0 = 100.0 + 0.25 - 0.0 = 100.25.
        // After ~4 s idle, host_now = 104.0; move-2's seg_t_start = 1.0 is
        // contiguous (no backward jump), but t0 + seg_t_start = 100.25 + 1.0 =
        // 101.25 < 104.0 → condition (b) fires → re-anchor.
        let mut a = Anchor::new();
        let (_t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
        let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 104.0);
        assert!(fresh, "idle gap must trigger re-anchor");
        // New T0 = host_now + lead - seg_t_start = 104.0 + 0.25 - 1.0 = 103.25
        // so the piece at u_start=1.0 lands at t0_b + 1.0 = 104.25 = now+lead.
        let expected_t0 = 104.0 + DEFAULT_LEAD_SECS - 1.0;
        assert!(
            (t0_b - expected_t0).abs() < 1e-9,
            "t0_b={t0_b} expected={expected_t0}"
        );
        // Verify the piece start lands exactly lead_secs ahead of host_now.
        assert!((t0_b + 1.0 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
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
