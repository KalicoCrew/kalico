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
    let _ = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
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
    let _ = a.anchor_segment(0.0, 5.0, 100.0).unwrap();
    let result = a.anchor_segment(0.0, 1.0, 130.0);
    let (t0_new, fresh) = result.expect("backward jump must re-anchor, not error");
    assert!(fresh, "backward jump must be fresh");
    assert!(
        (t0_new - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}
