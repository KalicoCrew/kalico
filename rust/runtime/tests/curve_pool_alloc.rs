//! Curve-pool alloc predicate tests. Spec §10.2 / §10.3 + Round-1 Codex #4
//! ordering invariant. Refactored for per-axis scalar curve pool (Step 7-B).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use runtime::curve_pool::{
    CURVE_POOL_N, CurveHandle, CurvePool, CurvePoolError, MAX_CONTROL_POINTS, MAX_DEGREE,
};

/// Clamped degree-1 linear scalar curve: 2 CPs, 4 knots.
fn linear_knots() -> [f32; 4] {
    [0.0, 0.0, 1.0, 1.0]
}
fn linear_cps() -> [f32; 2] {
    [0.0, 10.0]
}

#[test]
fn first_alloc_succeeds() {
    let pool = CurvePool::new();
    let h = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert_eq!(h.slot_idx, 0);
    assert_eq!(h.generation, 1); // bumped from 0 -> 1
}

#[test]
fn second_alloc_blocked_until_retired() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
            .is_none(),
        "second alloc should be blocked"
    );
    pool.confirm_retired(h1);
    let h2 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert_eq!(h2.generation, 2);
}

#[test]
fn lookup_validates_generation() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert!(pool.lookup(h1).is_ok());

    pool.confirm_retired(h1);
    let _h2 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    // Stale h1 must now reject (current_gen != h1.generation).
    assert!(pool.lookup(h1).is_err(), "stale handle must reject");
}

#[test]
fn wrap_u16_modulo_no_deadlock() {
    let pool = CurvePool::new();
    for _ in 0..65536 {
        let h = pool
            .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
            .unwrap();
        pool.confirm_retired(h);
    }
    // Slot is allocatable post-wrap.
    let h_post_wrap = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert_eq!(h_post_wrap.slot_idx, 0);
}

#[test]
fn out_of_range_slot_rejects() {
    let pool = CurvePool::new();
    assert!(pool
        .try_alloc_and_load(CURVE_POOL_N, 1, &linear_knots(), &linear_cps())
        .is_none());
    assert!(pool
        .lookup(CurveHandle::new(CURVE_POOL_N as u16, 1))
        .is_err());
}

#[test]
fn handle_pack_round_trips_through_wire() {
    let h = CurveHandle::new(7, 0xCAFE);
    assert_eq!(h.pack(), (0xCAFE_u32 << 16) | 7);
    assert_eq!(CurveHandle::unpack(h.pack()), h);
}

// --- Step 7-B scalar-specific tests ---

#[test]
fn load_degree1_linear_scalar_curve() {
    let pool = CurvePool::new();
    let cps = [0.0_f32, 10.0];
    let knots = [0.0_f32, 0.0, 1.0, 1.0];
    let h = pool.try_alloc_and_load(0, 1, &knots, &cps).unwrap();
    let view = pool.resolve(h).unwrap();
    assert_eq!(view.control_points, &cps[..]);
    assert_eq!(view.knots, &knots[..]);
    assert_eq!(view.degree, 1);
}

#[test]
fn load_degree9_curve_64_cps() {
    let pool = CurvePool::new();
    let n_cp = 64;
    let degree: u8 = 9;
    let cps: Vec<f32> = (0..n_cp).map(|i| i as f32).collect();
    let n_knots = n_cp + degree as usize + 1; // 74
    let mut knots = vec![0.0_f32; n_knots];
    // Clamped: first (degree+1) = 0.0, last (degree+1) = 1.0
    for k in &mut knots[..10] {
        *k = 0.0;
    }
    for k in &mut knots[n_knots - 10..] {
        *k = 1.0;
    }
    // Interior knots: monotone in (0, 1)
    let interior = n_knots - 20;
    for i in 0..interior {
        knots[10 + i] = (i + 1) as f32 / (interior + 1) as f32;
    }
    let h = pool.try_alloc_and_load(0, degree, &knots, &cps).unwrap();
    let view = pool.resolve(h).unwrap();
    assert_eq!(view.control_points.len(), n_cp);
    assert_eq!(view.knots.len(), n_knots);
    assert_eq!(view.degree, 9);
}

#[test]
fn load_degree10_curve_80_cps_at_limit() {
    let pool = CurvePool::new();
    let n_cp = MAX_CONTROL_POINTS; // 80
    let degree: u8 = MAX_DEGREE; // 10
    let cps: Vec<f32> = (0..n_cp).map(|i| i as f32).collect();
    let n_knots = n_cp + degree as usize + 1; // 91
    let mut knots = vec![0.0_f32; n_knots];
    let p1 = degree as usize + 1; // 11
    for k in &mut knots[..p1] {
        *k = 0.0;
    }
    for k in &mut knots[n_knots - p1..] {
        *k = 1.0;
    }
    let interior = n_knots - 2 * p1;
    for i in 0..interior {
        knots[p1 + i] = (i + 1) as f32 / (interior + 1) as f32;
    }
    let h = pool.try_alloc_and_load(0, degree, &knots, &cps).unwrap();
    let view = pool.resolve(h).unwrap();
    assert_eq!(view.control_points.len(), 80);
    assert_eq!(view.knots.len(), 91);
    assert_eq!(view.degree, 10);
}

#[test]
fn degree11_rejected() {
    let pool = CurvePool::new();
    // degree=11 exceeds MAX_DEGREE=10
    let cps: Vec<f32> = (0..12).map(|i| i as f32).collect();
    let knots = vec![0.0_f32; 12 + 11 + 1]; // 24 knots
    let result = pool.validate_and_load(0, 11, &knots, &cps);
    assert_eq!(result, Err(CurvePoolError::DegreeTooHigh));
}

#[test]
fn too_many_cps_rejected() {
    let pool = CurvePool::new();
    // 81 CPs exceeds MAX_CONTROL_POINTS=80
    let cps = vec![0.0_f32; 81];
    let knots = vec![0.0_f32; 81 + 1 + 1]; // degree=0, 83 knots
    let result = pool.validate_and_load(0, 0, &knots, &cps);
    assert_eq!(result, Err(CurvePoolError::InvalidLengths));
}

#[test]
fn resolve_returns_correct_scalar_data() {
    let pool = CurvePool::new();
    let cps = [1.0_f32, 2.0, 3.0, 4.0];
    let knots = [0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]; // degree-3
    let h = pool.validate_and_load(0, 3, &knots, &cps).unwrap();
    let view = pool.resolve(h).unwrap();
    assert_eq!(view.control_points, &[1.0, 2.0, 3.0, 4.0]);
    assert_eq!(view.knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    assert_eq!(view.degree, 3);
}

#[test]
fn unused_sentinel_is_detected() {
    assert!(CurveHandle::UNUSED_SENTINEL.is_unused_sentinel());
    // Regular handles and HOLD_SEGMENT_SENTINEL are NOT the unused sentinel.
    assert!(!CurveHandle::new(0, 0).is_unused_sentinel());
    assert!(!CurveHandle::HOLD_SEGMENT_SENTINEL.is_unused_sentinel());
}
