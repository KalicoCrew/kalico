//! Curve-pool alloc predicate tests. Spec §10.2 / §10.3 + Round-1 Codex #4
//! ordering invariant.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use runtime::curve_pool::{CURVE_POOL_N, CurveHandle, CurvePool, LoadedCurve};

fn dummy_curve() -> LoadedCurve {
    let mut c = LoadedCurve::empty();
    c.n_cp = 2;
    c.n_knots = 4;
    c.degree = 1;
    c
}

#[test]
fn first_alloc_succeeds() {
    let pool = CurvePool::new();
    let h = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h.slot_idx, 0);
    assert_eq!(h.generation, 1); // bumped from 0 -> 1
}

#[test]
fn second_alloc_blocked_until_retired() {
    let pool = CurvePool::new();
    let h1 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert!(
        pool.try_alloc_and_load(0, dummy_curve()).is_none(),
        "second alloc should be blocked"
    );
    pool.confirm_retired(h1);
    let h2 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h2.generation, 2);
}

#[test]
fn lookup_validates_generation() {
    let pool = CurvePool::new();
    let h1 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert!(pool.lookup(h1).is_ok());

    pool.confirm_retired(h1);
    let _h2 = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    // Stale h1 must now reject (current_gen != h1.generation).
    assert!(pool.lookup(h1).is_err(), "stale handle must reject");
}

#[test]
fn wrap_u16_modulo_no_deadlock() {
    let pool = CurvePool::new();
    // Force generation through wrap. Alloc + retire in sequence 65536 times
    // — current_gen wraps 0 → 65535 → 0 (last_retired_gen follows). The
    // alloc predicate `current_gen == last_retired_gen` keeps holding, so
    // there is no special wrap-cooldown machinery (Round-1 review fix
    // removed it).
    for _ in 0..65536 {
        let h = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
        pool.confirm_retired(h);
    }
    // Slot is allocatable post-wrap.
    let h_post_wrap = pool.try_alloc_and_load(0, dummy_curve()).unwrap();
    assert_eq!(h_post_wrap.slot_idx, 0);
}

#[test]
fn out_of_range_slot_rejects() {
    let pool = CurvePool::new();
    assert!(pool.try_alloc_and_load(CURVE_POOL_N, dummy_curve()).is_none());
    assert!(pool.lookup(CurveHandle::new(CURVE_POOL_N as u16, 1)).is_err());
}

#[test]
fn handle_pack_round_trips_through_wire() {
    // Wire schema (§5.3 push_segment, §5.1 load_curve_response): handle
    // travels as `(generation << 16) | slot_idx` u32. round-trip the pack /
    // unpack helpers through every issued handle.
    let h = CurveHandle::new(7, 0xCAFE);
    assert_eq!(h.pack(), (0xCAFE_u32 << 16) | 7);
    assert_eq!(CurveHandle::unpack(h.pack()), h);
}
