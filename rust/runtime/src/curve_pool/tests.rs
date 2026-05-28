use super::*;

/// Build a minimal valid wire-piece array: `count` pieces, each with a
/// trivial Bernstein quartet `(0, 0.33, 0.66, 1)` and duration `1.0`.
fn wire_pieces(count: usize) -> Vec<WirePiece> {
    let bp0 = 0.0f32.to_bits();
    let bp1 = 0.333_333_3_f32.to_bits();
    let bp2 = 0.666_666_7_f32.to_bits();
    let bp3 = 1.0f32.to_bits();
    let dur = 1.0f32.to_bits();
    (0..count)
        .map(|_| WirePiece {
            bp0_bits: bp0,
            bp1_bits: bp1,
            bp2_bits: bp2,
            bp3_bits: bp3,
            duration_bits: dur,
        })
        .collect()
}

#[test]
fn fresh_pool_lookup_active_unloaded_returns_none() {
    let pool = CurvePool::new();
    // gen=1 against current_gen=0 must reject.
    assert!(pool.lookup_active(CurveHandle::new(0, 1)).is_none());
    assert!(pool.lookup_active(CurveHandle::new(15, 1)).is_none());
}

#[test]
fn out_of_bounds_handle_returns_none() {
    let pool = CurvePool::new();
    assert!(
        pool.lookup_active(CurveHandle::new(CURVE_POOL_N as u16, 1))
            .is_none()
    );
    assert!(pool.lookup_active(CurveHandle::new(u16::MAX, 1)).is_none());
}

#[test]
fn alloc_and_load_then_lookup_returns_ptr() {
    let pool = CurvePool::new();
    let wire = wire_pieces(4);
    let handle = pool.try_alloc_and_load(0, &wire).expect("alloc+load");
    assert_eq!(handle.slot_idx, 0);
    assert_eq!(handle.generation, 1);
    let ptr = pool.lookup_active(handle).expect("lookup_active");
    // SAFETY: pool is alive for the duration of the test; we are the
    // sole reader because we have not yet retired the slot.
    let piece_count = unsafe { (*ptr).piece_count };
    assert_eq!(piece_count, 4);
}

#[test]
fn alloc_twice_into_same_slot_blocks_until_retired() {
    let pool = CurvePool::new();
    let wire = wire_pieces(4);
    let h1 = pool.try_alloc_and_load(0, &wire).expect("first");
    // Second alloc into the same slot must fail (slot busy).
    assert!(pool.try_alloc_and_load(0, &wire).is_none());
    pool.confirm_retired(h1);
    let h2 = pool.try_alloc_and_load(0, &wire).expect("second");
    assert_eq!(h2.generation, 2);
}

#[test]
fn invalid_piece_data_rejected_without_bumping_gen() {
    let pool = CurvePool::new();
    let mut wire = wire_pieces(1);
    // Inject a NaN into the first Bernstein point.
    wire[0].bp0_bits = f32::NAN.to_bits();
    assert!(pool.try_alloc_and_load(0, &wire).is_none());
    // Generation must NOT have bumped — slot still free for retry.
    assert!(pool.is_slot_free(0));
}

#[test]
fn empty_wire_rejected() {
    let pool = CurvePool::new();
    let wire: Vec<WirePiece> = Vec::new();
    assert!(pool.try_alloc_and_load(0, &wire).is_none());
}

#[test]
fn pack_unpack_round_trips() {
    let h = CurveHandle::new(7, 0xCAFE);
    let packed = h.pack();
    assert_eq!(packed, (0xCAFE_u32 << 16) | 7);
    let h2 = CurveHandle::unpack(packed);
    assert_eq!(h, h2);
}
