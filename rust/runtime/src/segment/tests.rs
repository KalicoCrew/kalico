use super::*;

#[test]
fn segment_size_is_under_64_bytes() {
    // Spec §4.7 / §3.1: small POD to minimize SPSC enqueue/dequeue memcpy cost.
    assert!(
        core::mem::size_of::<Segment>() <= 64,
        "Segment grew too large: {} bytes",
        core::mem::size_of::<Segment>()
    );
}

#[test]
fn segment_size_under_64_bytes_with_consumers_mask() {
    // Step-7-B base layout (48 B) + consumers_remaining (u16) =
    // ≤ 56 B after natural alignment. The repr(C) field ordering plus
    // tail-padding to 8-byte alignment lands the actual size on the
    // host build at 56 B. The looser ≤ 64 B assertion in
    // [`segment_size_is_under_64_bytes`] stays load-bearing for the
    // SPSC enqueue/dequeue memcpy budget; this exact-size assertion
    // is the canary for accidental field-order regressions.
    assert_eq!(core::mem::size_of::<Segment>(), 56);
}

#[test]
fn segment_duration_returns_t_end_minus_t_start() {
    let seg = Segment {
        id: 1,
        x_handle: CurveHandle::new(0, 1),
        y_handle: CurveHandle::new(1, 1),
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 100,
        t_end: 350,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::CoupledToXy,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    assert_eq!(seg.duration(), 250);
}

#[test]
fn motor_has_remaining_work_cartesian() {
    let mut seg = Segment {
        id: 1,
        x_handle: CurveHandle::new(0, 1),
        y_handle: CurveHandle::new(1, 1),
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::new(3, 1),
        t_start: 0,
        t_end: 1000,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    seg.consumers_remaining = Segment::compute_consumers_remaining(
        seg.kinematics,
        seg.x_handle,
        seg.y_handle,
        seg.z_handle,
        seg.e_handle,
    );

    // Cartesian: motor 0 ← X, 1 ← Y, 2 ← Z, 3 ← E. Z is UNUSED so
    // motor 2 has no work even before any clearing.
    assert!(seg.motor_has_remaining_work(0));
    assert!(seg.motor_has_remaining_work(1));
    assert!(!seg.motor_has_remaining_work(2)); // Z is UNUSED
    assert!(seg.motor_has_remaining_work(3));

    // Clear motor 0's X bit; motor 0 no longer has work.
    seg.consumers_remaining &= !(1u16 << CONS_REMAINING_X_SHIFT);
    assert!(!seg.motor_has_remaining_work(0));
    assert!(seg.motor_has_remaining_work(1));
    assert!(seg.motor_has_remaining_work(3));

    // Clear motor 1, motor 3. Now no motor has work; consumers_done.
    seg.consumers_remaining &= !(1u16 << (CONS_REMAINING_Y_SHIFT + 1));
    seg.consumers_remaining &= !(1u16 << (CONS_REMAINING_E_SHIFT + 3));
    assert!(!seg.motor_has_remaining_work(0));
    assert!(!seg.motor_has_remaining_work(1));
    assert!(!seg.motor_has_remaining_work(3));
    assert!(seg.consumers_done());
}

// motor_has_remaining_work_corexy was deleted (2026-05-21): the old test
// asserted the CoreXY cross-coupling semantics (motors 0 and 1 both
// consuming X AND Y). After the bridge-side transform fix, `Segment`
// curves are always motor-frame, so the consumer mapping is Cartesian
// (motor i reads slot i) regardless of kinematics tag. The
// motor_has_remaining_work_cartesian test above covers the same code path.

/// Lock the new "KinematicTag does not alter consumer mask" semantic.
///
/// Before the round-2 bridge-side CoreXY transform fix, `compute_consumers_remaining`
/// branched on `kinematics` and produced different bitmasks for `CoreXyAndE`
/// vs `CartesianXyzAndE`. After the fix, both tags yield the same Cartesian-
/// identity mask for a given set of handles. This test pins that invariant
/// so a future refactor cannot silently reintroduce the old branchy code.
#[test]
fn consumer_mask_is_identical_regardless_of_kinematic_tag() {
    let x = CurveHandle::new(0, 1);
    let y = CurveHandle::new(1, 1);
    let z = CurveHandle::new(2, 1);
    let e = CurveHandle::new(3, 1);

    let mask_cartesian = Segment::compute_consumers_remaining(
        KinematicTag::CartesianXyzAndE,
        x, y, z, e,
    );
    let mask_corexy = Segment::compute_consumers_remaining(
        KinematicTag::CoreXyAndE,
        x, y, z, e,
    );

    assert_eq!(
        mask_cartesian, mask_corexy,
        "consumer mask must be identical for CartesianXyzAndE ({mask_cartesian:#06x}) \
         and CoreXyAndE ({mask_corexy:#06x}) — the KinematicTag no longer alters the \
         motor-frame consumer mapping (bridge applies transform before segment construction)",
    );

    // Also verify motor_has_remaining_work agrees for each motor index.
    for motor in 0_u8..4 {
        let seg_cart = Segment {
            id: 0,
            x_handle: x,
            y_handle: y,
            z_handle: z,
            e_handle: e,
            t_start: 0,
            t_end: 1000,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: mask_cartesian,
        };
        let seg_corexy = Segment {
            kinematics: KinematicTag::CoreXyAndE,
            consumers_remaining: mask_corexy,
            ..seg_cart
        };
        assert_eq!(
            seg_cart.motor_has_remaining_work(motor),
            seg_corexy.motor_has_remaining_work(motor),
            "motor_has_remaining_work({motor}) differs between kinematic tags — \
             CoreXy tag must produce the same result as Cartesian",
        );
    }
}

#[test]
fn segment_is_copy_clone() {
    let seg = Segment {
        id: 0,
        x_handle: CurveHandle::new(0, 1),
        y_handle: CurveHandle::new(1, 1),
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 100,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::CoupledToXy,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    let _ = seg; // copy
    // Verify Clone derive exists; suppress lint since Copy is also derived.
    #[allow(clippy::clone_on_copy)]
    let _ = seg.clone();
}
