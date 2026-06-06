use super::*;

fn ease_entry(from: f32, to: f32, start_time_ns: u64, dur_s: f32) -> PieceEntry {
    PieceEntry {
        start_time: start_time_ns,
        coeffs: [from, from, to, to],
        duration: dur_s,
        _reserved: 0,
    }
}

#[test]
fn ring_walk_eval_single_piece() {
    let mut ring = AxisRing::new();
    let t0: u64 = 1_000_000_000;
    let dur: f32 = 1.0;

    ring.push_entry(ease_entry(0.0, 10.0, t0, dur)).unwrap();

    let (pos_start, _vel) = ring.sample(t0).unwrap();
    assert!((pos_start - 0.0).abs() < 1e-3, "start pos={pos_start}");

    let (pos_mid, _vel) = ring.sample(t0 + 500_000_000).unwrap();
    assert!((pos_mid - 5.0).abs() < 0.2, "mid pos={pos_mid}");

    let pos_after = ring.sample(t0 + 2_000_000_000);
    assert!(
        pos_after.is_none(),
        "ring must be empty after piece expires"
    );
    assert_eq!(ring.retired_count(), 1);
}

#[test]
fn ring_walk_two_pieces_contiguous() {
    let mut ring = AxisRing::new();

    let t0: u64 = 1_000_000_000;
    let dur_ns: u64 = 1_000_000;
    let dur_s: f32 = 0.001;

    ring.push_entry(ease_entry(0.0, 10.0, t0, dur_s)).unwrap();
    ring.push_entry(ease_entry(10.0, 0.0, t0 + dur_ns, dur_s))
        .unwrap();

    let (pos_start, _) = ring.sample(t0).unwrap();
    assert!((pos_start - 0.0).abs() < 1e-3, "start pos={pos_start}");
    assert_eq!(ring.retired_count(), 0);

    let (pos_boundary, _) = ring.sample(t0 + dur_ns).unwrap();
    assert!(
        (pos_boundary - 10.0).abs() < 0.1,
        "pos at piece0 end={pos_boundary}"
    );
    assert_eq!(ring.retired_count(), 1, "piece 0 retired at boundary");

    let (pos_p1_near_end, _) = ring.sample(t0 + 2 * dur_ns - 1).unwrap();
    assert!(
        (pos_p1_near_end - 0.0).abs() < 0.1,
        "piece1 near-end pos={pos_p1_near_end}"
    );

    let pos_gone = ring.sample(t0 + 2 * dur_ns);
    assert!(pos_gone.is_none(), "ring must be empty at piece 1 end_time");
    assert_eq!(ring.retired_count(), 2);
}

#[test]
fn push_from_bytes_round_trips() {
    let entry = ease_entry(0.0, 5.0, 500_000_000, 0.5);
    let bytes = entry.to_le_bytes();
    let mut all_bytes = bytes.to_vec();
    all_bytes.extend_from_slice(&bytes); // two identical pieces

    let mut ring = AxisRing::new();
    let pushed = ring.push_from_bytes(2, &all_bytes);
    assert_eq!(pushed, 2);
    assert_eq!(ring.desc.len(), 2);
}

#[test]
fn retired_count_heartbeat() {
    let mut ring = AxisRing::new();
    assert_eq!(ring.retired_count(), 0);

    ring.push_entry(ease_entry(0.0, 1.0, 0, 0.001)).unwrap();
    ring.sample(0);
    assert_eq!(ring.retired_count(), 0, "not yet retired");

    let mut ring2 = AxisRing::new();
    ring2.push_entry(ease_entry(0.0, 1.0, 0, 0.001)).unwrap();
    ring2.sample(0);
    ring2.sample(500_000);
    assert_eq!(ring2.retired_count(), 0, "not yet retired at 0.5ms");
    ring2.sample(2_000_000);
    assert_eq!(ring2.retired_count(), 1, "should be retired at 2ms");
}

#[test]
fn reset_clears_ring() {
    let mut ring = AxisRing::new();
    ring.push_entry(ease_entry(0.0, 1.0, 0, 1.0)).unwrap();
    ring.push_entry(ease_entry(1.0, 0.0, 1_000_000_000, 1.0))
        .unwrap();
    ring.reset();
    assert!(ring.is_empty());
    assert!(ring.armed.is_none());
    assert_eq!(
        ring.take_fault(),
        None,
        "reset must clear the fault register"
    );
}

#[test]
fn ethercat_fault_latches() {
    let mut ring = AxisRing::new();

    assert_eq!(ring.take_fault(), None, "no fault on empty ring");

    let piece_start_ns: u64 = 0;
    let sample_now_ns: u64 = 20_000_000;
    ring.push_entry(ease_entry(0.0, 1.0, piece_start_ns, 100.0))
        .unwrap();

    let result = ring.sample(sample_now_ns);
    assert!(result.is_none(), "PieceStartInPast must return None");

    let fault_val = ring.take_fault().expect("fault must be latched");

    let code_u16 = (fault_val & 0xFFFF) as u16;
    #[allow(clippy::cast_sign_loss)]
    let expected_code = (-308_i32 as i16) as u16;
    assert_eq!(
        code_u16, expected_code,
        "fault code must be PieceStartInPast wire value"
    );

    let deficit_us_hi = (fault_val >> 16) as u16;
    assert_eq!(
        deficit_us_hi, 20_000_u16,
        "fault high 16 bits must be deficit_us=20000 (0x4E20)"
    );

    assert_eq!(
        ring.take_fault(),
        None,
        "fault register must be cleared after take"
    );

    assert_eq!(ring.retired_count(), 0, "fault must not retire the piece");
}

#[test]
fn no_jump_at_origin_capture() {
    use crate::scale::CountMap;

    let mut ring = AxisRing::new();
    let t0: u64 = 5_000_000_000;
    let dur_s: f32 = 0.001;
    let pos_mm = 5.0_f32;

    ring.push_entry(PieceEntry {
        start_time: t0,
        coeffs: [pos_mm; 4],
        duration: dur_s,
        _reserved: 0,
    })
    .unwrap();

    let (sampled_pos, _vel) = ring.sample(t0).expect("sample at t0 must return Some");
    assert!(
        (sampled_pos - pos_mm).abs() < 1e-4_f32,
        "sample at t0 must return b0={pos_mm:.4}, got {sampled_pos:.6}"
    );

    let counts_per_mm = 3276.8_f64;
    let actual_counts = 20_000_i32;
    let cmap = CountMap::new(counts_per_mm, actual_counts, f64::from(sampled_pos));

    assert_eq!(
        cmap.target_counts(f64::from(sampled_pos)),
        actual_counts,
        "CountMap origin capture must produce target_counts == actual_counts (no startup jump)"
    );
}

#[test]
fn piece_boundary_c0_c1_continuity() {
    let mut ring = AxisRing::new();

    let t0: u64 = 2_000_000_000;
    let dur_ns: u64 = 1_000_000;
    let dur_s: f32 = 0.001_f32;
    let boundary_ns: u64 = t0 + dur_ns;

    // De Casteljau midpoint split of the linear ramp 0→10 mm over 2ms.
    let b0_piece0: [f32; 4] = [0.0, 5.0 / 3.0, 10.0 / 3.0, 5.0];
    let b0_piece1: [f32; 4] = [5.0, 5.0 + 5.0 / 3.0, 5.0 + 10.0 / 3.0, 10.0];

    ring.push_entry(PieceEntry {
        start_time: t0,
        coeffs: b0_piece0,
        duration: dur_s,
        _reserved: 0,
    })
    .unwrap();
    ring.push_entry(PieceEntry {
        start_time: boundary_ns,
        coeffs: b0_piece1,
        duration: dur_s,
        _reserved: 0,
    })
    .unwrap();

    let (pos_before, vel_before) = ring
        .sample(boundary_ns - 1)
        .expect("sample before boundary must return Some");
    assert_eq!(
        ring.take_fault(),
        None,
        "no fault expected for in-window piece 0 sample"
    );

    let (pos_after, vel_after) = ring
        .sample(boundary_ns + 1)
        .expect("sample after boundary must return Some");
    assert_eq!(
        ring.take_fault(),
        None,
        "no fault expected for in-window piece 1 sample"
    );

    let pos_gap = (pos_after - pos_before).abs();
    assert!(
        pos_gap < 0.01_f32,
        "C0 continuity violated: |pos_after({pos_after:.6}) - pos_before({pos_before:.6})| \
         = {pos_gap:.6} >= 0.01 mm"
    );

    let vel_gap = (vel_after - vel_before).abs();
    assert!(
        vel_gap < 1.0_f32,
        "C1 continuity violated: |vel_after({vel_after:.3}) - vel_before({vel_before:.3})| \
         = {vel_gap:.3} >= 1.0 mm/s"
    );

    assert!(
        vel_before > 0.0_f32,
        "vel_before={vel_before:.3} must be positive (monotone-increasing ramp)"
    );
    assert!(
        vel_after > 0.0_f32,
        "vel_after={vel_after:.3} must be positive (monotone-increasing ramp)"
    );

    assert!(
        (vel_before - 5000.0_f32).abs() < 250.0_f32,
        "vel_before={vel_before:.1} should be ~5000 mm/s for a linear ramp"
    );
    assert!(
        (vel_after - 5000.0_f32).abs() < 250.0_f32,
        "vel_after={vel_after:.1} should be ~5000 mm/s for a linear ramp"
    );
}

#[test]
fn fault_boundary_exact() {
    const MAX_START_IN_PAST_SECS: f32 = 200e-6;
    let drift_budget = (MAX_START_IN_PAST_SECS * CLOCK_FREQ_HZ) as u64;
    let fault_tolerance_ns = drift_budget + u64::from(EC_DC_PERIOD_NS);

    let now_a: u64 = 10_000_000_000;
    let start_a: u64 = now_a - fault_tolerance_ns;

    let mut ring_a = AxisRing::new();
    ring_a
        .push_entry(PieceEntry {
            start_time: start_a,
            coeffs: [0.0_f32; 4],
            duration: 10.0_f32,
            _reserved: 0,
        })
        .unwrap();

    let result_a = ring_a.sample(now_a);
    assert!(
        result_a.is_some(),
        "gap == fault_tolerance ({fault_tolerance_ns} ns) must NOT fault (strictly-greater-than); got None"
    );
    assert_eq!(
        ring_a.take_fault(),
        None,
        "no fault must be latched when gap == tolerance"
    );

    let start_b: u64 = now_a - fault_tolerance_ns - 1;

    let mut ring_b = AxisRing::new();
    ring_b
        .push_entry(PieceEntry {
            start_time: start_b,
            coeffs: [0.0_f32; 4],
            duration: 10.0_f32,
            _reserved: 0,
        })
        .unwrap();

    let result_b = ring_b.sample(now_a);
    assert!(
        result_b.is_none(),
        "gap == fault_tolerance + 1 must fault and return None"
    );

    let fault_val = ring_b
        .take_fault()
        .expect("fault register must be latched when gap > fault_tolerance");

    #[allow(clippy::cast_sign_loss)]
    let expected_code = (-308_i32 as i16) as u16;
    let code_u16 = (fault_val & 0xFFFF) as u16;
    assert_eq!(
        code_u16, expected_code,
        "fault register low 16 bits must be 0xFECC (−308)"
    );

    let deficit_us_hi = (fault_val >> 16) as u16;
    assert_eq!(
        deficit_us_hi, 1200_u16,
        "fault register high 16 bits must be 1200 µs (deficit at tolerance+1 boundary)"
    );
}

#[test]
fn end_time_ns_precision() {
    use runtime::piece_ring::PieceEntry;

    let start: u64 = 7_000_000_000;
    let entry = PieceEntry {
        start_time: start,
        coeffs: [0.0_f32; 4],
        duration: 0.001_f32, // 1 ms
        _reserved: 0,
    };

    let end = entry.end_time(CLOCK_FREQ_HZ);
    assert_eq!(
        end,
        start + 1_000_000,
        "end_time for 1ms piece must be start + 1_000_000 ns exactly; \
         got {} (delta={})",
        end,
        end.wrapping_sub(start)
    );
}

#[test]
fn fault_then_in_window_recovers() {
    let mut ring = AxisRing::new();

    let now_ns: u64 = 20_000_000;
    let start_ns: u64 = 0;
    ring.push_entry(ease_entry(0.0, 1.0, start_ns, 0.1))
        .unwrap();

    let r1 = ring.sample(now_ns);
    assert!(r1.is_none(), "stale unarm must return None");
    assert_eq!(ring.retired_count(), 0, "no retirement on fault");
    let fault1 = ring.take_fault().expect("fault must be latched");
    assert_ne!(fault1, FAULT_REG_NONE, "fault register must be non-zero");

    ring.reset();
    assert_eq!(ring.take_fault(), None, "reset clears fault register");

    let now2: u64 = 1_000_000_000;
    ring.push_entry(ease_entry(0.0, 1.0, now2, 0.1)).unwrap();
    let r2 = ring.sample(now2);
    assert!(r2.is_some(), "in-window piece must return Some after reset");
    assert_eq!(ring.take_fault(), None, "no fault for in-window piece");
}
