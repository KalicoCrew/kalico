//! Integration tests for `PieceEntry` — the 32-byte MCU ring-buffer entry.
//!
//! Validates:
//! 1. Size and alignment are exactly as the C ABI contract requires.
//! 2. `to_monomial` returns the correct coefficients for degenerate cases
//!    (constant and linear Bernstein polynomials).
//! 3. `end_time` arithmetic is correct against an H7 clock frequency.

use runtime::piece_ring::PieceEntry;

// ── Layout ────────────────────────────────────────────────────────────────────

#[test]
fn piece_entry_is_32_bytes() {
    assert_eq!(core::mem::size_of::<PieceEntry>(), 32);
}

#[test]
fn piece_entry_is_8_byte_aligned() {
    assert_eq!(core::mem::align_of::<PieceEntry>(), 8);
}

// ── to_monomial ───────────────────────────────────────────────────────────────

#[test]
fn piece_entry_to_monomial_constant() {
    // Bernstein [5.0, 5.0, 5.0, 5.0] with duration 0.001s = constant 5.0 mm.
    // After Bernstein → monomial: c0=5, c1=0, c2=0, c3=0.
    // Duration rescale divides by d^k (all zero coeffs stay zero).
    // Velocity coefficients are all zero.
    let entry = PieceEntry {
        start_time: 0,
        coeffs: [5.0, 5.0, 5.0, 5.0],
        duration: 0.001,
        _reserved: 0,
    };
    let (pos, vel) = entry.to_monomial();

    assert!(
        (pos[0] - 5.0).abs() < 1e-5,
        "constant c0 expected 5.0, got {}",
        pos[0]
    );
    for k in 1..4 {
        assert!(
            pos[k].abs() < 1e-5,
            "constant c{k} expected 0.0, got {}",
            pos[k]
        );
    }
    for (k, &v) in vel.iter().enumerate() {
        assert!(
            v.abs() < 1e-5,
            "constant vel_coeff[{k}] expected 0.0, got {v}"
        );
    }
}

#[test]
fn piece_entry_to_monomial_linear() {
    // Linear ramp from 0 to 1 mm over duration 0.01 s.
    // Unit-interval Bernstein for P(τ) = τ: [0, 1/3, 2/3, 1].
    // Unit-interval monomial: c0=0, c1=1, c2=0, c3=0.
    // Duration-rescaled: c1' = 1 / 0.01 = 100 mm/s.
    // Velocity: vc0 = c1' = 100, vc1 = 0, vc2 = 0.
    let entry = PieceEntry {
        start_time: 0,
        coeffs: [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0],
        duration: 0.01,
        _reserved: 0,
    };
    let (pos, vel) = entry.to_monomial();

    assert!(
        pos[0].abs() < 1e-5,
        "linear c0 expected 0.0, got {}",
        pos[0]
    );
    assert!(
        (pos[1] - 100.0).abs() < 1e-3,
        "linear c1 expected 100.0, got {}",
        pos[1]
    );
    assert!(
        pos[2].abs() < 1e-3,
        "linear c2 expected 0.0, got {}",
        pos[2]
    );
    assert!(
        pos[3].abs() < 1e-3,
        "linear c3 expected 0.0, got {}",
        pos[3]
    );

    // Velocity constant at 100 mm/s.
    assert!(
        (vel[0] - 100.0).abs() < 1e-3,
        "linear vel[0] expected 100.0, got {}",
        vel[0]
    );
    assert!(
        vel[1].abs() < 1e-3,
        "linear vel[1] expected 0.0, got {}",
        vel[1]
    );
    assert!(
        vel[2].abs() < 1e-3,
        "linear vel[2] expected 0.0, got {}",
        vel[2]
    );
}

// ── end_time ──────────────────────────────────────────────────────────────────

#[test]
fn piece_entry_end_time() {
    // start=1000, duration=0.001 s, clock_freq=550_000_000 Hz (H7 @ 550 MHz).
    // end = 1000 + (0.001 * 550_000_000) as u64 = 1000 + 550_000 = 551_000.
    let entry = PieceEntry {
        start_time: 1000,
        coeffs: [0.0; 4],
        duration: 0.001,
        _reserved: 0,
    };
    let end = entry.end_time(550_000_000.0_f32);
    assert_eq!(end, 551_000, "end_time mismatch: got {end}");
}

// ── PieceRing ─────────────────────────────────────────────────────────────────

use runtime::piece_ring::PieceRing;

fn make_piece(start: u64, duration: f32) -> PieceEntry {
    PieceEntry {
        start_time: start,
        coeffs: [0.0, 0.0, 0.0, 0.0],
        duration,
        _reserved: 0,
    }
}

fn make_storage<const N: usize>() -> [PieceEntry; N] {
    [PieceEntry {
        start_time: 0,
        coeffs: [0.0; 4],
        duration: 0.0,
        _reserved: 0,
    }; N]
}

#[test]
fn ring_new_empty() {
    let mut storage = make_storage::<8>();
    let ring = PieceRing::new(&mut storage);
    assert_eq!(ring.len(), 0);
    assert_eq!(ring.capacity(), 8);
    assert!(ring.is_empty());
    assert!(!ring.is_full());
}

#[test]
fn ring_push_and_peek() {
    let mut storage = make_storage::<8>();
    let mut ring = PieceRing::new(&mut storage);
    assert!(ring.push(make_piece(1000, 0.001)).is_ok());
    assert_eq!(ring.len(), 1);
    let front = ring.peek().unwrap();
    assert_eq!(front.start_time, 1000);
}

#[test]
fn ring_pop_advances_read() {
    let mut storage = make_storage::<8>();
    let mut ring = PieceRing::new(&mut storage);
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    assert_eq!(ring.len(), 2);
    ring.pop();
    assert_eq!(ring.len(), 1);
    assert_eq!(ring.peek().unwrap().start_time, 200);
}

#[test]
fn ring_full_rejects_push() {
    let mut storage = make_storage::<4>();
    let mut ring = PieceRing::new(&mut storage);
    for i in 0..4u64 {
        assert!(ring.push(make_piece(i * 100, 0.001)).is_ok());
    }
    assert!(ring.push(make_piece(400, 0.001)).is_err());
    assert_eq!(ring.len(), 4);
    assert!(ring.is_full());
}

#[test]
fn ring_wrap_around() {
    let mut storage = make_storage::<4>();
    let mut ring = PieceRing::new(&mut storage);
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    ring.pop();
    ring.pop();
    // head=2, tail=2, count=0. Now push 4 more (wraps around).
    for i in 0..4u64 {
        assert!(ring.push(make_piece((i + 3) * 100, 0.001)).is_ok());
    }
    assert_eq!(ring.len(), 4);
    assert_eq!(ring.peek().unwrap().start_time, 300);
}

#[test]
fn ring_consumed_count_monotonic() {
    let mut storage = make_storage::<4>();
    let mut ring = PieceRing::new(&mut storage);
    assert_eq!(ring.consumed_count(), 0);
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    ring.pop();
    assert_eq!(ring.consumed_count(), 1);
    ring.pop();
    assert_eq!(ring.consumed_count(), 2);
}

#[test]
fn ring_peek_empty_returns_none() {
    let mut storage = make_storage::<4>();
    let ring = PieceRing::new(&mut storage);
    assert!(ring.peek().is_none());
}

#[test]
fn ring_pop_empty_is_noop() {
    let mut storage = make_storage::<4>();
    let mut ring = PieceRing::new(&mut storage);
    ring.pop(); // should not panic
    assert_eq!(ring.consumed_count(), 0);
}
