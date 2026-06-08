use runtime::piece_ring::PieceEntry;

#[test]
fn piece_entry_is_32_bytes() {
    assert_eq!(core::mem::size_of::<PieceEntry>(), 32);
}

#[test]
fn piece_entry_is_8_byte_aligned() {
    assert_eq!(core::mem::align_of::<PieceEntry>(), 8);
}

#[test]
fn piece_entry_to_monomial_constant() {
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

#[test]
fn piece_entry_end_time() {
    let entry = PieceEntry {
        start_time: 1000,
        coeffs: [0.0; 4],
        duration: 0.001,
        _reserved: 0,
    };
    let end = entry.end_time(550_000_000.0_f32);
    assert_eq!(end, 551_000, "end_time mismatch: got {end}");
}

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
    for i in 0..4u64 {
        assert!(ring.push(make_piece((i + 3) * 100, 0.001)).is_ok());
    }
    assert_eq!(ring.len(), 4);
    assert_eq!(ring.peek().unwrap().start_time, 300);
}

#[test]
fn ring_retired_count_monotonic() {
    let mut storage = make_storage::<4>();
    let mut ring = PieceRing::new(&mut storage);
    assert_eq!(ring.retired_count(), 0);
    ring.push(make_piece(100, 0.001)).unwrap();
    ring.push(make_piece(200, 0.001)).unwrap();
    ring.pop();
    assert_eq!(ring.retired_count(), 1);
    ring.pop();
    assert_eq!(ring.retired_count(), 2);
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
    ring.pop();
    assert_eq!(ring.retired_count(), 0);
}

use runtime::piece_ring::RingDescriptor;

fn make_rd_storage<const N: usize>() -> [PieceEntry; N] {
    [PieceEntry {
        start_time: 0,
        coeffs: [0.0; 4],
        duration: 0.0,
        _reserved: 0,
    }; N]
}

fn pe(start: u64) -> PieceEntry {
    PieceEntry {
        start_time: start,
        coeffs: [0.0; 4],
        duration: 0.0,
        _reserved: 0,
    }
}

#[test]
fn write_slot_lands_at_absolute_index_without_advancing_head() {
    let mut storage = make_rd_storage::<8>();
    let ring = RingDescriptor::new(0, 8);
    ring.write_slot(&mut storage, 5, pe(1234));
    assert_eq!(storage[5].start_time, 1234);
    assert_eq!(ring.len(), 0);
    assert!(ring.is_empty());
}

#[test]
fn commit_head_makes_slots_visible_and_is_monotone() {
    let mut storage = make_rd_storage::<8>();
    let mut ring = RingDescriptor::new(0, 8);
    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    assert_eq!(ring.len(), 2);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 10);
    ring.commit_head(1); // stale re-send ignored
    assert_eq!(ring.len(), 2);
}

#[test]
fn advance_counter_retires_front_and_increments_retired() {
    let mut storage = make_rd_storage::<4>();
    let mut ring = RingDescriptor::new(0, 4);
    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    ring.advance_counter();
    assert_eq!(ring.retired_count(), 1);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 20);
    assert_eq!(ring.len(), 1);
}

#[test]
fn empty_full_distinct_via_monotonic_difference() {
    let mut storage = make_rd_storage::<2>();
    let mut ring = RingDescriptor::new(0, 2);
    assert!(ring.is_empty());
    ring.write_slot(&mut storage, 0, pe(1));
    ring.write_slot(&mut storage, 1, pe(2));
    ring.commit_head(2);
    assert!(ring.is_full());
    assert!(!ring.is_empty());
}

#[test]
fn rd_retired_cursor_wraps_after_depth_advances() {
    let mut storage = make_rd_storage::<2>();
    let mut ring = RingDescriptor::new(0, 2);

    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    assert_eq!(ring.len(), 2);

    ring.advance_counter();
    assert_eq!(ring.retired_count(), 1);
    assert_eq!(ring.tail, 1, "tail must be 1 after first advance");
    assert_eq!(ring.peek(&storage).unwrap().start_time, 20);

    ring.advance_counter();
    assert_eq!(ring.retired_count(), 2);
    assert_eq!(
        ring.tail, 0,
        "tail must wrap to 0 after ring_depth advances"
    );
    assert!(ring.is_empty());
    ring.write_slot(&mut storage, 0, pe(30));
    ring.write_slot(&mut storage, 1, pe(40));
    ring.commit_head(4);
    assert_eq!(ring.len(), 2);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 30);
}

#[test]
fn rd_commit_head_rejects_over_capacity_and_stale_behind_retired() {
    let mut storage = make_rd_storage::<4>();
    let mut ring = RingDescriptor::new(0, 4);
    ring.write_slot(&mut storage, 0, pe(1));
    ring.write_slot(&mut storage, 1, pe(2));
    ring.write_slot(&mut storage, 2, pe(3));
    ring.write_slot(&mut storage, 3, pe(4));

    ring.commit_head(3);
    assert_eq!(ring.len(), 3);

    ring.advance_counter();
    assert_eq!(ring.retired_count(), 1);
    assert_eq!(ring.len(), 2);

    let head_before = ring.head;
    ring.commit_head(6);
    assert_eq!(
        ring.head, head_before,
        "over-capacity commit_head must be rejected"
    );

    ring.commit_head(0);
    assert_eq!(
        ring.head, head_before,
        "behind-retired commit_head must be rejected"
    );

    ring.write_slot(&mut storage, ring.head as usize % 4, pe(50));
    ring.write_slot(&mut storage, (ring.head as usize + 1) % 4, pe(60));
    ring.commit_head(5);
    assert_eq!(
        ring.len(),
        4,
        "commit to exactly ring_depth occupancy must be accepted"
    );
}
