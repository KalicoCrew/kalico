use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, EC_DC_PERIOD_NS};
use runtime::piece_ring::PieceEntry;

fn ramp_piece(from_mm: f32, to_mm: f32, start_ns: u64) -> PieceEntry {
    let d = to_mm - from_mm;
    PieceEntry {
        start_time: start_ns,
        coeffs: [from_mm, from_mm + d / 3.0, from_mm + 2.0 * d / 3.0, to_mm],
        duration: 0.001_f32,
        _reserved: 0,
    }
}

#[test]
fn sustained_streaming_past_ring_depth() {
    const TOTAL: usize = AXIS_RING_CAPACITY + 20;
    const PIECE_DUR_NS: u64 = EC_DC_PERIOD_NS as u64;
    const BASE_NS: u64 = 10_000_000_000_u64;

    let mm_per_piece = 1.0_f32 / TOTAL as f32;
    let pieces: Vec<PieceEntry> = (0..TOTAL)
        .map(|i| {
            let from = i as f32 * mm_per_piece;
            let to = (i + 1) as f32 * mm_per_piece;
            let start_ns = BASE_NS + i as u64 * PIECE_DUR_NS;
            ramp_piece(from, to, start_ns)
        })
        .collect();

    let mut ring = AxisRing::new();

    let mut next_to_push: usize = 0;
    let mut last_retired: u32 = 0;

    while next_to_push < TOTAL {
        if ring.push_entry(pieces[next_to_push]).is_err() {
            break;
        }
        next_to_push += 1;
    }

    let max_iterations = TOTAL * 4 + AXIS_RING_CAPACITY * 4;
    let mut now = BASE_NS;
    let mut iterations = 0usize;

    loop {
        assert!(
            iterations < max_iterations,
            "streaming stalled: retired_count={}/{} next_to_push={}/{} after {} iterations",
            ring.retired_count(),
            TOTAL,
            next_to_push,
            TOTAL,
            iterations
        );
        iterations += 1;

        let _sample = ring.sample(now);

        assert_eq!(
            ring.take_fault(),
            None,
            "spurious PieceStartInPast fault at iteration {iterations}, now={now}, \
             retired={}/{}",
            ring.retired_count(),
            TOTAL
        );

        now = now.saturating_add(PIECE_DUR_NS);

        let current_retired = ring.retired_count();
        if current_retired != last_retired {
            last_retired = current_retired;
            while next_to_push < TOTAL {
                if ring.push_entry(pieces[next_to_push]).is_err() {
                    break;
                }
                next_to_push += 1;
            }
        }

        if next_to_push == TOTAL && ring.retired_count() == TOTAL as u32 {
            break;
        }
    }

    assert_eq!(
        ring.retired_count(),
        TOTAL as u32,
        "all {} pieces must be retired; got {}",
        TOTAL,
        ring.retired_count()
    );
    assert_eq!(
        ring.take_fault(),
        None,
        "no fault must remain latched after full stream"
    );
    assert!(
        ring.is_empty(),
        "ring must be empty after all pieces retire"
    );
}
