use super::*;

/// Build a queue from (tick_start, host_time) pairs.
fn q_with_host(ring_depth: u32, starts: &[(u64, f64)]) -> AxisQueue {
    let mut q = AxisQueue::new(ring_depth);
    for &(s, h) in starts {
        q.pieces.push_back((
            PieceEntry {
                start_time: s,
                coeffs: [0.0; 4],
                duration: 0.001,
                _reserved: 0,
            },
            h,
        ));
    }
    q
}

/// Convenience wrapper for single-MCU tests where host_time == tick as f64.
fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
    let pairs: Vec<(u64, f64)> = starts.iter().map(|&s| (s, s as f64)).collect();
    q_with_host(ring_depth, &pairs)
}

/// No-cohort cap: every axis is uncapped.
fn no_cap(_: &AxisKey) -> usize {
    usize::MAX
}

#[test]
fn idle_when_empty() {
    let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    assert!(matches!(
        schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| None, no_cap),
        Schedule::Idle
    ));
}

#[test]
fn stalls_when_global_head_ring_full() {
    let mut queues = BTreeMap::new();
    let mut a = q_with(2, &[10]);
    a.pushed = 2; // full
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
    assert!(matches!(
        schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| None, no_cap),
        Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
    ));
}

#[test]
fn batches_contiguous_same_mcu_prefix_only() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
    let s = schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| None, no_cap);
    match s {
        Schedule::Send(frames) => {
            let ax: Vec<_> = frames.iter().map(|f| (f.key, f.pieces.len())).collect();
            assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 0 }, 1)));
            assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 1 }, 1)));
            assert!(!ax.iter().any(|(k, _)| k.mcu_id == 2));
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn frame_cap_splits() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
    let s = schedule(&queues, 2, |_: &AxisKey, _: &AxisQueue| None, no_cap);
    match s {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 2);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn full_axis_does_not_block_same_mcu_sibling() {
    let mut q: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    let yq = q_with(8, &[0, 2]);
    let mut xq = q_with(1, &[1]);
    xq.pushed = 1;
    q.insert(AxisKey { mcu_id: 1, axis: 1 }, yq);
    q.insert(AxisKey { mcu_id: 1, axis: 0 }, xq);
    match schedule(&q, 255, |_: &AxisKey, _: &AxisQueue| None, no_cap) {
        Schedule::Send(frames) => {
            let yf = frames
                .iter()
                .find(|f| f.key == AxisKey { mcu_id: 1, axis: 1 });
            assert!(yf.is_some(), "Y should be batched despite full sibling X");
            assert!(
                !frames
                    .iter()
                    .any(|f| f.key == AxisKey { mcu_id: 1, axis: 0 }),
                "full X must not appear in the batch"
            );
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn time_gate_blocks_piece_beyond_horizon() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[100]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[200]));
    match schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| Some(150), no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1, "only axis 0 should be batched");
            assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
            assert_eq!(frames[0].pieces.len(), 1);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn all_beyond_horizon_returns_stall_ahead() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[1000]));
    assert!(
        matches!(
            schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| Some(500), no_cap),
            Schedule::StallAhead(AxisKey { mcu_id: 1, axis: 0 })
        ),
        "expected StallAhead when sole piece is beyond horizon"
    );
}

#[test]
fn no_horizon_none_uses_count_only_gate() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[u64::MAX]));
    match schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| None, no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 1);
        }
        other => panic!("expected Send (no time gate), got {other:?}"),
    }
}

/// Regression: bench shape from 2026-06-04.
///
/// mcu1 (F446, 180 MHz) has a far-future piece whose tick value is numerically
/// small (~4.8e12) but whose host time is large (beyond mcu1's horizon).
/// mcu0 (H7, 520 MHz) has past-due pieces whose tick value is numerically large
/// (~13.8e12) but whose host time is now-ish (within mcu0's horizon, room available).
///
/// Old code: `min_by` on raw ticks → mcu1's small ticks win → StallAhead(mcu1),
/// H7 starved for up to 2+ seconds.
/// New code: `min_by` on host time → mcu0's smaller host time wins → Send(mcu0).
#[test]
fn cross_mcu_host_time_ordering_bench_regression() {
    let f446_tick: u64 = 4_790_000_000_000;
    let h7_tick: u64 = 13_800_000_000_000;

    // mcu1 (F446): numerically smaller tick, but host time is far in the future
    let f446_host: f64 = 1_000.0; // far future, beyond any horizon
    // mcu0 (H7): numerically larger tick, but host time is now-ish
    let h7_host: f64 = 1.0; // near present

    let mut queues = BTreeMap::new();
    queues.insert(
        AxisKey { mcu_id: 1, axis: 2 }, // F446, Z axis
        q_with_host(8, &[(f446_tick, f446_host)]),
    );
    queues.insert(
        AxisKey { mcu_id: 0, axis: 0 }, // H7, X axis
        q_with_host(8, &[(h7_tick, h7_host)]),
    );

    // mcu0 horizon covers h7_tick; mcu1 horizon does NOT cover f446_tick.
    let horizon_of = |k: &AxisKey, _q: &AxisQueue| -> Option<u64> {
        if k.mcu_id == 0 {
            Some(h7_tick + 1_000_000)
        } else {
            Some(f446_tick - 1) // mcu1 piece is ahead of this horizon
        }
    };

    match schedule(&queues, 255, horizon_of, no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].key.mcu_id, 0,
                "H7 (mcu0) should be selected, not F446 (mcu1)"
            );
        }
        other => {
            panic!("expected Send(mcu0) — cross-MCU host-time ordering regression, got {other:?}")
        }
    }
}

/// The globally host-earliest piece's queue having room==0 must block everything
/// (intentional global-gating invariant).
#[test]
fn homing_lead_gates_piece_release() {
    let freq: f64 = 1_000_000.0;
    let ack_now: u64 = 0;

    // Two pieces: first within 50ms, second beyond.
    let piece_inside = (25_000_u64, 0.025_f64);
    let piece_beyond = (75_000_u64, 0.075_f64);

    let mut queues = BTreeMap::new();
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut q = q_with_host(8, &[piece_inside, piece_beyond]);
    q.lead_secs = 0.05;
    queues.insert(key, q);

    let horizon_of = |k: &AxisKey, q: &AxisQueue| -> Option<u64> {
        if k.mcu_id == 1 {
            Some(ack_now + (q.lead_secs * freq) as u64)
        } else {
            None
        }
    };

    // With 50ms lead: only the first piece should release.
    match schedule(&queues, 255, &horizon_of, no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 1, "only the inside-50ms piece must release");
            assert_eq!(frames[0].pieces[0].start_time, 25_000);
        }
        other => panic!("expected Send with one piece, got {other:?}"),
    }

    // With MAX_LEAD_SECS lead: both pieces release.
    let mut queues2 = BTreeMap::new();
    let mut q2 = q_with_host(8, &[piece_inside, piece_beyond]);
    q2.lead_secs = MAX_LEAD_SECS;
    queues2.insert(key, q2);

    let horizon_of_max = |k: &AxisKey, q: &AxisQueue| -> Option<u64> {
        if k.mcu_id == 1 {
            Some(ack_now + (q.lead_secs * freq) as u64)
        } else {
            None
        }
    };

    match schedule(&queues2, 255, &horizon_of_max, no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 2, "both pieces must release under MAX_LEAD_SECS");
        }
        other => panic!("expected Send with two pieces, got {other:?}"),
    }
}

/// Per-queue horizon: queue A has a narrow lead so its far piece is gated,
/// queue B has MAX_LEAD_SECS so its identically-far piece releases.
///
/// Pins the invariant that `horizon_of` is evaluated independently per queue.
/// A refactor that computes the horizon once from the head queue and applies it
/// to all queues would gate B's piece incorrectly and break this test.
#[test]
fn cross_lead_per_queue_horizon_independent() {
    let freq: f64 = 1_000_000.0;
    let ack_now: u64 = 0;

    // Queue A: lead_secs=0.05 → horizon = 50_000 ticks.
    // Queue B: lead_secs=MAX_LEAD_SECS → horizon = 1_000_000 ticks.
    // Both on mcu_id=1.
    //
    // Piece layout (host_time == tick/freq for simplicity):
    //   A-inside:  tick=25_000 → within A's horizon → must send
    //   A-beyond:  tick=75_000 → beyond A's horizon → must not send this pass
    //   B-at-75k:  tick=75_000 → within B's horizon (1_000_000) → must send
    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };

    let mut queues = BTreeMap::new();

    let mut qa = q_with_host(8, &[(25_000, 0.025), (75_000, 0.075)]);
    qa.lead_secs = 0.05;
    queues.insert(key_a, qa);

    let mut qb = q_with_host(8, &[(75_000, 0.075)]);
    qb.lead_secs = MAX_LEAD_SECS;
    queues.insert(key_b, qb);

    let horizon_of = |_k: &AxisKey, q: &AxisQueue| -> Option<u64> {
        Some(ack_now + (q.lead_secs * freq) as u64)
    };

    // Head is A-inside (host=0.025 is earliest, same MCU as B).
    // Inner loop evaluates each queue's horizon independently:
    //   A-inside (25_000 <= 50_000): sent.
    //   A-beyond (75_000 >  50_000): gated.
    //   B-at-75k (75_000 <= 1_000_000): sent.
    match schedule(&queues, 255, &horizon_of, no_cap) {
        Schedule::Send(frames) => {
            let a_frame = frames.iter().find(|f| f.key == key_a);
            let b_frame = frames.iter().find(|f| f.key == key_b);

            let a_frame = a_frame.expect("queue A must have a frame");
            assert_eq!(
                a_frame.pieces.len(),
                1,
                "A should send only the inside-50ms piece; got {} pieces",
                a_frame.pieces.len()
            );
            assert_eq!(
                a_frame.pieces[0].start_time, 25_000,
                "A's sent piece must be the inside-horizon one"
            );

            let b_frame = b_frame.expect("queue B must have a frame (MAX_LEAD_SECS horizon)");
            assert_eq!(
                b_frame.pieces.len(),
                1,
                "B should send its piece (within MAX_LEAD_SECS horizon); got {} pieces",
                b_frame.pieces.len()
            );
            assert_eq!(b_frame.pieces[0].start_time, 75_000);
        }
        other => panic!("expected Send with both A-inside and B pieces; got {other:?}"),
    }
}

#[test]
fn stall_full_on_globally_earliest_gates_all() {
    let mut queues = BTreeMap::new();

    // mcu0: host-earliest piece, but ring is full
    let mut mcu0_q = q_with_host(2, &[(100, 1.0)]);
    mcu0_q.pushed = 2; // full
    queues.insert(AxisKey { mcu_id: 0, axis: 0 }, mcu0_q);

    // mcu1: host-later piece, ring has room
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with_host(8, &[(50, 5.0)]));

    assert!(
        matches!(
            schedule(&queues, 255, |_: &AxisKey, _: &AxisQueue| None, no_cap),
            Schedule::StallFull(AxisKey { mcu_id: 0, axis: 0 })
        ),
        "StallFull on the globally host-earliest queue must gate all issuance"
    );
}
