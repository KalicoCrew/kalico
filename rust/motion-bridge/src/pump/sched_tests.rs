use super::*;

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

fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
    let pairs: Vec<(u64, f64)> = starts.iter().map(|&s| (s, s as f64)).collect();
    q_with_host(ring_depth, &pairs)
}

#[test]
fn idle_when_empty() {
    let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    assert!(matches!(schedule(&queues, 255, |_| None), Schedule::Idle));
}

#[test]
fn stalls_when_global_head_ring_full() {
    let mut queues = BTreeMap::new();
    let mut a = q_with(2, &[10]);
    a.pushed = 2;
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
    assert!(matches!(
        schedule(&queues, 255, |_| None),
        Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
    ));
}

#[test]
fn batches_contiguous_same_mcu_prefix_only() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
    let s = schedule(&queues, 255, |_| None);
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
    let s = schedule(&queues, 2, |_| None);
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
    match schedule(&q, 255, |_| None) {
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
    match schedule(&queues, 255, |_| Some(150)) {
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
            schedule(&queues, 255, |_| Some(500)),
            Schedule::StallAhead(AxisKey { mcu_id: 1, axis: 0 })
        ),
        "expected StallAhead when sole piece is beyond horizon"
    );
}

#[test]
fn no_horizon_none_uses_count_only_gate() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[u64::MAX]));
    match schedule(&queues, 255, |_| None) {
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

    let f446_host: f64 = 1_000.0;
    let h7_host: f64 = 1.0;

    let mut queues = BTreeMap::new();
    queues.insert(
        AxisKey { mcu_id: 1, axis: 2 },
        q_with_host(8, &[(f446_tick, f446_host)]),
    );
    queues.insert(
        AxisKey { mcu_id: 0, axis: 0 },
        q_with_host(8, &[(h7_tick, h7_host)]),
    );

    let horizon_of = |mcu_id: u32| -> Option<u64> {
        if mcu_id == 0 {
            Some(h7_tick + 1_000_000)
        } else {
            Some(f446_tick - 1)
        }
    };

    match schedule(&queues, 255, horizon_of) {
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

#[test]
fn stall_full_on_globally_earliest_gates_all() {
    let mut queues = BTreeMap::new();

    let mut mcu0_q = q_with_host(2, &[(100, 1.0)]);
    mcu0_q.pushed = 2;
    queues.insert(AxisKey { mcu_id: 0, axis: 0 }, mcu0_q);

    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with_host(8, &[(50, 5.0)]));

    assert!(
        matches!(
            schedule(&queues, 255, |_| None),
            Schedule::StallFull(AxisKey { mcu_id: 0, axis: 0 })
        ),
        "StallFull on the globally host-earliest queue must gate all issuance"
    );
}
