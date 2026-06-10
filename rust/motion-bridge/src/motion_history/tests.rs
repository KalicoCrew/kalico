use runtime::piece_ring::PieceEntry;

use crate::motion_history::{HISTORY_CAPACITY, HistoryError, HistoryPiece, HistoryStore};
use crate::pump::AxisKey;

const FREQ: u32 = 520_000_000;

fn key() -> AxisKey {
    AxisKey { mcu_id: 7, axis: 2 }
}

fn entry(start_time: u64, duration: f32, coeffs: [f32; 4]) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs,
        duration,
        _reserved: 0,
    }
}

fn linear(start_time: u64, duration: f32, p0: f32, p1: f32) -> PieceEntry {
    let third = (p1 - p0) / 3.0;
    entry(start_time, duration, [p0, p0 + third, p0 + 2.0 * third, p1])
}

#[test]
fn end_clock_matches_isr_formula() {
    let e = entry(1_000, 0.0123, [0.0; 4]);
    let h = HistoryPiece::from_entry(&e, FREQ);
    assert_eq!(h.end_clock, e.end_time(FREQ as f32));
    assert_eq!(h.start_clock, 1_000);
}

#[test]
fn linear_piece_position_velocity_acceleration() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 1.0, 0.0, 10.0), FREQ);
    let mid = FREQ as u64 / 2;
    let st = store.state_at_clock(key(), mid, Some(u64::MAX)).unwrap();
    assert!((st.position - 5.0).abs() < 1e-6);
    assert!((st.velocity - 10.0).abs() < 1e-6);
    assert!(st.acceleration.abs() < 1e-6);
}

#[test]
fn quadratic_piece_derivatives() {
    let mut store = HistoryStore::default();
    store.record(key(), &entry(0, 1.0, [0.0, 0.0, 5.0, 15.0]), FREQ);
    let mid = FREQ as u64 / 2;
    let st = store.state_at_clock(key(), mid, Some(u64::MAX)).unwrap();
    assert!((st.position - 3.75).abs() < 1e-5);
    assert!((st.velocity - 15.0).abs() < 1e-5);
    assert!((st.acceleration - 30.0).abs() < 1e-4);
}

#[test]
fn gap_between_pieces_holds_previous_endpoint() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.001, 0.0, 10.0), FREQ);
    let gap_start = HistoryPiece::from_entry(&linear(0, 0.001, 0.0, 10.0), FREQ).end_clock;
    store.record(
        key(),
        &linear(gap_start + 1_000_000, 0.001, 10.0, 20.0),
        FREQ,
    );
    let st = store
        .state_at_clock(key(), gap_start + 500_000, Some(u64::MAX))
        .unwrap();
    assert!((st.position - 10.0).abs() < 1e-6);
    assert_eq!(st.velocity, 0.0);
    assert_eq!(st.acceleration, 0.0);
}

#[test]
fn after_last_piece_holds_when_not_future() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.001, 0.0, 10.0), FREQ);
    let end = store
        .state_at_clock(key(), 519_999, Some(u64::MAX))
        .unwrap();
    assert!((end.position - 10.0).abs() < 1e-4);
    let held = store
        .state_at_clock(key(), 5_000_000, Some(10_000_000))
        .unwrap();
    assert!((held.position - 10.0).abs() < 1e-6);
}

#[test]
fn hold_in_the_future_is_an_error() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.001, 0.0, 10.0), FREQ);
    let err = store
        .state_at_clock(key(), 5_000_000, Some(1_000_000))
        .unwrap_err();
    assert!(matches!(err, HistoryError::QueryInFuture { .. }));
}

#[test]
fn inside_committed_future_piece_evaluates() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 1.0, 0.0, 10.0), FREQ);
    let st = store
        .state_at_clock(key(), FREQ as u64 / 2, Some(1_000))
        .unwrap();
    assert!((st.position - 5.0).abs() < 1e-6);
}

#[test]
fn before_window_is_an_error() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(1_000_000, 0.001, 0.0, 10.0), FREQ);
    let err = store
        .state_at_clock(key(), 500, Some(u64::MAX))
        .unwrap_err();
    assert!(matches!(err, HistoryError::BeforeRetainedWindow { .. }));
}

#[test]
fn unknown_axis_is_an_error() {
    let store = HistoryStore::default();
    let err = store.state_at_clock(key(), 0, Some(u64::MAX)).unwrap_err();
    assert!(matches!(err, HistoryError::NoHistoryForAxis(_)));
}

#[test]
fn rebase_clears_ring_and_answers_from_register() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 1.0, 0.0, 10.0), FREQ);
    store.rebase_axis(key(), 2_000_000_000, 42.0);
    let held = store
        .state_at_clock(key(), 2_000_000_500, Some(3_000_000_000))
        .unwrap();
    assert!((held.position - 42.0).abs() < 1e-9);
    let err = store
        .state_at_clock(key(), 1_000, Some(u64::MAX))
        .unwrap_err();
    assert!(matches!(err, HistoryError::BeforeRetainedWindow { .. }));
}

#[test]
fn eviction_keeps_capacity_and_reports_true_window() {
    let mut store = HistoryStore::default();
    let dur = 0.001_f32;
    let dur_ticks = (dur * FREQ as f32) as u64;
    for i in 0..(HISTORY_CAPACITY as u64 + 10) {
        store.record(key(), &linear(i * dur_ticks, dur, 0.0, 1.0), FREQ);
    }
    let err = store.state_at_clock(key(), 0, Some(u64::MAX)).unwrap_err();
    match err {
        HistoryError::BeforeRetainedWindow { window_start, .. } => {
            assert_eq!(window_start, 10 * dur_ticks);
        }
        other => panic!("expected BeforeRetainedWindow, got {other:?}"),
    }
}
