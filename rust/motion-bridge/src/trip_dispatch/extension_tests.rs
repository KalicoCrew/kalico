use super::extension::{
    clock32_to_64, host_time_to_ticks, ticks_to_host_time, ExtensionEngine, Participant,
};

fn engine(n: usize, expire_s: f64, min_extend_s: f64, start: f64) -> ExtensionEngine {
    ExtensionEngine::new(
        (0..n)
            .map(|_| Participant {
                last_status_time: start,
                expire_time: start + expire_s,
            })
            .collect(),
        expire_s,
        min_extend_s,
    )
}

#[test]
fn report_extends_others_anchored_to_minimum() {
    let mut e = engine(2, 0.025, 0.0, 0.0);
    let sends = e.on_report(0, 1.0);
    let find = |s: &[(usize, f64)], i| s.iter().find(|(p, _)| *p == i).map(|(_, t)| *t);
    assert_eq!(find(&sends, 1), Some(1.0 + 0.025));
    assert_eq!(find(&sends, 0), None);
}

#[test]
fn participant_cannot_extend_itself() {
    let mut e = engine(2, 0.025, 0.0, 0.0);
    e.on_report(0, 1.0);
    let sends = e.on_report(0, 2.0);
    assert!(sends.iter().all(|(p, _)| *p != 0));
    assert_eq!(sends, vec![(1, 2.0 + 0.025)]);
}

#[test]
fn silence_means_no_extension_for_anyone_anchored_to_the_silent_one() {
    let mut e = engine(3, 0.025, 0.0, 0.0);
    e.on_report(0, 1.0);
    e.on_report(1, 1.0);
    let sends = e.on_report(0, 2.0);
    assert!(sends.iter().any(|(p, t)| *p == 2 && *t > 1.0));
    assert!(sends.iter().all(|(p, _)| *p != 0 && *p != 1));
}

#[test]
fn hysteresis_suppresses_small_advances() {
    let mut e = engine(2, 0.025, 0.006, 0.0);
    let sends = e.on_report(0, 0.004);
    assert!(sends.is_empty());
    let sends = e.on_report(0, 0.010);
    assert_eq!(sends, vec![(1, 0.035)]);
}

#[test]
fn single_participant_extends_on_own_report() {
    let mut e = engine(1, 0.25, 0.0, 0.0);
    let sends = e.on_report(0, 1.0);
    assert_eq!(sends, vec![(0, 1.25)]);
}

#[test]
fn clock32_reconstruction_handles_wrap() {
    assert_eq!(clock32_to_64(0x1_0000_0010, 0xFFFF_FFF0), 0x0_FFFF_FFF0);
    assert_eq!(clock32_to_64(0x1_0000_0010, 0x0000_0020), 0x1_0000_0020);
}

#[test]
fn tick_time_round_trip() {
    let freq = 520_000_000.0;
    let host_now = 100.0;
    let now_ticks: u64 = 52_000_000_000;
    let t = ticks_to_host_time(now_ticks + 5_200_000, now_ticks, host_now, freq);
    assert!((t - 100.01).abs() < 1e-9);
    let back = host_time_to_ticks(t, now_ticks, host_now, freq);
    assert_eq!(back, now_ticks + 5_200_000);
}
