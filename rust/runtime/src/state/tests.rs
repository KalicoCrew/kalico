use super::*;

#[test]
fn shared_state_default_is_idle() {
    let s = SharedState::new();
    assert_eq!(
        s.runtime_status.load(core::sync::atomic::Ordering::Relaxed),
        crate::engine::RuntimeStatus::Idle as u8
    );
    assert!(!s.stream_open.load(core::sync::atomic::Ordering::Relaxed));
    assert!(!s.force_idle.load(core::sync::atomic::Ordering::Relaxed));
}

#[test]
fn shared_state_default_widened_now_zero() {
    let s = SharedState::new();
    assert_eq!(
        s.widened_now_lo.load(core::sync::atomic::Ordering::Relaxed),
        0
    );
    assert_eq!(
        s.widened_now_hi.load(core::sync::atomic::Ordering::Relaxed),
        0
    );
    assert_eq!(
        s.widened_now_seq
            .load(core::sync::atomic::Ordering::Relaxed),
        0
    );
}
