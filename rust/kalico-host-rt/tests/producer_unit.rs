//! Phase 10 Task 10.5 unit tests: `producer::push_segment` against
//! `MockTransport`.

mod mock_transport;

use std::time::Duration;

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::producer::{ProducerError, push_segment};
use kalico_host_rt::transport::{MessageValue, TransportError};

use mock_transport::{MockTransport, mp_with};

#[test]
fn happy_path_pushes_and_returns_accepted_id_and_epoch() {
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(4);
    io.enqueue_response(
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(0)),
            ("accepted_segment_id", MessageValue::U32(42)),
            ("credit_epoch", MessageValue::U32(7)),
        ]),
    );

    let info = push_segment(&mut io, &credit, 42, 0xCAFE_BABE, 1000, 2000, 0).expect("happy push");
    assert_eq!(info.accepted_segment_id, 42);
    assert_eq!(info.credit_epoch, 7);
    assert_eq!(credit.available(), 3, "credit decremented exactly once");
    let last = io.last_sent().unwrap();
    // Closure-review fix: assertions MUST mirror the firmware's
    // DECL_COMMAND format string verbatim — `curve_handle=` (not
    // `_packed`), `kinematics=` (not `kin`), and `t_start_hi` ahead of
    // `t_start_lo`.
    assert!(last.contains("id=42"));
    assert!(last.contains("curve_handle=3405691582"));
    assert!(
        !last.contains("curve_handle_packed="),
        "must not use the legacy `_packed` suffix; firmware DECL_COMMAND \
         is `curve_handle=%u`"
    );
    assert!(last.contains("t_start_lo=1000"));
    assert!(last.contains("t_end_lo=2000"));
    assert!(last.contains("kinematics=0"));
    assert!(
        !last.contains("kin=0"),
        "must not use legacy `kin=`; firmware is `kinematics=%c`"
    );
    // Field ordering — `t_start_hi` must come before `t_start_lo`.
    let hi_pos = last.find("t_start_hi=").expect("t_start_hi missing");
    let lo_pos = last.find("t_start_lo=").expect("t_start_lo missing");
    assert!(
        hi_pos < lo_pos,
        "t_start_hi must precede t_start_lo to match firmware DECL_COMMAND"
    );
    let end_hi_pos = last.find("t_end_hi=").expect("t_end_hi missing");
    let end_lo_pos = last.find("t_end_lo=").expect("t_end_lo missing");
    assert!(
        end_hi_pos < end_lo_pos,
        "t_end_hi must precede t_end_lo to match firmware DECL_COMMAND"
    );
}

#[test]
fn no_credit_returns_nocredit_without_sending() {
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(1);
    credit.try_acquire().unwrap(); // exhaust
    let err = push_segment(&mut io, &credit, 0, 1, 0, 100, 0).unwrap_err();
    assert!(
        matches!(err, ProducerError::NoCredit),
        "expected NoCredit, got {err:?}"
    );
    assert!(io.sent.is_empty(), "must not send when out of credit");
}

#[test]
fn mcu_rejection_releases_credit() {
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(2);
    io.enqueue_response(
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(-103)), // POOL_NOT_LOADED
            ("accepted_segment_id", MessageValue::U32(0)),
            ("credit_epoch", MessageValue::U32(0)),
        ]),
    );
    let err = push_segment(&mut io, &credit, 0, 0, 0, 0, 0).unwrap_err();
    match err {
        ProducerError::McuRejected(r) => assert_eq!(r, -103),
        other => panic!("expected McuRejected, got {other:?}"),
    }
    assert_eq!(credit.available(), 2, "credit must be restored on reject");
}

#[test]
fn transport_timeout_releases_credit() {
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(1);
    // No response queued → transport returns Timeout.
    let err = push_segment(&mut io, &credit, 0, 0, 0, 0, 0).unwrap_err();
    assert!(
        matches!(err, ProducerError::Transport(_)),
        "expected Transport(_), got {err:?}"
    );
    assert_eq!(credit.available(), 1, "credit must be restored on timeout");
}

#[test]
fn high_64bit_t_start_t_end_split_lo_hi() {
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(1);
    io.enqueue_response(
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(0)),
            ("accepted_segment_id", MessageValue::U32(0)),
            ("credit_epoch", MessageValue::U32(0)),
        ]),
    );
    // 2^32 + 5 → lo=5, hi=1
    let _ = push_segment(&mut io, &credit, 0, 0, 0x1_0000_0005, 0x1_0000_0006, 0).unwrap();
    let last = io.last_sent().unwrap();
    assert!(
        last.contains("t_start_lo=5") && last.contains("t_start_hi=1"),
        "unexpected wire encoding: {last}"
    );
    assert!(
        last.contains("t_end_lo=6") && last.contains("t_end_hi=1"),
        "unexpected wire encoding: {last}"
    );
}

#[test]
fn missing_result_field_surfaces_parse_error_and_releases_credit() {
    // I1 fix: a malformed `kalico_push_response` with no `result`
    // field must NOT silently succeed (treating absent as 0). Instead
    // surface a Transport(Parse(...)) and roll back the credit.
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(2);
    io.enqueue_response(
        "kalico_push_response",
        mp_with(&[
            // No "result" key → try_get_i32 returns None.
            ("accepted_segment_id", MessageValue::U32(1)),
            ("credit_epoch", MessageValue::U32(1)),
        ]),
    );
    let err = push_segment(&mut io, &credit, 0, 0, 0, 0, 0).unwrap_err();
    match err {
        ProducerError::Transport(TransportError::Parse(msg)) => {
            assert!(
                msg.contains("missing 'result' field"),
                "expected diagnostic to mention missing 'result', got {msg}"
            );
        }
        other => panic!("expected Transport(Parse(_)), got {other:?}"),
    }
    assert_eq!(
        credit.available(),
        2,
        "credit must be restored when malformed response trips parse error"
    );
}

// Suppress the dead-code warning that fires when this test binary is
// compiled without referring to every helper from `mock_transport.rs`.
const _: Duration = Duration::from_millis(0);
