use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::transport::{MessageParams, MessageValue};
use std::sync::mpsc::sync_channel;
use std::time::Duration;

fn submit_with_call_id(
    h: &mut ReactorHarness,
    call_id: u64,
) -> std::sync::mpsc::Receiver<Result<MessageParams, TransportError>> {
    let (tx, rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        call_id,
        vec![call_id as u8],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(60),
    );
    rx
}

#[test]
fn is_shutdown_fails_all_pending_calls_immediately() {
    let mut h = ReactorHarness::new();
    let rx1 = submit_with_call_id(&mut h, 1);
    let rx2 = submit_with_call_id(&mut h, 2);
    h.tick();
    assert_eq!(h.awaiting_depth(), 2);

    let params = MessageParams::default();
    h.reactor
        .fail_pending_on_mcu_shutdown("is_shutdown", &params);

    assert_eq!(h.awaiting_depth(), 0);
    for rx in [rx1, rx2] {
        let result = rx
            .recv_timeout(Duration::from_millis(100))
            .expect("completion delivered");
        assert!(
            matches!(result, Err(TransportError::McuShutdown(_))),
            "expected McuShutdown, got {result:?}"
        );
    }
}

#[test]
fn unresolved_static_string_id_keeps_response_name_in_reason() {
    let mut h = ReactorHarness::new();
    let rx = submit_with_call_id(&mut h, 1);
    h.tick();

    let mut params = MessageParams::default();
    params
        .fields
        .insert("static_string_id".into(), MessageValue::U32(9999));
    h.reactor.fail_pending_on_mcu_shutdown("shutdown", &params);

    let result = rx
        .recv_timeout(Duration::from_millis(100))
        .expect("completion delivered");
    match result {
        Err(TransportError::McuShutdown(reason)) => {
            assert!(
                reason.contains("shutdown"),
                "reason should carry the response name, got {reason:?}"
            );
        }
        other => panic!("expected McuShutdown, got {other:?}"),
    }
}

#[test]
fn shutdown_with_no_pending_calls_is_a_no_op() {
    let mut h = ReactorHarness::new();
    let params = MessageParams::default();
    h.reactor
        .fail_pending_on_mcu_shutdown("is_shutdown", &params);
    assert_eq!(h.awaiting_depth(), 0);
}
