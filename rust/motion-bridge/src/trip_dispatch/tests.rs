use super::*;
use std::cell::RefCell;
use std::sync::atomic::Ordering;

use kalico_host_rt::transport::{MessageParams, MessageValue};

/// Build a `MessageParams` with a single u32 field — enough to exercise
/// the closure filters without any real transport.
fn params_u32(key: &str, val: u32) -> MessageParams {
    let mut p = MessageParams::new();
    p.insert(key, MessageValue::U32(val));
    p
}

// ---------------------------------------------------------------------------
// Closure-filter tests
// ---------------------------------------------------------------------------

/// `BridgeGpio` closure: a frame whose `arm_id` does NOT match the
/// registered arm must be silently ignored; a matching frame must fan out.
#[test]
fn bridge_gpio_closure_filters_by_arm_id() {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(vec![SinkSpec { mcu: 1, trsync_oid: 10 }]));
    let sent = Arc::new(std::sync::Mutex::new(Vec::<(u32, String)>::new()));

    let want_arm_id: Option<u32> = Some(42_u32);
    let fan_clone = Arc::clone(&fan);
    let triggered_clone = Arc::clone(&triggered);
    let sent_clone = Arc::clone(&sent);
    // Simulate what `prepare` builds for a BridgeGpio source.
    let closure = move |params: &MessageParams| {
        if let Some(want) = want_arm_id {
            if params.get_u32("arm_id") != want {
                return;
            }
        }
        fan_clone.on_trip(|mcu, cmd| {
            sent_clone.lock().unwrap().push((mcu, cmd.to_string()));
        });
        triggered_clone.store(true, Ordering::Release);
    };

    // Wrong arm_id — must be ignored.
    closure(&params_u32("arm_id", 99));
    assert!(!triggered.load(Ordering::Acquire), "wrong arm_id must not trigger");
    assert!(sent.lock().unwrap().is_empty(), "no send on arm_id mismatch");

    // Correct arm_id — must fan out.
    closure(&params_u32("arm_id", 42));
    assert!(triggered.load(Ordering::Acquire), "matching arm_id must trigger");
    assert_eq!(sent.lock().unwrap().len(), 1, "one send per sink on first trip");
}

/// `Trsync` closure: `can_trigger != 0` (still armed) must be ignored;
/// `can_trigger == 0` (probe hit / soft-trip) must fan out.
#[test]
fn trsync_closure_ignores_nonzero_can_trigger() {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(vec![SinkSpec { mcu: 2, trsync_oid: 11 }]));
    let sent = Arc::new(std::sync::Mutex::new(Vec::<(u32, String)>::new()));

    let want_arm_id: Option<u32> = None; // Trsync path
    let fan_clone = Arc::clone(&fan);
    let triggered_clone = Arc::clone(&triggered);
    let sent_clone = Arc::clone(&sent);
    let closure = move |params: &MessageParams| {
        if let Some(want) = want_arm_id {
            if params.get_u32("arm_id") != want {
                return;
            }
        } else {
            if params.get_u32("can_trigger") != 0 {
                return;
            }
        }
        fan_clone.on_trip(|mcu, cmd| {
            sent_clone.lock().unwrap().push((mcu, cmd.to_string()));
        });
        triggered_clone.store(true, Ordering::Release);
    };

    // Still armed (can_trigger = 1) — must be ignored.
    closure(&params_u32("can_trigger", 1));
    assert!(!triggered.load(Ordering::Acquire), "can_trigger=1 must not trigger");
    assert!(sent.lock().unwrap().is_empty(), "no send while still armed");

    // Probe hit (can_trigger = 0) — must fan out.
    closure(&params_u32("can_trigger", 0));
    assert!(triggered.load(Ordering::Acquire), "can_trigger=0 must trigger");
    assert_eq!(sent.lock().unwrap().len(), 1, "one send per sink on first trip");
}

// ---------------------------------------------------------------------------

#[test]
fn first_trip_fans_trigger_to_all_sinks_once() {
    let sinks = vec![
        SinkSpec { mcu: 1, trsync_oid: 10 },
        SinkSpec { mcu: 2, trsync_oid: 11 },
        SinkSpec { mcu: 3, trsync_oid: 12 },
    ];
    let sent = RefCell::new(Vec::<(u32, String)>::new());
    let dispatch = FanOut::new(sinks);

    dispatch.on_trip(|mcu, cmd| sent.borrow_mut().push((mcu, cmd.to_string())));
    // second trip is a no-op (one-shot)
    dispatch.on_trip(|mcu, cmd| sent.borrow_mut().push((mcu, cmd.to_string())));

    let sent = sent.into_inner();
    assert_eq!(sent.len(), 3, "exactly one trigger per sink, one-shot");
    assert_eq!(sent[0], (1, "trsync_trigger oid=10 reason=1".to_string()));
    assert_eq!(sent[1], (2, "trsync_trigger oid=11 reason=1".to_string()));
    assert_eq!(sent[2], (3, "trsync_trigger oid=12 reason=1".to_string()));
}

#[test]
fn build_trigger_cmd_formats_reason_endstop_hit() {
    assert_eq!(build_trigger_cmd(42), "trsync_trigger oid=42 reason=1");
}
