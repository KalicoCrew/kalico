use super::*;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

#[test]
fn dispatch_fires_matching_callback() {
    let mut table = InterceptorTable::new();
    let count = Arc::new(AtomicU32::new(0));
    let count_clone = Arc::clone(&count);

    table.register(
        "trsync_state".into(),
        Some(0),
        InterceptorCallback(Box::new(move |_| {
            count_clone.fetch_add(1, Ordering::Relaxed);
        })),
    );

    let mut params = MessageParams::new();
    params.insert("oid", crate::transport::MessageValue::U32(0));
    params.insert("can_trigger", crate::transport::MessageValue::U32(0));

    table.dispatch("trsync_state", Some(0), &params);
    assert_eq!(count.load(Ordering::Relaxed), 1);
}

#[test]
fn dispatch_ignores_wrong_oid() {
    let mut table = InterceptorTable::new();
    let count = Arc::new(AtomicU32::new(0));
    let count_clone = Arc::clone(&count);

    table.register(
        "trsync_state".into(),
        Some(0),
        InterceptorCallback(Box::new(move |_| {
            count_clone.fetch_add(1, Ordering::Relaxed);
        })),
    );

    let params = MessageParams::new();
    table.dispatch("trsync_state", Some(1), &params);
    assert_eq!(count.load(Ordering::Relaxed), 0);
}

#[test]
fn dispatch_ignores_wrong_name() {
    let mut table = InterceptorTable::new();
    let count = Arc::new(AtomicU32::new(0));
    let count_clone = Arc::clone(&count);

    table.register(
        "trsync_state".into(),
        Some(0),
        InterceptorCallback(Box::new(move |_| {
            count_clone.fetch_add(1, Ordering::Relaxed);
        })),
    );

    let params = MessageParams::new();
    table.dispatch("analog_in_state", Some(0), &params);
    assert_eq!(count.load(Ordering::Relaxed), 0);
}

#[test]
fn unregister_removes_callback() {
    let mut table = InterceptorTable::new();
    let count = Arc::new(AtomicU32::new(0));
    let count_clone = Arc::clone(&count);

    let id = table.register(
        "trsync_state".into(),
        Some(0),
        InterceptorCallback(Box::new(move |_| {
            count_clone.fetch_add(1, Ordering::Relaxed);
        })),
    );

    let params = MessageParams::new();
    table.dispatch("trsync_state", Some(0), &params);
    assert_eq!(count.load(Ordering::Relaxed), 1);

    table.unregister(id);
    table.dispatch("trsync_state", Some(0), &params);
    assert_eq!(count.load(Ordering::Relaxed), 1, "should not fire after unregister");
}

#[test]
fn callback_receives_params() {
    let mut table = InterceptorTable::new();
    let seen_value = Arc::new(AtomicU32::new(999));
    let seen_clone = Arc::clone(&seen_value);

    table.register(
        "trsync_state".into(),
        Some(0),
        InterceptorCallback(Box::new(move |params| {
            seen_clone.store(params.get_u32("can_trigger"), Ordering::Relaxed);
        })),
    );

    let mut params = MessageParams::new();
    params.insert("oid", crate::transport::MessageValue::U32(0));
    params.insert("can_trigger", crate::transport::MessageValue::U32(0));
    params.insert("trigger_reason", crate::transport::MessageValue::U32(1));

    table.dispatch("trsync_state", Some(0), &params);
    assert_eq!(seen_value.load(Ordering::Relaxed), 0);
}
