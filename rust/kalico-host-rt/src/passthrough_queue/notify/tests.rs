use super::*;
use std::sync::{Arc, Mutex};

#[test]
fn dispatch_fires_callback_once() {
    let mut table = NotifyTable::new();
    let fired = Arc::new(Mutex::new(0u32));
    let fired2 = Arc::clone(&fired);

    let id = table.register(Box::new(move |_resp| {
        *fired2.lock().unwrap() += 1;
    }));

    table.dispatch(id, NotifyResponse::default());
    assert_eq!(*fired.lock().unwrap(), 1);

    // Second dispatch is a no-op
    table.dispatch(id, NotifyResponse::default());
    assert_eq!(*fired.lock().unwrap(), 1);
}

#[test]
fn unique_ids() {
    let mut table = NotifyTable::new();
    let id1 = table.register(Box::new(|_| {}));
    let id2 = table.register(Box::new(|_| {}));
    assert_ne!(id1, id2);
    assert!(!id1.is_none());
    assert!(!id2.is_none());
}

#[test]
fn dispatch_propagates_response_payload() {
    let mut table = NotifyTable::new();
    let captured = Arc::new(Mutex::new(Vec::new()));
    let captured2 = Arc::clone(&captured);

    let id = table.register(Box::new(move |resp| {
        *captured2.lock().unwrap() = resp.bytes;
    }));

    table.dispatch(
        id,
        NotifyResponse {
            bytes: vec![0xDE, 0xAD],
            sent_time: 1.0,
            receive_time: 2.0,
        },
    );

    assert_eq!(*captured.lock().unwrap(), vec![0xDE, 0xAD]);
}
