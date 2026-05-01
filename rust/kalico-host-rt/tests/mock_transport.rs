//! Phase-B MockTransport: `&self` + internal synchronization.
//!
//! Two response modes:
//!
//! 1. **Static** (`enqueue_static`): pre-load a response that `call()`
//!    returns immediately — zero latency, suitable for quality-gate tests
//!    that need the `ClockSyncEstimator` residual < 100 µs.
//!
//! 2. **Async** (`wait_for_call` + `complete_call`): a test thread blocks
//!    on `wait_for_call`, then drives the response from outside `call()`.
//!    Suitable for tests that need to inspect call ordering but don't have
//!    tight timing constraints.
//!
//! The `call_time` in the pending-call entry is the `Instant` captured
//! inside `call()` just before it blocks; static responses use it too.

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::SyncSender;
use std::time::{Duration, Instant};

use kalico_host_rt::host_io::parser::FieldValue;
use kalico_host_rt::transport::{MessageParams, MessageValue, Transport, TransportError};

type Responder = Box<dyn Fn(&str, Instant) -> MessageParams + Send + Sync>;

struct MockPendingCall {
    expected_response_name: String,
    cmd:                    String,
    call_time:              Instant,
    completion:             Option<SyncSender<Result<MessageParams, TransportError>>>,
}

struct MockState {
    pending_calls: HashMap<u64, MockPendingCall>,
    next_call_id:  u64,
    sent_cmds:     Vec<String>,
    /// Per-response-name responder: called synchronously inside call() with
    /// the call_time Instant, returns the MessageParams to complete the call.
    static_responders: HashMap<String, Responder>,
}

pub struct MockTransport {
    state:        Mutex<MockState>,
    call_arrived: Condvar,
}

impl MockTransport {
    pub fn new() -> Self {
        Self {
            state: Mutex::new(MockState {
                pending_calls: HashMap::new(),
                next_call_id: 1,
                sent_cmds: Vec::new(),
                static_responders: HashMap::new(),
            }),
            call_arrived: Condvar::new(),
        }
    }

    /// Register a synchronous responder for `expected_response_name`.
    /// It is called inside `call()` with the just-sent `cmd` and the
    /// `call_time` Instant. The mock state mutex is held while the
    /// responder runs, so the closure must NOT call back into
    /// `MockTransport` (which would deadlock).
    pub fn install_responder<F>(&self, expected_response_name: &str, f: F)
    where
        F: Fn(&str, Instant) -> MessageParams + Send + Sync + 'static,
    {
        self.state.lock().unwrap()
            .static_responders
            .insert(expected_response_name.to_string(), Box::new(f));
    }

    pub fn pending_count(&self) -> usize {
        self.state.lock().unwrap().pending_calls.len()
    }

    pub fn sent_count(&self) -> usize {
        self.state.lock().unwrap().sent_cmds.len()
    }

    pub fn any_sent_starting_with(&self, prefix: &str) -> bool {
        self.state.lock().unwrap().sent_cmds.iter().any(|s| s.starts_with(prefix))
    }

    pub fn last_sent(&self) -> Option<String> {
        self.state.lock().unwrap().sent_cmds.last().cloned()
    }

    pub fn sent_starting_with(&self, prefix: &str) -> Vec<String> {
        self.state.lock().unwrap().sent_cmds.iter()
            .filter(|s| s.starts_with(prefix))
            .cloned()
            .collect()
    }

    /// Block until a call with `expected_response_name` is pending.
    /// Returns `(cmd, call_time)`.
    pub fn wait_for_call(&self, expected_response_name: &str) -> (String, Instant) {
        let mut guard = self.state.lock().unwrap();
        loop {
            let found = guard.pending_calls.iter()
                .find(|(_, c)| c.expected_response_name == expected_response_name)
                .map(|(_, c)| (c.cmd.clone(), c.call_time));
            if let Some(info) = found {
                return info;
            }
            guard = self.call_arrived.wait(guard).unwrap();
        }
    }

    pub fn complete_call(&self, name: &str, params: MessageParams) {
        let mut state = self.state.lock().unwrap();
        let id = state.pending_calls.iter()
            .find(|(_, c)| c.expected_response_name == name)
            .map(|(id, _)| *id);
        if let Some(id) = id {
            if let Some(call) = state.pending_calls.remove(&id) {
                drop(state);
                if let Some(tx) = call.completion {
                    let _ = tx.send(Ok(params));
                }
            }
        }
    }

    pub fn drop_pending(&self, name: &str) {
        let mut state = self.state.lock().unwrap();
        let id = state.pending_calls.iter()
            .find(|(_, c)| c.expected_response_name == name)
            .map(|(id, _)| *id);
        if let Some(id) = id {
            state.pending_calls.remove(&id);
        }
    }
}

impl Transport for MockTransport {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let call_time = Instant::now();
        let rx = {
            let mut state = self.state.lock().unwrap();
            state.sent_cmds.push(cmd.to_string());

            // Static responder: call synchronously, return immediately.
            if let Some(responder) = state.static_responders.get(expected_response_name) {
                let params = responder(cmd, call_time);
                return Ok(params);
            }

            let id = state.next_call_id;
            state.next_call_id += 1;
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            state.pending_calls.insert(id, MockPendingCall {
                expected_response_name: expected_response_name.to_string(),
                cmd: cmd.to_string(),
                call_time,
                completion: Some(tx),
            });
            rx
        };
        self.call_arrived.notify_all();
        match rx.recv_timeout(timeout) {
            Ok(result) => result,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    fn call_typed(
        &self,
        name: &str,
        _args: &[(&str, FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.call(name, expected_response_name, timeout)
    }
}

/// Newtype wrapper around `Arc<MockTransport>` that implements `Transport`.
/// Needed because the orphan rule prevents `impl Transport for Arc<MockTransport>`.
#[derive(Clone)]
pub struct SharedMock(pub Arc<MockTransport>);

impl SharedMock {
    pub fn new() -> Self {
        Self(Arc::new(MockTransport::new()))
    }
}

impl std::ops::Deref for SharedMock {
    type Target = MockTransport;
    fn deref(&self) -> &MockTransport { &self.0 }
}

impl Transport for SharedMock {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.0.call(cmd, expected_response_name, timeout)
    }

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.0.call_typed(name, args, expected_response_name, timeout)
    }
}

// --- shared helpers used by other test files ---------------------------------

#[allow(dead_code)]
pub fn mp_with(values: &[(&str, MessageValue)]) -> MessageParams {
    let mut p = MessageParams::new();
    for (k, v) in values {
        p.insert((*k).to_string(), v.clone());
    }
    p
}

// --- in-file tests -----------------------------------------------------------

#[test]
fn complete_call_returns_to_caller() {
    let mock = Arc::new(MockTransport::new());
    let clone = mock.clone();
    let t = std::thread::spawn(move || {
        clone.call("ping", "pong", Duration::from_secs(1))
    });
    let mock_b = mock.clone();
    let _waiter = std::thread::spawn(move || {
        let _ = mock_b.wait_for_call("pong");
        let mut params = MessageParams::new();
        params.insert("result", MessageValue::I32(0));
        mock_b.complete_call("pong", params);
    });
    assert!(t.join().unwrap().is_ok());
    assert_eq!(mock.pending_count(), 0);
}

#[test]
fn timeout_leaves_pending_until_dropped() {
    let mock = MockTransport::new();
    let result = mock.call("ping", "pong", Duration::from_millis(20));
    assert!(matches!(result, Err(TransportError::Timeout)));
    assert_eq!(mock.pending_count(), 1);
    mock.drop_pending("pong");
    assert_eq!(mock.pending_count(), 0);
}
