//! Step-6 unit-test transport. Records every `send`, replays canned
//! responses on `wait_for_response`. The integration tests for
//! `producer`, `stream::arm_all_mcus`, etc. all build on top of this.
//!
//! Plan-decision C: Step-6 phase-10 modules consume `&mut dyn Transport`
//! / `T: Transport` so they're testable here without the real
//! USB-CDC port. The implementation deliberately mirrors the
//! `KalicoHostIo` shape — same `send`, same `wait_for_response`, same
//! `poll_events` — so the tests exercise the exact code paths
//! production runs.

use std::collections::{HashMap, VecDeque};
use std::time::Duration;

use kalico_host_rt::transport::{
    MessageParams, MessageValue, Transport, TransportError,
};

/// A test-only callback the mock invokes at `wait_for_response` time
/// (rather than at queueing time) so a response whose contents depend
/// on call-time wall clock — e.g. `kalico_clock_sync_response` whose
/// `mcu_clock` must lie on the regression line at the actual
/// `host_send` instant — can be encoded fresh every time the
/// estimator picks it up. Used by `arm_flow_unit.rs` to stabilise the
/// dedicated-sync test against wall-clock jitter between fixture
/// setup and the call into `arm_all_mcus`.
#[allow(clippy::type_complexity)]
pub type DynamicResponder =
    Box<dyn FnMut() -> MessageParams + Send>;

#[derive(Default)]
pub struct MockTransport {
    pub sent: Vec<String>,
    /// Queued responses for `wait_for_response` to consume in order.
    pub responses: VecDeque<(String, MessageParams)>,
    /// Per-message-name dynamic responders. If a name has both an
    /// entry here AND a queued response, the dynamic one wins. Each
    /// invocation calls the closure once.
    pub dynamic_responders: HashMap<String, DynamicResponder>,
    /// Queued events for `poll_events` to drain.
    pub events: VecDeque<(String, MessageParams)>,
    /// If set, `wait_for_response` returns `Err(Timeout)` after popping
    /// this many responses (used to model deadline-miss scenarios).
    pub force_timeout_after: Option<usize>,
}

impl std::fmt::Debug for MockTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MockTransport")
            .field("sent", &self.sent)
            .field("responses", &self.responses)
            .field(
                "dynamic_responders",
                &self.dynamic_responders.keys().collect::<Vec<_>>(),
            )
            .field("events", &self.events)
            .field("force_timeout_after", &self.force_timeout_after)
            .finish()
    }
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn enqueue_response(&mut self, name: &str, params: MessageParams) {
        self.responses.push_back((name.into(), params));
    }

    /// Register a closure invoked at `wait_for_response(name)` time;
    /// the returned `MessageParams` becomes the reply. The closure
    /// remains installed across calls (so multiple matches reuse the
    /// same responder).
    #[allow(dead_code)]
    pub fn install_dynamic_responder(
        &mut self,
        name: &str,
        responder: DynamicResponder,
    ) {
        self.dynamic_responders.insert(name.into(), responder);
    }

    #[allow(dead_code)]
    pub fn enqueue_event(&mut self, name: &str, params: MessageParams) {
        self.events.push_back((name.into(), params));
    }

    pub fn last_sent(&self) -> Option<&str> {
        self.sent.last().map(String::as_str)
    }
}

impl Transport for MockTransport {
    fn send(&mut self, cmd: &str) -> Result<(), TransportError> {
        self.sent.push(cmd.into());
        Ok(())
    }

    fn wait_for_response(
        &mut self,
        name: &str,
        _timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        if let Some(remaining) = self.force_timeout_after.as_mut() {
            if *remaining == 0 {
                return Err(TransportError::Timeout);
            }
            *remaining -= 1;
        }
        // Dynamic responder wins if installed for this name (call-time
        // resolution; lets a test pin the response to the actual
        // `Instant::now()` inside the SUT).
        if let Some(responder) = self.dynamic_responders.get_mut(name) {
            return Ok(responder());
        }
        // Walk the queue: first matching response wins, drop earlier
        // unmatched-but-still-queued entries (mirrors how a real port
        // delivers in-order frames).
        while let Some((n, p)) = self.responses.pop_front() {
            if n == name {
                return Ok(p);
            }
            // Different message arrived first — stash it as an event
            // so a later `poll_events` call can pick it up.
            self.events.push_back((n, p));
        }
        Err(TransportError::Timeout)
    }

    fn poll_events(&mut self, name: &str) -> Vec<MessageParams> {
        let mut out = Vec::new();
        let mut keep = VecDeque::with_capacity(self.events.len());
        while let Some((n, p)) = self.events.pop_front() {
            if n == name {
                out.push(p);
            } else {
                keep.push_back((n, p));
            }
        }
        self.events = keep;
        out
    }
}

// --- shared helpers used by other test files -------------------------------

#[allow(dead_code)]
pub fn mp_with(values: &[(&str, MessageValue)]) -> MessageParams {
    let mut p = MessageParams::new();
    for (k, v) in values {
        p.insert((*k).to_string(), v.clone());
    }
    p
}

// --- in-file sanity tests --------------------------------------------------

#[test]
fn send_records_command() {
    let mut io = MockTransport::new();
    io.send("hello").unwrap();
    assert_eq!(io.last_sent(), Some("hello"));
    assert_eq!(io.sent.len(), 1);
}

#[test]
fn wait_for_response_returns_first_match() {
    let mut io = MockTransport::new();
    io.enqueue_response(
        "kalico_push_response",
        mp_with(&[("result", MessageValue::I32(0))]),
    );
    let resp = io
        .wait_for_response("kalico_push_response", Duration::from_secs(1))
        .unwrap();
    assert_eq!(resp.get_i32("result"), 0);
}

#[test]
fn wait_for_response_times_out_when_queue_empty() {
    let mut io = MockTransport::new();
    let err = io
        .wait_for_response("anything", Duration::from_millis(1))
        .unwrap_err();
    assert!(
        matches!(err, TransportError::Timeout),
        "expected Timeout, got {err:?}"
    );
}

#[test]
fn unmatched_responses_become_events() {
    let mut io = MockTransport::new();
    io.enqueue_response(
        "kalico_credit_freed",
        mp_with(&[("free_slots", MessageValue::U32(3))]),
    );
    // wait_for_response on a different name must not consume the
    // queued frame; instead the mock park it as an event.
    let _ = io.wait_for_response("never_arrives", Duration::ZERO);
    let drained = io.poll_events("kalico_credit_freed");
    assert_eq!(drained.len(), 1);
    assert_eq!(drained[0].get_u32("free_slots"), 3);
}
