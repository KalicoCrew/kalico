//! `RouterTransport` — a `kalico_host_rt::transport::Transport` impl backed
//! by a shared `PassthroughRouter` instead of an owned serial port.
//!
//! Used by the bridge's planner-dispatch closure so `producer::load_curve`
//! and `producer::push_segment` can issue their wire calls through the
//! same router that klippy already drives. Synchronous request/response
//! is implemented by registering a notify callback that hands the raw
//! response bytes to a `crossbeam_channel::Sender` and decoding them
//! against an `Arc<MsgProtoParser>` provided by the bridge owner.
//!
//! ## Parser availability
//!
//! The parser is supplied externally (via `PyMotionBridge::set_msgproto_dict`).
//! Until it has been installed, both `call` and `call_typed` return
//! `TransportError::Parse("msgproto parser not configured")`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crossbeam_channel::bounded;

use kalico_host_rt::host_io::parser::{FieldValue, MsgProtoParser};
use kalico_host_rt::passthrough_queue::{
    CommandQueueId, McuHandle, NotifyId, PassthroughEntry, PassthroughRouter,
};
use kalico_host_rt::transport::{MessageParams, Transport, TransportError};

/// Adapter that maps the `Transport` request/response calls onto a single
/// `(McuHandle, CommandQueueId)` slot of a shared `PassthroughRouter`.
pub struct RouterTransport {
    pub router: Arc<Mutex<PassthroughRouter>>,
    pub mcu: McuHandle,
    pub queue: CommandQueueId,
    /// Optional msgproto parser. `None` until klippy hands the data
    /// dictionary JSON over via `PyMotionBridge::set_msgproto_dict`.
    pub parser: Arc<Mutex<Option<Arc<MsgProtoParser>>>>,
}

impl std::fmt::Debug for RouterTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RouterTransport")
            .field("mcu", &self.mcu.raw())
            .field("queue", &self.queue.raw())
            .finish()
    }
}

impl RouterTransport {
    pub fn new(
        router: Arc<Mutex<PassthroughRouter>>,
        mcu: McuHandle,
        queue: CommandQueueId,
        parser: Arc<Mutex<Option<Arc<MsgProtoParser>>>>,
    ) -> Self {
        Self {
            router,
            mcu,
            queue,
            parser,
        }
    }

    fn parser_snapshot(&self) -> Result<Arc<MsgProtoParser>, TransportError> {
        self.parser
            .lock()
            .unwrap()
            .clone()
            .ok_or_else(|| TransportError::Parse("msgproto parser not configured".to_string()))
    }

    /// Common path: register notify + push entry + block on receiver +
    /// decode body bytes against the parser, filtering by
    /// `expected_response_name`.
    ///
    /// Relies on the `dispatch_response` contract: the bytes delivered via
    /// `NotifyResponse::bytes` are `[msgid VLQ | fields...]`, directly
    /// decodable by `MsgProtoParser::decode_body`. See
    /// `kalico_host_rt::passthrough_queue::router::PassthroughRouter::dispatch_response`.
    fn submit_and_wait(
        &self,
        wire_bytes: Vec<u8>,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let parser = self.parser_snapshot()?;

        let (tx, rx) = bounded::<Vec<u8>>(1);
        let notify_id: NotifyId = {
            let mut router = self.router.lock().unwrap();
            router
                .register_notify(
                    self.mcu,
                    Box::new(move |resp| {
                        // Best-effort: receiver dropped on timeout, ignore.
                        let _ = tx.send(resp.bytes);
                    }),
                )
                .map_err(|e| TransportError::Parse(format!("register_notify: {e}")))?
        };

        {
            let mut router = self.router.lock().unwrap();
            let entry = PassthroughEntry::new(wire_bytes, 0, 0, notify_id);
            router
                .push(self.mcu, self.queue, entry)
                .map_err(|e| TransportError::Parse(format!("router push: {e}")))?;
        }

        let body = match rx.recv_timeout(timeout) {
            Ok(b) => b,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                return Err(TransportError::Timeout);
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                return Err(TransportError::Closed);
            }
        };

        let (name, params) = parser
            .decode_body(&body)
            .map_err(|e| TransportError::Parse(format!("decode_body: {e:?}")))?;

        if name != expected_response_name {
            return Err(TransportError::Parse(format!(
                "expected response '{expected_response_name}', got '{name}'"
            )));
        }
        Ok(params)
    }
}

impl Transport for RouterTransport {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let parser = self.parser_snapshot()?;
        let bytes = parser
            .encode(cmd)
            .map_err(|e| TransportError::Parse(format!("encode: {e:?}")))?;
        self.submit_and_wait(bytes, expected_response_name, timeout)
    }

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let parser = self.parser_snapshot()?;
        let bytes = parser
            .encode_typed(name, args)
            .map_err(|e| TransportError::Parse(format!("encode_typed: {e:?}")))?;
        self.submit_and_wait(bytes, expected_response_name, timeout)
    }

    fn send_typed(
        &self,
        name: &str,
        args: &[(&str, FieldValue<'_>)],
    ) -> Result<(), TransportError> {
        let parser = self.parser_snapshot()?;
        let wire_bytes = parser
            .encode_typed(name, args)
            .map_err(|e| TransportError::Parse(format!("encode_typed: {e:?}")))?;
        // Fire-and-forget: push the encoded payload through the router with
        // no notify registration. The router's `PassthroughEntry::new` takes
        // a `NotifyId`; `NotifyId::default()` is the documented "no notify"
        // sentinel — see `kalico_host_rt::passthrough_queue::router`.
        let mut router = self.router.lock().unwrap();
        let entry = PassthroughEntry::new(wire_bytes, 0, 0, NotifyId::none());
        router
            .push(self.mcu, self.queue, entry)
            .map_err(|e| TransportError::Parse(format!("router push: {e}")))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kalico_host_rt::clock::RealClock;

    fn router() -> Arc<Mutex<PassthroughRouter>> {
        let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
        Arc::new(Mutex::new(PassthroughRouter::with_clock(clock)))
    }

    #[test]
    fn no_parser_returns_parse_error() {
        let router = router();
        let (mcu, queue) = {
            let mut r = router.lock().unwrap();
            let h = r.claim_mcu("test");
            let q = r.alloc_command_queue(h).unwrap();
            (h, q)
        };
        let parser_slot: Arc<Mutex<Option<Arc<MsgProtoParser>>>> = Arc::new(Mutex::new(None));
        let t = RouterTransport::new(router, mcu, queue, parser_slot);

        match t.call(
            "kalico_load_curve",
            "kalico_load_curve_response",
            Duration::from_millis(10),
        ) {
            Err(TransportError::Parse(s)) => assert!(s.contains("not configured")),
            other => panic!("expected Parse error, got {other:?}"),
        }
    }

    #[test]
    fn host_time_to_mcu_clock_helper_round_trips() {
        // Sanity-check the new router accessor used by the dispatch closure.
        let router = router();
        let mcu = {
            let mut r = router.lock().unwrap();
            r.claim_mcu("clk")
        };
        // freq=1e6, offset=0, last_clock=0 -> 1.5s -> 1_500_000.
        {
            let mut r = router.lock().unwrap();
            r.set_clock_est(mcu, 1_000_000.0, 0.0, 0).unwrap();
        }
        let r = router.lock().unwrap();
        let got = r.host_time_to_mcu_clock(mcu, 1.5).unwrap();
        assert_eq!(got, 1_500_000);
    }

    #[test]
    fn host_time_to_mcu_clock_returns_zero_when_unset() {
        let router = router();
        let mcu = {
            let mut r = router.lock().unwrap();
            r.claim_mcu("clk")
        };
        let r = router.lock().unwrap();
        assert_eq!(r.host_time_to_mcu_clock(mcu, 1.5).unwrap(), 0);
    }
}
