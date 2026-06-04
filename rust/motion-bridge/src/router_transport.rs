use std::sync::{Arc, Mutex};
use std::time::Duration;

use crossbeam_channel::bounded;

use kalico_host_rt::host_io::parser::{FieldValue, MsgProtoParser};
use kalico_host_rt::passthrough_queue::{
    CommandQueueId, McuHandle, NotifyId, PassthroughEntry, PassthroughRouter,
};
use kalico_host_rt::transport::{MessageParams, Transport, TransportError};

pub struct RouterTransport {
    pub router: Arc<Mutex<PassthroughRouter>>,
    pub mcu: McuHandle,
    pub queue: CommandQueueId,
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
        let mut router = self.router.lock().unwrap();
        let entry = PassthroughEntry::new(wire_bytes, 0, 0, NotifyId::none());
        router
            .push(self.mcu, self.queue, entry)
            .map_err(|e| TransportError::Parse(format!("router push: {e}")))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
