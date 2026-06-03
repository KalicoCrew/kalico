//! `NativeCall`: the single request/response primitive the curve/segment
//! producers need. Hoisted off `KalicoHostIo` so the producer functions can
//! drive any kalico-native peer (a serial MCU via `KalicoHostIo`, or a
//! same-host EtherCAT RT process via `UnixNativeConn`) without caring which.
//!
//! One frame out (`kind` + `body`), one frame in (matching `correlation_id`).
//! Object-safe: callers use `&dyn NativeCall`.

use std::time::Duration;

use kalico_protocol::MessageKind;

use crate::transport::TransportError;

pub trait NativeCall: Send + Sync {
    /// Issue a kalico-native control-channel call: send `kind` + `body`, block
    /// until the correlation-matched response arrives or `timeout` elapses.
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError>;
}

impl NativeCall for crate::host_io::KalicoHostIo {
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        crate::host_io::KalicoHostIo::kalico_call(self, kind, body, timeout)
    }
}
