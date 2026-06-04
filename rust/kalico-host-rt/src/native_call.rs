use std::time::Duration;

use kalico_protocol::MessageKind;

use crate::transport::TransportError;

pub trait NativeCall: Send + Sync {
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
