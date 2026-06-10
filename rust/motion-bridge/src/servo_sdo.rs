use std::time::Duration;

use kalico_host_rt::native_call::NativeCall as _;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode as _, Encode as _};
use kalico_protocol::messages::{
    ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE, ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH,
    MessageKind, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse,
};

const SDO_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_sdo_read(
    conn: &UnixNativeConn,
    index: u16,
    subindex: u8,
) -> Result<SdoReadResponse, String> {
    let body = SdoRead { index, subindex }.encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoRead, body, SDO_TIMEOUT)
        .map_err(|e| format!("SdoRead transport: {e:?}"))?;
    if kind != MessageKind::SdoReadResponse {
        return Err(format!(
            "SdoRead: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    SdoReadResponse::decode(&resp).map_err(|e| format!("SdoReadResponse decode: {e:?}"))
}

pub fn send_sdo_write(
    conn: &UnixNativeConn,
    index: u16,
    subindex: u8,
    size: u8,
    value: i64,
) -> Result<SdoWriteResponse, String> {
    let body = SdoWrite {
        index,
        subindex,
        size,
        value,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoWrite, body, SDO_TIMEOUT)
        .map_err(|e| format!("SdoWrite transport: {e:?}"))?;
    if kind != MessageKind::SdoWriteResponse {
        return Err(format!(
            "SdoWrite: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    SdoWriteResponse::decode(&resp).map_err(|e| format!("SdoWriteResponse decode: {e:?}"))
}

pub fn failure_text(result: i32) -> String {
    match result {
        ERR_SDO_UNSUPPORTED_SIZE => "object size unsupported (must be 1..=4 bytes)".into(),
        ERR_SDO_VERIFY_MISMATCH => "readback mismatch".into(),
        ERR_SDO_TRANSPORT => "SDO transport failure (no CoE abort code)".into(),
        ERR_SDO_VALUE_RANGE => "value does not fit the object width".into(),
        code if code > 0 => format!("CoE abort 0x{:08x}", code as u32),
        code => format!("endpoint error {code}"),
    }
}

#[cfg(test)]
mod tests;
