use std::time::Duration;

use kalico_host_rt::native_call::NativeCall as _;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode as _, Encode as _};
use kalico_protocol::messages::{MessageKind, SetTorque, SetTorqueResponse};

/// Worst-case enable: 3000 DC cycles of ladder (~3 s) plus margin.
const SET_TORQUE_TIMEOUT: Duration = Duration::from_secs(8);

pub fn send_set_torque(
    conn: &UnixNativeConn,
    value: bool,
    execute_at_ns: u64,
) -> Result<i32, String> {
    let body = SetTorque {
        value: u8::from(value),
        execute_at_ns,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SetTorque, body, SET_TORQUE_TIMEOUT)
        .map_err(|e| format!("SetTorque transport: {e:?}"))?;
    if kind != MessageKind::SetTorqueResponse {
        return Err(format!(
            "SetTorque: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r =
        SetTorqueResponse::decode(&resp).map_err(|e| format!("SetTorqueResponse decode: {e:?}"))?;
    Ok(r.result)
}

#[cfg(test)]
mod tests;
