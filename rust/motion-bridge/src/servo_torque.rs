use std::time::Duration;

use kalico_host_rt::native_call::NativeCall as _;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode as _, Encode as _};
use kalico_protocol::messages::{
    MessageKind, RestoreDriveLimitsResponse, SetDriveLimits, SetDriveLimitsResponse, SetTorque,
    SetTorqueResponse,
};

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

const DRIVE_LIMITS_TIMEOUT: Duration = Duration::from_secs(10);

pub fn send_drive_limits(
    conn: &UnixNativeConn,
    following_error_counts: u32,
    max_torque_tenth_pct: u16,
) -> Result<i32, String> {
    let body = SetDriveLimits {
        following_error_counts,
        max_torque_tenth_pct,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SetDriveLimits, body, DRIVE_LIMITS_TIMEOUT)
        .map_err(|e| format!("SetDriveLimits transport: {e:?}"))?;
    if kind != MessageKind::SetDriveLimitsResponse {
        return Err(format!(
            "SetDriveLimits: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = SetDriveLimitsResponse::decode(&resp)
        .map_err(|e| format!("SetDriveLimitsResponse decode: {e:?}"))?;
    Ok(r.result)
}

pub fn send_restore_drive_limits(conn: &UnixNativeConn) -> Result<i32, String> {
    let (kind, resp) = conn
        .kalico_call(
            MessageKind::RestoreDriveLimits,
            Vec::new(),
            DRIVE_LIMITS_TIMEOUT,
        )
        .map_err(|e| format!("RestoreDriveLimits transport: {e:?}"))?;
    if kind != MessageKind::RestoreDriveLimitsResponse {
        return Err(format!(
            "RestoreDriveLimits: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = RestoreDriveLimitsResponse::decode(&resp)
        .map_err(|e| format!("RestoreDriveLimitsResponse decode: {e:?}"))?;
    Ok(r.result)
}

#[cfg(test)]
mod tests;
