use std::time::Duration;

use kalico_host_rt::native_call::NativeCall as _;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode as _, Encode as _};
use kalico_protocol::messages::{
    MessageKind, StartCapture, StartCaptureResponse, StopCapture, StopCaptureResponse,
};

// Capture start/stop only touch the command path (no CiA402 ladder); a stop
// additionally joins the writer thread, which flushes at most one fsync.
const CAPTURE_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_start_capture(
    conn: &UnixNativeConn,
    path: &str,
    started_utc: &str,
    drive_name: &str,
) -> Result<i32, String> {
    let body = StartCapture {
        path: path.to_owned(),
        started_utc: started_utc.to_owned(),
        drive_name: drive_name.to_owned(),
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::StartCapture, body, CAPTURE_TIMEOUT)
        .map_err(|e| format!("StartCapture transport: {e:?}"))?;
    if kind != MessageKind::StartCaptureResponse {
        return Err(format!(
            "StartCapture: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = StartCaptureResponse::decode(&resp)
        .map_err(|e| format!("StartCaptureResponse decode: {e:?}"))?;
    Ok(r.result)
}

pub fn send_stop_capture(conn: &UnixNativeConn) -> Result<StopCaptureResponse, String> {
    let (kind, resp) = conn
        .kalico_call(
            MessageKind::StopCapture,
            StopCapture.encoded_to_vec(),
            CAPTURE_TIMEOUT,
        )
        .map_err(|e| format!("StopCapture transport: {e:?}"))?;
    if kind != MessageKind::StopCaptureResponse {
        return Err(format!(
            "StopCapture: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    StopCaptureResponse::decode(&resp).map_err(|e| format!("StopCaptureResponse decode: {e:?}"))
}

#[cfg(test)]
mod tests;
