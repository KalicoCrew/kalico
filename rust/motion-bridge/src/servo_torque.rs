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
mod tests {
    use super::*;
    use kalico_native_transport::demux::{Demuxer, Frame};
    use kalico_native_transport::frame::{CHANNEL_CONTROL, encode_frame};
    use kalico_native_transport::wire_helpers::{
        MESSAGE_VERSION_DEFAULT, decode_message_header, encode_message_header,
    };
    use std::io::{Read, Write};
    use std::os::unix::net::UnixStream;

    fn spawn_endpoint(peer: UnixStream, result: i32) -> std::sync::mpsc::Receiver<SetTorque> {
        spawn_endpoint_with_kind(peer, MessageKind::SetTorqueResponse, {
            let resp = SetTorqueResponse { result };
            resp.encoded_to_vec()
        })
    }

    fn spawn_endpoint_with_kind(
        mut peer: UnixStream,
        kind: MessageKind,
        body: Vec<u8>,
    ) -> std::sync::mpsc::Receiver<SetTorque> {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let mut demux = Demuxer::new();
            let mut buf = [0u8; 4096];
            loop {
                let n = match peer.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                let (frames, _e) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    if let Frame::Kalico { payload, .. } = f {
                        let (hdr, _body) =
                            decode_message_header(&payload).expect("valid message header");
                        let msg = SetTorque::decode(_body).expect("valid SetTorque body");
                        let _ = tx.send(msg);
                        let mut out = encode_message_header(
                            kind,
                            MESSAGE_VERSION_DEFAULT,
                            hdr.correlation_id,
                        )
                        .to_vec();
                        out.extend_from_slice(&body);
                        let frame = encode_frame(CHANNEL_CONTROL, &out);
                        peer.write_all(&frame).unwrap();
                        return;
                    }
                }
            }
        });
        rx
    }

    #[test]
    fn round_trips_enable_and_result() {
        let (client, server) = UnixStream::pair().unwrap();
        let rx = spawn_endpoint(server, 0);
        let conn = UnixNativeConn::from_stream(client);
        let result = send_set_torque(&conn, true, 42_000).expect("call");
        assert_eq!(result, 0);
        let seen = rx.recv().expect("endpoint saw the command");
        assert_eq!(seen.value, 1);
        assert_eq!(seen.execute_at_ns, 42_000);
    }

    #[test]
    fn surfaces_nonzero_result() {
        let (client, server) = UnixStream::pair().unwrap();
        let _rx = spawn_endpoint(server, -312);
        let conn = UnixNativeConn::from_stream(client);
        assert_eq!(send_set_torque(&conn, false, 99).expect("call"), -312);
    }

    #[test]
    fn transport_error_is_an_err() {
        let (client, server) = UnixStream::pair().unwrap();
        drop(server); // peer gone
        let conn = UnixNativeConn::from_stream(client);
        assert!(send_set_torque(&conn, true, 1).is_err());
    }

    #[test]
    fn wrong_kind_response_is_rejected() {
        let (client, server) = UnixStream::pair().unwrap();
        let _rx = spawn_endpoint_with_kind(server, MessageKind::PushPiecesResponse, vec![0u8; 20]);
        let conn = UnixNativeConn::from_stream(client);
        let err = send_set_torque(&conn, true, 42_000).expect_err("should error on wrong kind");
        assert!(err.contains("unexpected response kind"));
    }
}
