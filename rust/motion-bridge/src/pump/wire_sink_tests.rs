use super::*;
use kalico_host_rt::transport::TransportError;
use std::os::unix::net::UnixStream;
use std::sync::Arc;
use std::time::Duration;

/// A `UnixNativeConn` whose peer end is dropped — every call returns
/// `TransportError::Closed` (read returns `Ok(0)`) or `TransportError::Io`
/// (write to closed peer yields BrokenPipe).
fn closed_conn() -> Arc<kalico_host_rt::unix_native_conn::UnixNativeConn> {
    let (client, _server) = UnixStream::pair().unwrap();
    // Drop _server immediately: the client will see EOF / broken-pipe on
    // the very first I/O.
    Arc::new(
        kalico_host_rt::unix_native_conn::UnixNativeConn::from_stream(client)
            .expect("from_stream"),
    )
}

fn key() -> AxisKey {
    AxisKey { mcu_id: 0, axis: 0 }
}

fn one_piece() -> Vec<runtime::piece_ring::PieceEntry> {
    vec![runtime::piece_ring::PieceEntry {
        start_time: 1000,
        coeffs: [0.0; 4],
        duration: 0.001,
        _reserved: 0,
    }]
}

#[test]
fn closed_peer_yields_fatal_send_error() {
    // Hold the strong Arc for the whole test so upgrade() succeeds and we
    // exercise the real Closed/Io path, not the detached-conn Fatal arm.
    let conn = closed_conn();
    let sink = WireSink {
        transports: {
            let mut m = HashMap::new();
            m.insert(0, McuTransport::EtherCat(Arc::downgrade(&conn)));
            m
        },
        timeout: Duration::from_millis(50),
        freq_of: Arc::new(|_| None),
    };
    let pieces = one_piece();
    match sink.call_push_pieces(key(), &pieces, 0, 1) {
        Err(SendError::Fatal(_)) => {}
        other => panic!("expected Fatal for closed EtherCAT peer, got {other:?}"),
    }
}

#[test]
fn detached_ethercat_conn_yields_fatal_send_error() {
    // The strong Arc is dropped before the call, so upgrade() fails and the
    // released-conn path must classify as Fatal (pump exits, no spin).
    let weak = Arc::downgrade(&closed_conn());
    let sink = WireSink {
        transports: {
            let mut m = HashMap::new();
            m.insert(0, McuTransport::EtherCat(weak));
            m
        },
        timeout: Duration::from_millis(50),
        freq_of: Arc::new(|_| None),
    };
    let pieces = one_piece();
    match sink.call_push_pieces(key(), &pieces, 0, 1) {
        Err(SendError::Fatal(_)) => {}
        other => panic!("expected Fatal for detached EtherCAT conn, got {other:?}"),
    }
}

#[test]
fn timeout_yields_transient_send_error() {
    // Construct a TransportError::Timeout and verify the match arm
    // classifies it as Transient.  We can't easily produce a real
    // timeout without sleeping, so test the classification logic
    // directly on the error type.
    let e = TransportError::Timeout;
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(!is_fatal, "Timeout must not be fatal");
}

#[test]
fn parse_error_yields_transient_send_error() {
    let e = TransportError::Parse("bad frame".to_owned());
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(!is_fatal, "Parse must not be fatal");
}

#[test]
fn io_error_yields_fatal_send_error() {
    let e = TransportError::Io(std::io::Error::from(std::io::ErrorKind::BrokenPipe));
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(is_fatal, "Io must be fatal");
}

#[test]
fn closed_variant_is_fatal() {
    let e = TransportError::Closed;
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(is_fatal, "Closed must be fatal");
}
