use kalico_ethercat_rt::server::FrameServer;
use std::os::unix::net::UnixStream;
use std::thread;
use std::time::Duration;

#[test]
fn frame_server_detects_client_disconnect() {
    let socket_path = format!("/tmp/kalico-srv-disc-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    let mut server = FrameServer::bind(&socket_path).expect("bind");

    let client = UnixStream::connect(&socket_path).expect("connect");

    // Poll until the server accepts the connection (non-blocking; bounded).
    let accept_deadline = std::time::Instant::now() + Duration::from_secs(1);
    loop {
        let _ = server.poll_commands();
        if server.client_connected() {
            break;
        }
        assert!(
            std::time::Instant::now() < accept_deadline,
            "server did not accept connection within 1 s"
        );
        thread::sleep(Duration::from_millis(1));
    }

    assert!(
        !server.client_disconnected(),
        "should not be disconnected while client is live"
    );

    drop(client);

    // Poll until EOF is observed (bounded).
    let eof_deadline = std::time::Instant::now() + Duration::from_secs(1);
    loop {
        let _ = server.poll_commands();
        if server.client_disconnected() {
            break;
        }
        assert!(
            std::time::Instant::now() < eof_deadline,
            "server did not detect disconnect within 1 s after client drop"
        );
        thread::sleep(Duration::from_millis(1));
    }

    assert!(
        server.client_disconnected(),
        "EOF must set disconnected flag"
    );

    let _ = std::fs::remove_file(&socket_path);
}

#[test]
fn frame_server_never_connected_is_not_disconnected() {
    let socket_path = format!("/tmp/kalico-srv-noconn-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    let server = FrameServer::bind(&socket_path).expect("bind");

    assert!(
        !server.client_disconnected(),
        "a server that never accepted a client must not report disconnected"
    );

    let _ = std::fs::remove_file(&socket_path);
}
