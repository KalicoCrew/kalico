use std::fs::OpenOptions;
use std::io::{self, Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::{AsRawFd, RawFd};
use std::time::Duration;

use kalico_native_transport::connection::Connection;
use kalico_native_transport::{ConnectionState, KalicoNativeTransport};

const SIM_SOCKET: &str = "/tmp/klipper_sim_socket";

struct PtyConn {
    fd: RawFd,
    file: std::fs::File,
}

impl Connection for PtyConn {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.file.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.file.write_all(buf)
    }
}

fn open_nonblocking_pty(path: &str) -> io::Result<PtyConn> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
        .open(path)?;
    let fd = file.as_raw_fd();
    Ok(PtyConn { fd, file })
}

#[allow(dead_code)]
fn pty_fd_unused(c: &PtyConn) -> RawFd {
    c.fd
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let _ = env_logger::try_init();

    println!("[sim_handshake] opening {SIM_SOCKET}");
    let conn = open_nonblocking_pty(SIM_SOCKET)?;

    let transport = KalicoNativeTransport::new(conn);
    println!("[sim_handshake] sending Identify, awaiting response...");
    let epoch = transport.identify(Duration::from_secs(5))?;
    let state = transport.state();

    println!("[sim_handshake] state = {state:?}");
    println!("[sim_handshake] reset_epoch = 0x{epoch:08x}");
    println!(
        "[sim_handshake] expected schema_hash = {}",
        hex(&kalico_protocol::SCHEMA_HASH)
    );

    assert!(
        matches!(state, ConnectionState::Identified { reset_epoch } if reset_epoch == epoch),
        "transport not in Identified state after identify()"
    );
    assert_ne!(epoch, 0, "reset_epoch must be nonzero");
    println!("[sim_handshake] PASS");
    Ok(())
}

fn hex(b: &[u8]) -> String {
    let mut s = String::with_capacity(b.len() * 2);
    for byte in b {
        s.push_str(&format!("{byte:02x}"));
    }
    s
}

extern crate libc;
