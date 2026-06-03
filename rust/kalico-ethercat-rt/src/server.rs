//! Unix-socket server: decode kalico-native command frames, hand them to a
//! handler, write framed responses. Non-blocking poll suited to a DC loop.

use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};

use kalico_native_transport::demux::{Demuxer, Frame};

use crate::wire::{Command, decode_command};

pub struct FrameServer {
    listener: UnixListener,
    conn: Option<UnixStream>,
    demux: Demuxer,
    buf: [u8; 4096],
}

impl core::fmt::Debug for FrameServer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FrameServer")
            .field("conn", &self.conn.is_some())
            .finish_non_exhaustive()
    }
}

impl FrameServer {
    /// Bind a unix-domain socket at `path`. Removes any stale socket file first.
    pub fn bind(path: &str) -> std::io::Result<Self> {
        let _ = std::fs::remove_file(path);
        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;
        // The endpoint runs as root (raw EtherCAT socket), so the bind would
        // otherwise leave the socket root-only. Relax to 0o666 so a non-root
        // client — the bench `ec-test-client` now, the klipper-user motion-bridge
        // later — can connect. This is a local-only UDS on a single-purpose host.
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o666))?;
        Ok(Self { listener, conn: None, demux: Demuxer::new(), buf: [0u8; 4096] })
    }

    /// Accept a pending client if we do not already have one.
    fn try_accept(&mut self) {
        if self.conn.is_none() {
            match self.listener.accept() {
                Ok((stream, _)) => {
                    // Non-blocking, NOT a read timeout. This server's poll runs
                    // inside the endpoint's single-threaded 1 ms EtherCAT DC loop.
                    // A blocking read — even a 200 µs timeout — stalls that loop:
                    // once a client connected, the per-cycle read latency pushed
                    // the PDO exchange past the A6-EC's sync watchdog and faulted
                    // the drive (wkc 3->1, errc1.1). Non-blocking returns WouldBlock
                    // instantly when the socket is idle; partial/burst frames simply
                    // accumulate in the demuxer across DC cycles.
                    let _ = stream.set_nonblocking(true);
                    self.conn = Some(stream);
                    eprintln!("ec-rt: client connected");
                }
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => eprintln!("ec-rt: accept error: {e}"),
            }
        }
    }

    /// Drain whatever bytes are available from the client and return decoded
    /// commands. Responses are written by the caller via [`respond`].
    pub fn poll_commands(&mut self) -> Vec<Command> {
        self.try_accept();
        let mut cmds = Vec::new();
        let Some(stream) = self.conn.as_mut() else { return cmds };
        match stream.read(&mut self.buf) {
            Ok(0) => {
                eprintln!("ec-rt: client disconnected");
                self.conn = None;
            }
            Ok(n) => {
                let (frames, errs) = self.demux.feed_slice(&self.buf[..n]);
                for e in &errs {
                    eprintln!("ec-rt: stream error: {e:?}");
                }
                for f in frames {
                    if let Frame::Kalico { channel, payload } = f {
                        match decode_command(channel, &payload) {
                            Ok(cmd) => cmds.push(cmd),
                            Err(e) => eprintln!("ec-rt: bad command: {e:?}"),
                        }
                    }
                }
            }
            Err(ref e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(e) => {
                eprintln!("ec-rt: read error: {e}");
                self.conn = None;
            }
        }
        cmds
    }

    /// Write a framed response to the connected client. Drops the connection on write error.
    pub fn respond(&mut self, frame: &[u8]) {
        if let Some(stream) = self.conn.as_mut() {
            if let Err(e) = stream.write_all(frame) {
                eprintln!("ec-rt: write error: {e}");
                self.conn = None;
            }
        }
    }
}
