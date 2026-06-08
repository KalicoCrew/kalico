use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};

use kalico_native_transport::demux::{Demuxer, Frame};

use crate::wire::{decode_command, Command};

pub struct FrameServer {
    listener: UnixListener,
    conn: Option<UnixStream>,
    demux: Demuxer,
    buf: [u8; 4096],
    /// Set on peer EOF, read error, write error, or deliberate `respond_and_close`.
    /// Once true the server refuses further accept calls (one-shot session contract).
    session_ended: bool,
}

impl core::fmt::Debug for FrameServer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FrameServer")
            .field("conn", &self.conn.is_some())
            .finish_non_exhaustive()
    }
}

impl FrameServer {
    pub fn bind(path: &str) -> std::io::Result<Self> {
        let _ = std::fs::remove_file(path);
        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;
        // 0o666: endpoint runs as root; non-root clients (motion-bridge) must connect.
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o666))?;
        Ok(Self {
            listener,
            conn: None,
            demux: Demuxer::new(),
            buf: [0u8; 4096],
            session_ended: false,
        })
    }

    fn try_accept(&mut self) {
        if self.conn.is_none() && !self.session_ended {
            match self.listener.accept() {
                Ok((stream, _)) => {
                    // Non-blocking: a blocking read stalls the 1 ms DC loop and
                    // pushes the PDO exchange past the A6-EC sync watchdog (wkc 3→1).
                    let _ = stream.set_nonblocking(true);
                    self.conn = Some(stream);
                    eprintln!("ec-rt: client connected");
                }
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => eprintln!("ec-rt: accept error: {e}"),
            }
        }
    }

    pub fn poll_commands(&mut self) -> Vec<Command> {
        self.try_accept();
        let mut cmds = Vec::new();
        let Some(stream) = self.conn.as_mut() else {
            return cmds;
        };
        match stream.read(&mut self.buf) {
            Ok(0) => {
                eprintln!("ec-rt: client disconnected");
                self.conn = None;
                self.session_ended = true;
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
                self.session_ended = true;
            }
        }
        cmds
    }

    pub fn respond(&mut self, frame: &[u8]) {
        if let Some(stream) = self.conn.as_mut() {
            if let Err(e) = stream.write_all(frame) {
                eprintln!("ec-rt: write error: {e}");
                self.conn = None;
                self.session_ended = true;
            }
        }
    }

    pub fn client_connected(&self) -> bool {
        self.conn.is_some()
    }

    pub fn session_ended(&self) -> bool {
        self.session_ended
    }

    pub fn respond_and_close(&mut self, frame: &[u8]) {
        if self.conn.is_none() {
            eprintln!("ec-rt: respond_and_close called with no client — frame dropped");
            self.session_ended = true;
            return;
        }
        self.respond(frame);
        self.conn = None;
        self.session_ended = true;
    }
}
