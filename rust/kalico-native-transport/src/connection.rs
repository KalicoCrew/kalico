//! Layer-0 stub: byte-pipe abstraction so the transport is testable without
//! actual USB-CDC hardware. The real implementation lives elsewhere
//! (kalico-host-rt or a future kalico-native-io crate); this trait is the
//! seam.

use std::collections::VecDeque;
use std::io;
use std::sync::{Arc, Mutex};

pub trait Connection: Send {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()>;
}

/// In-memory bidirectional connection for tests. Two `MockConnection`
/// halves share a pair of buffers — `tx` writes go into the peer's `rx`
/// view and vice versa.
#[derive(Clone, Debug)]
pub struct MockConnection {
    /// Bytes the host has written; the peer reads from here.
    pub host_to_peer: Arc<Mutex<VecDeque<u8>>>,
    /// Bytes the peer has written; the host reads from here.
    pub peer_to_host: Arc<Mutex<VecDeque<u8>>>,
}

impl MockConnection {
    pub fn pair() -> (HostHalf, PeerHalf) {
        let h2p = Arc::new(Mutex::new(VecDeque::new()));
        let p2h = Arc::new(Mutex::new(VecDeque::new()));
        (
            HostHalf {
                tx: h2p.clone(),
                rx: p2h.clone(),
            },
            PeerHalf {
                tx: p2h,
                rx: h2p,
            },
        )
    }
}

/// Host-side connection half: writes go onto the wire toward the MCU, reads
/// pull bytes the MCU has emitted.
#[derive(Debug, Clone)]
pub struct HostHalf {
    pub tx: Arc<Mutex<VecDeque<u8>>>,
    pub rx: Arc<Mutex<VecDeque<u8>>>,
}

/// Peer-side helper for tests pretending to be the MCU.
#[derive(Debug, Clone)]
pub struct PeerHalf {
    pub tx: Arc<Mutex<VecDeque<u8>>>,
    pub rx: Arc<Mutex<VecDeque<u8>>>,
}

impl Connection for HostHalf {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut q = self.rx.lock().unwrap();
        let n = buf.len().min(q.len());
        for slot in buf.iter_mut().take(n) {
            *slot = q.pop_front().unwrap();
        }
        Ok(n)
    }
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let mut q = self.tx.lock().unwrap();
        q.extend(buf.iter().copied());
        Ok(())
    }
}

impl PeerHalf {
    pub fn read_all_pending(&self) -> Vec<u8> {
        let mut q = self.rx.lock().unwrap();
        let v: Vec<u8> = q.iter().copied().collect();
        q.clear();
        v
    }

    pub fn write(&self, buf: &[u8]) {
        let mut q = self.tx.lock().unwrap();
        q.extend(buf.iter().copied());
    }
}
