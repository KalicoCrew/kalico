//! Off-loop executor for CoE mailbox traffic.
//!
//! SDO transactions block for milliseconds to seconds (mailbox round trips,
//! the drive's internal EEPROM save, SOEM's 700 ms per-attempt timeout). The
//! DC loop must keep process data flowing every cycle — a slave in OP drops
//! to SAFE-OP (ErC1.1, emergency 0x8700) when cyclic frames pause past its
//! sync watchdog. So mailbox work runs on this dedicated thread while the DC
//! loop keeps cycling; SOEM's Linux port serializes socket access internally,
//! which is exactly the concurrent PDO-thread + mailbox-thread split its own
//! examples use.
//!
//! Requests execute strictly in submission order (single worker, FIFO
//! channel), preserving write-then-readback semantics per client call.
//!
//! The worker shares SOEM's one raw socket with the DC loop, so its
//! scheduling matters as much as its existence: see
//! [`crate::thread_prio::assume_companion_rt_scheduling`] for why the
//! hardware endpoint must run it as a pinned low-priority SCHED_FIFO
//! companion rather than SCHED_OTHER.

use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::thread::JoinHandle;

use kalico_protocol::messages::{SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse};

use crate::sdo::{execute_sdo_read, execute_sdo_write, SdoBus};
use crate::thread_prio::{assume_companion_rt_scheduling, demote_to_normal_scheduling};

pub enum WorkerScheduling {
    RealtimeCompanion { cpu: usize, priority: i32 },
    Normal,
}

pub enum MailboxRequest {
    SdoRead {
        correlation_id: u32,
        msg: SdoRead,
    },
    SdoWrite {
        correlation_id: u32,
        msg: SdoWrite,
    },
    WriteLimits {
        correlation_id: u32,
        ferr_counts: u32,
        torque_tenth_pct: u16,
        restore: bool,
    },
}

pub enum MailboxReply {
    SdoRead {
        correlation_id: u32,
        resp: SdoReadResponse,
    },
    SdoWrite {
        correlation_id: u32,
        resp: SdoWriteResponse,
    },
    WriteLimits {
        correlation_id: u32,
        rc: i32,
        ferr_counts: u32,
        torque_tenth_pct: u16,
        restore: bool,
    },
}

pub struct MailboxWorker {
    requests: Sender<MailboxRequest>,
    replies: Receiver<MailboxReply>,
    handle: Option<JoinHandle<()>>,
}

impl MailboxWorker {
    pub fn spawn<B, L>(mut bus: B, mut write_limits: L, scheduling: WorkerScheduling) -> Self
    where
        B: SdoBus + Send + 'static,
        L: FnMut(u32, u16) -> i32 + Send + 'static,
    {
        let (req_tx, req_rx) = channel::<MailboxRequest>();
        let (rep_tx, rep_rx) = channel::<MailboxReply>();
        let handle = std::thread::Builder::new()
            .name("ec-rt-mailbox".into())
            .spawn(move || {
                match scheduling {
                    WorkerScheduling::RealtimeCompanion { cpu, priority } => {
                        assume_companion_rt_scheduling(cpu, priority)
                    }
                    WorkerScheduling::Normal => demote_to_normal_scheduling(),
                }
                while let Ok(req) = req_rx.recv() {
                    let reply = match req {
                        MailboxRequest::SdoRead {
                            correlation_id,
                            msg,
                        } => MailboxReply::SdoRead {
                            correlation_id,
                            resp: execute_sdo_read(&mut bus, &msg),
                        },
                        MailboxRequest::SdoWrite {
                            correlation_id,
                            msg,
                        } => MailboxReply::SdoWrite {
                            correlation_id,
                            resp: execute_sdo_write(&mut bus, &msg),
                        },
                        MailboxRequest::WriteLimits {
                            correlation_id,
                            ferr_counts,
                            torque_tenth_pct,
                            restore,
                        } => MailboxReply::WriteLimits {
                            correlation_id,
                            rc: write_limits(ferr_counts, torque_tenth_pct),
                            ferr_counts,
                            torque_tenth_pct,
                            restore,
                        },
                    };
                    if rep_tx.send(reply).is_err() {
                        return;
                    }
                }
            })
            .expect("spawn ec-rt-mailbox thread");
        Self {
            requests: req_tx,
            replies: rep_rx,
            handle: Some(handle),
        }
    }

    /// Queue a mailbox transaction; never blocks. Panics if the worker thread
    /// died — that is a bug, not a runtime condition to recover from.
    pub fn submit(&self, req: MailboxRequest) {
        self.requests
            .send(req)
            .expect("ec-rt-mailbox thread is gone");
    }

    /// Non-blocking poll for one completed transaction; call from the DC loop
    /// each cycle until it returns None.
    pub fn try_recv(&self) -> Option<MailboxReply> {
        match self.replies.try_recv() {
            Ok(reply) => Some(reply),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                panic!("ec-rt-mailbox thread is gone");
            }
        }
    }
}

impl Drop for MailboxWorker {
    fn drop(&mut self) {
        let (sink, _) = channel();
        let _ = std::mem::replace(&mut self.requests, sink);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests;
