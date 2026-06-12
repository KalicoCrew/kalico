use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use kalico_protocol::messages::{SdoRead, SdoWrite};

use super::{MailboxReply, MailboxRequest, MailboxWorker, WorkerScheduling};
use crate::sdo::{DictObject, DictSdoBus, SdoBus};

fn dict() -> DictSdoBus {
    DictSdoBus::new([
        (
            (0x2001, 0x02),
            DictObject {
                size: 2,
                value: [250, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x2001, 0x03),
            DictObject {
                size: 2,
                value: [0x70, 0x0C, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
    ])
}

struct SlowBus<B> {
    inner: B,
    delay: Duration,
}

impl<B: SdoBus> SdoBus for SlowBus<B> {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        std::thread::sleep(self.delay);
        self.inner.read(index, subindex)
    }

    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        std::thread::sleep(self.delay);
        self.inner.write(index, subindex, bytes)
    }
}

fn drain_one(worker: &MailboxWorker, deadline: Duration) -> MailboxReply {
    let start = Instant::now();
    loop {
        if let Some(reply) = worker.try_recv() {
            return reply;
        }
        assert!(start.elapsed() < deadline, "no reply within {deadline:?}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn submit_never_blocks_while_transaction_is_slow() {
    let worker = MailboxWorker::spawn(
        SlowBus {
            inner: dict(),
            delay: Duration::from_millis(50),
        },
        |_, _| 0,
        WorkerScheduling::Normal,
    );
    let start = Instant::now();
    worker.submit(MailboxRequest::SdoWrite {
        correlation_id: 1,
        msg: SdoWrite {
            index: 0x2001,
            subindex: 0x02,
            size: 2,
            value: 1000,
        },
    });
    worker.submit(MailboxRequest::SdoRead {
        correlation_id: 2,
        msg: SdoRead {
            index: 0x2001,
            subindex: 0x02,
        },
    });
    assert!(
        start.elapsed() < Duration::from_millis(20),
        "submit must not wait for the bus"
    );

    match drain_one(&worker, Duration::from_secs(2)) {
        MailboxReply::SdoWrite {
            correlation_id,
            msg,
            resp,
        } => {
            assert_eq!(correlation_id, 1);
            assert_eq!((msg.index, msg.subindex), (0x2001, 0x02));
            assert_eq!(resp.result, 0);
        }
        _ => panic!("expected the write reply first"),
    }
    match drain_one(&worker, Duration::from_secs(2)) {
        MailboxReply::SdoRead {
            correlation_id,
            msg,
            resp,
        } => {
            assert_eq!(correlation_id, 2);
            assert_eq!((msg.index, msg.subindex), (0x2001, 0x02));
            assert_eq!(resp.result, 0);
            assert_eq!(
                i64::from_le_bytes([resp.data[0], resp.data[1], 0, 0, 0, 0, 0, 0]),
                1000,
                "read must observe the earlier write (FIFO order)"
            );
        }
        _ => panic!("expected the read reply second"),
    }
}

#[test]
fn replies_preserve_submission_order() {
    let worker = MailboxWorker::spawn(dict(), |_, _| 0, WorkerScheduling::Normal);
    for cid in 0..16u32 {
        worker.submit(MailboxRequest::SdoRead {
            correlation_id: cid,
            msg: SdoRead {
                index: 0x2001,
                subindex: 0x02,
            },
        });
    }
    for expected in 0..16u32 {
        match drain_one(&worker, Duration::from_secs(2)) {
            MailboxReply::SdoRead { correlation_id, .. } => {
                assert_eq!(correlation_id, expected)
            }
            _ => panic!("expected SdoRead reply"),
        }
    }
}

#[test]
fn write_limits_routes_through_callback_with_restore_flag() {
    let calls = Arc::new(AtomicU32::new(0));
    let seen = calls.clone();
    let worker = MailboxWorker::spawn(
        dict(),
        move |ferr, tq| {
            seen.store(ferr * 10 + u32::from(tq), Ordering::SeqCst);
            7
        },
        WorkerScheduling::Normal,
    );
    worker.submit(MailboxRequest::WriteLimits {
        correlation_id: 9,
        ferr_counts: 4,
        torque_tenth_pct: 2,
        restore: true,
    });
    match drain_one(&worker, Duration::from_secs(2)) {
        MailboxReply::WriteLimits {
            correlation_id,
            rc,
            ferr_counts,
            torque_tenth_pct,
            restore,
        } => {
            assert_eq!(
                (correlation_id, rc, ferr_counts, torque_tenth_pct, restore),
                (9, 7, 4, 2, true)
            );
            assert_eq!(calls.load(Ordering::SeqCst), 42);
        }
        _ => panic!("expected WriteLimits reply"),
    }
}

#[test]
fn drop_joins_the_worker_cleanly() {
    let worker = MailboxWorker::spawn(
        SlowBus {
            inner: dict(),
            delay: Duration::from_millis(20),
        },
        |_, _| 0,
        WorkerScheduling::Normal,
    );
    worker.submit(MailboxRequest::SdoRead {
        correlation_id: 1,
        msg: SdoRead {
            index: 0x2001,
            subindex: 0x02,
        },
    });
    drop(worker);
}
