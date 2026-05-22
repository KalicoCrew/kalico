//! A6 — Status snapshot monotonicity under concurrent reads. Spec §3.6.
//!
//! 8 reader threads, 1 writer thread. Writer publishes monotonically-increasing
//! generation values into an `ArcSwap<Snapshot>`; each reader's observed
//! sequence must be monotonically non-decreasing (no torn snapshots, no
//! K-then-K-1 inversions).
//!
//! Reads-per-reader chosen so the test runs in <1s under release. Debug builds
//! still finish in a few seconds.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use arc_swap::ArcSwap;

#[derive(Debug, Clone)]
struct Snapshot {
    generation: u64,
}

#[test]
fn arcswap_reads_are_monotonic_under_concurrent_writer() {
    const READERS: usize = 8;
    const READS: usize = 200_000;
    const WRITES: u64 = 50_000;

    let snapshot: Arc<ArcSwap<Snapshot>> =
        Arc::new(ArcSwap::from_pointee(Snapshot { generation: 0 }));
    let writer_done = Arc::new(AtomicBool::new(false));

    // Writer.
    let writer_snap = Arc::clone(&snapshot);
    let writer_done_w = Arc::clone(&writer_done);
    let writer = thread::spawn(move || {
        for g in 1..=WRITES {
            writer_snap.store(Arc::new(Snapshot { generation: g }));
        }
        writer_done_w.store(true, Ordering::SeqCst);
    });

    // Readers.
    let mut handles = Vec::with_capacity(READERS);
    for _ in 0..READERS {
        let snap = Arc::clone(&snapshot);
        let writer_done_r = Arc::clone(&writer_done);
        handles.push(thread::spawn(move || {
            let mut last: u64 = 0;
            for _ in 0..READS {
                if writer_done_r.load(Ordering::Relaxed) {
                    break;
                }
                let cur = snap.load();
                let g = cur.generation;
                assert!(
                    g >= last,
                    "non-monotonic snapshot read: saw generation {g} after {last}"
                );
                last = g;
            }
        }));
    }

    writer.join().unwrap();
    for h in handles {
        h.join().unwrap();
    }
}

#[test]
fn arcswap_load_returns_complete_snapshot() {
    // Sanity: each load() returns a self-consistent snapshot — no torn read.
    // Snapshot is small (single u64) so Rust's Arc swap atomicity makes this
    // trivially true; we're sanity-checking the test infrastructure.
    let snap: Arc<ArcSwap<Snapshot>> = Arc::new(ArcSwap::from_pointee(Snapshot { generation: 42 }));
    let g1 = snap.load();
    snap.store(Arc::new(Snapshot { generation: 43 }));
    let g2 = snap.load();
    assert_eq!(g1.generation, 42);
    assert_eq!(g2.generation, 43);
}
