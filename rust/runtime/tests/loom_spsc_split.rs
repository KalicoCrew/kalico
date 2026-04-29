//! Loom model of the half-split SPSC producer/consumer pattern.
//!
//! `heapless::spsc::Queue` doesn't natively swap to loom atomics, so this
//! test models the abstract pattern using loom primitives (head/tail
//! cursors over a fixed-size ring) instead. The real `heapless::spsc` is
//! covered separately by host-target integration tests + criterion stress
//! tests; loom's job here is to confirm the *ownership-discipline shape*
//! has no observable inconsistency under arbitrary interleavings — i.e.
//! the consumer never observes pops that the producer didn't first push.
//!
//! The producer pushes exactly one item and the consumer pops at most
//! one. Loom's branch budget is tight, so the model deliberately stays
//! small; broader coverage of the SPSC machinery lives in proptest /
//! criterion suites that the regular host CI runs.
//!
//! Run with:
//!
//! ```bash
//! RUSTFLAGS="--cfg loom" cargo test -p runtime --release --test loom_spsc_split
//! ```

#![cfg(loom)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use loom::sync::Arc;
use loom::sync::atomic::{AtomicUsize, Ordering};
use loom::thread;

#[test]
fn loom_spsc_pattern_producer_consumer() {
    loom::model(|| {
        const N: usize = 4;
        let head = Arc::new(AtomicUsize::new(0));
        let tail = Arc::new(AtomicUsize::new(0));

        let p_head = head.clone();
        let p_tail = tail.clone();
        let producer = thread::spawn(move || {
            // Producer pushes exactly ONE item. Empty ring at start, so
            // next_head=1, tail=0 — never full. Single iteration; no
            // unbounded retry (loom's branch budget would explode).
            let h = p_head.load(Ordering::Relaxed);
            let t = p_tail.load(Ordering::Acquire);
            let next_h = (h + 1) % N;
            assert_ne!(next_h, t, "fresh ring should never be full on first push");
            p_head.store(next_h, Ordering::Release);
        });

        let c_head = head.clone();
        let c_tail = tail.clone();
        let consumer = thread::spawn(move || {
            // Consumer makes a small finite number of attempts to pop the
            // single item. If it sees an empty ring on every attempt,
            // that's a legal outcome (the producer might not have run yet
            // in this loom interleaving).
            for _ in 0..3 {
                let h = c_head.load(Ordering::Acquire);
                let t = c_tail.load(Ordering::Relaxed);
                if h == t {
                    continue;
                }
                c_tail.store((t + 1) % N, Ordering::Release);
                // Cardinality assertion: the producer pushes 1, so the
                // consumer can pop at most 1. We popped exactly one and
                // we're done.
                break;
            }
        });

        producer.join().expect("producer thread panicked");
        consumer.join().expect("consumer thread panicked");

        // Final invariant: head ≥ tail (modulo wrap), never the other way
        // around. With one push and at most one pop, the legal terminal
        // states are (head=1, tail=0) or (head=1, tail=1).
        let final_head = head.load(Ordering::Acquire);
        let final_tail = tail.load(Ordering::Acquire);
        assert!(
            final_tail == 0 || final_tail == final_head,
            "tail outran head: head={final_head}, tail={final_tail}"
        );
    });
}
