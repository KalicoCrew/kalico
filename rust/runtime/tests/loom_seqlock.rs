#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Loom model of the §11.4 widened-clock seqlock.
//!
//! Loom exhaustively explores the legal interleavings of a writer and a
//! reader thread under the relaxed memory model. The model here mirrors the
//! actual `publish_widened_now` / `read_widened_now` shape but uses
//! `loom::sync::atomic` so loom can track the orderings.
//!
//! The test deliberately publishes ONE value (not a sequence of writes)
//! and bounds the reader to a small finite retry budget so loom's branch
//! exploration terminates. Loom's strength is in the tiny-state-space
//! exhaustive search, not in long-running models.
//!
//! Run with:
//!
//! ```bash
//! RUSTFLAGS="--cfg loom" cargo test -p runtime --release --test loom_seqlock
//! ```

#![cfg(loom)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use loom::sync::Arc;
use loom::sync::atomic::{AtomicU32, Ordering};
use loom::thread;

#[test]
fn loom_seqlock_writer_reader() {
    loom::model(|| {
        let lo = Arc::new(AtomicU32::new(0));
        let hi = Arc::new(AtomicU32::new(0));
        let seq = Arc::new(AtomicU32::new(0));

        let writer_lo = lo.clone();
        let writer_hi = hi.clone();
        let writer_seq = seq.clone();

        let writer = thread::spawn(move || {
            writer_seq.store(1, Ordering::Release); // → odd
            writer_lo.store(0xCAFE_BABE, Ordering::Release);
            writer_hi.store(0xDEAD_BEEF, Ordering::Release);
            writer_seq.store(2, Ordering::Release); // → even
        });

        let mut observation: Option<(u32, u32)> = None;
        for _ in 0..4 {
            let s_before = seq.load(Ordering::Acquire);
            if s_before & 1 != 0 {
                continue;
            }
            let l = lo.load(Ordering::Acquire);
            let h = hi.load(Ordering::Acquire);
            let s_after = seq.load(Ordering::Acquire);
            if s_after == s_before {
                observation = Some((l, h));
                break;
            }
        }

        // The observed pair must be from the legal set: either the
        // pre-write (0, 0) or the post-write (CAFEBABE, DEADBEEF). A torn
        // mix-and-match (e.g. CAFEBABE × 0) would violate the seqlock
        // invariant and surface the bug.
        if let Some((l, h)) = observation {
            let coherent = (l, h) == (0, 0) || (l, h) == (0xCAFE_BABE, 0xDEAD_BEEF);
            assert!(coherent, "torn read: lo={l:#x} hi={h:#x}");
        }
        // observation == None is also fine — it means the reader bailed
        // without observing a stable seq, which is allowed under the
        // seqlock contract (the foreground production code is a real
        // unbounded retry loop; the test bound exists only for loom).

        writer.join().expect("writer thread panicked");
    });
}
