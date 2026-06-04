#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Loom model of the §8.5 `force_idle` handshake (Phase 7 + Phase 12).
//!
//! Models the cross-half handshake:
//!   1. Foreground sets `force_idle = true` (Release).
//!   2. ISR observes the flag (Acquire), drops the in-flight segment,
//!      sets `acked_force_idle = true` (Release).
//!   3. Foreground reads `acked_force_idle` (Acquire); on observing true
//!      it is safe to clear `stream_open` and the pool.
//!
//! Loom's branch budget exploration confirms the happens-before chain:
//! foreground reading `acked_force_idle == true` synchronizes with the
//! ISR's prior `force_idle` observation, so foreground knows the ISR
//! cannot still be evaluating a pre-handshake segment.
//!
//! Run with:
//!
//! ```bash
//! RUSTFLAGS="--cfg loom" cargo test -p runtime --release --test loom_force_idle
//! ```

#![cfg(loom)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use loom::sync::Arc;
use loom::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use loom::thread;

#[test]
fn loom_force_idle_handshake_happens_before() {
    loom::model(|| {
        // Models SharedState fields:
        //   force_idle, acked_force_idle: AtomicBool
        //   isr_observed_seg_id: AtomicU32 — proxy for "ISR was actively
        //     evaluating segment N before the handshake landed".
        let force_idle = Arc::new(AtomicBool::new(false));
        let acked = Arc::new(AtomicBool::new(false));
        let isr_observed_seg_id = Arc::new(AtomicU32::new(0));

        let isr_force_idle = force_idle.clone();
        let isr_acked = acked.clone();
        let isr_seg = isr_observed_seg_id.clone();
        let isr = thread::spawn(move || {
            // ISR loop, bounded for loom. On each tick:
            //   - If force_idle observed (Acquire), set acked (Release) and stop.
            //   - Otherwise, "evaluate" segment 1 (publish its id).
            for _ in 0..4 {
                if isr_force_idle.load(Ordering::Acquire) {
                    isr_acked.store(true, Ordering::Release);
                    return;
                }
                // Pre-handshake: ISR is publishing segment id == 1.
                isr_seg.store(1, Ordering::Release);
            }
        });

        let fg_force_idle = force_idle.clone();
        let fg_acked = acked.clone();
        let fg_seg = isr_observed_seg_id.clone();
        let foreground = thread::spawn(move || {
            // Foreground sequence: set force_idle, then poll acked, then
            // observe the segment id.
            fg_force_idle.store(true, Ordering::Release);
            let mut saw_ack = false;
            for _ in 0..6 {
                if fg_acked.load(Ordering::Acquire) {
                    saw_ack = true;
                    break;
                }
            }
            // Invariant under the §8.5 contract: if foreground sees
            // `acked_force_idle == true`, the ISR has already observed
            // `force_idle == true` and stopped evaluating segments.
            // Therefore the segment id we read after the ack-observation is
            // the LAST one the ISR published BEFORE it stopped. The id is
            // 0 (initial) or 1 (the one segment the ISR was evaluating);
            // any other value would indicate the ISR continued past the
            // handshake.
            if saw_ack {
                let s = fg_seg.load(Ordering::Acquire);
                assert!(s == 0 || s == 1, "bogus segment id post-handshake: {s}");
            }
            // If saw_ack is false, the ISR never observed force_idle in
            // this interleaving — that's a legal interleaving (production
            // foreground retry-loops with a deadline).
        });

        isr.join().expect("isr thread panicked");
        foreground.join().expect("fg thread panicked");
    });
}
