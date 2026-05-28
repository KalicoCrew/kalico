#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Loom model of the curve-pool alloc / confirm-retired race (Phase 12).
//!
//! Spec §10.2 + Round-1 Codex #4 ordering invariant: the foreground
//! `try_alloc_and_load` writes the curve data BEFORE bumping `current_gen`
//! with Release; the ISR's `lookup` does an Acquire load of `current_gen`
//! before dereferencing the curve. Concurrently, the ISR-side trace stream
//! drives foreground `confirm_retired(handle)` which publishes
//! `last_retired_gen = handle.gen`.
//!
//! The invariant under loom: at any instant, the live (slot, gen) handle
//! observable to the ISR must have a coherent curve-data view (no torn
//! mix of pre-publish and post-publish memory). The model uses a single
//! "data" word per slot to stand in for the curve's bulk data; loom
//! verifies the writer never publishes a new `current_gen` before the
//! data write lands, and the reader never observes the new gen with the
//! old data.
//!
//! Run with:
//!
//! ```bash
//! RUSTFLAGS="--cfg loom" cargo test -p runtime --release --test loom_curve_pool_alloc
//! ```

#![cfg(loom)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use loom::sync::Arc;
use loom::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use loom::thread;

#[test]
fn loom_alloc_confirm_retired_race() {
    loom::model(|| {
        // One slot — minimal model. Initial state: empty (gen=0,
        // last_retired=0). Foreground will alloc → publish gen=1; ISR will
        // observe and signal retirement.
        let current_gen = Arc::new(AtomicU16::new(0));
        let last_retired_gen = Arc::new(AtomicU16::new(0));
        // "data" stands in for the curve's bulk-data word. Foreground
        // writes it BEFORE bumping current_gen.
        let curve_data = Arc::new(AtomicU32::new(0));

        let fg_cur = current_gen.clone();
        let fg_last = last_retired_gen.clone();
        let fg_data = curve_data.clone();
        let foreground = thread::spawn(move || {
            // Alloc predicate: current_gen == last_retired_gen.
            let cur = fg_cur.load(Ordering::Acquire);
            let last = fg_last.load(Ordering::Acquire);
            if cur != last {
                return; // Slot busy in this interleaving — skip.
            }
            // Write data BEFORE bumping gen (Round-1 Codex #4).
            fg_data.store(0xCAFE_BABE, Ordering::Release);
            // Release-store the new gen. The ISR's Acquire-load on
            // current_gen synchronizes with this store, so any ISR-side
            // read of `data` after observing the new gen sees the published
            // bulk data.
            fg_cur.store(cur.wrapping_add(1), Ordering::Release);
        });

        let isr_cur = current_gen.clone();
        let isr_last = last_retired_gen.clone();
        let isr_data = curve_data.clone();
        let isr = thread::spawn(move || {
            // ISR-side lookup: observe a (slot, gen) handle from the wire,
            // verify it matches `current_gen`, then dereference the data.
            // The HANDLE the ISR observed in the trace stream encodes the
            // gen it expects.
            let observed_gen = isr_cur.load(Ordering::Acquire);
            if observed_gen == 0 {
                // Foreground hasn't published yet in this interleaving; no
                // segment refers to this slot, so no lookup happens. Legal.
                return;
            }
            // The ISR is now resolving handle{slot=0, gen=observed_gen}.
            // Per the contract, the data load (Acquire) MUST see the
            // foreground's pre-bump-gen Release write.
            let d = isr_data.load(Ordering::Acquire);
            assert_eq!(
                d, 0xCAFE_BABE,
                "torn read: ISR observed current_gen={observed_gen} but \
                 stale data={d:#x} — Acquire/Release sync failed"
            );

            // Simulate the segment retiring. Foreground reclaim observes
            // SEGMENT_END(handle) and bumps last_retired_gen.
            // (In production this happens on a different drain pass; here
            // we model the eventual reclaim hand-off as part of the same
            // ISR thread for loom branch-budget reasons — semantic effect
            // is identical.)
            isr_last.store(observed_gen, Ordering::Release);
        });

        foreground.join().expect("fg thread panicked");
        isr.join().expect("isr thread panicked");

        // Final invariant: last_retired_gen ≤ current_gen (modulo wrap is
        // out of scope for a single-alloc model). With 0 or 1 alloc, the
        // legal terminal states are (cur=0, last=0) or (cur=1, last∈{0,1}).
        let cur = current_gen.load(Ordering::Acquire);
        let last = last_retired_gen.load(Ordering::Acquire);
        assert!(
            last <= cur,
            "last_retired_gen ({last}) outran current_gen ({cur})"
        );
    });
}
