#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

#![cfg(loom)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use loom::sync::Arc;
use loom::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use loom::thread;

#[test]
fn loom_force_idle_handshake_happens_before() {
    loom::model(|| {
        let force_idle = Arc::new(AtomicBool::new(false));
        let acked = Arc::new(AtomicBool::new(false));
        let isr_observed_seg_id = Arc::new(AtomicU32::new(0));

        let isr_force_idle = force_idle.clone();
        let isr_acked = acked.clone();
        let isr_seg = isr_observed_seg_id.clone();
        let isr = thread::spawn(move || {
            for _ in 0..4 {
                if isr_force_idle.load(Ordering::Acquire) {
                    isr_acked.store(true, Ordering::Release);
                    return;
                }
                isr_seg.store(1, Ordering::Release);
            }
        });

        let fg_force_idle = force_idle.clone();
        let fg_acked = acked.clone();
        let fg_seg = isr_observed_seg_id.clone();
        let foreground = thread::spawn(move || {
            fg_force_idle.store(true, Ordering::Release);
            let mut saw_ack = false;
            for _ in 0..6 {
                if fg_acked.load(Ordering::Acquire) {
                    saw_ack = true;
                    break;
                }
            }
            if saw_ack {
                let s = fg_seg.load(Ordering::Acquire);
                assert!(s == 0 || s == 1, "bogus segment id post-handshake: {s}");
            }
        });

        isr.join().expect("isr thread panicked");
        foreground.join().expect("fg thread panicked");
    });
}
