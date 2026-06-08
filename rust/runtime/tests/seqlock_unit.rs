#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use runtime::clock::{publish_widened_now, read_widened_now};
use runtime::state::SharedState;

#[test]
fn seqlock_round_trip() {
    let shared = SharedState::new();
    publish_widened_now(&shared, 0xDEAD_BEEF_CAFE_BABE);
    let got = read_widened_now(&shared);
    assert_eq!(got, 0xDEAD_BEEF_CAFE_BABE);
}

#[test]
fn seqlock_zero_initial_read() {
    // A reader that hits the seqlock before the ISR ever publishes must see
    // the default-zero value, not spin forever — the initial seq value (0)
    // is even, so the loop's `seq_before & 1 != 0` branch is not taken and
    // both halves load 0.
    let shared = SharedState::new();
    let got = read_widened_now(&shared);
    assert_eq!(got, 0);
}

#[test]
fn seqlock_multiple_writes() {
    let shared = SharedState::new();
    for i in 0u64..1000 {
        publish_widened_now(&shared, i.wrapping_mul(0x1234_5678));
    }
    let got = read_widened_now(&shared);
    assert_eq!(got, 999u64.wrapping_mul(0x1234_5678));
}

#[test]
fn seqlock_value_with_high_word_set() {
    // The lo/hi split is the load-bearing piece; verify a value with a
    // non-zero high u32 round-trips through the two AtomicU32 stores.
    let shared = SharedState::new();
    let v = (0x0000_0001u64 << 32) | 0xDEAD_BEEFu64;
    publish_widened_now(&shared, v);
    let got = read_widened_now(&shared);
    assert_eq!(got, v);
}

#[test]
fn seqlock_back_to_back_reads_consistent() {
    let shared = SharedState::new();
    publish_widened_now(&shared, 0xBADD_F00D_5EED_FACE);
    let a = read_widened_now(&shared);
    let b = read_widened_now(&shared);
    assert_eq!(a, b);
}
