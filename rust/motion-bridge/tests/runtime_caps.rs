//! Task 10 — bridge bootstrap queries `RuntimeCaps` after Identify.
//!
//! The bootstrap path's transport call (`KalicoHostIo::kalico_call`) requires
//! a live reactor + serial port (real hardware or Renode), so the
//! end-to-end attach is exercised in higher-level integration tests. This
//! file pins the wire-format contract that the bridge's bootstrap helper
//! relies on: a `RuntimeCapsResponse` body produced by the protocol crate's
//! `Encode` round-trips back through the bridge's decode path.
//!
//! The companion test file `tests/sim_motion.rs` does not compile against
//! `sota-motion` HEAD due to unrelated stale producer call sites, so this
//! test lives in a fresh test target.

use kalico_protocol::codec::{Cursor, Decode, Encode};
use kalico_protocol::messages::{RuntimeCapsResponse, RUNTIME_CAPS_RESPONSE_BODY_LEN};

/// The bootstrap helper decodes a `RuntimeCapsResponse` body by calling
/// `RuntimeCapsResponse::decode_from(&mut Cursor::new(body))`. Any change to
/// the wire layout that breaks this round-trip would silently regress the
/// per-MCU sizing path landed in Task 10.
#[test]
fn query_runtime_caps_roundtrip_via_codec() {
    let original = RuntimeCapsResponse {
        max_control_points: 512,
        max_knot_vector_len: 524,
        max_degree: 10,
        curve_pool_n: 4,
    };

    let mut body = Vec::new();
    original.encode(&mut body);
    assert_eq!(
        body.len(),
        RUNTIME_CAPS_RESPONSE_BODY_LEN,
        "encoded body must match the documented 11-byte layout",
    );

    let mut c = Cursor::new(&body);
    let decoded = RuntimeCapsResponse::decode_from(&mut c)
        .expect("RuntimeCapsResponse decodes from its own encoding");
    assert_eq!(decoded, original);
}

/// A short body must surface as a decode error rather than panicking; the
/// bootstrap path maps the error to a `log::warn!` + fallback, so this
/// guards the "older firmware doesn't reply" branch from accidentally
/// becoming a hard panic.
#[test]
fn query_runtime_caps_short_body_errors() {
    let body = [0u8; 3];
    let mut c = Cursor::new(&body);
    let r = RuntimeCapsResponse::decode_from(&mut c);
    assert!(r.is_err(), "short body must fail to decode, got {r:?}");
}
