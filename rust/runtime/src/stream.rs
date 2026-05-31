//! Stream lifecycle — reduced to the `flush` command surface.
//!
//! `open` / `arm` / `terminal` (segment-era stubs) and `FgStreamState` have
//! been removed. The piece ring is the only live data path; `flush` remains
//! as a no-op shell until the host-side force-idle mechanism is wired.

#![allow(unsafe_code)]

use crate::error::KALICO_OK;
use crate::state::RuntimeContext;

/// `force_idle` handshake (§8.5). No-op shell — the real cancel logic
/// lands when the host-side rewrite reaches this layer.
///
/// # Safety
/// `ctx` must be non-null and point to a valid `RuntimeContext`.
/// `out_credit_epoch` may be null; if non-null it must be a valid `*mut u32`.
pub unsafe fn flush(_ctx: *mut RuntimeContext, _out_credit_epoch: *mut u32) -> i32 {
    KALICO_OK
}
