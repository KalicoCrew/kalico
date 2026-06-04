#![allow(unsafe_code)]

use crate::error::KALICO_OK;
use crate::state::RuntimeContext;

/// # Safety
/// `ctx` must be non-null and point to a valid `RuntimeContext`.
/// `out_credit_epoch` may be null; if non-null it must be a valid `*mut u32`.
pub unsafe fn flush(_ctx: *mut RuntimeContext, _out_credit_epoch: *mut u32) -> i32 {
    KALICO_OK
}
