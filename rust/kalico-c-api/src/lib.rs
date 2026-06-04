#![cfg_attr(not(feature = "host"), no_std)]
#![allow(unsafe_code)]

mod nurbs_ffi;
mod runtime_ffi;

#[cfg(feature = "header-nurbs")]
pub use nurbs_ffi::exports::*;
#[cfg(feature = "header-runtime")]
pub use runtime_ffi::exports::*;

pub use runtime::error::*;

#[cfg(not(feature = "host"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe extern "C" {
        fn rust_panic_latch() -> !;
    }
    // SAFETY: rust_panic_latch is __noreturn (calls Klipper's shutdown() which never returns).
    unsafe { rust_panic_latch() }
}
