//! Kalico C-FFI staticlib. Umbrella for nurbs (Layer 0) and runtime (Layer 4).
//! Spec §2.2 / §3.2.
//!
//! Generated headers:
//! - `kalico-c-api/include/kalico_nurbs.h` (cbindgen, gated by `header-nurbs`).
//! - `kalico-c-api/include/kalico_runtime.h` (cbindgen, gated by `header-runtime`).
//!
//! CI verifies that regenerating the headers produces a no-op diff.

#![cfg_attr(not(feature = "host"), no_std)]
// FFI surface is inherently unsafe; the workspace-wide `unsafe_code = deny`
// applies to the pure Rust nurbs/runtime crates, but this crate's reason to
// exist is the `extern "C"` boundary, so we opt out at the crate level.
#![allow(unsafe_code)]

mod nurbs_ffi;
mod runtime_ffi;

// Re-export FFI symbols at crate root so integration tests can call them
// (they're declared inside `mod exports` per cfg-feature gating).
#[cfg(feature = "header-nurbs")]
pub use nurbs_ffi::exports::*;
#[cfg(feature = "header-runtime")]
pub use runtime_ffi::exports::*;

// Re-export error code constants used by integration tests.
pub use runtime::error::*;

/// Panic handler for MCU `no_std` staticlib builds.
///
/// Routes into the C-side fault-latch (`rust_panic_latch` in
/// `src/runtime_panic.c`), which calls Klipper's `shutdown("Rust panic")`.
/// This emits a shutdown-report frame the host sees, services the IWDG one
/// last time, and surfaces the failure in the klippy log — instead of
/// silently locking the MCU inside whatever context the panic occurred
/// (TIM5 ISR, stepper timer callback, etc.).
///
/// Pre-2026-05-19 (A5 boundary audit) this spun forever, which on the
/// bench appeared as a frozen MCU with no diagnostics. The spin path
/// also prevented IWDG service from inside an interrupt context.
///
/// Host builds use the std panic machinery and skip this.
#[cfg(not(feature = "host"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe extern "C" {
        fn rust_panic_latch() -> !;
    }
    // SAFETY: rust_panic_latch is __noreturn on the C side; it calls
    // Klipper's shutdown() macro which never returns. The Rust panic
    // handler's `-> !` return type is satisfied by the function's
    // noreturn signature.
    unsafe { rust_panic_latch() }
}
