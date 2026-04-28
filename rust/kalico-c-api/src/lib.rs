//! Umbrella staticlib + cbindgen FFI surface for kalico's Rust crates.
//!
//! All NURBS symbols are namespaced `kalico_nurbs_*` and exposed via cbindgen.
//! The generated header lives at `kalico-c-api/include/kalico_nurbs.h`
//! and is checked into source control; CI verifies that regenerating it
//! produces a no-op diff.

#![cfg_attr(not(feature = "host"), no_std)]
// FFI surface is inherently unsafe; the workspace-wide `unsafe_code = deny`
// applies to the pure Rust nurbs crate, but this crate's reason to exist is the
// `extern "C"` boundary, so we opt out at the crate level.
#![allow(unsafe_code)]

use nurbs::{ArcLengthTableRef, ScalarNurbsRef, VectorNurbsRef};

/// Evaluate a scalar NURBS at parameter `u`. Returns the position.
///
/// Caller must guarantee `curve` is a valid (non-null, properly initialized)
/// pointer to a `ScalarNurbsRef<f32>` with stable lifetime through the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_nurbs_eval_f32(
    curve: *const ScalarNurbsRef<'_, f32>,
    u: f32,
) -> f32 {
    let curve_ref: &ScalarNurbsRef<'_, f32> = unsafe { &*curve };
    nurbs::eval::eval(curve_ref, u)
}

/// Evaluate a vector NURBS in R^3 at parameter `u`. Writes the resulting
/// 3-vector into `out` (caller-allocated, length 3).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_nurbs_vector_eval_3_f32(
    curve: *const VectorNurbsRef<'_, f32, 3>,
    u: f32,
    out: *mut f32,
) {
    let curve_ref: &VectorNurbsRef<'_, f32, 3> = unsafe { &*curve };
    let result = nurbs::eval::vector_eval(curve_ref, u);
    let out_slice = unsafe { core::slice::from_raw_parts_mut(out, 3) };
    out_slice.copy_from_slice(&result);
}

/// Look up a parameter `u` corresponding to arc length `s` in a precomputed table.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_nurbs_param_from_arc_length_f32(
    table: *const ArcLengthTableRef<'_, f32>,
    s: f32,
) -> f32 {
    let table_ref: &ArcLengthTableRef<'_, f32> = unsafe { &*table };
    nurbs::arc_length::param_from_arc_length(table_ref, s)
}

/// Panic handler for MCU `no_std` staticlib builds. The Klipper C runtime
/// provides no Rust panic infra; on panic we loop forever, leaving the
/// machine in a deterministic locked state for the watchdog/host to detect.
///
/// Host builds use the std panic machinery and skip this.
#[cfg(not(feature = "host"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
