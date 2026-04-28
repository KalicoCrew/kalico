//! Kalico nurbs C-FFI surface. cfg-gated by `header-nurbs`.
//!
//! Exposes the Layer-0 NURBS evaluation entrypoints to the Klipper C build.
//! All symbols are namespaced `kalico_nurbs_*` and consumed via cbindgen-
//! generated headers (`include/kalico_nurbs.h`, checked into source).

#![allow(unsafe_code)]

#[cfg(feature = "header-nurbs")]
pub mod exports {
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
}
