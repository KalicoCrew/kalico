#![allow(unsafe_code)]

#[cfg(feature = "header-nurbs")]
pub mod exports {
    use nurbs::{ArcLengthTableRef, ScalarNurbsRef, VectorNurbsRef};

    // SAFETY: `curve` must be non-null and valid for the duration of the call.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_nurbs_eval_f32(
        curve: *const ScalarNurbsRef<'_, f32>,
        u: f32,
    ) -> f32 {
        let curve_ref: &ScalarNurbsRef<'_, f32> = unsafe { &*curve };
        nurbs::eval::eval(curve_ref, u)
    }

    // SAFETY: `curve` non-null and valid; `out` points to a writable [f32; 3].
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

    // SAFETY: `table` must be non-null and valid for the duration of the call.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_nurbs_param_from_arc_length_f32(
        table: *const ArcLengthTableRef<'_, f32>,
        s: f32,
    ) -> f32 {
        let table_ref: &ArcLengthTableRef<'_, f32> = unsafe { &*table };
        nurbs::arc_length::param_from_arc_length(table_ref, s)
    }
}
