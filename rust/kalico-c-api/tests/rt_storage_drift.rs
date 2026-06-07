#![cfg(feature = "host")]

use runtime::RT_STORAGE_SIZE;
use runtime::state::RuntimeContext;

#[test]
fn rt_storage_size_within_plausible_bounds() {
    // Lower bound: must be at least 32 KB (Kconfig RUNTIME_STORAGE_SIZE_SMALL
    // range floor is 32768).
    assert!(
        RT_STORAGE_SIZE >= 32 * 1024,
        "RT_STORAGE_SIZE = {} bytes — implausibly small for RuntimeContext. \
         Check CONFIG_RUNTIME_STORAGE_SIZE_LARGE/_SMALL in src/Kconfig.",
        RT_STORAGE_SIZE
    );
    // Upper bound: cap at 1 MB. RUNTIME_STORAGE_SIZE_LARGE range ceiling is
    // 524288 (512 KB); 1 MB gives slack but flags absurd values.
    assert!(
        RT_STORAGE_SIZE <= 1024 * 1024,
        "RT_STORAGE_SIZE = {} bytes — implausibly large. Check Kconfig.",
        RT_STORAGE_SIZE
    );
}

#[test]
fn runtime_context_fits_in_rt_storage() {
    let ctx_size = core::mem::size_of::<RuntimeContext>();
    assert!(
        ctx_size <= RT_STORAGE_SIZE,
        "RuntimeContext is {} bytes but RT_STORAGE_SIZE is only {} — \
         bump CONFIG_RUNTIME_STORAGE_SIZE_LARGE/_SMALL in src/Kconfig.",
        ctx_size,
        RT_STORAGE_SIZE
    );
}

#[test]
fn runtime_context_alignment_within_c_alignas() {
    // The C side declares rt_storage with _Alignas(16). RuntimeContext's
    // alignment must not exceed that. Mirrors the const_assert in
    // runtime_ffi.rs; this test fails with a clearer message.
    let ctx_align = core::mem::align_of::<RuntimeContext>();
    assert!(
        ctx_align <= 16,
        "RuntimeContext requires {}-byte alignment but rt_storage is only \
         _Alignas(16) — bump _Alignas in src/runtime_storage.c.",
        ctx_align
    );
}
