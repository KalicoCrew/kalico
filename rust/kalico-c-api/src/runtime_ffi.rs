//! Kalico runtime C-FFI surface. Spec §3.2 / §4.4 / §5.2 / §5.6 / §11.2.
//!
//! cfg-gated by `header-runtime`. Exposes the opaque `*mut KalicoRuntime`
//! handle plus the eight `kalico_runtime_*` entrypoints used by the Klipper
//! C ISR shim and the foreground producer task.
//!
//! ## Half-split + raw-pointer projection (Step 6 Phase 1)
//!
//! Step 5's entrypoints converted `*mut KalicoRuntime` to `&mut
//! RuntimeContext`. Concurrent ISR/foreground entry through that pattern
//! creates overlapping `&mut`s under Rust's strict aliasing model — latent
//! UB acknowledged in spec §6.8 / Step-5 plan Task 13 and slated for Step 6
//! hardening. This module now uses `core::ptr::addr_of!` +
//! `UnsafeCell::raw_get` to project to either `&mut FgState` or `&mut
//! IsrState` (disjoint memory regions) at most once per FFI entry — no
//! `&mut RuntimeContext` is ever materialised. Sound under stacked-borrows
//! / tree-borrows.

#![allow(unsafe_code)]

#[cfg(feature = "header-runtime")]
pub mod exports {
    use core::cell::UnsafeCell;
    use core::sync::atomic::{AtomicBool, Ordering};

    use runtime::RT_STORAGE_SIZE;
    use runtime::engine::RuntimeStatus;
    use runtime::error::{
        KALICO_ERR_CAPABILITY_MISSING, KALICO_ERR_INVALID_ARG, KALICO_ERR_INVALID_HANDLE,
        KALICO_ERR_NOT_INIT, KALICO_ERR_NULL_PTR, KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED,
        KALICO_OK,
    };
    use runtime::state::{IsrState, RuntimeContext, SharedState};

    /// The opaque type C sees — never dereferenced on the C side.
    /// Matches spec §3.2 / §5.6 handle discipline.
    #[allow(missing_debug_implementations)] // opaque to C; never inspected
    #[repr(C)]
    pub struct KalicoRuntime {
        _private: [u8; 0],
    }

    // rt_storage — backing buffer for RuntimeContext, declared on the C
    // side (src/runtime_storage.c) per docs/kalico-rewrite/mcu-c-rust-boundary.md
    // rule B2 (C owns linker-section placement on the MCU).
    //
    // The UnsafeCell wrapper is layout-compatible with the C-side
    // `uint8_t rt_storage[RT_STORAGE_SIZE]`; it exists purely to grant
    // interior-mutability rights to pointers derived from it via .get().
    // No shared `&` reference to rt_storage is ever formed by Rust —
    // the only access path is rt_storage.get().cast::<RuntimeContext>()
    // in runtime_handle_create, after which all writes flow through the
    // existing half-split addr_of_mut! projection chain in
    // RuntimeContext::init.
    //
    // Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md.
    //
    // On the MCU (target_os = "none") rt_storage is C-declared in
    // src/runtime_storage.c with a cfg-gated section attribute. On the host
    // (cargo test, integration test harnesses), nothing links against that
    // C file — so the same name is provided here as a Rust-side static.
    // The host static doesn't need section placement; it just exists so
    // tests and host-build paths find the symbol.
    #[cfg(target_os = "none")]
    unsafe extern "C" {
        static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>;
    }

    // Host backing storage. UnsafeCell isn't Sync by default; wrap it in a
    // transparent newtype with unsafe impl Sync. The cast site in
    // runtime_handle_create reaches the *mut [u8; N] via `.0.get()` on host
    // and `.get()` on MCU (UnsafeCell::get directly); a small cfg-split
    // there keeps the wrapper minimal.
    //
    // Same discipline as the pre-refactor RuntimeCell — synchronization is
    // the INIT_DONE guard + the half-split aliasing pattern, not
    // type-system Sync.
    #[cfg(not(target_os = "none"))]
    #[repr(C, align(16))]
    struct HostRtStorage(UnsafeCell<[u8; RT_STORAGE_SIZE]>);
    // SAFETY: see runtime_handle_create's SAFETY comment.
    #[cfg(not(target_os = "none"))]
    unsafe impl Sync for HostRtStorage {}
    // Lowercase to match the C-side `uint8_t rt_storage[N]` symbol that
    // the MCU extern declaration above resolves to. The same identifier
    // must work in both modes.
    #[cfg(not(target_os = "none"))]
    #[allow(non_upper_case_globals)]
    static rt_storage: HostRtStorage = HostRtStorage(UnsafeCell::new([0u8; RT_STORAGE_SIZE]));

    // Compile-time size contract: RuntimeContext must fit in rt_storage.
    // Bump CONFIG_RUNTIME_STORAGE_SIZE_LARGE/_SMALL in src/Kconfig if this
    // fails after a RuntimeContext field addition.
    const _: () = {
        assert!(
            core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
            "RuntimeContext outgrew RT_STORAGE_SIZE — bump Kconfig storage size"
        );
    };

    // Compile-time alignment contract: rt_storage is _Alignas(16) on the
    // C side; RuntimeContext's alignment must not exceed that. If this
    // fails, bump the _Alignas value in src/runtime_storage.c.
    const _: () = {
        assert!(
            core::mem::align_of::<RuntimeContext>() <= 16,
            "RuntimeContext alignment > 16 — bump _Alignas in runtime_storage.c"
        );
    };

    /// Single-shot init guard. `compare_exchange(false → true)` succeeds
    /// exactly once; subsequent calls observe `Err(true)` and return null.
    pub(super) static INIT_DONE: AtomicBool = AtomicBool::new(false);

    // C-side cycle-counter helper — defined in src/stm32/runtime_tick_h7.c
    // on the MCU and stubbed by the integration-test harness on host.
    unsafe extern "C" {
        fn runtime_cyccnt_read() -> u32;
    }

    /// Init-once. Spec §3.2.
    ///
    /// Returns a valid handle on the first successful call; null on any
    /// subsequent call. The handle is the address of the static
    /// `RuntimeContext` storage; its lifetime is `'static`.
    #[unsafe(no_mangle)]
    pub extern "C" fn runtime_handle_create() -> *mut KalicoRuntime {
        // Guard against double-init. Klipper calls this exactly once from a
        // single-threaded DECL_INIT sequence before TIM5 is armed, so a plain
        // relaxed load is sufficient — there is no concurrent caller.
        //
        // We intentionally avoid compare_exchange here: on Cortex-M7 the Rust
        // compiler lowers it to LDREXB/STREXB (exclusive monitor). Renode's
        // H7 model (v1.16) silently drops the exclusive store — STREXB
        // returns r2=0 (success) but does not write to memory — leaving
        // INIT_DONE=0 even though the code proceeds into init(). Using a
        // plain non-exclusive STRB (via store) avoids that Renode bug.
        if INIT_DONE.load(Ordering::Relaxed) {
            return core::ptr::null_mut();
        }
        // SAFETY: single-threaded init; no other context can observe
        // rt_storage until INIT_DONE is published below. RuntimeContext::init
        // writes through raw-pointer projections and never forms `&mut
        // RuntimeContext`, matching the §11.2 aliasing discipline.
        //
        // rt_storage.get() returns *mut [u8; N] with provenance over the
        // full C-declared buffer; the cast to *mut RuntimeContext inherits
        // that provenance (the const_assert above ensures RuntimeContext
        // fits within the buffer).
        unsafe {
            // On MCU rt_storage is UnsafeCell<[u8; N]> (extern from C);
            // on host it's HostRtStorage(UnsafeCell<[u8; N]>) (Rust-defined).
            // .get() vs .0.get() — same underlying *mut [u8; N], same provenance.
            #[cfg(target_os = "none")]
            let rt_ptr: *mut RuntimeContext = rt_storage.get().cast::<RuntimeContext>();
            #[cfg(not(target_os = "none"))]
            let rt_ptr: *mut RuntimeContext = rt_storage.0.get().cast::<RuntimeContext>();
            debug_assert_eq!(
                (rt_ptr as usize) % core::mem::align_of::<RuntimeContext>(),
                0,
                "rt_storage alignment mismatch — linker placed it unaligned"
            );
            RuntimeContext::init(rt_ptr);
            // Publish after full init — ISR sees either INIT_DONE=false
            // (before enable) or a fully-initialised context (after).
            // Release ordering pairs with the Acquire loads in every FFI call.
            INIT_DONE.store(true, Ordering::Release);
            rt_ptr.cast::<KalicoRuntime>()
        }
    }

    /// Validate a versioned blob payload's leading version byte (§4.2).
    /// Foreground entrypoint for the C handler that reads payload bytes off
    /// the wire and routes the post-version-byte slice into the Step-5
    /// flat-pointer load path. Returns `KALICO_OK` on a recognised version
    /// or `KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED` otherwise.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_check_blob_version(
        payload_ptr: *const u8,
        payload_len: u32,
    ) -> i32 {
        if payload_ptr.is_null() || payload_len == 0 {
            return KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED;
        }
        // SAFETY: caller-provided pointer-and-length pair is contracted to
        // be a valid byte slice of length `payload_len`.
        let blob = unsafe { core::slice::from_raw_parts(payload_ptr, payload_len as usize) };
        match runtime::wire::check_version(blob) {
            Ok(()) => KALICO_OK,
            Err(_) => KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED,
        }
    }

    /// Task 6 TIM5 ISR body — piece-ring walker.
    ///
    /// Projects `&mut IsrState`, `&SharedState`, and a `&mut [PieceEntry]`
    /// slice over `piece_storage` from `RuntimeContext` and delegates to
    /// `runtime::tick::isr_sample_tick`.
    ///
    /// Called from `TIM5_IRQHandler` in `src/stm32/runtime_tick_{h7,f4}.c`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick_sample(rt: *mut KalicoRuntime) {
        if rt.is_null() {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null and INIT_DONE=true. TIM5 is the SOLE writer
        // of `IsrState` (§11.1). `piece_storage` is `UnsafeCell` — projecting
        // a `&mut [PieceEntry]` slice here is sound because:
        //   - The ISR is the sole mutator of the ring tail (pop) and per-axis
        //     ISR-cache fields.
        //   - The foreground pushes into ring HEAD positions; it never writes
        //     to the slot currently pointed to by an axis's ring tail (the ISR
        //     never pops a slot while the foreground holds it in the "current
        //     piece" cache).
        //   - `UnsafeCell::raw_get` yields provenance over the full array
        //     without forming a `&UnsafeCell<[..]>` shared reference.
        unsafe {
            let raw = runtime_cyccnt_read();
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            // Project piece_storage as a mutable slice of PieceEntry.
            let ps_ptr: *mut [runtime::piece_ring::PieceEntry; runtime::state::TOTAL_RING_PIECES] =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).piece_storage));
            let storage: &mut [runtime::piece_ring::PieceEntry] = &mut *ps_ptr;
            let isr: &mut IsrState = &mut *isr_ptr;
            let shared: &SharedState = &*shared_ptr;
            runtime::tick::isr_sample_tick(isr, shared, storage, raw);
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_status(rt: *mut KalicoRuntime) -> u8 {
        if rt.is_null() {
            return RuntimeStatus::Fault as u8;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return RuntimeStatus::Fault as u8;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState is read-only here; project to the atomics-
        // bearing field via `addr_of!` and form `&SharedState`. No `&mut`
        // forms on this path.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).runtime_status.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_error(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only access to SharedState atomics.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).last_error.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_counter(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only access to the ISR-half's tick counter (an
        // AtomicU32 on Engine). The §11.1 invariant says the ISR is the sole
        // *writer* of IsrState; foreground may form `&IsrState` (shared
        // borrow) for read-only access of atomics — the atomic itself
        // provides the synchronization. We do that by forming `&IsrState`
        // through the UnsafeCell and reading through its embedded atomic.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.tick_counter()
        }
    }

    /// Alias for `runtime_handle_tick_counter` with the
    /// `kalico_runtime_get_*` naming the bench diag rotation uses. Same
    /// underlying read — `Engine::tick_counter.snapshot()`. Returns 0 on a
    /// null handle or before `INIT_DONE`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_tick_counter(rt: *mut KalicoRuntime) -> u32 {
        // SAFETY: delegates to the existing tick-counter accessor; same
        // shared-borrow contract.
        unsafe { runtime_handle_tick_counter(rt) }
    }

    // ---- Phase 11 §5.3 status-frame accessors -----------------------------
    //
    // Each helper projects to `&SharedState` (atomics-only) and reads one
    // field. Released as a separate FFI per Klipper's "one C-side `sendf`
    // call passes scalar args" pattern: the status-frame DECL_TASK assembles
    // the values via these accessors, the `runtime_handle_widened_now` helper
    // reads the §11.4 seqlock-protected widened clock, and the periodic
    // `kalico_status_v6` frame goes out at ~10 Hz.

    /// Read the widened MCU clock. Spec §3.9 — on-demand widening from
    /// Klipper's `timer_read_time` + the `stats_send_time` / `stats_send_time_high`
    /// counters that Klipper's stats task maintains (basecmd.c). TIM5 now
    /// free-runs from boot, so the engine seqlock is continuously republished
    /// while the firmware is alive; the stats-task path remains the correct
    /// choice here because it avoids ISR-context re-entrancy with the
    /// `timer_read_time` wrap update. The stats task runs unconditionally,
    /// so this widening advances regardless of engine activity.
    ///
    /// Mirrors the C-side `runtime_widened_host_clock` in `src/runtime_tick.c`.
    /// Foreground-only — `timer_read_time()` is not re-entrant with the
    /// stats-task wrap update; do not call from ISR context.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_widened_now(rt: *mut KalicoRuntime) -> u64 {
        // `rt` is unused — the widening reads only Klipper-side globals — but
        // the parameter is retained so the C ABI stays stable across the
        // pre/post seqlock refactor.
        let _ = rt;
        unsafe extern "C" {
            fn timer_read_time() -> u32;
            static stats_send_time: u32;
            static stats_send_time_high: u32;
        }
        // SAFETY: `timer_read_time` is a single u32 read of an MMIO timer
        // register (or a software counter in sim builds), safe from any
        // non-ISR caller. `stats_send_time` / `stats_send_time_high` are
        // u32 globals written by the stats DECL_TASK; a concurrent update
        // can produce a torn read, but the "low < stats_send_time" probe
        // self-corrects within one stats cadence (~5 s) and the resulting
        // error is bounded by one u32 wrap (a ~37 s window on 120 MHz F4,
        // ~16 s on H7 — both far longer than any drift the host tolerates).
        unsafe {
            let low = timer_read_time();
            let high = stats_send_time_high + ((low < stats_send_time) as u32);
            ((high as u64) << 32) | (low as u64)
        }
    }

    /// 2026-05-17 F4-retire-stall diagnostic: read low 32 bits of the most
    /// recent `now - seg.t_start` observed inside `runtime_modulated_tick`.
    /// `0` if no segment has been processed yet OR if the engine clock is
    /// behind the segment's t_start (saturating_sub clamps to 0).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_modulated_elapsed_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .last_modulated_elapsed_lo
                .load(Ordering::Acquire)
        }
    }

    /// Companion to `runtime_handle_last_modulated_elapsed_lo`: low 32 bits
    /// of the active segment's `duration()`. If `elapsed_lo` >= this value,
    /// the retirement branch should fire on the next tick.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_modulated_duration_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .last_modulated_duration_lo
                .load(Ordering::Acquire)
        }
    }

    /// Number of times `runtime_modulated_tick`'s retirement branch was
    /// entered (`elapsed >= duration` was true). Pair with the success
    /// counter via `runtime_handle_modulated_retire_successes` to decide
    /// whether retirement is failing on the `consumers_done` check.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_modulated_retire_attempts(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .modulated_retire_attempts
                .load(Ordering::Acquire)
        }
    }

    /// Number of times `runtime_modulated_tick` actually retired a segment
    /// (`consumers_done` was true after clearing the motor bits). If this
    /// is less than `runtime_handle_modulated_retire_attempts`, some entries
    /// to the retirement branch are still leaving consumer bits set.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_modulated_retire_successes(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .modulated_retire_successes
                .load(Ordering::Acquire)
        }
    }

    /// Last snapshot of seg.consumers_remaining AFTER the clear-all-motors
    /// loop in modulated_tick's retirement branch. If retire_attempts >
    /// retire_successes and this is non-zero, the bits in this value are
    /// what the per-motor clear didn't reach.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_last_retire_consumers_after_clear(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            (*core::ptr::addr_of!((*ctx).shared))
                .last_retire_consumers_after_clear
                .load(Ordering::Acquire)
        }
    }

    /// Read the latched `fault_detail` payload (§9.2). Mirrors the value
    /// the foreground emits with the async `kalico_fault` event. `0` when
    /// no fault has latched OR the latched fault carries no detail.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_fault_detail(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState atomics-only access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).fault_detail.load(Ordering::Acquire)
        }
    }

    /// Read the scheduler dispatch-history func addr at the most recent `-311`.
    /// Reference-only, not wired into the fault event — `runtime_handle_tick_blocker_pc`
    /// carries the addr2line target instead. `0` before any `-311` fires.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState atomics-only access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_func.load(Ordering::Acquire)
        }
    }

    /// Read the stacked exception-frame return address (PC) captured at TIM5
    /// handler entry on the most recent `-311 TickIntervalExceeded` fault.
    /// This is the instruction that was about to execute when TIM5
    /// preempted/resumed — the addr2line target naming the code that held the
    /// CPU / PRIMASK across the late tick. `0` before any `-311` fires (or on
    /// host/test builds). Wired into the fault event's `segment_id` field by
    /// `runtime_tick.c`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_pc(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState atomics-only access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_pc.load(Ordering::Acquire)
        }
    }

    /// Read the stacked xPSR exception number captured at TIM5 handler entry
    /// on the most recent `-311 TickIntervalExceeded` fault. `0` =
    /// thread/foreground was the interrupted context; nonzero = that IRQ/
    /// exception number was the interrupted context (TIM5 tail-chained behind
    /// it). `0` before any `-311` fires (or on host/test builds).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_exc(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState atomics-only access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_exc.load(Ordering::Acquire)
        }
    }

    /// Diagnostic: read the configured `steps_per_mm` for axis `oid` (0..=3
    /// in motor space). Returns 0.0 if `oid` is out of range or runtime
    /// uninitialised. Used by Phase 4 sim test to verify axis configuration
    /// reached the engine.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_get_axis_steps_per_mm(
        rt: *mut KalicoRuntime,
        oid: u8,
    ) -> f32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.debug_steps_per_mm(oid as usize)
        }
    }

    /// Seed the engine's widen state with a u64 baseline so the engine's
    /// `now` agrees with Klipper's widened MCU clock. Called by the Linux
    /// sim host once at runtime_init, BEFORE the engine pthread starts
    /// ticking. `baseline_widened_clock` is whatever value timer_read_time()
    /// would map to in the widened (no-wrap) frame at this instant; the
    /// caller computes it from clock_gettime so wrap counts already passed
    /// in u32 timer_read_time space are folded in.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_seed_widen(
        rt: *mut KalicoRuntime,
        baseline_widened_clock: u64,
    ) {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).widen_state.seed_high(baseline_widened_clock);
        }
    }

    /// Diagnostic: read most recent post-PA/IS motor position for axis `oid`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_axis_motor(rt: *mut KalicoRuntime, oid: u8) -> f32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.debug_last_motor(oid as usize)
        }
    }

    /// Diagnostic: read most recent (now, t_start, duration) into the three
    /// out pointers. All u64 cycle counts.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_last_timing(
        rt: *mut KalicoRuntime,
        now_out: *mut u64,
        t_start_out: *mut u64,
        duration_out: *mut u64,
    ) {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let (n, ts, dur) = (*isr_ptr).engine.debug_last_timing();
            if !now_out.is_null() {
                *now_out = n;
            }
            if !t_start_out.is_null() {
                *t_start_out = ts;
            }
            if !duration_out.is_null() {
                *duration_out = dur;
            }
        }
    }

    /// Diagnostic: read step accumulator (sub-step residual + integer state)
    /// for axis `oid`.  Returns 0.0 if invalid.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_axis_accumulator(
        rt: *mut KalicoRuntime,
        oid: u8,
    ) -> f64 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.debug_accumulator(oid as usize)
        }
    }

    /// Read the cumulative signed step count for stepper `oid` (0-indexed).
    /// Returns 0 for an invalid `rt` / uninitialised runtime / out-of-range oid.
    /// Used by the sim diagnostic command `runtime_sim_stepper_count_query`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_stepper_count(
        rt: *mut KalicoRuntime,
        oid: u8,
    ) -> i32 {
        use runtime::state::MAX_STEPPER_OIDS;
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        if oid as usize >= MAX_STEPPER_OIDS {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).stepper_counts[oid as usize].load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_xdirect_write_count() -> u32 {
        #[cfg(not(target_os = "none"))]
        {
            runtime::test_xdirect_capture::count() as u32
        }
        #[cfg(target_os = "none")]
        {
            0
        }
    }

    /// Seed the engine's `prev_x/y/z` position origin and `StepMotorState`
    /// accumulators so the first segment after `SET_KINEMATIC_POSITION`
    /// computes its delta against the correct origin rather than `(0, 0, 0)`.
    ///
    /// Called by the C `DECL_COMMAND` handler for `runtime_seed_position`
    /// immediately before the host sends the first `PushSegment` of the
    /// new stream. Fire-and-forget from the host side; no response is
    /// required because the following `PushSegment` provides ordering.
    ///
    /// `x_q16`, `y_q16`, `z_q16` are Q16.16 fixed-point mm values
    /// (`i32 = mm * 65536`). Decoded to `f32` here; the precision loss
    /// (≈ 15 µm at 1 m) is negligible relative to the step-size floor.
    ///
    /// Foreground-only. Projects `&mut IsrState` under the same
    /// single-threaded-foreground precondition as `kalico_runtime_configure_axis`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_seed_position(
        rt: *mut KalicoRuntime,
        x_q16: i32,
        y_q16: i32,
        z_q16: i32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let x = x_q16 as f32 / 65536.0;
        let y = y_q16 as f32 / 65536.0;
        let z = z_q16 as f32 / 65536.0;
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only projection. `engine` lives in `IsrState`;
        // we project `&mut IsrState` under the single-threaded-foreground
        // precondition (no ISR running) — this command arrives before the first
        // PushSegment. No other `&mut IsrState` or `&mut FgState` may be live
        // on this call path.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.seed_position([x, y, z]);
        }
        KALICO_OK
    }

    // ---- Stream lifecycle + clock-sync FFI ------------
    //
    // `open` / `arm` / `terminal` (segment-era stubs) have been removed.
    // `flush` is retained as a no-op shell; the host rewrite (same branch)
    // replaces it with the real force-idle / cancel mechanism.

    /// `kalico_stream_flush` — `force_idle` handshake (§8.5).
    ///
    /// `flush()` projects to both halves under disabled-IRQ, so we hand it
    /// the raw `*mut RuntimeContext` directly rather than going through
    /// the foreground-only `project_fg` helper. SAFETY: caller is the
    /// single-threaded foreground command dispatch.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_stream_flush(
        rt: *mut KalicoRuntime,
        out_credit_epoch: *mut u32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: rt is the published RuntimeContext pointer (verified
        // non-null + INIT_DONE above). flush() does its own half-split
        // projections internally per the §8.5 ordering contract.
        unsafe { runtime::stream::flush(rt.cast::<RuntimeContext>(), out_credit_epoch) }
    }

    /// `kalico_clock_sync_request` — RTT-aware clock-sync ping (§12.1).
    ///
    /// Returns the on-demand widened MCU clock (timer_read_time +
    /// stats_send_time_high), NOT the engine seqlock value. The seqlock is
    /// updated only from the TIM5 ISR, so this path computes the widened clock
    /// on-demand and is correct regardless of whether TIM5 is running.
    ///
    /// The on-demand widening uses Klipper's `stats_send_time_high` (updated
    /// by the stats DECL_TASK at ~0.2 Hz). Its ~5 s lag in the high half is
    /// invisible to the bridge's RTT-aware linear regression — the
    /// regression amortises samples over many ticks.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_clock_sync_request(
        rt: *mut KalicoRuntime,
        request_id: u32,
        host_send_time_lo: u32,
        host_send_time_hi: u32,
        out_mcu_clock: *mut u64,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: identical to `runtime_handle_widened_now` — single u32 reads of
        // Klipper-owned globals. Safe from any non-ISR caller; clock-sync runs in
        // command-handler foreground context.
        let mcu_clock = unsafe {
            unsafe extern "C" {
                fn timer_read_time() -> u32;
                static stats_send_time: u32;
                static stats_send_time_high: u32;
            }
            let low = timer_read_time();
            let high = stats_send_time_high + ((low < stats_send_time) as u32);
            ((high as u64) << 32) | (low as u64)
        };
        let _ = (request_id, host_send_time_lo, host_send_time_hi);
        if !out_mcu_clock.is_null() {
            // SAFETY: out_mcu_clock checked non-null.
            unsafe { *out_mcu_clock = mcu_clock };
        }
        KALICO_OK
    }

    /// Sim escape hatch: load a pre-baked NURBS fixture into a curve-pool slot.
    ///
    /// Per Step-6 plan Phase 0 Task 0.2 GDB-attach diagnosis: under Renode,
    /// the H7 platform model silently ignores `SCB->CPACR` writes from
    /// `SystemInit()`, leaving the FPU disabled. The regular
    /// `runtime_handle_load_curve` path runs `is_finite()` / `> 0.0` checks
    /// on caller-supplied data; those FPU instructions raise a UsageFault
    /// that lands in Klipper's `DefaultHandler` infinite loop. This entrypoint
    /// uses static pre-validated fixtures and the
    /// `CurvePool::load_unchecked` integer-only-copy variant so Step-6
    /// protocol iteration can land segments in sim without touching the FPU.
    ///
    /// Compiled only with the `kalico-sim` Cargo feature, gated on
    /// `CONFIG_KALICO_SIM=y` in the Klipper Makefile. NEVER include in
    /// production firmware.
    #[cfg(feature = "kalico-sim")]
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_load_fixture(
        _rt: *mut KalicoRuntime,
        _slot_idx: u16,
        _fixture_id: u16,
        _out_handle_packed: *mut u32,
    ) -> i32 {
        // Fixture loading used the old NURBS CurvePool API. The pool now
        // uses WirePiece-based cubic loading. This escape hatch is unused
        // by homing / motion tests — stub it to unblock the sim build.
        use runtime::error::KALICO_ERR_INVALID_CURVE;
        KALICO_ERR_INVALID_CURVE
    }

    // ---- Step 7-D: endstop arm/disarm/poll_trip ----------------------------

    use runtime::endstop::{
        ArmMsg, ArmPolicy, ArmStatus, DisarmStatus, MAX_SOURCES, MAX_STEPPERS, SourceConfig,
        SourceKind, VelocityAxis,
    };

    const SOURCE_RECORD_LEN: usize = 11;
    const STEPPER_RECORD_LEN: usize = 1;

    // Trip event format v1: arm_id(4) + clock_lo(4) + clock_hi(4)
    // + source_idx(1) + fmt_version(1) + stepper_count(1)
    // + stepper_data(stepper_count * 5).
    pub const KALICO_TRIP_EVENT_V1_HEADER_LEN: usize = 15;
    pub const KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN: usize = 5;
    pub const KALICO_TRIP_EVENT_V1_FMT_VERSION: u8 = 1;
    pub const KALICO_TRIP_EVENT_V1_MAX_LEN: usize =
        KALICO_TRIP_EVENT_V1_HEADER_LEN + MAX_STEPPERS * KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN;

    /// Arm an endstop. The blob layouts match spec §3.1:
    /// - `sources`: `source_count` records of 11 bytes each
    ///   (kind u8, gpio u16 LE, polarity u8, arm_policy u8, sample_n u8,
    ///    velocity_axis u8, v_min_q16 u32 LE).
    /// - `steppers`: `stepper_count` records of 1 byte (stepper_oid u8).
    ///
    /// Writes one of the spec §3.2 status values into `*out_status`:
    /// 0 = Armed, 1 = AlreadyTripped, 2 = Rejected.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_arm(
        arm_id: u32,
        arm_clock_lo: u32,
        arm_clock_hi: u32,
        source_count: u8,
        sources_ptr: *const u8,
        sources_len: usize,
        stepper_count: u8,
        steppers_ptr: *const u8,
        steppers_len: usize,
        grant_ticks_lo: u32,
        grant_ticks_hi: u32,
        out_status: *mut u8,
    ) -> i32 {
        if out_status.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        unsafe { *out_status = 2 }; // default: Rejected (overwritten on success)
        if source_count == 0 || source_count as usize > MAX_SOURCES {
            return KALICO_ERR_NULL_PTR;
        }
        if stepper_count == 0 || stepper_count as usize > MAX_STEPPERS {
            return KALICO_ERR_NULL_PTR;
        }
        if sources_len != source_count as usize * SOURCE_RECORD_LEN {
            return KALICO_ERR_NULL_PTR;
        }
        if steppers_len != stepper_count as usize * STEPPER_RECORD_LEN {
            return KALICO_ERR_NULL_PTR;
        }
        if sources_ptr.is_null() || steppers_ptr.is_null() {
            return KALICO_ERR_NULL_PTR;
        }

        let sources_blob: &[u8] = unsafe { core::slice::from_raw_parts(sources_ptr, sources_len) };
        let steppers_blob: &[u8] =
            unsafe { core::slice::from_raw_parts(steppers_ptr, steppers_len) };

        let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
        for i in 0..source_count as usize {
            let r = &sources_blob[i * SOURCE_RECORD_LEN..(i + 1) * SOURCE_RECORD_LEN];
            let kind = match r[0] {
                0 => SourceKind::Physical,
                1 => SourceKind::TmcDiag,
                2 => SourceKind::Software,
                _ => return KALICO_ERR_NULL_PTR,
            };
            let gpio = u16::from_le_bytes([r[1], r[2]]);
            let active_high = r[3] != 0;
            let policy = match r[4] {
                0 => ArmPolicy::TripImmediately,
                1 => ArmPolicy::WaitForClear,
                2 => ArmPolicy::IgnoreUntilMoving,
                _ => return KALICO_ERR_NULL_PTR,
            };
            let sample_n = r[5];
            let velocity_axis = VelocityAxis::from_bits_truncate(r[6]);
            let v_min_q16 = u32::from_le_bytes([r[7], r[8], r[9], r[10]]);
            sources[i] = SourceConfig {
                kind,
                gpio,
                active_high,
                policy,
                sample_n,
                velocity_axis,
                v_min_q16,
            };
        }

        let mut stepper_oids = [0u8; MAX_STEPPERS];
        for i in 0..stepper_count as usize {
            stepper_oids[i] = steppers_blob[i];
        }

        let arm_clock = (u64::from(arm_clock_hi) << 32) | u64::from(arm_clock_lo);
        let grant_ticks = (u64::from(grant_ticks_hi) << 32) | u64::from(grant_ticks_lo);
        let msg = ArmMsg {
            arm_id,
            arm_clock,
            source_count,
            sources,
            stepper_count,
            stepper_oids,
            grant_ticks,
        };

        match runtime::endstop::arm(msg) {
            Ok(ArmStatus::Armed) => {
                unsafe { *out_status = 0 };
                KALICO_OK
            }
            Ok(ArmStatus::AlreadyTripped) => {
                unsafe { *out_status = 1 };
                KALICO_OK
            }
            Err(_e) => {
                unsafe { *out_status = 2 };
                KALICO_OK
            }
        }
    }

    /// Disarm an active endstop arm. `out_status` writes spec §3.5 codes:
    /// 0 = Disarmed, 1 = AlreadyTripped, 2 = Unknown.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_disarm(arm_id: u32, out_status: *mut u8) -> i32 {
        if out_status.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        let s = match runtime::endstop::disarm(arm_id) {
            DisarmStatus::Disarmed => 0u8,
            DisarmStatus::AlreadyTripped => 1u8,
            DisarmStatus::Unknown => 2u8,
        };
        unsafe { *out_status = s };
        KALICO_OK
    }

    /// Software-trip an armed endstop. Called from the C command handler
    /// `command_runtime_software_trip`. Writes a status byte into `*out_status`:
    /// 0 = Tripped, 1 = NotArmed, 2 = WrongArmId.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_software_trip(
        arm_id: u32,
        clock_lo: u32,
        clock_hi: u32,
        out_status: *mut u8,
    ) -> i32 {
        if out_status.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        let clock = (u64::from(clock_hi) << 32) | u64::from(clock_lo);
        // Pass empty stepper_counts — the MCU's step counters are not available
        // from the command handler context. The snapshot will have zero counts;
        // the host uses the retained curve for position reconstruction instead.
        let result = runtime::endstop::software_trip(arm_id, clock, &[]);
        let status = match result {
            runtime::endstop::TripResult::Tripped => 0u8,
            runtime::endstop::TripResult::NotArmed => 1u8,
            runtime::endstop::TripResult::WrongArmId => 2u8,
        };
        unsafe { *out_status = status };
        KALICO_OK
    }

    /// Extend the homing deadline by one grant window. Called from the C
    /// command handler `command_runtime_extend_homing_deadline`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_extend_deadline(
        arm_id: u32,
        clock_lo: u32,
        clock_hi: u32,
    ) -> i32 {
        let clock = (u64::from(clock_hi) << 32) | u64::from(clock_lo);
        runtime::endstop::extend_deadline(arm_id, clock);
        KALICO_OK
    }

    /// Drain the next pending trip event into a host-side buffer.
    ///
    /// Wire format v1, little-endian, total length =
    /// `KALICO_TRIP_EVENT_V1_HEADER_LEN + stepper_count *
    /// KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN`. Header layout:
    /// `arm_id u32 | trip_clock_lo u32 | trip_clock_hi u32 |
    ///  trip_source_idx u8 | fmt_version u8 (=1) | stepper_count u8`.
    /// Each stepper record: `stepper_oid u8 | step_count i32`.
    ///
    /// Returns:
    /// - `1`  + `*out_actual_len` = encoded length: an event was drained.
    /// - `0`  + `*out_actual_len = 0`: no event ready.
    /// - `KALICO_ERR_NULL_PTR` on argument errors (incl. out_buf too small).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_poll_trip(
        out_buf: *mut u8,
        out_buf_len: usize,
        out_actual_len: *mut usize,
    ) -> i32 {
        if out_buf.is_null() || out_actual_len.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        unsafe { *out_actual_len = 0 };

        let Some(evt) = runtime::endstop::poll_trip() else {
            return 0;
        };

        let stepper_count = usize::from(evt.stepper_count);
        let needed =
            KALICO_TRIP_EVENT_V1_HEADER_LEN + stepper_count * KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN;
        if out_buf_len < needed {
            return KALICO_ERR_NULL_PTR;
        }
        let buf: &mut [u8] = unsafe { core::slice::from_raw_parts_mut(out_buf, needed) };
        buf[0..4].copy_from_slice(&evt.arm_id.to_le_bytes());
        buf[4..8].copy_from_slice(&(evt.trip_clock as u32).to_le_bytes());
        buf[8..12].copy_from_slice(&((evt.trip_clock >> 32) as u32).to_le_bytes());
        buf[12] = evt.trip_source_idx;
        buf[13] = KALICO_TRIP_EVENT_V1_FMT_VERSION;
        buf[14] = evt.stepper_count;
        for i in 0..stepper_count {
            let off = KALICO_TRIP_EVENT_V1_HEADER_LEN + i * KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN;
            buf[off] = evt.steppers[i].oid;
            buf[off + 1..off + 5].copy_from_slice(&evt.steppers[i].step_count.to_le_bytes());
        }
        unsafe { *out_actual_len = needed };
        1
    }

    /// Push a sampled GPIO level into the runtime's abstract pin table.
    ///
    /// The runtime's endstop module reads pin levels from an internal
    /// `PIN_LEVELS: [AtomicBool; MAX_GPIO_PINS]` table (rust/runtime/src/
    /// endstop.rs:311). The C ISR shim samples real GPIOs via
    /// `gpio_in_read` once per modulation tick (TIM5_IRQHandler at
    /// src/stm32/runtime_tick_h7.c, just before `runtime_handle_tick`)
    /// and pushes each result through this FFI before `endstop::tick`
    /// observes it. Sim builds (Renode e2e at
    /// tools/test_renode_endstop_e2e.py) call the same FFI directly via
    /// the `command_runtime_sim_endstop_set_pin` shim, bypassing real GPIO.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_set_pin_level(gpio: u16, level: u8) -> i32 {
        if runtime::endstop::set_pin_level(gpio, level != 0) {
            KALICO_OK
        } else {
            KALICO_ERR_NULL_PTR
        }
    }

    // ---- Step-time scheduling FFI (spec §5) --------------------------------

    /// Flip a stepper's `StepMode` at runtime. Spec §10.
    ///
    /// `mode`: 0 = Modulated (phase-stepping), 1 = StepTime (classic).
    /// `mcu_supports_phase`: non-zero if the MCU advertises phase-stepping
    /// capability.
    ///
    /// Returns:
    /// - `KALICO_OK` on success.
    /// - `KALICO_ERR_INVALID_HANDLE` if `handle` is null or
    ///   `stepper_idx >= MAX_STEPPER_OIDS`.
    /// - `KALICO_ERR_INVALID_ARG` if `mode` is not a recognised `StepMode`
    ///   discriminant.
    /// - `KALICO_ERR_CAPABILITY_MISSING` if `mode == Modulated` and
    ///   `mcu_supports_phase == 0`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_set_step_mode(
        rt: *mut KalicoRuntime,
        stepper_idx: u8,
        mode: u8,
        mcu_supports_phase: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_INVALID_HANDLE;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let mode = match runtime::state::StepMode::from_u8(mode) {
            Some(m) => m,
            None => return KALICO_ERR_INVALID_ARG,
        };
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState is atomics-only; no `&mut` is formed on this
        // path. The `set_step_mode` function touches only per-stepper
        // `AtomicU8` entries in `SharedState::step_modes`, which are
        // explicitly designed for concurrent foreground writes and ISR reads.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match runtime::state::set_step_mode(shared, stepper_idx, mode, mcu_supports_phase != 0)
            {
                Ok(()) => {
                    // TIM5 lifecycle is decoupled from step mode (spec
                    // 2026-05-28): the timer is armed at runtime_tick_init and
                    // disabled only on Klipper shutdown. Setting a step mode no
                    // longer arms/disarms the tick.
                    KALICO_OK
                }
                Err(runtime::state::SetStepModeError::CapabilityMissing) => {
                    KALICO_ERR_CAPABILITY_MISSING
                }
                Err(runtime::state::SetStepModeError::OutOfRange) => KALICO_ERR_INVALID_HANDLE,
            }
        }
    }

    // ---- Legacy Newton / step-ring producer surface (DELETED) ------------
    //
    // The `kalico_runtime_producer_step`, `kalico_runtime_step_ring_*`,
    // `kalico_runtime_kick_producer`, `kalico_runtime_get_producer_pending`,
    // `kalico_runtime_force_idle`, `kalico_runtime_apply_step`, and all
    // associated producer/Newton diagnostic accessors were removed by the
    // 2026-05-20 stepping-redesign (Task 13). The new path is the
    // per-sample `kalico_runtime_tick_sample` ISR driving the cubic
    // evaluator over the curve pool — no producer timer, no per-motor
    // step rings, no Newton fill.

    /// Diagnostic: read the low 32 bits of `producer_enqueue_success_total`.
    /// Bumps AFTER `fg.queue_producer.enqueue(seg)` returns Ok in
    /// `push_segment_impl`. If non-zero while
    /// `producer_segment_dequeued_total` is 0, the queue split is broken
    /// (producer and consumer ends not sharing the backing buffer).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_enqueue_success_lo(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared
                .producer_enqueue_success_total
                .load(Ordering::Acquire) as u32
        }
    }

    /// 2026-05-15 live diagnosis: read the low 32 bits of
    /// `push_segment_all_unused_total`. If this counter advances during a
    /// jog, the bridge sent push_segment frames with every handle set to
    /// the UNUSED sentinel — the segment retires on producer dequeue
    /// without ever invoking motor processing, which matches the
    /// "energized but no motion" symptom.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_push_seg_all_unused_lo(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.push_segment_all_unused_total.load(Ordering::Acquire) as u32
        }
    }

    /// 2026-05-15 live diagnosis: read the packed last `x_handle` from
    /// `push_segment_impl`. Layout: `(gen << 16) | slot_idx`. UNUSED
    /// sentinel = 0xFFFE_FFFE.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_push_x_handle(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_x_handle_packed.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis: read the packed last `y_handle` from
    /// `push_segment_impl`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_push_y_handle(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_y_handle_packed.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis: read the last `consumers_remaining`
    /// mask computed by `push_segment_impl`. Zero means every handle on
    /// the most recent push_segment was UNUSED.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_push_consumers_remaining(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_consumers_remaining.load(Ordering::Acquire)
        }
    }

    /// Read the current `StepMode` discriminant for `stepper_idx`.
    ///
    /// Used by the C-side `arm_step_time_steppers_after_push` to determine
    /// whether a stepper should be registered with Klipper's scheduler (mode
    /// `StepTime = 1`) or driven by the TIM5 ISR (mode `Modulated = 0`).
    ///
    /// Returns:
    /// - `0`    — `StepMode::Modulated` (phase-stepping via TIM5 ISR).
    /// - `1`    — `StepMode::StepTime`  (classic Klipper timer scheduling).
    /// - `0xFF` — null `rt`, `INIT_DONE == false`, or `stepper_idx` out of range.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_step_mode(
        rt: *mut KalicoRuntime,
        stepper_idx: u8,
    ) -> u8 {
        use runtime::state::MAX_STEPPER_OIDS;
        if rt.is_null() || (stepper_idx as usize) >= MAX_STEPPER_OIDS {
            return 0xFF;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0xFF;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` is the published RT_CELL pointer (non-null, INIT_DONE=true).
        // `step_modes` are per-stepper `AtomicU8` fields in `SharedState`;
        // we read via a shared `&SharedState` reference — no `&mut` is formed.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            shared.step_modes[stepper_idx as usize].load(Ordering::Acquire)
        }
    }

    /// Read back the parsed phase-stepping SPI config for motor `motor_idx`
    /// (Task 4 / spec §4.1 introspection).
    ///
    /// Returns the packed `AtomicU16` payload: high byte = `spi_bus_id`, low
    /// byte = `cs_pin_id`. `0xFFFF` means no phase config is installed on
    /// that motor (the default), and is also returned for a null `rt`,
    /// uninitialised runtime, or `motor_idx >= 4`.
    ///
    /// Use `runtime::phase_config::PhaseConfig::unpack` on the host side to
    /// decode.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_query_phase_config(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
    ) -> u16 {
        if rt.is_null() || motor_idx >= 4 {
            return 0xFFFF;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0xFFFF;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` is the published RT_CELL pointer; `phase_config` is a
        // shared array of `AtomicU16` slots accessed via a shared
        // `&SharedState` — no `&mut` is formed.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match shared.phase_config.get(motor_idx as usize) {
                Some(slot) => slot.load(Ordering::Acquire),
                None => 0xFFFF,
            }
        }
    }

    // ─── Stepping-redesign Task 10 ──────────────────────────────────────
    //
    // Per-axis Klipper SysTick consumers (one timer per axis: X=0, Y=1,
    // Z=2, E=3) read these two scheduler tunables every dispatch. They
    // live on `SharedState` so the foreground config path (Task 11's
    // `configure_kinematics`) publishes once and every per-axis timer
    // observes a consistent pair on its next wake. Both accessors take no
    // runtime handle — the per-axis timer is dispatched from Klipper's
    // scheduler context where threading an `rt` arg through the `struct
    // timer.func` typedef would force a parallel ABI. Internally they
    // reach `rt_storage` (the published runtime context buffer) directly,
    // gated by `INIT_DONE`, and return 0 if the runtime hasn't initialised
    // yet — a safe default that makes the timer body fall back to "wake
    // `now` with no floor" until configure_kinematics lands.

    /// Project the `rt_storage` byte buffer to a `*const RuntimeContext`,
    /// returning `None` if `INIT_DONE` hasn't been published yet. Used by
    /// the handle-free FFI accessors (e.g. `kalico_runtime_get_*`) that
    /// run in scheduler / ISR contexts where threading the opaque `*mut
    /// KalicoRuntime` is impractical.
    fn runtime_handle_or_null() -> Option<*const RuntimeContext> {
        if !INIT_DONE.load(Ordering::Acquire) {
            return None;
        }
        // Same projection pattern as `runtime_handle_create`. `rt_storage`
        // is `UnsafeCell<[u8; N]>` on the MCU and `HostRtStorage` on the
        // host; in both cases `.get()` / `.0.get()` yields a `*mut [u8; N]`
        // with provenance over the full buffer, which we cast to
        // `*const RuntimeContext`. The const_assert above guarantees the
        // struct fits.
        #[cfg(target_os = "none")]
        let rt_ptr: *const RuntimeContext = {
            // SAFETY: `rt_storage` is a C-declared extern static; reading
            // its address (`.get()`) does not form an aliasing reference.
            // The actual access is gated by `INIT_DONE` above.
            #[allow(unsafe_code)]
            unsafe {
                rt_storage.get().cast::<RuntimeContext>()
            }
        };
        #[cfg(not(target_os = "none"))]
        let rt_ptr: *const RuntimeContext = rt_storage.0.get().cast::<RuntimeContext>();
        Some(rt_ptr)
    }

    /// Read the per-axis-timer dispatcher floor (cycles). The minimum
    /// number of MCU clock cycles into the future the per-axis timer adds
    /// to `now` when computing its next waketime; prevents runaway
    /// re-entry. Published by Task 11's `configure_kinematics`. Returns 0
    /// until the runtime is initialised.
    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_dispatcher_floor_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: `rt_ptr` is the published rt_storage projection, valid
        // for the lifetime of the program once INIT_DONE is set. Read-only
        // access to `SharedState` atomics via a shared `&` projection — no
        // `&mut` reaches this path.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr)
                .dispatcher_floor_cycles
                .load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

    /// Read the per-axis-timer empty-queue poll cadence (cycles). Used by
    /// the per-axis timer when its queue is empty: it reschedules at
    /// `now + sample_period_cycles`. Typically set to the modulation-rate
    /// period (25 µs at 40 kHz). Published by Task 11's
    /// `configure_kinematics`. Returns 0 until the runtime is initialised
    /// — `0` cycles means "wake `now`," which the next dispatch will
    /// immediately reschedule.
    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_sample_period_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: see `kalico_runtime_get_dispatcher_floor_cycles`.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr).sample_period_cycles.load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

    // ─── Stepping-redesign Task 11 ──────────────────────────────────────
    //
    // Three foreground configuration entry points. Each unpacks f32 bits
    // from the wire (Klipper protocol carries floats as u32-bits), forms
    // `&mut IsrState.engine` under the §11.2 raw-pointer projection
    // discipline, and delegates validation + state-publish to the engine
    // method. Foreground-only — these handlers run from the
    // single-threaded command dispatcher between segments, never during
    // a TIM5 ISR tick on the same axis state, so projecting `&mut
    // IsrState` here is sound under the same precondition that
    // `kalico_runtime_seed_position` and `kalico_runtime_configure_axis`
    // rely on.

    /// Stepping-redesign Task 14. Publish per-axis configuration with
    /// explicit stepper bindings. `microstep_distance_f32_bits` is
    /// `f32::to_bits` of the per-step distance (Klipper carries f32 as u32
    /// on the wire). `mode` is `0` for Pulse; `1` for Phase (TMC5160
    /// XDIRECT coil-current modulation). Other mode values return
    /// `KALICO_ERR_INVALID_ARG`. `ring_depth` is the number of
    /// [`PieceEntry`] slots to allocate for this axis's ring region
    /// from the shared `piece_storage`; the engine bump-allocates
    /// contiguously. The C caller currently passes a compile-time
    /// default (see `command_kalico_configure_axis` in `src/stepper.c`);
    /// a future protocol revision will let the host drive this via
    /// the wire. `bindings_ptr` points to an array of `stepper_count`
    /// [`runtime::stepping_state::StepperBindingRust`] entries; a null
    /// pointer with `stepper_count == 0` is legal (axis with no steppers,
    /// e.g. virtual / logical-only). Returns `0` on success, negative on
    /// validation failure. The C handler treats any non-zero return as a
    /// hard error and shuts the MCU down.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_configure_axis(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        mode: u8,
        microstep_distance_f32_bits: u32,
        ring_depth: u16,
        bindings_ptr: *const runtime::stepping_state::StepperBindingRust,
        stepper_count: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let mode_enum = match mode {
            0 => runtime::stepping_state::StepMode::Pulse,
            1 => runtime::stepping_state::StepMode::Phase,
            _ => return KALICO_ERR_INVALID_ARG,
        };
        let mstep_dist = f32::from_bits(microstep_distance_f32_bits);
        // Reconstruct the bindings slice. A null pointer with count 0 is
        // valid (axis with no steppers); null + non-zero is rejected to
        // avoid forming a slice over an invalid pointer.
        let bindings: &[runtime::stepping_state::StepperBindingRust] = if stepper_count == 0 {
            &[]
        } else if bindings_ptr.is_null() {
            return KALICO_ERR_NULL_PTR;
        } else {
            // SAFETY: caller guarantees `bindings_ptr` is valid for
            // `stepper_count` elements of `StepperBindingRust` (4-byte
            // `#[repr(C)]` struct). The C command dispatcher passes the
            // address of a stack-allocated array it owns for the duration
            // of this call. The slice borrow does not outlive this
            // function.
            unsafe { core::slice::from_raw_parts(bindings_ptr, stepper_count as usize) }
        };
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only entry; spec §11.2 raw-pointer projection.
        // Engine state is on `IsrState`; foreground may form `&mut
        // IsrState` here under the precondition that TIM5 is not
        // concurrently ticking the same per-axis state (Klipper command
        // dispatch is single-threaded and serialised against the modulated
        // tick by priority arbitration during configuration windows).
        let total_ring_pieces = runtime::state::TOTAL_RING_PIECES;
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let rc = (*isr_ptr).engine.configure_axis(
                axis_idx,
                mode_enum,
                mstep_dist,
                ring_depth as usize,
                bindings,
                total_ring_pieces,
            );
            if rc != KALICO_OK {
                return rc;
            }
            // If this axis was configured as Modulated (via SharedState step_modes),
            // upgrade axis.mode from Pulse to Phase so the ISR routes through
            // dispatch_phase for XDIRECT coil modulation.
            let shared_ptr: *const runtime::state::SharedState = core::ptr::addr_of!((*ctx).shared);
            if (axis_idx as usize) < runtime::state::MAX_STEPPER_OIDS {
                let step_mode = (*shared_ptr).step_modes[axis_idx as usize]
                    .load(core::sync::atomic::Ordering::Acquire);
                if step_mode == runtime::state::StepMode::Modulated as u8 {
                    // stepping_axes[axis_idx] is now Option<AxisState>; it was
                    // just configured above so it's Some.
                    if let Some(Some(axis)) =
                        (*isr_ptr).engine.stepping_axes.get_mut(axis_idx as usize)
                    {
                        axis.mode.store(
                            runtime::stepping_state::StepMode::Phase as u8,
                            core::sync::atomic::Ordering::Release,
                        );
                    }
                }
            }
            KALICO_OK
        }
    }

    /// Reset the motion engine to a clean state — issued by the host on every
    /// (re)connect before reconfiguring axes. Rewinds the ring bump allocator
    /// and clears all per-axis state so re-sent `configure_axis` commands never
    /// overflow `piece_storage` on a reconnect-without-reboot.
    ///
    /// Must be called from foreground inside an IRQ-disabled window (the C
    /// command handler holds `irq_save`/`irq_restore`): the engine state and the
    /// per-axis step queues this clears are concurrently touched by the TIM5
    /// sample ISR and the per-axis step-event timers.
    ///
    /// Returns `KALICO_OK` (0), `KALICO_ERR_NULL_PTR`, or `KALICO_ERR_NOT_INIT`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_reset(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only entry under the C-side IRQ guard; spec §11.2
        // raw-pointer projection. No other `&mut IsrState` may be live.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.reset();
        }
        // Clear the C-owned per-axis step queues (MCU build only); host/test
        // builds have no `step_queues` global.
        #[cfg(not(any(test, feature = "host")))]
        runtime::step_queue::reset_all_queues();
        KALICO_OK
    }

    /// Write one 32-byte [`PieceEntry`] to absolute physical slot
    /// `(start_slot + index) mod ring_depth` for `axis_idx`. Does **not**
    /// advance the frontier — the slot becomes visible to the ISR consumer
    /// only after a subsequent [`kalico_runtime_commit_head`] call.
    /// Streamed pre-CRC by the transport (Task 7); the CRC verification step
    /// calls `commit_head` to expose the batch atomically.
    ///
    /// `piece_ptr` points to exactly 32 raw bytes in `PieceEntry` wire format
    /// (little-endian, 8-byte-aligned field layout). The pointer may be
    /// unaligned relative to 8-byte boundaries — `read_unaligned` is used
    /// internally, matching the same `read_unaligned` discipline used throughout this FFI.
    ///
    /// Returns `KALICO_OK` on success; `KALICO_ERR_NULL_PTR` if `rt` or
    /// `piece_ptr` is null; `KALICO_ERR_NOT_INIT` if the runtime has not been
    /// initialised; `KALICO_ERR_INVALID_ARG` if `axis_idx` is out of range or
    /// the axis has not been configured.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_write_piece(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        start_slot: u16,
        index: u8,
        piece_ptr: *const u8,
    ) -> i32 {
        if rt.is_null() || piece_ptr.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only entry. The §11.2 split guarantees:
        //   - The ISR (TIM5) exclusively pops from each axis ring's tail cursor;
        //     the foreground exclusively writes to ring slots. Head is NOT
        //     advanced here — that is deferred to a subsequent
        //     `kalico_runtime_commit_head` call. The two sides never touch the
        //     same slot simultaneously.
        //   - `UnsafeCell::raw_get` on `piece_storage` yields a `*mut [..]`
        //     with provenance over the full array without forming a shared
        //     reference to the `UnsafeCell`.
        //   - `read_unaligned` is used for the incoming `PieceEntry` because
        //     `piece_ptr` is a byte offset into a protocol frame buffer and is
        //     not guaranteed to satisfy the 8-byte alignment required by
        //     `PieceEntry`. `read_unaligned` is sound for any `Copy` type as
        //     long as the source address is valid for reads of `size_of::<T>()`
        //     bytes and all bit patterns are valid — both hold here
        //     (`PieceEntry` has no invalid-bit-pattern invariants).
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let ps_ptr: *mut [runtime::piece_ring::PieceEntry; runtime::state::TOTAL_RING_PIECES] =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).piece_storage));
            let storage: &mut [runtime::piece_ring::PieceEntry] = &mut *ps_ptr;
            let Some(axis) = (*isr_ptr)
                .engine
                .stepping_axes
                .get_mut(axis_idx as usize)
                .and_then(|s| s.as_mut())
            else {
                return KALICO_ERR_INVALID_ARG;
            };
            if !axis.ring.is_configured() {
                return KALICO_ERR_INVALID_ARG;
            }
            let depth = axis.ring.ring_depth;
            let slot = (start_slot as usize + index as usize) % depth;
            let entry =
                core::ptr::read_unaligned(piece_ptr.cast::<runtime::piece_ring::PieceEntry>());
            axis.ring.write_slot(storage, slot, entry);
        }
        KALICO_OK
    }

    /// Advance the axis ring's monotonic valid frontier to `new_head`.
    ///
    /// The advance is accepted only when `new_head` represents a strict
    /// increase over the current frontier and keeps occupancy within
    /// `ring_depth` (flow-control invariant). A stale re-send with a lower
    /// `new_head` is silently ignored. Called post-CRC by the transport
    /// (Task 7) after a batch of slots has been written via
    /// [`kalico_runtime_write_piece`].
    ///
    /// Returns `KALICO_OK` on success (including the monotone no-op case);
    /// `KALICO_ERR_NULL_PTR` if `rt` is null; `KALICO_ERR_NOT_INIT` if the
    /// runtime has not been initialised; `KALICO_ERR_INVALID_ARG` if
    /// `axis_idx` is out of range or the axis has not been configured.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_commit_head(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        new_head: u32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: same §11.2 foreground-only precondition as write_piece.
        // `ring.head` is a plain u32: the foreground is the sole writer and
        // the ISR reads it via peek()/is_empty(). On single-core ARMv7E-M,
        // exception entry/return act as memory barriers (boundary rule B5),
        // so the ISR's read is sequenced after the foreground's store with no
        // explicit fence required. No overlapping mutable projections.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let Some(axis) = (*isr_ptr)
                .engine
                .stepping_axes
                .get_mut(axis_idx as usize)
                .and_then(|s| s.as_mut())
            else {
                return KALICO_ERR_INVALID_ARG;
            };
            if !axis.ring.is_configured() {
                return KALICO_ERR_INVALID_ARG;
            }
            axis.ring.commit_head(new_head);
        }
        KALICO_OK
    }

    /// Stepping-redesign Task 11. Publish the kinematic scale factor
    /// relating logical-XY velocity to physical motor-coordinate velocity
    /// (`1.0` Cartesian, `1/√2` CoreXY). `k_xy_f32_bits` is `f32::to_bits`
    /// of the scalar. Rejected if not finite or not strictly positive.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_configure_kinematics(
        rt: *mut KalicoRuntime,
        k_xy_f32_bits: u32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let k_xy = f32::from_bits(k_xy_f32_bits);
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: see `kalico_runtime_configure_axis` SAFETY note.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.configure_kinematics(k_xy)
        }
    }

    /// Stepping-redesign Task 11. Publish asymmetric pressure-advance
    /// coefficients (seconds). `0.0` on either side disables PA in that
    /// phase. Negative values are rejected (PA is never physically
    /// negative).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_configure_pressure_advance(
        rt: *mut KalicoRuntime,
        advance_accel_f32_bits: u32,
        advance_decel_f32_bits: u32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let advance_accel = f32::from_bits(advance_accel_f32_bits);
        let advance_decel = f32::from_bits(advance_decel_f32_bits);
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: see `kalico_runtime_configure_axis` SAFETY note.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr)
                .engine
                .configure_pressure_advance(advance_accel, advance_decel)
        }
    }

    /// Stepping-redesign Task 12. Flip a logical axis between `Pulse` and
    /// `Phase` output modes. `axis_idx` is `0..N_AXES` (X=0, Y=1, Z=2,
    /// E=3); `new_mode` is `0` for Pulse, `1` for Phase. Rejected (returns
    /// `-2`) if any axis currently has an active Bezier piece — the flip
    /// must happen between segments. Other validation failures return
    /// `-1`. On success, the engine flushes the per-axis step queue,
    /// resyncs the Phase-side `last_phase_target` counters when entering
    /// Phase mode, and publishes the new mode atomically. See
    /// `Engine::set_axis_mode` for the full spec-step sequence.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_set_axis_mode(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        new_mode: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: see `kalico_runtime_configure_axis` SAFETY note. The
        // command dispatcher is single-threaded and serialised against
        // the modulated tick — projecting `&mut IsrState` here is sound
        // under that precondition (foreground-only entry point, never
        // re-entered from the TIM5 ISR).
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.set_axis_mode(axis_idx, new_mode)
        }
    }

    /// Stepping-redesign Task 12. Add `delta_microsteps` to a single
    /// physical stepper's `phase_offset_target`. The Task-13 TIM5 ramp
    /// helper walks `phase_offset_microsteps` toward this target at no
    /// more than `max_microsteps_per_sample` microsteps per sample.
    /// `stepper_idx` is the global stepper index across all configured
    /// axes (sum of per-axis stepper counts in axis order). Validated
    /// `max_microsteps_per_sample ∈ 1..=256`; an invalid argument latches
    /// `FaultCode::JogParametersInvalid` and returns `-1`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_set_stepper_offset(
        rt: *mut KalicoRuntime,
        stepper_idx: u8,
        delta_microsteps: i32,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: see `kalico_runtime_configure_axis` SAFETY note. The
        // `&SharedState` borrow uses `addr_of!` (no `&mut` ever formed —
        // SharedState is atomics-only) and is independent of the
        // `&mut IsrState` projection.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            (*isr_ptr).engine.set_stepper_offset(
                shared,
                stepper_idx,
                delta_microsteps,
                max_microsteps_per_sample,
            )
        }
    }

    /// Fill caller buffers with the current heartbeat snapshot.
    ///
    /// Writes `engine_state`, `fault_code`, and up to `max_axes`
    /// per-axis retired piece counts into the caller-provided buffers.
    ///
    /// Returns the number of axes actually written (>= 0) on success, or a
    /// negative error code:
    /// - `-1` (`KALICO_ERR_NULL_PTR`)  — `rt` or any out-pointer is null.
    /// - `-2` (`KALICO_ERR_NOT_INIT`)  — runtime not yet initialised.
    ///
    /// Only `[..min(num_axes, max_axes)]` entries are written to
    /// `out_retired`; the caller must allocate at least `max_axes` u32s.
    ///
    /// # Safety
    /// - `rt` must be the handle returned by `runtime_handle_create`.
    /// - `out_engine_state`, `out_fault_code` must be valid for a single-byte
    ///   write.
    /// - `out_retired` must be valid for `max_axes` u32 writes (≥ 4-byte
    ///   aligned, as a C `uint32_t[8]` is).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_heartbeat(
        rt: *mut KalicoRuntime,
        out_engine_state: *mut u8,
        out_fault_code: *mut u8,
        out_retired: *mut u32,
        max_axes: usize,
    ) -> i32 {
        if rt.is_null()
            || out_engine_state.is_null()
            || out_fault_code.is_null()
            || out_retired.is_null()
        {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only access to IsrState atomics via the §11.2
        // shared-borrow discipline — same pattern as
        // `runtime_handle_tick_counter`. `Engine::status()` and
        // `Engine::last_error()` both load through atomics; `num_axes` is
        // a plain u8 written only during foreground configure_axis calls
        // (not during ISR ticks); `retired_counts()` reads per-axis
        // ring descriptors through plain u32 loads that are written
        // exclusively by the ISR retire path. A transient torn read of one
        // count (non-atomic u32 on 32-bit Cortex-M is single-instruction
        // aligned, so effectively atomic in practice) is tolerable for a
        // 10 Hz diagnostic heartbeat.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let engine = &(*isr_ptr).engine;

            let engine_state = engine.status() as u8;
            let fault_code = (engine.last_error() as u32 & 0xFF) as u8;
            let num_axes = engine.num_axes as usize;
            let counts = engine.retired_counts();
            let n_write = num_axes.min(max_axes);

            core::ptr::write(out_engine_state, engine_state);
            core::ptr::write(out_fault_code, fault_code);
            for i in 0..n_write {
                out_retired.add(i).write(counts[i]);
            }
            #[allow(clippy::cast_possible_truncation)]
            let result = n_write as i32;
            result
        }
    }

    /// Install C-owned `step_queues` into the Rust engine on host/MACH_LINUX
    /// builds so `tick_sample`'s `dispatch_axis` can push step entries.
    ///
    /// On the MCU the engine resolves the C `step_queues` extern directly
    /// via `#[cfg(not(target_os = "none"))]`; on host builds
    /// `test_queue_ptrs` stays null unless explicitly installed here.
    ///
    /// Called once from `runtime_tick_enable` in `src/linux/runtime_tick_host.c`
    /// before the tick thread is armed, so there is no concurrent ISR writer.
    ///
    /// Returns `KALICO_OK` (0) on success, or a negative error code:
    /// - `KALICO_ERR_NULL_PTR`  — `rt` or `queues` is null.
    /// - `KALICO_ERR_NOT_INIT`  — runtime has not been initialised yet.
    #[cfg(feature = "host")]
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_install_step_queues(
        rt: *mut KalicoRuntime,
        queues: *mut u8,
    ) -> i32 {
        if rt.is_null() || queues.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let q0 = queues.cast::<runtime::step_queue::StepQueue>();
            let ptrs: [*mut runtime::step_queue::StepQueue; runtime::stepping_state::N_AXES] = [
                q0,
                q0.add(1),
                q0.add(2),
                q0.add(3),
                q0.add(4),
                q0.add(5),
                q0.add(6),
                q0.add(7),
            ];
            (*isr_ptr).engine.test_install_step_queues(ptrs);
        }
        KALICO_OK
    }
}
