//! Kalico runtime C-FFI surface. Spec §3.2 / §4.4 / §5.2 / §5.6 / §11.2.
//!
//! cfg-gated by `header-runtime`. Exposes the opaque `*mut KalicoRuntime`
//! handle plus the `kalico_runtime_*` entrypoints used by the Klipper C ISR
//! shim and the foreground producer task.
//!
//! ## Half-split aliasing discipline
//!
//! Concurrent ISR/foreground entry via a shared `&mut RuntimeContext` would
//! create overlapping `&mut`s — UB under stacked/tree borrows. Instead, every
//! FFI entry projects to either `&mut FgState` or `&mut IsrState` (disjoint
//! memory regions) via `core::ptr::addr_of!` + `UnsafeCell::raw_get`. No
//! `&mut RuntimeContext` is ever materialised. See docs/kalico-rewrite/mcu-c-rust-boundary.md.

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

    /// Opaque handle type — never dereferenced on the C side (spec §3.2 / §5.6).
    #[allow(missing_debug_implementations)] // opaque to C; never inspected
    #[repr(C)]
    pub struct KalicoRuntime {
        _private: [u8; 0],
    }

    // rt_storage — backing buffer for RuntimeContext. On MCU: C-declared in
    // src/runtime_storage.c with linker-section placement (boundary rule B2
    // from docs/kalico-rewrite/mcu-c-rust-boundary.md — C owns placement).
    // On host: Rust-side static below; section placement not needed.
    // UnsafeCell grants interior-mutability rights to pointers derived via
    // .get(); no shared `&rt_storage` reference is ever formed by Rust.
    #[cfg(target_os = "none")]
    unsafe extern "C" {
        static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>;
    }

    // Host backing storage. UnsafeCell isn't Sync; wrap in a newtype with
    // unsafe impl Sync. Synchronization is the INIT_DONE guard + the
    // half-split aliasing pattern — not type-system Sync.
    // Lowercase name matches the C-side `uint8_t rt_storage[N]` symbol so
    // both cfg branches resolve to the same identifier.
    #[cfg(not(target_os = "none"))]
    #[repr(C, align(16))]
    struct HostRtStorage(UnsafeCell<[u8; RT_STORAGE_SIZE]>);
    // SAFETY: see runtime_handle_create's SAFETY comment.
    #[cfg(not(target_os = "none"))]
    unsafe impl Sync for HostRtStorage {}
    #[cfg(not(target_os = "none"))]
    #[allow(non_upper_case_globals)]
    static rt_storage: HostRtStorage = HostRtStorage(UnsafeCell::new([0u8; RT_STORAGE_SIZE]));

    const _: () = {
        assert!(
            core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
            "RuntimeContext outgrew RT_STORAGE_SIZE — bump Kconfig storage size"
        );
    };

    const _: () = {
        assert!(
            core::mem::align_of::<RuntimeContext>() <= 16,
            "RuntimeContext alignment > 16 — bump _Alignas in runtime_storage.c"
        );
    };

    /// Single-shot init guard. Set true exactly once in `runtime_handle_create`
    /// and never cleared; all FFI entries Acquire-load it before dereferencing rt.
    pub(super) static INIT_DONE: AtomicBool = AtomicBool::new(false);

    // C-side cycle-counter helper — defined in src/stm32/runtime_tick_h7.c
    // on the MCU and stubbed by the integration-test harness on host.
    unsafe extern "C" {
        fn runtime_cyccnt_read() -> u32;
    }

    /// Init-once; returns the static `RuntimeContext` handle or null on
    /// subsequent calls. Spec §3.2.
    #[unsafe(no_mangle)]
    pub extern "C" fn runtime_handle_create() -> *mut KalicoRuntime {
        // Plain load/store instead of compare_exchange: on Cortex-M7 the
        // compiler lowers CAS to LDREXB/STREXB (exclusive monitor). Renode
        // H7 v1.16 silently drops the exclusive store (STREXB returns r2=0
        // but does not write to memory), leaving INIT_DONE=0 even after the
        // code proceeds into init(). Non-exclusive STRB (via store) avoids
        // that Renode bug. Klipper calls this exactly once before TIM5 is
        // armed, so there is no real concurrent caller to guard against.
        if INIT_DONE.load(Ordering::Relaxed) {
            return core::ptr::null_mut();
        }
        // SAFETY: single-threaded init; no other context observes rt_storage
        // until INIT_DONE is published below. RuntimeContext::init writes
        // through raw-pointer projections and never forms `&mut RuntimeContext`
        // (§11.2 aliasing discipline). rt_storage.get() yields *mut [u8; N]
        // with provenance over the full buffer; cast to *mut RuntimeContext
        // inherits that provenance (const_assert above ensures it fits).
        unsafe {
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
            // Release pairs with Acquire loads in every FFI call; ISR sees
            // either INIT_DONE=false (before enable) or fully-initialised context.
            INIT_DONE.store(true, Ordering::Release);
            rt_ptr.cast::<KalicoRuntime>()
        }
    }

    /// Validate the leading version byte of a blob payload (§4.2).
    /// Returns `KALICO_OK` on a recognised version or
    /// `KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED` otherwise.
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

    /// TIM5 ISR body — piece-ring walker.
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
        // SAFETY: rt non-null and INIT_DONE=true. TIM5 is the SOLE writer of
        // IsrState (§11.1). piece_storage aliasing is sound: ISR owns the ring
        // tail (pop) and per-axis ISR-cache fields; foreground writes only to
        // HEAD positions and never reuses a slot the ISR may still read —
        // anything between retired and head, including ISR-cached "current
        // piece" slots — until the ISR retires it (simple-mcu-contract design
        // §4.2 slot-freeing invariant).
        // UnsafeCell::raw_get yields provenance without forming a shared ref.
        unsafe {
            let raw = runtime_cyccnt_read();
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
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
        // SAFETY: read-only SharedState atomics access; no `&mut` on this path.
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
        // SAFETY: read-only SharedState atomics access.
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
        // SAFETY: §11.1 — ISR is sole writer of IsrState; foreground may form
        // `&IsrState` for atomic reads (the atomic provides synchronisation).
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.tick_counter()
        }
    }

    /// Alias for `runtime_handle_tick_counter` with the `kalico_runtime_get_*` naming.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_tick_counter(rt: *mut KalicoRuntime) -> u32 {
        unsafe { runtime_handle_tick_counter(rt) }
    }

    // ---- §5.3 status-frame accessors ---------------------------------------
    //
    // Each helper projects to &SharedState (atomics-only). Separate FFI
    // functions per Klipper's "one sendf call per scalar" pattern; the
    // DECL_TASK assembles them into the ~10 Hz `kalico_status_v6` frame.

    /// On-demand widened MCU clock using `timer_read_time` +
    /// `stats_send_time_high` (spec §3.9). The stats-task path is used
    /// (not the engine seqlock) because `timer_read_time` is not re-entrant
    /// with the stats-task wrap update — do not call from ISR context.
    /// Mirrors `runtime_widened_host_clock` in `src/runtime_tick.c`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_widened_now(rt: *mut KalicoRuntime) -> u64 {
        // rt is unused (widening reads Klipper globals), but kept for ABI stability.
        let _ = rt;
        unsafe extern "C" {
            fn timer_read_time() -> u32;
            static stats_send_time: u32;
            static stats_send_time_high: u32;
        }
        // SAFETY: `timer_read_time` is a single u32 MMIO/software-counter read,
        // safe from any non-ISR caller. `stats_send_time*` are u32 globals
        // written by the stats DECL_TASK; a concurrent torn read self-corrects
        // within one stats cadence (~5 s) — error bounded by one u32 wrap
        // (~37 s on 120 MHz F4, ~16 s on H7), far longer than host tolerance.
        unsafe {
            let low = timer_read_time();
            let high = stats_send_time_high + ((low < stats_send_time) as u32);
            ((high as u64) << 32) | (low as u64)
        }
    }

    /// Low 32 bits of the most recent `now - seg.t_start` from
    /// `runtime_modulated_tick`. `0` before any segment or when clock
    /// is behind `t_start` (saturating_sub clamps to 0).
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

    /// Low 32 bits of the active segment's `duration()`. If `elapsed_lo`
    /// >= this, the retirement branch fires on the next tick.
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

    /// How many times the modulated-tick retirement branch was entered
    /// (`elapsed >= duration`). Pair with `retire_successes` to diagnose
    /// whether retirement stalls on the `consumers_done` check.
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

    /// How many times modulated-tick actually retired a segment
    /// (`consumers_done` after clearing motor bits). Less than
    /// `retire_attempts` means some entries leave consumer bits set.
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

    /// `consumers_remaining` after the clear-all-motors loop in modulated_tick
    /// retirement. Non-zero when `retire_attempts > retire_successes` reveals
    /// which bits the per-motor clear didn't reach.
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

    /// Latched `fault_detail` payload (§9.2); `0` if no fault or no detail.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_fault_detail(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).fault_detail.load(Ordering::Acquire)
        }
    }

    /// Scheduler dispatch-history func addr at the most recent `-311` fault.
    /// Not wired into the fault event (use `tick_blocker_pc` for addr2line).
    /// `0` before any `-311` fires.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_func.load(Ordering::Acquire)
        }
    }

    /// Exception-frame return address (PC) captured at TIM5 handler entry on
    /// the most recent `-311 TickIntervalExceeded` fault — the addr2line
    /// target naming code that held the CPU/PRIMASK across the late tick.
    /// Wired into the fault event's `segment_id` field by `runtime_tick.c`.
    /// `0` before any `-311` fires (or on host/test builds).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_pc(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_pc.load(Ordering::Acquire)
        }
    }

    /// Stacked xPSR exception number from the most recent `-311` fault entry.
    /// `0` = foreground was interrupted; nonzero = that IRQ was interrupted
    /// (TIM5 tail-chained). `0` before any `-311` fires or on host builds.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_exc(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics access.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_exc.load(Ordering::Acquire)
        }
    }

    /// Configured `steps_per_mm` for axis `oid` (0..=3). Returns 0.0 if
    /// out of range or uninitialised.
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

    /// Seed the engine's widen state so `now` agrees with Klipper's widened
    /// MCU clock. Called by the Linux sim host once at runtime_init before
    /// the engine pthread starts ticking. `baseline_widened_clock` folds in
    /// any u32 wrap counts already passed (computed from `clock_gettime`).
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

    /// `force_idle` handshake (§8.5). Passes raw `*mut RuntimeContext`
    /// directly because flush projects to both halves under disabled-IRQ.
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
        // SAFETY: rt non-null + INIT_DONE verified above; flush() does its
        // own half-split projections per §8.5.
        unsafe { runtime::stream::flush(rt.cast::<RuntimeContext>(), out_credit_epoch) }
    }

    /// RTT-aware clock-sync ping (§12.1). Returns the on-demand widened MCU
    /// clock (`timer_read_time` + `stats_send_time_high`), not the engine
    /// seqlock, so it is correct regardless of whether TIM5 is running. The
    /// ~5 s lag of `stats_send_time_high` is invisible to the bridge's
    /// RTT-aware linear regression.
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
        // SAFETY: same as `runtime_handle_widened_now` — single u32 reads of
        // Klipper globals, safe from any non-ISR caller.
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
            unsafe { *out_mcu_clock = mcu_clock };
        }
        KALICO_OK
    }

    // ---- Endstop arm/disarm/poll_trip -------------------------------------

    use runtime::endstop::{
        ArmMsg, ArmPolicy, ArmStatus, DisarmStatus, MAX_SOURCES, MAX_STEPPERS, SourceConfig,
        SourceKind, VelocityAxis,
    };

    const SOURCE_RECORD_LEN: usize = 11;
    const STEPPER_RECORD_LEN: usize = 1;

    // Trip event v1 wire format: arm_id(4) + clock_lo(4) + clock_hi(4)
    // + source_idx(1) + fmt_version(1) + stepper_count(1) + stepper_data(N*5).
    pub const KALICO_TRIP_EVENT_V1_HEADER_LEN: usize = 15;
    pub const KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN: usize = 5;
    pub const KALICO_TRIP_EVENT_V1_FMT_VERSION: u8 = 1;
    pub const KALICO_TRIP_EVENT_V1_MAX_LEN: usize =
        KALICO_TRIP_EVENT_V1_HEADER_LEN + MAX_STEPPERS * KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN;

    /// Arm an endstop (spec §3.1 blob layout). `sources`: 11-byte records
    /// (kind u8, gpio u16 LE, polarity u8, arm_policy u8, sample_n u8,
    /// velocity_axis u8, v_min_q16 u32 LE). `steppers`: 1-byte records
    /// (stepper_oid u8). `*out_status`: 0=Armed, 1=AlreadyTripped, 2=Rejected.
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

    /// Disarm an active endstop arm. `*out_status`: 0=Disarmed,
    /// 1=AlreadyTripped, 2=Unknown (spec §3.5).
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

    /// Software-trip an armed endstop. `*out_status`: 0=Tripped,
    /// 1=NotArmed, 2=WrongArmId.
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
        // Empty stepper counts: MCU step counters are unavailable in command
        // handler context; the host uses the retained curve for position
        // reconstruction instead.
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
    /// Wire format v1 LE: `arm_id u32 | clock_lo u32 | clock_hi u32 |
    /// source_idx u8 | fmt_version u8 (=1) | stepper_count u8`; then
    /// per stepper: `oid u8 | step_count i32`.
    /// Returns 1 + `*out_actual_len` = encoded length on success; 0 if no
    /// event ready; `KALICO_ERR_NULL_PTR` on bad args or undersized buffer.
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

    /// Push a sampled GPIO level into the abstract pin table.
    ///
    /// The C ISR shim samples GPIOs via `gpio_in_read` once per TIM5 tick
    /// and pushes each result here before `endstop::tick` observes it. Sim
    /// builds call this directly via `command_runtime_sim_endstop_set_pin`,
    /// bypassing real GPIO.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_set_pin_level(gpio: u16, level: u8) -> i32 {
        if runtime::endstop::set_pin_level(gpio, level != 0) {
            KALICO_OK
        } else {
            KALICO_ERR_NULL_PTR
        }
    }

    // ---- Step-time scheduling FFI (spec §5) --------------------------------

    /// Flip a stepper's `StepMode` at runtime (spec §10).
    /// `mode`: 0=Modulated (phase-stepping), 1=StepTime (classic).
    /// `mcu_supports_phase`: non-zero if the MCU advertises phase-stepping.
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
        // SAFETY: SharedState atomics-only; no `&mut`. `step_modes` are
        // AtomicU8 designed for concurrent foreground writes and ISR reads.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match runtime::state::set_step_mode(shared, stepper_idx, mode, mcu_supports_phase != 0)
            {
                Ok(()) => {
                    // TIM5 lifecycle is decoupled from step mode: armed at
                    // runtime_tick_init and disabled only on Klipper shutdown.
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
    // `kalico_runtime_producer_step`, `*_step_ring_*`, `*_kick_producer`,
    // `*_get_producer_pending`, `*_force_idle`, `*_apply_step`, and all
    // associated producer/Newton diagnostics were removed by the stepping
    // redesign. The new path is `kalico_runtime_tick_sample` driving the
    // cubic evaluator — no producer timer, no per-motor step rings, no Newton.

    /// Low 32 bits of `producer_enqueue_success_total`. If non-zero while
    /// `producer_segment_dequeued_total` is 0, the SPSC queue split is
    /// broken (producer and consumer not sharing the same backing buffer).
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

    /// Low 32 bits of `push_segment_all_unused_total`. Advancing during a jog
    /// means the bridge sent push_segment frames with every handle set to the
    /// UNUSED sentinel — segment retires without motor processing ("energized
    /// but no motion" symptom).
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

    /// Packed last `x_handle` from `push_segment_impl`:
    /// `(gen << 16) | slot_idx`. UNUSED sentinel = 0xFFFE_FFFE.
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

    /// Packed last `y_handle` from `push_segment_impl` (same layout as x_handle).
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

    /// Last `consumers_remaining` mask from `push_segment_impl`.
    /// Zero means every handle in the most recent push_segment was UNUSED.
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

    /// Current `StepMode` for `stepper_idx`: 0=Modulated (TIM5 ISR),
    /// 1=StepTime (Klipper scheduler), 0xFF=invalid/uninitialised.
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
        // SAFETY: rt published and INIT_DONE=true; step_modes are AtomicU8 in
        // SharedState; shared `&SharedState` — no `&mut` formed.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            shared.step_modes[stepper_idx as usize].load(Ordering::Acquire)
        }
    }

    /// Phase-stepping SPI config for motor `motor_idx` (spec §4.1).
    /// Packed u16: high byte = `spi_bus_id`, low byte = `cs_pin_id`.
    /// `0xFFFF` = no config / null rt / uninitialised / out of range.
    /// Use `runtime::phase_config::PhaseConfig::unpack` to decode.
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
        // SAFETY: rt published; phase_config is AtomicU16 in SharedState —
        // shared `&SharedState`, no `&mut`.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match shared.phase_config.get(motor_idx as usize) {
                Some(slot) => slot.load(Ordering::Acquire),
                None => 0xFFFF,
            }
        }
    }

    // ─── Handle-free scheduler tunables ────────────────────────────────
    //
    // Per-axis Klipper SysTick timers (X=0, Y=1, Z=2, E=3) read these two
    // tunables every dispatch. They take no `rt` argument because the Klipper
    // `struct timer.func` typedef makes threading a pointer impractical;
    // instead they reach `rt_storage` directly via `runtime_handle_or_null`,
    // gated by INIT_DONE. Return a safe default (5 MHz) before init.

    /// Project `rt_storage` to `*const RuntimeContext`, or `None` before
    /// `INIT_DONE`. Same projection pattern as `runtime_handle_create`.
    fn runtime_handle_or_null() -> Option<*const RuntimeContext> {
        if !INIT_DONE.load(Ordering::Acquire) {
            return None;
        }
        #[cfg(target_os = "none")]
        let rt_ptr: *const RuntimeContext = {
            // SAFETY: reading .get() on a C extern static yields a raw pointer
            // without forming an aliasing reference; access gated by INIT_DONE.
            #[allow(unsafe_code)]
            unsafe {
                rt_storage.get().cast::<RuntimeContext>()
            }
        };
        #[cfg(not(target_os = "none"))]
        let rt_ptr: *const RuntimeContext = rt_storage.0.get().cast::<RuntimeContext>();
        Some(rt_ptr)
    }

    /// Minimum MCU clock cycles the per-axis timer adds to `now` when
    /// computing its next wake time (prevents runaway re-entry).
    /// Returns 5 MHz default until initialised.
    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_dispatcher_floor_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: rt_ptr is the static rt_storage projection; read-only
        // SharedState atomics access via shared `&` — no `&mut`.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr)
                .dispatcher_floor_cycles
                .load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

    /// Empty-queue poll cadence for the per-axis timer (cycles). Timer
    /// reschedules at `now + sample_period_cycles` when idle; typically the
    /// modulation period (25 µs at 40 kHz). Returns 5 MHz default until
    /// initialised.
    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_sample_period_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: same as `kalico_runtime_get_dispatcher_floor_cycles`.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr).sample_period_cycles.load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

    // ─── Foreground configuration entry points ──────────────────────────
    //
    // Each unpacks f32 bits from the wire (Klipper carries floats as u32),
    // projects `&mut IsrState.engine` via §11.2 raw-pointer discipline, and
    // delegates to the engine method. Foreground-only — single-threaded
    // command dispatcher, never concurrent with a TIM5 tick on the same
    // axis state.

    /// Publish per-axis configuration with explicit stepper bindings.
    /// `microstep_distance_f32_bits` = `f32::to_bits` of per-step distance.
    /// `mode`: 0=Pulse, 1=Phase (TMC5160 XDIRECT). `ring_depth`: PieceEntry
    /// slots bump-allocated from `piece_storage` for this axis. `bindings_ptr`
    /// may be null when `stepper_count == 0` (logical-only axis). Non-zero
    /// return is treated as a hard error by the C handler.
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
        // Null ptr + count 0 is valid (no-stepper axis); null + non-zero
        // is rejected to avoid forming a slice over an invalid pointer.
        let bindings: &[runtime::stepping_state::StepperBindingRust] = if stepper_count == 0 {
            &[]
        } else if bindings_ptr.is_null() {
            return KALICO_ERR_NULL_PTR;
        } else {
            // SAFETY: caller guarantees bindings_ptr is valid for
            // stepper_count elements (#[repr(C)] StepperBindingRust).
            // C dispatcher passes a stack-allocated array that outlives
            // this call; the slice borrow does not escape.
            unsafe { core::slice::from_raw_parts(bindings_ptr, stepper_count as usize) }
        };
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only entry; §11.2 raw-pointer projection.
        // Klipper command dispatch is single-threaded and serialised against
        // the modulated tick during configuration windows.
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
            // If step_modes says Modulated, upgrade axis.mode to Phase so
            // the ISR routes through dispatch_phase for XDIRECT modulation.
            let shared_ptr: *const runtime::state::SharedState = core::ptr::addr_of!((*ctx).shared);
            if (axis_idx as usize) < runtime::state::MAX_STEPPER_OIDS {
                let step_mode = (*shared_ptr).step_modes[axis_idx as usize]
                    .load(core::sync::atomic::Ordering::Acquire);
                if step_mode == runtime::state::StepMode::Modulated as u8 {
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
    /// so re-sent `configure_axis` commands never overflow `piece_storage`.
    ///
    /// Must be called inside an IRQ-disabled window (C handler holds
    /// `irq_save`/`irq_restore`): engine state and step queues are touched
    /// concurrently by the TIM5 ISR and per-axis step-event timers.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_reset(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground under C-side IRQ guard; §11.2 raw-pointer
        // projection. No other `&mut IsrState` may be live.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.reset();
        }
        // MCU only: clear C-owned step queues (host/test builds have none).
        #[cfg(not(any(test, feature = "host")))]
        runtime::step_queue::reset_all_queues();
        KALICO_OK
    }

    /// Write one 32-byte [`PieceEntry`] to physical slot
    /// `(start_slot + index) mod ring_depth` for `axis_idx`. Does NOT advance
    /// the frontier — call [`kalico_runtime_commit_head`] post-CRC to expose
    /// the batch atomically. `piece_ptr` may be unaligned; `read_unaligned`
    /// is used internally.
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
        // SAFETY: §11.2 foreground-only entry. ISR pops from the ring tail;
        // foreground writes to slots and does NOT advance head here — the two
        // sides never touch the same slot simultaneously. UnsafeCell::raw_get
        // yields provenance without a shared ref. read_unaligned is used
        // because piece_ptr is a byte-offset into a protocol frame and need
        // not satisfy PieceEntry's 8-byte alignment; PieceEntry has no
        // invalid-bit-pattern invariants so all reads are sound.
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

    /// Advance the axis ring's monotonic frontier to `new_head`. A stale
    /// re-send with a lower `new_head` is silently ignored. Called post-CRC
    /// after a batch written via [`kalico_runtime_write_piece`].
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
        // ring.head is a plain u32 (foreground sole writer, ISR reads via
        // peek/is_empty). On single-core ARMv7E-M, exception entry/return are
        // memory barriers (boundary rule B5), so no explicit fence is needed.
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

    /// Publish the kinematic scale factor (logical-XY → motor-coord velocity):
    /// `1.0` Cartesian, `1/√2` CoreXY. Rejected if not finite or ≤ 0.
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

    /// Publish asymmetric pressure-advance coefficients (seconds).
    /// `0.0` disables PA for that phase; negative values are rejected.
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

    /// Flip axis `axis_idx` between Pulse (0) and Phase (1) output modes.
    /// Rejected (`-2`) if any axis has an active Bezier piece — must happen
    /// between segments. On success the engine flushes the step queue and
    /// resyncs `last_phase_target` counters when entering Phase mode.
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
        // SAFETY: see `kalico_runtime_configure_axis` SAFETY note.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.set_axis_mode(axis_idx, new_mode)
        }
    }

    /// Add `delta_microsteps` to a stepper's `phase_offset_target`. The TIM5
    /// ramp helper walks `phase_offset_microsteps` toward the target at no
    /// more than `max_microsteps_per_sample` per sample (valid range 1..=256;
    /// out-of-range latches `FaultCode::JogParametersInvalid` and returns `-1`).
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
        // SAFETY: see `kalico_runtime_configure_axis`. The &SharedState
        // borrow is independent of &mut IsrState — SharedState is atomics-only.
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

    /// Fill caller buffers with the heartbeat snapshot: `engine_state`,
    /// `fault_code`, and up to `max_axes` per-axis retired piece counts.
    /// Returns the number of axes written on success; negative on error.
    ///
    /// # Safety
    /// `out_engine_state` and `out_fault_code` must be valid for a 1-byte
    /// write; `out_retired` must be valid for `max_axes` u32 writes.
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
        // SAFETY: §11.2 shared-borrow on IsrState atomics. `num_axes` is a
        // plain u8 written only during foreground configure_axis. `retired_counts()`
        // reads plain u32 fields written exclusively by the ISR retire path;
        // a transient torn read is tolerable for a 10 Hz heartbeat (on
        // 32-bit Cortex-M, aligned u32 is single-instruction, so effectively
        // atomic in practice).
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

    /// Install C-owned `step_queues` into the Rust engine on host builds.
    /// On MCU the engine resolves the C extern directly; on host,
    /// `test_queue_ptrs` is null until this is called. Called once before
    /// the tick thread is armed — no concurrent ISR writer.
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
