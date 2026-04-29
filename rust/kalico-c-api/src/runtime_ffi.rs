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
    use core::mem::MaybeUninit;
    use core::sync::atomic::{AtomicBool, Ordering};

    use runtime::curve_pool::{CurvePool, MAX_DIM};
    use runtime::engine::RuntimeStatus;
    use runtime::error::{
        KALICO_ERR_FAULT_LATCHED, KALICO_ERR_INVALID_CURVE, KALICO_ERR_INVALID_DURATION,
        KALICO_ERR_INVALID_HANDLE, KALICO_ERR_INVALID_KINEMATICS, KALICO_ERR_NOT_INIT,
        KALICO_ERR_NULL_PTR, KALICO_ERR_QUEUE_FULL, KALICO_OK,
    };
    use runtime::segment::{CurveHandle, KinematicTag, Segment};
    use runtime::state::{FgState, IsrState, RuntimeContext, SharedState};
    use runtime::trace::TraceSample;

    /// The opaque type C sees — never dereferenced on the C side.
    /// Matches spec §3.2 / §5.6 handle discipline.
    #[allow(missing_debug_implementations)] // opaque to C; never inspected
    #[repr(C)]
    pub struct KalicoRuntime {
        _private: [u8; 0],
    }

    /// Concrete singleton storage. Spec §3.2 init-once protocol.
    ///
    /// Wrapped in `MaybeUninit` because `RuntimeContext::init` writes
    /// through raw-pointer projections (no constructor returns a
    /// fully-formed `RuntimeContext`). Wrapped in `UnsafeCell` so we can
    /// take a raw pointer to the storage from a shared `&` static without
    /// undefined behaviour.
    pub(super) struct RuntimeCell(UnsafeCell<MaybeUninit<RuntimeContext>>);
    // SAFETY: synchronization is done externally via `INIT_DONE` (only one
    // thread of control can take the `false → true` transition) and at
    // runtime by the §11.1 foreground/ISR ownership discipline. The
    // half-split (`FgState` / `IsrState`) projection through raw-pointer
    // helpers in each FFI entry preserves strict aliasing.
    unsafe impl Sync for RuntimeCell {}

    pub(super) static RT_CELL: RuntimeCell = RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));

    /// Single-shot init guard. `compare_exchange(false → true)` succeeds
    /// exactly once; subsequent calls observe `Err(true)` and return null.
    pub(super) static INIT_DONE: AtomicBool = AtomicBool::new(false);

    // C-side `kalico_clock_freq` constant — defined in src/runtime_tick.c
    // (or, on host builds, by the integration-test harness).
    //
    // NOTE: `RuntimeContext::init` re-imports this same symbol on the
    // runtime-crate side; the import here is kept so the existing
    // producer-protocol re-enable path can read the freq for
    // `min_segment_cycles` arithmetic.
    unsafe extern "C" {
        pub(super) static kalico_clock_freq: u32;
    }

    // C-side timer-control helpers — defined in src/stm32/kalico_h7_timer.c
    // on the MCU and stubbed by the integration-test harness on host.
    unsafe extern "C" {
        fn kalico_h7_enable_tim5();
        #[allow(dead_code)]
        fn kalico_h7_disable_tim5();
        fn kalico_h7_read_cyccnt() -> u32;
    }

    /// Init-once. Spec §3.2.
    ///
    /// Returns a valid handle on the first successful call; null on any
    /// subsequent call. The handle is the address of the static
    /// `RuntimeContext` storage; its lifetime is `'static`.
    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_init() -> *mut KalicoRuntime {
        // Atomic compare-exchange — exactly one caller takes the
        // false → true transition; everyone else observes Err and bails.
        if INIT_DONE
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return core::ptr::null_mut();
        }
        // SAFETY: we hold the INIT_DONE token; no other context has access
        // to RT_CELL until we publish a non-null handle. RuntimeContext::init
        // writes through raw-pointer projections and never forms
        // `&mut RuntimeContext`, matching the §11.2 aliasing discipline.
        unsafe {
            let rt_ptr: *mut RuntimeContext = (*RT_CELL.0.get()).as_mut_ptr();
            RuntimeContext::init(rt_ptr);
            rt_ptr.cast::<KalicoRuntime>()
        }
    }

    /// Push a segment. Producer protocol per spec §4.4.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_push_segment(
        rt: *mut KalicoRuntime,
        id: u32,
        curve_handle: u16,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` is the published RT_CELL pointer (verified non-null
        // and INIT_DONE==true above). We project to the foreground half-state
        // and the SharedState atomics via raw pointers; no `&mut
        // RuntimeContext` ever exists on this path. The §11.1 ownership
        // discipline (foreground sole writer of FgState) is enforced by
        // code review — no other FFI entry forms `&mut FgState`.
        unsafe {
            let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let isr_ptr_const: *const IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr)).cast_const();
            let fg: &mut FgState = &mut *fg_ptr;
            let shared: &SharedState = &*shared_ptr;
            push_segment_impl(
                fg,
                shared,
                isr_ptr_const,
                id,
                curve_handle,
                t_start,
                t_end,
                kinematics,
            )
        }
    }

    /// Foreground push body. Operates on the half-split borrows projected by
    /// the FFI shim above. The Step-5 producer protocol at re-enable still
    /// reads back from the ISR half (`widen_state`); per spec §4.7 the ISR
    /// is paused at this point so the foreground can mutate `widen_state`
    /// without contention.
    ///
    /// SAFETY (caller): `isr_ptr_const` must point at the same `RuntimeContext`'s
    /// `IsrState`, and the ISR must be disabled while the producer-protocol
    /// re-enable branch runs (Klipper's `kalico_h7_disable_tim5()` does
    /// this; callers via the C shim hold to that contract).
    #[allow(clippy::too_many_arguments)]
    unsafe fn push_segment_impl(
        fg: &mut FgState,
        shared: &SharedState,
        isr_ptr_const: *const IsrState,
        id: u32,
        curve_handle: u16,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
    ) -> i32 {
        // Fault-latched short-circuit (preserves Step-5 behaviour).
        if shared.last_error.load(Ordering::Acquire) != 0
            && shared.runtime_status.load(Ordering::Acquire) == RuntimeStatus::Fault as u8
        {
            return KALICO_ERR_FAULT_LATCHED;
        }
        if t_end <= t_start {
            return KALICO_ERR_INVALID_DURATION;
        }
        // SAFETY: C-side immutable constant set at static-init time.
        let min_seg_cycles = u64::from(runtime::clock::min_segment_cycles(unsafe {
            super::exports::kalico_clock_freq
        }));
        if t_end - t_start < min_seg_cycles {
            return KALICO_ERR_INVALID_DURATION;
        }
        let kin = match kinematics {
            0 => KinematicTag::CoreXyAndE,
            1 => KinematicTag::CartesianXyzAndE,
            _ => return KALICO_ERR_INVALID_KINEMATICS,
        };
        let seg = Segment {
            id,
            curve: CurveHandle(curve_handle),
            t_start,
            t_end,
            kinematics: kin,
        };
        if fg.queue_producer.enqueue(seg).is_err() {
            return KALICO_ERR_QUEUE_FULL;
        }
        // §4.4 producer-protocol: re-enable TIM5 if observed status was IDLE/DRAINED.
        let cur_status = shared.runtime_status.load(Ordering::Acquire);
        if cur_status == RuntimeStatus::Idle as u8 || cur_status == RuntimeStatus::Drained as u8 {
            // SAFETY: foreground-context access; spec §4.7 invariant — TIM5
            // was disabled by C-side caller before push, so `widen_state`
            // has no concurrent ISR writer. We materialize a `&mut
            // WidenState` here under that contract.
            //
            // Per Round-3 fix B-R3-4, `widen_state` lives on `IsrState`,
            // not Engine. The shim borrows it by projecting through the
            // ISR-state UnsafeCell *only* under the ISR-disabled
            // discipline.
            unsafe {
                let raw = super::exports::kalico_h7_read_cyccnt();
                let isr_ptr_mut = isr_ptr_const.cast_mut();
                let widen_state = &mut (*isr_ptr_mut).widen_state;
                // Reconstruct last-widened high-water mark from the ISR's
                // pre-disable state. `WidenState` exposes its fields
                // crate-private but not pub, so we approximate by reading
                // the seqlock-published widened-now from SharedState
                // (§11.4) — that's the most recent widened sample the ISR
                // produced before being disabled.
                let last_widened = runtime::clock::read_widened_now(shared);
                widen_state.reinit(raw, last_widened);
                super::exports::kalico_h7_enable_tim5();
            }
        }
        KALICO_OK
    }

    /// Load a curve into a slab slot. Producer-side validation rejects bad data.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_load_curve(
        rt: *mut KalicoRuntime,
        slot_idx: u16,
        control_points_flat: *const f32,
        n_cp: u16,
        knots: *const f32,
        n_knots: u16,
        weights: *const f32,
        n_weights: u16,
        degree: u8,
    ) -> i32 {
        if rt.is_null() || control_points_flat.is_null() || knots.is_null() || weights.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null and INIT_DONE=true above. CurvePool is at the
        // top level of RuntimeContext; foreground is the sole writer per
        // §10.5 (Phase 2 hardens the ISR-side concurrency further). We
        // form `&mut CurvePool` through `addr_of_mut!` without ever
        // forming `&mut RuntimeContext`.
        unsafe {
            let pool_ptr: *mut CurvePool = core::ptr::addr_of_mut!((*ctx).curve_pool);
            // SAFETY: caller must ensure each pointer is valid for `n_*`
            // reads of f32 and that the buffers do not alias the curve pool.
            let cps_slice = core::slice::from_raw_parts(
                control_points_flat,
                n_cp as usize * MAX_DIM,
            );
            let knots_slice = core::slice::from_raw_parts(knots, n_knots as usize);
            let weights_slice = core::slice::from_raw_parts(weights, n_weights as usize);
            match (*pool_ptr).load(
                CurveHandle(slot_idx),
                cps_slice,
                knots_slice,
                weights_slice,
                degree,
            ) {
                Ok(()) => KALICO_OK,
                Err(
                    runtime::curve_pool::CurvePoolError::OutOfBounds
                    | runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded,
                ) => KALICO_ERR_INVALID_HANDLE,
                Err(_) => KALICO_ERR_INVALID_CURVE,
            }
        }
    }

    /// ISR entrypoint. Spec §3.2 / §4.2.
    /// `raw_cyccnt` is the raw 32-bit DWT->CYCCNT value; Rust widens to u64.
    /// Skips null-check (caller is the C ISR shim with stable handle).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
        // Defensive Acquire-load — guards against early-fire during init.
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null per the C ISR shim's stable-handle contract;
        // INIT_DONE=true above. We project to the IsrState UnsafeCell, the
        // top-level CurvePool, and SharedState via raw pointers. The §11.1
        // discipline guarantees the TIM5 ISR is the SOLE writer of IsrState,
        // and the half-split structure means we never form
        // `&mut RuntimeContext`.
        //
        // Field-disjoint borrow: `let IsrState { engine, widen_state, … }
        // = &mut *isr` splits the single `&mut IsrState` into multiple
        // disjoint `&mut`s the borrow checker accepts because each field
        // projection is non-overlapping.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let pool_ptr: *const CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let isr: &mut IsrState = &mut *isr_ptr;
            let pool: &CurvePool = &*pool_ptr;
            let shared: &SharedState = &*shared_ptr;
            let IsrState {
                engine,
                widen_state,
                queue_consumer,
                trace_producer,
                ..
            } = isr;
            let _ = engine.tick(
                raw_cyccnt,
                widen_state,
                pool,
                queue_consumer,
                trace_producer,
                shared,
            );
            // Mirror the engine's status into SharedState so the
            // foreground-only entrypoints (push_segment, status,
            // last_error) can read it through atomics rather than
            // reaching into IsrState.
            shared
                .runtime_status
                .store(engine.status() as u8, Ordering::Release);
            shared
                .last_error
                .store(engine.last_error(), Ordering::Release);
        }
    }

    /// Foreground drain. Returns count of samples written.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_drain_trace(
        rt: *mut KalicoRuntime,
        out_buf: *mut TraceSample,
        out_cap: u32,
    ) -> u32 {
        if rt.is_null() || out_buf.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: project to the foreground trace consumer only — no &mut
        // on IsrState forms here. Caller-provided out_buf must be valid for
        // out_cap writes of TraceSample.
        unsafe {
            let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
            let fg: &mut FgState = &mut *fg_ptr;
            let out_slice = core::slice::from_raw_parts_mut(out_buf, out_cap as usize);
            let mut count = 0usize;
            while count < out_slice.len() {
                let Some(sample) = fg.trace_consumer.dequeue() else {
                    break;
                };
                if let Some(slot) = out_slice.get_mut(count) {
                    *slot = sample;
                }
                count += 1;
            }
            #[allow(clippy::cast_possible_truncation)]
            let result = count as u32;
            result
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_status(rt: *mut KalicoRuntime) -> u8 {
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
    pub unsafe extern "C" fn kalico_runtime_last_error(rt: *mut KalicoRuntime) -> i32 {
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
    pub unsafe extern "C" fn kalico_runtime_tick_counter(rt: *mut KalicoRuntime) -> u32 {
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

    /// Sim escape hatch: load a pre-baked NURBS fixture into a curve-pool slot.
    ///
    /// Per Step-6 plan Phase 0 Task 0.2 GDB-attach diagnosis: under Renode,
    /// the H7 platform model silently ignores `SCB->CPACR` writes from
    /// `SystemInit()`, leaving the FPU disabled. The regular
    /// `kalico_runtime_load_curve` path runs `is_finite()` / `> 0.0` checks
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
        rt: *mut KalicoRuntime,
        slot_idx: u16,
        fixture_id: u16,
    ) -> i32 {
        use runtime::sim_fixtures::{FIXTURE_CPS_MAX, FIXTURE_KNOTS_MAX, FIXTURE_WEIGHTS_MAX};
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: project to the top-level CurvePool only — no `&mut
        // RuntimeContext` forms on this path. The fixture path uses the
        // FPU-free `load_unchecked` to avoid Renode's CPACR-disabled
        // UsageFault on the regular load() path.
        unsafe {
            let pool_ptr: *mut CurvePool = core::ptr::addr_of_mut!((*ctx).curve_pool);
            let mut cps = [0.0_f32; FIXTURE_CPS_MAX];
            let mut knots = [0.0_f32; FIXTURE_KNOTS_MAX];
            let mut weights = [0.0_f32; FIXTURE_WEIGHTS_MAX];
            let Some((degree, n_cp, n_knots, n_weights)) = runtime::sim_fixtures::lookup(
                fixture_id,
                &mut cps,
                &mut knots,
                &mut weights,
            ) else {
                return KALICO_ERR_INVALID_CURVE;
            };
            match (*pool_ptr).load_unchecked(
                CurveHandle(slot_idx),
                &cps[..n_cp * MAX_DIM],
                &knots[..n_knots],
                &weights[..n_weights],
                degree,
            ) {
                Ok(()) => KALICO_OK,
                Err(
                    runtime::curve_pool::CurvePoolError::OutOfBounds
                    | runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded,
                ) => KALICO_ERR_INVALID_HANDLE,
                Err(_) => KALICO_ERR_INVALID_CURVE,
            }
        }
    }
}
