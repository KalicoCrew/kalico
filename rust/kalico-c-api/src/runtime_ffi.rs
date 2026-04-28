//! Kalico runtime C-FFI surface. Spec §3.2 / §4.4 / §5.2 / §5.6.
//!
//! cfg-gated by `header-runtime`. Exposes the opaque `*mut KalicoRuntime`
//! handle plus the eight `kalico_runtime_*` entrypoints used by the Klipper
//! C ISR shim and the foreground producer task.
//!
//! ## Latent UB acknowledged at Step 5
//!
//! Each entrypoint converts `*mut KalicoRuntime` to `&mut RuntimeContext`,
//! which produces overlapping `&mut`s under Rust's strict aliasing model when
//! the ISR preempts foreground. Spec §6.8 / plan Task 13 explicitly accept
//! this as deferred to Step 6 hardening (proper SPSC half-split or
//! interrupt-free sections). The shared-state subfields (`SegmentQueue`,
//! `TraceRing`) use `heapless::spsc::Queue` which is atomic-correct on
//! ARMv7-M; atomic fields on `Engine` use interior mutability via `&self`.

#![allow(unsafe_code)]

#[cfg(feature = "header-runtime")]
pub mod exports {
    use core::cell::UnsafeCell;
    use core::mem::MaybeUninit;
    use core::sync::atomic::{AtomicU8, Ordering};

    use runtime::curve_pool::{CurvePool, MAX_DIM};
    use runtime::engine::{Engine, RuntimeStatus};
    use runtime::error::{
        KALICO_ERR_FAULT_LATCHED, KALICO_ERR_INVALID_CURVE, KALICO_ERR_INVALID_DURATION,
        KALICO_ERR_INVALID_HANDLE, KALICO_ERR_INVALID_KINEMATICS, KALICO_ERR_NOT_INIT,
        KALICO_ERR_NULL_PTR, KALICO_ERR_QUEUE_FULL, KALICO_OK,
    };
    use runtime::queue::SegmentQueue;
    use runtime::segment::{CurveHandle, KinematicTag, Segment};
    use runtime::slot::{NoopIs, NoopPa};
    use runtime::trace::{TraceRing, TraceSample};

    // Compile-time choice of slot impls. Spec §3.1.
    //
    // Step 5 hardcodes the Noop ZSTs. Step 8 (`input-shaper`) and Step 9
    // (`pa-tanh`) will introduce real impls and add cfg-feature arms here
    // that swap these aliases — until those features are declared in
    // `kalico-c-api/Cargo.toml`, the alias is unconditional.
    type Pa = NoopPa;
    type Is = NoopIs;

    /// The opaque type C sees — never dereferenced on the C side.
    /// Matches spec §3.2 / §5.6 handle discipline.
    #[allow(missing_debug_implementations)] // opaque to C; never inspected
    #[repr(C)]
    pub struct KalicoRuntime {
        _private: [u8; 0],
    }

    /// Concrete singleton storage. Spec §3.2 init-once protocol.
    pub(super) struct RuntimeCell(UnsafeCell<MaybeUninit<RuntimeContext>>);
    // SAFETY: synchronization is done externally via `INIT_STATE` (only one
    // thread of control transitions UNINIT → INITING → READY) and at runtime
    // by the §4.7 foreground/ISR protocol. The latent-UB acknowledgement at
    // module-doc top covers concurrent &mut aliasing.
    unsafe impl Sync for RuntimeCell {}

    pub(super) struct RuntimeContext {
        pub(super) engine: Engine<Pa, Is>,
        pub(super) queue: SegmentQueue,
        pub(super) pool: CurvePool,
        pub(super) trace: TraceRing<128>,
    }

    pub(super) static RT_CELL: RuntimeCell = RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));

    pub(super) const INIT_UNINIT: u8 = 0;
    pub(super) const INIT_INITING: u8 = 1;
    pub(super) const INIT_READY: u8 = 2;

    pub(super) static INIT_STATE: AtomicU8 = AtomicU8::new(INIT_UNINIT);

    // C-side `kalico_clock_freq` constant — defined in src/runtime_tick.c
    // (or, on host builds, by the integration-test harness).
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
    /// Returns valid handle on first successful call; null otherwise.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_init() -> *mut KalicoRuntime {
        match INIT_STATE.compare_exchange(
            INIT_UNINIT,
            INIT_INITING,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => {
                // SAFETY: C-side immutable constant set at static-init time in src/runtime_tick.c.
                let clock_freq = unsafe { kalico_clock_freq };
                // SAFETY: we hold the INIT_INITING token; no other context
                // has access to RT_CELL until we publish READY.
                unsafe {
                    (*RT_CELL.0.get()).write(RuntimeContext {
                        engine: Engine::<Pa, Is>::new(clock_freq),
                        queue: SegmentQueue::new(),
                        pool: CurvePool::new(),
                        trace: TraceRing::<128>::new(),
                    });
                }
                INIT_STATE.store(INIT_READY, Ordering::Release);
                RT_CELL.0.get().cast::<KalicoRuntime>()
            }
            Err(_) => core::ptr::null_mut(), // Already INITING or READY
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
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &mut *rt.cast::<RuntimeContext>() };
        if ctx.engine.status() == RuntimeStatus::Fault {
            return KALICO_ERR_FAULT_LATCHED;
        }
        if t_end <= t_start {
            return KALICO_ERR_INVALID_DURATION;
        }
        // MIN_SEGMENT_CYCLES check.
        // SAFETY: C-side immutable constant set at static-init time in src/runtime_tick.c.
        let min_seg_cycles = u64::from(runtime::clock::min_segment_cycles(unsafe {
            kalico_clock_freq
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
        if ctx.queue.try_push(seg).is_err() {
            return KALICO_ERR_QUEUE_FULL;
        }
        // §4.4 producer-protocol: re-enable TIM5 if observed status was IDLE/DRAINED.
        match ctx.engine.status() {
            RuntimeStatus::Idle | RuntimeStatus::Drained => {
                // Reinit CYCCNT widening before re-enabling TIM5. ISR was
                // disabled in `kalico_h7_disable_tim5()` and is still off
                // here — single-thread access to widen_state is safe per
                // spec §4.7.
                // SAFETY: foreground-context access; spec §4.7 invariant — TIM5 was
                // disabled by C-side caller before push, so widen_state has no
                // concurrent ISR writer.
                let raw = unsafe { kalico_h7_read_cyccnt() };
                ctx.engine.reinit_widen(raw);
                unsafe {
                    kalico_h7_enable_tim5();
                }
            }
            _ => {}
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
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &mut *rt.cast::<RuntimeContext>() };
        // SAFETY: caller must ensure each pointer is valid for `n_*` reads of f32
        // and that the buffers do not alias the curve pool. n_cp * MAX_DIM bounds
        // the cps buffer per the producer protocol.
        let cps_slice =
            unsafe { core::slice::from_raw_parts(control_points_flat, n_cp as usize * MAX_DIM) };
        let knots_slice = unsafe { core::slice::from_raw_parts(knots, n_knots as usize) };
        let weights_slice = unsafe { core::slice::from_raw_parts(weights, n_weights as usize) };
        match ctx.pool.load(
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

    /// ISR entrypoint. Spec §3.2 / §4.2.
    /// `raw_cyccnt` is the raw 32-bit DWT->CYCCNT value; Rust widens to u64.
    /// Skips null-check (caller is the C ISR shim with stable handle).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
        // Defensive Acquire-load — guards against early-fire during INITING.
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &mut *rt.cast::<RuntimeContext>() };
        let now = ctx.engine.widen(raw_cyccnt);
        let _ = ctx
            .engine
            .tick(now, &mut ctx.queue, &ctx.pool, &mut ctx.trace);
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
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return 0;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &mut *rt.cast::<RuntimeContext>() };
        // SAFETY: caller must ensure out_buf is valid for out_cap writes of TraceSample,
        // properly aligned, and not aliased.
        let out_slice = unsafe { core::slice::from_raw_parts_mut(out_buf, out_cap as usize) };
        ctx.trace.drain_into(out_slice) as u32
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_status(rt: *mut KalicoRuntime) -> u8 {
        if rt.is_null() {
            return RuntimeStatus::Fault as u8;
        }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return RuntimeStatus::Fault as u8;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // shared `&` ref form; ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &*rt.cast::<RuntimeContext>() };
        ctx.engine.status() as u8
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_error(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // shared `&` ref form; ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &*rt.cast::<RuntimeContext>() };
        ctx.engine.last_error()
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick_counter(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if INIT_STATE.load(Ordering::Acquire) != INIT_READY {
            return 0;
        }
        // SAFETY: rt is the published RT_CELL pointer (verified non-null and INIT_STATE==READY above);
        // shared `&` ref form; ISR/foreground latent-mut-aliasing acknowledged in module doc per spec §3.2.
        let ctx = unsafe { &*rt.cast::<RuntimeContext>() };
        ctx.engine.tick_counter()
    }
}
