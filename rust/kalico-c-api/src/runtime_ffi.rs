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

    use runtime::curve_pool::{CURVE_POOL_N, CurveHandle, CurvePool, MAX_DIM};
    use runtime::engine::RuntimeStatus;
    use runtime::error::{
        KALICO_ERR_FAULT_LATCHED, KALICO_ERR_INVALID_CURVE, KALICO_ERR_INVALID_DURATION,
        KALICO_ERR_INVALID_HANDLE, KALICO_ERR_INVALID_KINEMATICS, KALICO_ERR_NOT_INIT,
        KALICO_ERR_NULL_PTR, KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED, KALICO_ERR_QUEUE_FULL,
        KALICO_ERR_SEGMENT_ID_NON_MONOTONIC, KALICO_OK,
    };
    use runtime::segment::{KinematicTag, Segment};
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

    /// Push a segment. Producer protocol per spec §4.4 + §10.1.
    ///
    /// `curve_handle_packed` is the wire-encoded handle: `(generation << 16) |
    /// slot_idx`. Step-6 §10.1 widening over Step-5's bare `u16`.
    /// `out_accepted_segment_id` and `out_credit_epoch` may be NULL (host
    /// callers that don't need them); when present they receive the values
    /// published into `SharedState` on success — host caller sees the same
    /// values via the `kalico_push_response` schema (§5.3).
    #[unsafe(no_mangle)]
    #[allow(clippy::too_many_arguments)]
    pub unsafe extern "C" fn kalico_runtime_push_segment(
        rt: *mut KalicoRuntime,
        id: u32,
        curve_handle_packed: u32,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
        out_accepted_segment_id: *mut u32,
        out_credit_epoch: *mut u32,
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
                CurveHandle::unpack(curve_handle_packed),
                t_start,
                t_end,
                kinematics,
                out_accepted_segment_id,
                out_credit_epoch,
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
        curve_handle: CurveHandle,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
        out_accepted_segment_id: *mut u32,
        out_credit_epoch: *mut u32,
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
        // Round-2 B11-real / Round-3 B-R3-8 — strict monotonicity gated by
        // the `accepted_segment_id_seen` flag so the initial-state-no-prior-
        // push case does not collide with id=0. The flag is reset on flush /
        // new stream_open (Phase 7 will wire those resets).
        let prev_seen = shared.accepted_segment_id_seen.load(Ordering::Acquire);
        let prev_accepted = shared.accepted_segment_id.load(Ordering::Acquire);
        if prev_seen && id <= prev_accepted {
            return KALICO_ERR_SEGMENT_ID_NON_MONOTONIC;
        }
        let seg = Segment {
            id,
            curve_handle,
            t_start,
            t_end,
            kinematics: kin,
            flags: 0,
            _pad: [0; 2],
        };
        if fg.queue_producer.enqueue(seg).is_err() {
            return KALICO_ERR_QUEUE_FULL;
        }
        // Round-2 B6: on the FIRST push of a fresh stream (Opening or
        // StreamOpenPriming with no recorded first segment yet), capture
        // the priming segment's t_start in FgState so the §6.3 arm()
        // predicate can validate it without peeking the ISR-owned queue.
        // Also auto-transition StreamOpening → StreamOpenPriming on first
        // push so the state machine reflects priming-buffer accumulation.
        match fg.stream_state_machine {
            runtime::stream::FgStreamState::StreamOpening => {
                fg.stream_state_machine =
                    runtime::stream::FgStreamState::StreamOpenPriming;
                if fg.first_priming_segment_t_start.is_none() {
                    fg.first_priming_segment_t_start = Some(t_start);
                }
            }
            runtime::stream::FgStreamState::StreamOpenPriming => {
                if fg.first_priming_segment_t_start.is_none() {
                    fg.first_priming_segment_t_start = Some(t_start);
                }
            }
            runtime::stream::FgStreamState::Armed => {
                // Round-3 B-R3-9 implicit transition: once a push lands
                // after arm(), the stream is in steady-state motion.
                // Foreground state machine reflects that.
                fg.stream_state_machine = runtime::stream::FgStreamState::Running;
            }
            _ => {}
        }
        // Round-2 B14: foreground publishes the cumulative-accepted cursor
        // for both the periodic kalico_status frame and Gate-B observers.
        // Release pairs with foreground/host readers' Acquire on the same
        // atomics.
        shared
            .accepted_segment_id
            .store(id, Ordering::Release);
        shared
            .accepted_segment_id_seen
            .store(true, Ordering::Release);
        // Optional out-params for the host-side response schema (Phase 3.3).
        if !out_accepted_segment_id.is_null() {
            // SAFETY: caller-provided pointer is documented to be a valid
            // u32 location for writes when non-null.
            unsafe {
                *out_accepted_segment_id = id;
            }
        }
        if !out_credit_epoch.is_null() {
            let credit_epoch = shared.credit_epoch.load(Ordering::Acquire);
            // SAFETY: caller-provided pointer is documented to be a valid
            // u32 location for writes when non-null.
            unsafe {
                *out_credit_epoch = credit_epoch;
            }
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

    /// Load a curve into a slab slot. Producer-side validation rejects bad
    /// data. Returns the freshly issued `(slot, gen)` packed handle via
    /// `out_handle_packed` on success (Round-5 Codex #4 — host can't reference
    /// a curve it just loaded otherwise).
    ///
    /// `control_points_flat` / `knots` / `weights` are the legacy Step-5
    /// flat-pointer triple. The wire-format change to a single 1-byte-
    /// versioned blob (§4.2) lands in `kalico_runtime_load_curve_v1` at the
    /// C-side handler in `runtime_tick.c`; this FFI is the existing surface
    /// that the C handler unpacks into for the call across the FFI boundary.
    #[unsafe(no_mangle)]
    #[allow(clippy::too_many_arguments)]
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
        out_handle_packed: *mut u32,
    ) -> i32 {
        if rt.is_null() || control_points_flat.is_null() || knots.is_null() || weights.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null and INIT_DONE=true above. CurvePool is at the
        // top level of RuntimeContext; per-slot atomics in `PoolSlot`
        // bridge the foreground-writer / ISR-reader split (§10.2 + Round-1
        // Codex #4).
        unsafe {
            let pool_ptr: *const CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
            // SAFETY: caller must ensure each pointer is valid for `n_*`
            // reads of f32 and that the buffers do not alias the curve pool.
            let cps_slice = core::slice::from_raw_parts(
                control_points_flat,
                n_cp as usize * MAX_DIM,
            );
            let knots_slice = core::slice::from_raw_parts(knots, n_knots as usize);
            let weights_slice = core::slice::from_raw_parts(weights, n_weights as usize);
            match (*pool_ptr).validate_and_load(
                slot_idx,
                cps_slice,
                knots_slice,
                weights_slice,
                degree,
            ) {
                Ok(handle) => {
                    if !out_handle_packed.is_null() {
                        *out_handle_packed = handle.pack();
                    }
                    KALICO_OK
                }
                Err(
                    runtime::curve_pool::CurvePoolError::OutOfBounds
                    | runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded,
                ) => KALICO_ERR_INVALID_HANDLE,
                Err(_) => KALICO_ERR_INVALID_CURVE,
            }
        }
    }

    /// Validate a versioned blob payload's leading version byte (§4.2).
    /// Foreground entrypoint for the C handler that reads payload bytes off
    /// the wire and routes the post-version-byte slice into the Step-5
    /// flat-pointer load path. Returns `KALICO_OK` on a recognised version
    /// or `KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED` otherwise.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_check_blob_version(
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

    /// Diagnostic: per-slot generation snapshot (spec §10.4 + Round-1 B9).
    /// Used after a fault for host-side recovery decisions. Writes the
    /// per-slot `current_gen` and `last_retired_gen` into the out-params.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_query_pool_state(
        rt: *mut KalicoRuntime,
        slot_idx: u16,
        out_current_gen: *mut u16,
        out_last_retired_gen: *mut u16,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        if (slot_idx as usize) >= CURVE_POOL_N {
            return KALICO_ERR_INVALID_HANDLE;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only access through atomics; no `&mut` forms.
        unsafe {
            let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
            let Some(slot) = pool.slots.get(slot_idx as usize) else {
                return KALICO_ERR_INVALID_HANDLE;
            };
            if !out_current_gen.is_null() {
                *out_current_gen = slot.current_gen.load(Ordering::Acquire);
            }
            if !out_last_retired_gen.is_null() {
                *out_last_retired_gen = slot.last_retired_gen.load(Ordering::Acquire);
            }
        }
        KALICO_OK
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

    // ---- Stream lifecycle + clock-sync FFI (spec §8.3 / §12.1) ------------
    //
    // Phase 3.2 declares the FFI shape; Phase 6 wires the actual state-
    // machine bodies (`runtime::stream::open` / `arm` / `terminal` / `flush`
    // / `clock_sync_respond`). Until Phase 6 lands, the shims return
    // `KALICO_ERR_STREAM_STATE_VIOLATION` (-140) so the host sees a
    // recognisable "not-yet-implemented" code rather than silently passing.

    /// Project to `&mut FgState` + `&SharedState`. Used by the stream-
    /// lifecycle FFI shims below. Caller must guarantee `rt` non-null and
    /// `INIT_DONE=true`.
    ///
    /// SAFETY: same contract as `kalico_runtime_push_segment`'s projection.
    /// Only one `&mut FgState` may be live at a time across the FFI surface;
    /// the foreground task is single-threaded so this is enforced by call-
    /// site discipline, not the type system.
    unsafe fn project_fg<R, F>(rt: *mut KalicoRuntime, f: F) -> R
    where
        F: FnOnce(&mut FgState, &SharedState) -> R,
    {
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            f(&mut *fg_ptr, &*shared_ptr)
        }
    }

    /// `kalico_stream_open` — assert host-MCU stream identity (§8.3).
    /// Phase-6 stub.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_stream_open(
        rt: *mut KalicoRuntime,
        stream_id: u32,
        out_credit_epoch: *mut u32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: half-split projection per the discipline contract.
        unsafe {
            project_fg(rt, |fg, shared| {
                let r = runtime::stream::open(fg, shared, stream_id);
                if r == KALICO_OK && !out_credit_epoch.is_null() {
                    *out_credit_epoch = shared.credit_epoch.load(Ordering::Acquire);
                }
                r
            })
        }
    }

    /// `kalico_stream_arm` — commit the priming buffer (§6.4 / §8.3).
    /// Phase-6 stub.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_stream_arm(
        rt: *mut KalicoRuntime,
        t_start_t0: u64,
        arm_lead_cycles: u32,
        out_armed_t_start: *mut u64,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: half-split projection per the discipline contract.
        unsafe {
            project_fg(rt, |fg, shared| {
                let (r, armed_t) =
                    runtime::stream::arm(fg, shared, t_start_t0, arm_lead_cycles);
                if !out_armed_t_start.is_null() {
                    *out_armed_t_start = armed_t;
                }
                r
            })
        }
    }

    /// `kalico_stream_terminal` — mark the last segment id of the stream
    /// (§8.3). Phase-6 stub.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_stream_terminal(
        rt: *mut KalicoRuntime,
        segment_id: u32,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // SAFETY: half-split projection per the discipline contract.
        unsafe { project_fg(rt, |fg, shared| runtime::stream::terminal(fg, shared, segment_id)) }
    }

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
    /// Phase-6 stub. Out-param receives the MCU local-clock value sampled
    /// inside the FFI on success.
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
        // SAFETY: half-split projection per the discipline contract.
        unsafe {
            project_fg(rt, |fg, shared| {
                let (r, mcu_clock) = runtime::stream::clock_sync_respond(
                    fg,
                    shared,
                    request_id,
                    host_send_time_lo,
                    host_send_time_hi,
                );
                if !out_mcu_clock.is_null() {
                    *out_mcu_clock = mcu_clock;
                }
                r
            })
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
        out_handle_packed: *mut u32,
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
            let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
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
            match pool.load_unchecked(
                slot_idx,
                &cps[..n_cp * MAX_DIM],
                &knots[..n_knots],
                &weights[..n_weights],
                degree,
            ) {
                Ok(handle) => {
                    if !out_handle_packed.is_null() {
                        *out_handle_packed = handle.pack();
                    }
                    KALICO_OK
                }
                Err(
                    runtime::curve_pool::CurvePoolError::OutOfBounds
                    | runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded,
                ) => KALICO_ERR_INVALID_HANDLE,
                Err(_) => KALICO_ERR_INVALID_CURVE,
            }
        }
    }
}
