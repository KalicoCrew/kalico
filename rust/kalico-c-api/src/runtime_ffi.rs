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

    use runtime::curve_pool::{CURVE_POOL_N, CurveHandle, CurvePool};
    use runtime::engine::RuntimeStatus;
    use runtime::error::{
        KALICO_ERR_CAPABILITY_MISSING, KALICO_ERR_FAULT_LATCHED, KALICO_ERR_INVALID_ARG,
        KALICO_ERR_INVALID_CURVE, KALICO_ERR_INVALID_DURATION, KALICO_ERR_INVALID_HANDLE,
        KALICO_ERR_INVALID_KINEMATICS, KALICO_ERR_NOT_INIT, KALICO_ERR_NULL_PTR,
        KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED, KALICO_ERR_QUEUE_FULL,
        KALICO_ERR_SEGMENT_ID_NON_MONOTONIC, KALICO_ERR_ZERO_DURATION_SEGMENT, KALICO_OK,
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

    // Placed in AXI SRAM on H7 via the linker script (.axi_bss section).
    // The 282 KB RuntimeContext doesn't fit in the H723's 128 KB DTCM
    // region, but the H7 has 320 KB of AXI SRAM at 0x24000000 that is
    // unused by the rest of Klipper. Other targets (host / linux / non-H7
    // MCUs) ignore the section name and the static lands in regular .bss.
    #[cfg_attr(feature = "axi-bss-placement", unsafe(link_section = ".axi_bss"))]
    pub(super) static RT_CELL: RuntimeCell = RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));

    /// Single-shot init guard. `compare_exchange(false → true)` succeeds
    /// exactly once; subsequent calls observe `Err(true)` and return null.
    pub(super) static INIT_DONE: AtomicBool = AtomicBool::new(false);

    // C-side `runtime_clock_freq` constant — defined in src/runtime_tick.c
    // (or, on host builds, by the integration-test harness).
    //
    // NOTE: `RuntimeContext::init` re-imports this same symbol on the
    // runtime-crate side; the import here is kept so the existing
    // producer-protocol re-enable path can read the freq for
    // `min_segment_cycles` arithmetic.
    unsafe extern "C" {
        pub(super) static runtime_clock_freq: u32;
    }

    // C-side timer-control helpers — defined in src/stm32/runtime_tick_h7.c
    // on the MCU and stubbed by the integration-test harness on host.
    unsafe extern "C" {
        fn runtime_tick_enable();
        fn runtime_tick_disable();
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
        // SAFETY: single-threaded init; no other context can observe RT_CELL
        // until INIT_DONE is published below. RuntimeContext::init writes
        // through raw-pointer projections and never forms `&mut
        // RuntimeContext`, matching the §11.2 aliasing discipline.
        unsafe {
            let rt_ptr: *mut RuntimeContext = (*RT_CELL.0.get()).as_mut_ptr();
            RuntimeContext::init(rt_ptr);
            // Publish after full init — ISR sees either INIT_DONE=false
            // (before enable) or a fully-initialised context (after).
            // Release ordering pairs with the Acquire loads in every FFI call.
            INIT_DONE.store(true, Ordering::Release);
            rt_ptr.cast::<KalicoRuntime>()
        }
    }

    /// Push a segment. Producer protocol per spec §4.4 + §10.1.
    ///
    /// Step 7-B: four per-axis curve handles (x, y, z, e) replace the single
    /// `curve_handle_packed`. Each is a wire-encoded `(generation << 16) |
    /// slot_idx`. `e_mode` selects the extruder evaluation strategy (0 =
    /// CoupledToXy, 1 = Independent, 2 = Travel). `extrusion_ratio_bits` is
    /// `f32::to_bits()` of the extrusion_per_xy_mm scalar for CoupledToXy mode.
    ///
    /// `out_accepted_segment_id` and `out_credit_epoch` may be NULL (host
    /// callers that don't need them); when present they receive the values
    /// published into `SharedState` on success — host caller sees the same
    /// values via the `kalico_push_response` schema (§5.3).
    #[unsafe(no_mangle)]
    #[allow(clippy::too_many_arguments)]
    pub unsafe extern "C" fn runtime_handle_push_segment(
        rt: *mut KalicoRuntime,
        id: u32,
        x_handle_packed: u32,
        y_handle_packed: u32,
        z_handle_packed: u32,
        e_handle_packed: u32,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
        e_mode: u8,
        extrusion_ratio_bits: u32,
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
            let result = push_segment_impl(
                fg,
                shared,
                isr_ptr_const,
                id,
                CurveHandle::unpack(x_handle_packed),
                CurveHandle::unpack(y_handle_packed),
                CurveHandle::unpack(z_handle_packed),
                CurveHandle::unpack(e_handle_packed),
                t_start,
                t_end,
                kinematics,
                e_mode,
                extrusion_ratio_bits,
                out_accepted_segment_id,
                out_credit_epoch,
            );
            shared
                .last_push_segment_result
                .store(result, Ordering::Release);
            result
        }
    }

    /// Foreground push body. Operates on the half-split borrows projected by
    /// the FFI shim above. The Step-5 producer protocol at re-enable still
    /// reads back from the ISR half (`widen_state`); per spec §4.7 the ISR
    /// is paused at this point so the foreground can mutate `widen_state`
    /// without contention.
    ///
    /// Step 7-B: accepts 4 per-axis curve handles + e_mode + extrusion_ratio.
    /// Registers all 4 handles in the retirement table at push time so the
    /// trace-drain pipeline can retire them on `SEGMENT_END`.
    ///
    /// SAFETY (caller): `isr_ptr_const` must point at the same `RuntimeContext`'s
    /// `IsrState`, and the ISR must be disabled while the producer-protocol
    /// re-enable branch runs (Klipper's `runtime_tick_disable()` does
    /// this; callers via the C shim hold to that contract).
    #[allow(clippy::too_many_arguments)]
    unsafe fn push_segment_impl(
        fg: &mut FgState,
        shared: &SharedState,
        isr_ptr_const: *const IsrState,
        id: u32,
        x_handle: CurveHandle,
        y_handle: CurveHandle,
        z_handle: CurveHandle,
        e_handle: CurveHandle,
        t_start: u64,
        t_end: u64,
        kinematics: u8,
        e_mode_raw: u8,
        extrusion_ratio_bits: u32,
        out_accepted_segment_id: *mut u32,
        out_credit_epoch: *mut u32,
    ) -> i32 {
        use runtime::config::EMode;
        // Fault-latched short-circuit (preserves Step-5 behaviour).
        if shared.last_error.load(Ordering::Acquire) != 0
            && shared.runtime_status.load(Ordering::Acquire) == RuntimeStatus::Fault as u8
        {
            return KALICO_ERR_FAULT_LATCHED;
        }
        if t_end == t_start {
            return KALICO_ERR_ZERO_DURATION_SEGMENT;
        }
        if t_end < t_start {
            return KALICO_ERR_INVALID_DURATION;
        }
        // SAFETY: C-side immutable constant set at static-init time.
        let min_seg_cycles = u64::from(runtime::clock::min_segment_cycles(unsafe {
            super::exports::runtime_clock_freq
        }));
        if t_end - t_start < min_seg_cycles {
            return KALICO_ERR_INVALID_DURATION;
        }
        let kin = match kinematics {
            0 => KinematicTag::CoreXyAndE,
            1 => KinematicTag::CartesianXyzAndE,
            _ => return KALICO_ERR_INVALID_KINEMATICS,
        };
        let e_mode = match e_mode_raw {
            0 => EMode::CoupledToXy,
            1 => EMode::Independent,
            2 => EMode::Travel,
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
        // §3.8 consumer mask. Computed here at construction because the
        // host-side `Engine::push_segment` Rust API that also computes this
        // mask has no callers in the production path — the FFI bypasses it
        // and enqueues directly. Leaving the mask at 0 makes
        // `seg.consumers_done()` return true on the first `producer_step`
        // call AFTER motor 0 fills its first batch, retiring the segment
        // before motor 0 has finished its real work and silently losing
        // every step past PRODUCER_BATCH_CAP. Reproduces as
        // "audible clicks but no toolhead motion" / "G1 X1 emits 32 of 80
        // expected pulses" on the bench. Pin the mask now so retirement
        // gates on per-motor `SegmentExhausted` reports.
        let consumers_remaining = Segment::compute_consumers_remaining(
            kin, x_handle, y_handle, z_handle, e_handle,
        );
        // 2026-05-15 live diagnosis: capture handle packings and the
        // computed consumers_remaining so the host can read them back via
        // FFI/diag tags. If `consumers_remaining == 0`, every handle was
        // UNUSED — the bridge sent a no-op segment to the MCU.
        shared.last_push_x_handle_packed.store(x_handle.pack(), Ordering::Release);
        shared.last_push_y_handle_packed.store(y_handle.pack(), Ordering::Release);
        shared
            .last_push_consumers_remaining
            .store(consumers_remaining as u32, Ordering::Release);
        if consumers_remaining == 0 {
            shared.push_segment_all_unused_total.fetch_add(1, Ordering::AcqRel);
        }
        let seg = Segment {
            id,
            x_handle,
            y_handle,
            z_handle,
            e_handle,
            t_start,
            t_end,
            kinematics: kin,
            e_mode,
            extrusion_ratio: f32::from_bits(extrusion_ratio_bits),
            flags: 0,
            _pad: [0; 1],
            consumers_remaining,
        };
        if fg.queue_producer.enqueue(seg).is_err() {
            return KALICO_ERR_QUEUE_FULL;
        }
        shared
            .producer_enqueue_success_total
            .fetch_add(1, Ordering::AcqRel);
        // Register all 4 per-axis handles in the retirement table so the
        // trace-drain pipeline can retire them on SEGMENT_END.
        fg.retirement_table.register(id, [x_handle, y_handle, z_handle, e_handle]);
        // Round-2 B6: on the FIRST push of a fresh stream (Opening or
        // StreamOpenPriming with no recorded first segment yet), capture
        // the priming segment's t_start in FgState so the §6.3 arm()
        // predicate can validate it without peeking the ISR-owned queue.
        // Also auto-transition StreamOpening → StreamOpenPriming on first
        // push so the state machine reflects priming-buffer accumulation.
        match fg.stream_state_machine {
            runtime::stream::FgStreamState::StreamOpening => {
                fg.stream_state_machine = runtime::stream::FgStreamState::StreamOpenPriming;
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
        shared.accepted_segment_id.store(id, Ordering::Release);
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
                let raw = super::exports::runtime_cyccnt_read();
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
                super::exports::runtime_tick_enable();
            }
        }
        KALICO_OK
    }

    /// Load a scalar curve into a slab slot. Producer-side validation rejects
    /// bad data. Returns the freshly issued `(slot, gen)` packed handle via
    /// `out_handle_packed` on success.
    ///
    /// Step 7-B: accepts scalar control points (1D). No weights (polynomial-only).
    #[unsafe(no_mangle)]
    #[allow(clippy::too_many_arguments)]
    pub unsafe extern "C" fn runtime_handle_load_curve(
        rt: *mut KalicoRuntime,
        slot_idx: u16,
        control_points_flat: *const f32,
        n_cp: u16,
        knots: *const f32,
        n_knots: u16,
        degree: u8,
        out_handle_packed: *mut u32,
    ) -> i32 {
        if rt.is_null() || control_points_flat.is_null() || knots.is_null() {
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
            let cps_slice =
                core::slice::from_raw_parts(control_points_flat, n_cp as usize);
            let knots_slice = core::slice::from_raw_parts(knots, n_knots as usize);
            match (*pool_ptr).validate_and_load(
                slot_idx,
                degree,
                knots_slice,
                cps_slice,
            ) {
                Ok(handle) => {
                    if !out_handle_packed.is_null() {
                        *out_handle_packed = handle.pack();
                    }
                    KALICO_OK
                }
                Err(
                    err @ (runtime::curve_pool::CurvePoolError::OutOfBounds
                    | runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded),
                ) => {
                    // Diagnostic 2026-05-12: encode the slot's cur/last gens
                    // into `out_handle_packed` so the host can decode the
                    // rejection state. Layout: u32 = (kind << 30) |
                    // (current_gen << 16) | last_retired_gen. kind = 0 for
                    // OutOfBounds (gens left zero), kind = 1 for SlotAlreadyLoaded.
                    if !out_handle_packed.is_null() {
                        let pool: &CurvePool = &*pool_ptr;
                        let (kind_bits, cur, last) = match err {
                            runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded => {
                                if let Some(slot) = pool.slots.get(slot_idx as usize) {
                                    (
                                        1u32,
                                        slot.current_gen.load(Ordering::Acquire),
                                        slot.last_retired_gen.load(Ordering::Acquire),
                                    )
                                } else {
                                    (1u32, 0u16, 0u16)
                                }
                            }
                            _ => (0u32, 0u16, 0u16),
                        };
                        *out_handle_packed = (kind_bits << 30)
                            | ((cur as u32) << 16)
                            | (last as u32);
                    }
                    KALICO_ERR_INVALID_HANDLE
                }
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

    /// Diagnostic: per-slot generation snapshot (spec §10.4 + Round-1 B9).
    /// Used after a fault for host-side recovery decisions. Writes the
    /// per-slot `current_gen` and `last_retired_gen` into the out-params.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_query_pool_state(
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
    pub unsafe extern "C" fn runtime_handle_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
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

    /// TIM5 ISR callback for the Modulated (polled-tick StepAccumulator)
    /// path (spec §3.2, T10).
    ///
    /// Computes the widened MCU clock inline from
    /// `timer_read_time()` + `stats_send_time_high` (same widening rule as
    /// `runtime_handle_widened_now`), then dispatches into
    /// `Engine::runtime_modulated_tick`.
    ///
    /// Called from `TIM5_IRQHandler` in `src/stm32/runtime_tick_{h7,f4}.c`,
    /// which is itself only enabled when `count_modulated_steppers > 0`
    /// (see `runtime_tick_enable`). For the all-StepTime MVP this entry
    /// is never invoked because TIM5 stays disabled.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_modulated_tick(rt: *mut KalicoRuntime) {
        if rt.is_null() {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null and INIT_DONE=true. The TIM5 ISR is the
        // SOLE writer of `IsrState`, so the disjoint half-split borrow
        // discipline (engine via IsrState, curve pool + shared state via
        // shared `&`s) holds for this entry point exactly as it does for
        // `runtime_handle_tick`.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let pool_ptr: *const CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let isr: &mut IsrState = &mut *isr_ptr;
            let pool: &CurvePool = &*pool_ptr;
            let shared: &SharedState = &*shared_ptr;

            // Compute widened `now` via the C-side helper. The helper
            // (`runtime_widened_host_clock` in `src/runtime_tick.c`) wraps
            // `timer_read_time` + the `stats_send_time*` stats counters and
            // is marked `used, externally_visible` so LTO keeps the symbol
            // around for staticlib callers like this one. Mirrors the
            // foreground `runtime_handle_widened_now` widening rule.
            unsafe extern "C" {
                fn runtime_widened_host_clock() -> u64;
            }
            let now: u64 = runtime_widened_host_clock();

            // Field-disjoint borrow: `engine` and `queue_consumer` are
            // separate fields, so the borrow checker accepts two non-
            // overlapping `&mut` borrows out of the single `&mut IsrState`.
            // Symmetric to `runtime_handle_tick` (StepTime path).
            let IsrState {
                engine,
                queue_consumer,
                ..
            } = isr;
            engine.runtime_modulated_tick(now, queue_consumer, pool, shared);

            // Mirror the engine's status into SharedState so the
            // foreground entrypoints see fault latching.
            shared
                .runtime_status
                .store(engine.status() as u8, Ordering::Release);
            shared
                .last_error
                .store(engine.last_error(), Ordering::Release);
        }
    }

    /// Foreground drain. Returns count of samples written.
    ///
    /// Phase 11 §10.4 expansion: alongside writing the sample to the wire
    /// buffer, this also calls `pool.confirm_retired` for any sample
    /// carrying `TRACE_FLAG_SEGMENT_END`, so curve-pool slots get reclaimed
    /// in lockstep with the trace ship-out (one drain pass = one
    /// foreground-side wire emit + one reclaim cursor advance).
    ///
    /// `out_saw_segment_end` (optional, may be NULL): set to `1` on return
    /// when the drain consumed at least one `TRACE_FLAG_SEGMENT_END`
    /// sample, else `0`. Closure-review fix: `kalico_credit_freed` emission
    /// in `runtime_drain` previously gated only on the second
    /// (drain-and-reclaim) leg's bit, but the first leg routinely consumes
    /// every `SEGMENT_END` under steady-state push, suppressing the credit
    /// event and deadlocking host flow control. The C handler now ORs this
    /// bit with the reclaim leg's bit.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_drain_trace(
        rt: *mut KalicoRuntime,
        out_buf: *mut TraceSample,
        out_cap: u32,
        out_saw_segment_end: *mut u8,
    ) -> u32 {
        if !out_saw_segment_end.is_null() {
            // SAFETY: caller-provided pointer; documented to be a valid
            // u8 location for writes when non-null.
            unsafe {
                *out_saw_segment_end = 0;
            }
        }
        if rt.is_null() || out_buf.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: project to the foreground trace consumer + curve pool —
        // no `&mut` on IsrState forms here. Caller-provided out_buf must be
        // valid for out_cap writes of TraceSample.
        unsafe {
            let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
            let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
            let fg: &mut FgState = &mut *fg_ptr;
            let out_slice = core::slice::from_raw_parts_mut(out_buf, out_cap as usize);
            let mut count = 0usize;
            let mut saw_segment_end = false;
            while count < out_slice.len() {
                let Some(sample) = fg.trace_consumer.dequeue() else {
                    break;
                };
                if (sample.flags & runtime::trace::TRACE_FLAG_SEGMENT_END) != 0 {
                    // Retire all 4 per-axis handles via the retirement table.
                    if let Some(handles) = fg.retirement_table.lookup(sample.segment_id) {
                        for h in &handles {
                            if !h.is_unused_sentinel()
                                && *h
                                    != runtime::curve_pool::CurveHandle::HOLD_SEGMENT_SENTINEL
                            {
                                pool.confirm_retired(*h);
                            }
                        }
                    }
                    saw_segment_end = true;
                }
                if let Some(slot) = out_slice.get_mut(count) {
                    *slot = sample;
                }
                count += 1;
            }
            if !out_saw_segment_end.is_null() && saw_segment_end {
                *out_saw_segment_end = 1;
            }
            #[allow(clippy::cast_possible_truncation)]
            let result = count as u32;
            result
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
    /// counters that Klipper's stats task maintains (basecmd.c). Replaces the
    /// pre-emission-rewrite SharedState seqlock dependency: TIM5 is off when
    /// `count_modulated_steppers == 0`, so the seqlock would not be re-published
    /// in StepTime-only configurations. The stats task runs unconditionally,
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

    /// Read the credit-flow epoch counter (§5.3 + §10.4). Bumped on each
    /// `kalico_stream_flush` so the host can detect mid-stream resets.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_credit_epoch(rt: *mut KalicoRuntime) -> u32 {
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
            (*shared_ptr).credit_epoch.load(Ordering::Acquire)
        }
    }

    /// Read the cumulative-accepted segment id cursor (§5.3 + §4.1.5).
    /// Mirrors the value placed into the `kalico_push_response` schema.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_accepted_segment_id(rt: *mut KalicoRuntime) -> u32 {
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
            (*shared_ptr).accepted_segment_id.load(Ordering::Acquire)
        }
    }

    /// Read the retired-through segment id cursor (§5.3 + §4.1.5). Advances
    /// monotonically as the engine retires segments — host uses this to
    /// gate flow control and to know when a stream-terminal hand-off is
    /// safe to call.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_retired_through_segment_id(
        rt: *mut KalicoRuntime,
    ) -> u32 {
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
            (*shared_ptr)
                .retired_through_segment_id
                .load(Ordering::Acquire)
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

    /// Read the currently-active segment id (`0` if engine is Idle/Drained
    /// or pre-stream).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_current_segment_id(rt: *mut KalicoRuntime) -> u32 {
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
            (*shared_ptr).current_segment_id.load(Ordering::Acquire)
        }
    }

    /// Approximate queue depth — number of segments the foreground has
    /// pushed minus the number the ISR has retired through. Useful as a
    /// status-frame breadcrumb but NOT a synchronization primitive (both
    /// cursors lag the actual SPSC state by an unbounded number of ticks
    /// in the worst case). Returns saturating-subtraction in u8 range
    /// (`Q_N - 1` is the structural cap; saturate at 255 just in case).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_queue_depth(rt: *mut KalicoRuntime) -> u8 {
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
            let accepted = (*shared_ptr).accepted_segment_id.load(Ordering::Acquire);
            let retired = (*shared_ptr)
                .retired_through_segment_id
                .load(Ordering::Acquire);
            let depth = accepted.saturating_sub(retired);
            #[allow(clippy::cast_possible_truncation)]
            let r = depth.min(u32::from(u8::MAX)) as u8;
            r
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

    /// Diagnostic: read the configured `steps_per_mm` for axis `oid` (0..=3
    /// in motor space). Returns 0.0 if `oid` is out of range or runtime
    /// uninitialised. Used by Phase 4 sim test to verify that
    /// `configure_axes_blob` reached the engine.
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).widen_state.seed_high(baseline_widened_clock);
        }
    }

    /// Diagnostic: read most recent post-PA/IS motor position for axis `oid`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_axis_motor(
        rt: *mut KalicoRuntime,
        oid: u8,
    ) -> f32 {
        if rt.is_null() || !INIT_DONE.load(Ordering::Acquire) || oid >= 4 {
            return 0.0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let (n, ts, dur) = (*isr_ptr).engine.debug_last_timing();
            if !now_out.is_null() { *now_out = n; }
            if !t_start_out.is_null() { *t_start_out = ts; }
            if !duration_out.is_null() { *duration_out = dur; }
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
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

    /// Configure axis mapping and kinematics for this MCU. Minimal stub for
    /// Step 7-B MVP — accepts `kinematics_tag` (0 = CoreXyAndE, 1 =
    /// CartesianXyzAndE) and validates. Full motor-config blob
    /// deserialization is deferred to Step 7-C.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_configure_axes(
        rt: *mut KalicoRuntime,
        kinematics_tag: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // Validate the kinematics tag.
        match kinematics_tag {
            0 | 1 => {}
            _ => return KALICO_ERR_INVALID_KINEMATICS,
        }
        // Stub: full motor-config blob deserialization deferred to Step 7-C.
        let _ = rt;
        KALICO_OK
    }

    /// Configure axes from a packed motor blob delivered via the kalico-native
    /// transport. Layout (matches `kalico-protocol` `ConfigureAxes` body):
    ///   kinematics u8 | present_mask u8 | awd_mask u8 | invert_mask u8 |
    ///   steps_per_mm[4] f32 little-endian
    ///
    /// Total: 20 bytes. `kinematics`: 0 = CoreXyAndE, 1 = CartesianXyzAndE.
    /// Bits in masks index motors `[A/X, B/Y, Z, E]`.
    ///
    /// Caller invariant: this is one-shot, called from foreground before
    /// TIM5 is armed (i.e. before any tick can fire). The FFI projects
    /// `&mut IsrState` outside the ISR lock, which is sound only under
    /// that single-threaded precondition.
    /// Emit a tagged `#output` line to the host for live-step diagnostics.
    /// On the MCU this calls into `runtime_diag_progress` (defined in
    /// `src/runtime_tick.c`), which queues an `output(...)` line to the
    /// USB-CDC TX buffer. The host sees `rt_diag tag=<T> stage=<S> value=<V>`
    /// even if the chip resets shortly after — buffered bytes flush before
    /// the disconnect. No-op on host-test builds.
    #[inline]
    #[allow(unused_variables)]
    fn diag_progress(tag: u32, stage: u32, value: u32) {
        #[cfg(target_os = "none")]
        {
            unsafe extern "C" {
                fn runtime_diag_progress(tag: u32, stage: u32, value: u32);
            }
            // SAFETY: stable C ABI; foreground-only.
            unsafe { runtime_diag_progress(tag, stage, value); }
        }
    }

    /// Extended blob layout (25 bytes):
    ///
    /// ```text
    /// byte  0     kinematics_tag  (0 = CoreXY+E, 1 = Cartesian+E)
    /// byte  1     present_mask    (bit i set → motor i is present)
    /// byte  2     awd_mask        (bit i set → motor i is AWD)
    /// byte  3     invert_mask     (bit i set → motor i direction inverted)
    /// bytes 4-7   steps_per_mm[0] (f32 LE)
    /// bytes 8-11  steps_per_mm[1] (f32 LE)
    /// bytes 12-15 steps_per_mm[2] (f32 LE)
    /// bytes 16-19 steps_per_mm[3] (f32 LE)
    ///             -- present only in extended (25-byte) format --
    /// byte 20     mcu_caps        (bit 0 = mcu_supports_phase_stepping)
    /// byte 21     step_mode[0]    (0 = Modulated, 1 = StepTime)
    /// byte 22     step_mode[1]
    /// byte 23     step_mode[2]
    /// byte 24     step_mode[3]
    /// ```
    ///
    /// Legacy hosts emit the 20-byte format; the MCU defaults all steppers to
    /// `StepTime` in that case. Any other `blob_len` is rejected.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_configure_axes_blob(
        rt: *mut KalicoRuntime,
        blob_ptr: *const u8,
        blob_len: u32,
    ) -> i32 {
        use runtime::config::{EMode as _Unused, McuAxisConfig, MotorConfig};
        let _ = _Unused::CoupledToXy;
        diag_progress(0xCA, 1, 0); // ENTER
        if rt.is_null() || blob_ptr.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // Accept 20-byte (legacy) or 25-byte (extended, with StepMode array).
        if blob_len != 20 && blob_len != 25 {
            return KALICO_ERR_INVALID_KINEMATICS;
        }
        let blob = unsafe { core::slice::from_raw_parts(blob_ptr, blob_len as usize) };
        diag_progress(0xCA, 2, u32::from(blob[0]));
        let kinematics_tag = blob[0];
        let present_mask = blob[1];
        let awd_mask = blob[2];
        let invert_mask = blob[3];
        let mut steps = [0f32; 4];
        for i in 0..4 {
            let off = 4 + i * 4;
            steps[i] = f32::from_le_bytes([
                blob[off], blob[off + 1], blob[off + 2], blob[off + 3],
            ]);
        }
        diag_progress(0xCA, 3, u32::from(present_mask));
        let kinematics = match kinematics_tag {
            0 => KinematicTag::CoreXyAndE,
            1 => KinematicTag::CartesianXyzAndE,
            _ => return KALICO_ERR_INVALID_KINEMATICS,
        };
        let mut motors: [Option<MotorConfig>; 4] = [None, None, None, None];
        for i in 0..4 {
            if present_mask & (1 << i) != 0 {
                motors[i] = Some(MotorConfig {
                    steps_per_mm: steps[i],
                    is_awd: awd_mask & (1 << i) != 0,
                    invert_dir: invert_mask & (1 << i) != 0,
                });
            }
        }
        diag_progress(0xCA, 4, 0);
        let cfg = McuAxisConfig { motors, kinematics };
        if cfg.validate().is_err() {
            return KALICO_ERR_INVALID_KINEMATICS;
        }
        diag_progress(0xCA, 5, 0);
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: per the doc-comment precondition, no ISR is running yet.
        // Foreground is the sole writer; we project &mut IsrState to call
        // engine.configure(). No other &IsrState may be live at this point.
        unsafe {
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.configure(cfg);
        }
        diag_progress(0xCA, 6, 0);
        // Step 7-D: configure_axes is the bridge's "I'm starting a fresh
        // session" signal. Reset segment-id monotonicity tracking so the
        // new klippy session's segment_id sequence (which restarts from 1)
        // doesn't collide with any accepted_segment_id state preserved on
        // the MCU across the klippy reconnect (-141
        // SEGMENT_ID_NON_MONOTONIC). NOT calling full `stream::flush()` —
        // that path does an ISR force_idle handshake which times out into
        // LIVENESS_STALLED when TIM5 isn't yet running (configure_axes is
        // called before the first segment push, before ISR enable). Only
        // these two atomics gate the monotonic check (runtime_ffi.rs:265
        // and engine.rs / segment::activate paths); resetting them is
        // safe with no ISR running.
        unsafe {
            let shared_ptr: *const runtime::state::SharedState =
                core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).accepted_segment_id_seen
                .store(false, Ordering::Release);
            (*shared_ptr).accepted_segment_id
                .store(0, Ordering::Release);
        }
        // Same "fresh session" semantics: clear the C-side
        // motor→stepper binding table so the upcoming stream of
        // config_runtime_stepper commands populates a fresh slate.
        // Without this, the table accumulates across klippy
        // reconnects (the MCU stays powered) and motor 0 / motor 1 hit
        // RUNTIME_MAX_STEPPERS_PER_MOTOR=4 after two reconnects —
        // the third shutdowns with "too many steppers per motor".
        // Bench-observed 2026-05-11 after F446 KALICO_RUNTIME
        // enablement, when klippy began re-running
        // config_runtime_stepper for both H7 and F446 each session.
        diag_progress(0xCA, 7, 0);
        #[cfg(target_os = "none")]
        {
            unsafe extern "C" {
                fn runtime_reset_stepper_bindings();
            }
            // SAFETY: foreground-only, no preconditions; simply zeros
            // small static arrays in src/stepper.c.
            unsafe { runtime_reset_stepper_bindings(); }
        }
        diag_progress(0xCA, 8, 0);

        // --- Extended format: parse per-stepper StepMode array (spec §4 C1) ---
        //
        // byte 20: mcu_caps (bit 0 = mcu_supports_phase_stepping)
        // bytes 21-24: step_mode[0..4] (0 = Modulated, 1 = StepTime)
        //
        // Legacy 20-byte blobs land here with no step_mode bytes; SharedState
        // already initialises every step_modes[i] to StepTime (default), so no
        // further action is needed for the legacy path.
        if blob_len == 25 {
            let mcu_caps = blob[20];
            let mcu_supports_phase = (mcu_caps & 0x01) != 0;
            unsafe {
                let shared_ptr: *const runtime::state::SharedState =
                    core::ptr::addr_of!((*ctx).shared);
                let shared: &runtime::state::SharedState = &*shared_ptr;
                for i in 0..4usize {
                    let raw_mode = blob[21 + i];
                    let mode = match runtime::state::StepMode::from_u8(raw_mode) {
                        Some(m) => m,
                        // Unknown discriminant → treat as StepTime (safe default).
                        None => runtime::state::StepMode::StepTime,
                    };
                    match runtime::state::set_step_mode(
                        shared,
                        i as u8,
                        mode,
                        mcu_supports_phase,
                    ) {
                        Ok(()) => {}
                        Err(runtime::state::SetStepModeError::CapabilityMissing) => {
                            // Host requested Modulated on an MCU that doesn't
                            // advertise phase-stepping. Defense-in-depth check —
                            // the host (Task E1) is supposed to prevent this.
                            diag_progress(0xCA, 9, i as u32);
                            return KALICO_ERR_CAPABILITY_MISSING;
                        }
                        Err(runtime::state::SetStepModeError::OutOfRange) => {
                            // i is bounded to 0..4 above; unreachable in practice.
                            diag_progress(0xCA, 9, 0xFF);
                            return KALICO_ERR_INVALID_KINEMATICS;
                        }
                    }
                }
            }
            diag_progress(0xCA, 10, u32::from(mcu_caps));
        }

        // Spec §6.3: after all step_modes are written, atomically decide
        // whether TIM5 should be armed. `runtime_tick_enable` will no-op on
        // the C side when the Modulated count is zero (F4 with all-StepTime
        // config never starts TIM5 here). This covers both the legacy
        // 20-byte path (all-StepTime default, count == 0, no-op enable) and
        // the extended 25-byte path (enables TIM5 only if at least one
        // Modulated stepper was configured).
        unsafe { runtime_tick_enable() };

        KALICO_OK
    }

    /// Phase 11 Task 11.2 foreground reclaim drain pipeline. Drains up to
    /// `limit` trace samples from the ring, calls `pool.confirm_retired`
    /// for each `SEGMENT_END` observed, and returns a 32-bit packed
    /// status:
    ///
    /// - Bits 0..=15 — count of samples drained this call.
    /// - Bit 16     — set if a fresh trace-overflow fault latched (§13.1).
    /// - Bit 17     — set if at least one `SEGMENT_END` was observed
    ///   (caller emits one or more `kalico_credit_freed`
    ///   events keyed off the updated cursors).
    ///
    /// The C handler (`runtime_drain` `DECL_TASK` in `src/runtime_tick.c`)
    /// uses this single-call form so the trace-drain + reclaim + fault-
    /// latch pipeline is one round-trip per drain wake-up.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_drain_and_reclaim(
        rt: *mut KalicoRuntime,
        limit: u32,
    ) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only projection — touches FgState (sole writer)
        // for the trace consumer, &CurvePool for confirm_retired, and
        // &SharedState for the trace-overflow latch.
        unsafe {
            let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
            let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            let fg: &mut FgState = &mut *fg_ptr;
            let mut saw_segment_end = false;
            let drained = runtime::reclaim::drain_and_reclaim(
                pool,
                &fg.retirement_table,
                || {
                    let s = fg.trace_consumer.dequeue();
                    if let Some(sample) = s {
                        if (sample.flags & runtime::trace::TRACE_FLAG_SEGMENT_END) != 0 {
                            saw_segment_end = true;
                        }
                    }
                    s
                },
                limit as usize,
            );
            let overflow_latched = runtime::reclaim::check_trace_overflow_and_fault(shared);
            let mut packed: u32 = (drained as u32) & 0xFFFF;
            if overflow_latched {
                packed |= 1 << 16;
            }
            if saw_segment_end {
                packed |= 1 << 17;
            }
            packed
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
    /// SAFETY: same contract as `runtime_handle_push_segment`'s projection.
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
                let (r, armed_t) = runtime::stream::arm(fg, shared, t_start_t0, arm_lead_cycles);
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
        unsafe {
            project_fg(rt, |fg, shared| {
                runtime::stream::terminal(fg, shared, segment_id)
            })
        }
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
    ///
    /// Returns the on-demand widened MCU clock (timer_read_time +
    /// stats_send_time_high), NOT the engine seqlock value. Rationale: the
    /// seqlock published by `Engine::tick` is only updated from the TIM5 ISR,
    /// and TIM5 stays disabled in the all-StepTime MVP (see
    /// `runtime_tick_enable` in `src/stm32/runtime_tick_h7.c` — early-return
    /// when `count_modulated_steppers == 0`). Reading the seqlock in that
    /// configuration returns its default 0, which the bridge's clock-sync
    /// driver filters out as "MCU clock looks uninitialised" — the host's
    /// router clock estimate then never refreshes from its connect-time
    /// anchor, `compute_ack_clock` extrapolates linearly into the future,
    /// segment `t_start` lands tens of seconds ahead of the MCU's actual
    /// clock, and the in-flight credit window deadlocks waiting for
    /// retirements that can't happen.
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
            let Some((degree, n_cp, n_knots, _n_weights)) =
                runtime::sim_fixtures::lookup(fixture_id, &mut cps, &mut knots, &mut weights)
            else {
                return KALICO_ERR_INVALID_CURVE;
            };
            // Step 7-B: load_unchecked uses scalar API. Fixtures still
            // emit 3D data (3 floats per CP); extract first component (X).
            // Task 8 will update fixtures to native scalar.
            const FIXTURE_DIM: usize = 3; // sim_fixtures' per-CP dimension
            let mut cps_scalar = [0.0_f32; runtime::curve_pool::MAX_CONTROL_POINTS];
            for i in 0..n_cp {
                cps_scalar[i] = cps[i * FIXTURE_DIM];
            }
            match pool.load_unchecked(
                slot_idx,
                degree,
                &knots[..n_knots],
                &cps_scalar[..n_cp],
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

        let sources_blob: &[u8] = unsafe {
            core::slice::from_raw_parts(sources_ptr, sources_len)
        };
        let steppers_blob: &[u8] = unsafe {
            core::slice::from_raw_parts(steppers_ptr, steppers_len)
        };

        let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
        for i in 0..source_count as usize {
            let r = &sources_blob[i * SOURCE_RECORD_LEN..(i + 1) * SOURCE_RECORD_LEN];
            let kind = match r[0] {
                0 => SourceKind::Physical,
                1 => SourceKind::TmcDiag,
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
        let msg = ArmMsg {
            arm_id,
            arm_clock,
            source_count,
            sources,
            stepper_count,
            stepper_oids,
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

    /// Step-time trip evaluation. Called from each `step_time_event` ISR
    /// after the per-step GPIO sample, mirroring what `engine.tick()` does
    /// for the Modulated path (which dispatches `endstop::tick` via
    /// `engine::poll_endstop_trip`, see `rust/runtime/src/engine.rs`).
    ///
    /// In a StepTime-only firmware build (MVP), TIM5 — and therefore
    /// `engine.tick` and its embedded `endstop::tick` call — never runs.
    /// Without this entry point an armed endstop's per-step samples update
    /// `PIN_LEVELS` but trip evaluation never happens, and homing moves
    /// run forever even when the GPIO asserts.
    ///
    /// On `AbortNow` this also calls `Engine::abort_for_step_time_trip`
    /// (the shared `abort_for_homing_trip` helper under the hood), which
    /// retires every in-flight segment's curve-pool slots, publishes the
    /// retire cursor for the host's `kalico_credit_freed` emitter, and
    /// transitions the engine to `Drained`. Without that retirement, the
    /// host's slot pool deadlocks after CURVE_POOL_N / 2 G28 trips
    /// because trip-aborted segments leak their slots — observed in the
    /// `g28_shaped_xy_two_pass_homing_via_renode_monitor` sim test.
    ///
    /// `now` is the widened MCU clock at the call site (same widening as
    /// `command_runtime_clock_sync_request` uses). The endstop module
    /// records this as the `trip_clock` in the published snapshot, and
    /// gates on `now >= arm_clock`. Velocity-gated policies receive
    /// `[u32::MAX; 3]` from this entry — see
    /// `Engine::abort_for_step_time_trip` for the rationale.
    ///
    /// Returns 1 if a trip fired this call, 0 otherwise, or a negative
    /// `KALICO_ERR_*` on misuse.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_tick_step_time(
        rt: *mut KalicoRuntime,
        now: u64,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_INVALID_HANDLE;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `ctx` points at the published RT_CELL (verified non-null
        // and INIT_DONE==true above). The step_time_event ISR is the sole
        // writer of the per-stepper consumer state and shares the ISR
        // ownership discipline with kalico_runtime_tick — claiming IsrState
        // mutably here mirrors what `kalico_runtime_tick` does at the same
        // priority level.
        unsafe {
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let pool_ptr: *const CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let isr: &mut IsrState = &mut *isr_ptr;
            let pool: &CurvePool = &*pool_ptr;
            let shared: &SharedState = &*shared_ptr;
            let IsrState {
                engine,
                trace_producer,
                ..
            } = isr;
            if engine.abort_for_step_time_trip(now, pool, trace_producer, shared) {
                shared
                    .runtime_status
                    .store(engine.status() as u8, Ordering::Release);
                shared
                    .last_error
                    .store(engine.last_error(), Ordering::Release);
                1
            } else {
                0
            }
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
            match runtime::state::set_step_mode(shared, stepper_idx, mode, mcu_supports_phase != 0) {
                Ok(()) => {
                    // Spec §6.3: re-evaluate TIM5 arm state after every
                    // successful step-mode flip. Count Modulated steppers via
                    // the same loop used by `kalico_runtime_count_modulated_steppers`.
                    // `runtime_tick_enable` is a no-op when count == 0 (C-side
                    // guard added in the same commit), so calling it here is
                    // always safe. `runtime_tick_disable` is called only when
                    // the count reaches zero — idempotent if TIM5 was never
                    // started.
                    use runtime::state::MAX_STEPPER_OIDS;
                    let mut modulated_count = 0u8;
                    for i in 0..MAX_STEPPER_OIDS {
                        if shared.step_modes[i].load(Ordering::Acquire)
                            == runtime::state::StepMode::Modulated as u8
                        {
                            modulated_count = modulated_count.saturating_add(1);
                        }
                    }
                    if modulated_count == 0 {
                        runtime_tick_disable();
                    } else {
                        runtime_tick_enable();
                    }
                    KALICO_OK
                }
                Err(runtime::state::SetStepModeError::CapabilityMissing) => {
                    KALICO_ERR_CAPABILITY_MISSING
                }
                Err(runtime::state::SetStepModeError::OutOfRange) => KALICO_ERR_INVALID_HANDLE,
            }
        }
    }

    // ---- Step-emission rewrite FFI surface (Task 6) -----------------------
    //
    // The pre-emission `kalico_runtime_arm_step_timer` /
    // `kalico_runtime_compute_next_step_time` exports (per-segment schedule
    // architecture) were retired here in favour of the per-motor step-ring
    // surface below. The new model splits arming into:
    //
    //   - producer (1 shared Klipper timer)   →  `kalico_runtime_producer_step`
    //   - per-motor consumers (1 timer each)  →  `kalico_runtime_step_ring_peek_head`
    //                                         /  `peek_next` / `advance` / `available`
    //   - wake source for the producer        →  `kalico_runtime_kick_producer`
    //   - foreground synchronous flush        →  `kalico_runtime_force_idle` (T11 stub)
    //
    // Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md
    // §3.4 (producer), §3.5 (consumer), §3.10 (force_idle).

    /// Producer Klipper-timer callback entry. Runs one `Engine::producer_step`
    /// pass: Newton-fills `(cycles_abs_lo, dir)` entries into every StepTime
    /// motor's `StepRing` up to `PRODUCER_BATCH_CAP` per motor, retires any
    /// segment whose `consumers_remaining` mask drained, and republishes
    /// `producer_pending` / `producer_runs_total` diagnostics.
    ///
    /// Returns `true` if at least one motor still has unfinished work
    /// (caller should self-reschedule the producer timer); `false` if every
    /// StepTime motor reached `AllIdle` (caller waits for a wake from
    /// `push_segment`'s CAS or from `kalico_runtime_kick_producer`).
    ///
    /// SAFETY (caller): `rt` is the published `RT_CELL` pointer and the
    /// producer-side Klipper timer is the single serialised caller — no
    /// other context may concurrently form `&mut IsrState::engine` or
    /// pull from `IsrState::queue_consumer`. The C-side producer timer in
    /// Task 8 will enforce this by routing through one `DECL_TIMER`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_producer_step(rt: *mut KalicoRuntime) -> bool {
        if rt.is_null() {
            return false;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return false;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null + INIT_DONE=true above. The projection
        // mirrors `runtime_handle_tick` (the long-running ISR-half mutator):
        // we materialise `&mut IsrState` once via `UnsafeCell::raw_get`,
        // then field-disjoint-borrow `engine` and `queue_consumer` out of
        // it; the curve pool and shared half are projected through
        // `addr_of!` as `&` references (atomics-only access on those
        // halves). The §11.1 ownership discipline says the producer timer
        // and the TIM5 ISR never run concurrently against the same
        // `IsrState` (foreground producer timer fires from `sched_check_periodic`
        // with IRQs disabled; TIM5 ISR is suspended on entry to the same
        // critical section — Task 8 wires the gating).
        unsafe {
            use runtime::step_producer::ProducerTickResult;
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            let isr: &mut IsrState = &mut *isr_ptr;
            let IsrState {
                engine,
                queue_consumer,
                trace_producer,
                ..
            } = isr;
            // Snapshot the retire cursor BEFORE producer_step. After the call
            // returns, if the cursor advanced, the producer retired one or
            // more segments. The C-side `runtime_drain` -> `kalico_credit_freed`
            // path keys off SEGMENT_END trace samples (via `drain_and_reclaim`'s
            // bit-17 in the packed status word), so we must enqueue one per
            // retired segment for the host's slot pool to drain. Without
            // this, the host fills `capacity=4` segments to F446 / `capacity=16`
            // to H7, then blocks on "slot pool exhausted" and shuts down the
            // MCU after 4 G1 commands.
            let retired_before = shared.retired_through_segment_id.load(Ordering::Acquire);
            let result = engine.producer_step(pool, queue_consumer, shared);
            let retired_after = shared.retired_through_segment_id.load(Ordering::Acquire);
            if retired_after != retired_before {
                // One SEGMENT_END sample per retired segment id. Wraparound-safe
                // arithmetic across u32 — at worst we emit a couple of stale
                // samples per wrap, which the host tolerates (credit_freed
                // is idempotent in `retired_through_segment_id`).
                use runtime::trace::{TRACE_FLAG_SEGMENT_END, TraceSample};
                let mut id = retired_before.wrapping_add(1);
                loop {
                    let _ = trace_producer.enqueue(TraceSample {
                        tick: 0,
                        motor_a: 0.0,
                        motor_b: 0.0,
                        motor_z: 0.0,
                        motor_e: 0.0,
                        segment_id: id,
                        curve_handle: runtime::curve_pool::CurveHandle::UNUSED_SENTINEL,
                        flags: TRACE_FLAG_SEGMENT_END,
                        _pad: [0; 7],
                    });
                    if id == retired_after {
                        break;
                    }
                    id = id.wrapping_add(1);
                }
            }
            match result {
                ProducerTickResult::WorkPending => true,
                ProducerTickResult::AllIdle => false,
            }
        }
    }

    /// Per-motor consumer: peek the entry at the cursor without advancing.
    ///
    /// Returns `true` and writes `*out_cycles_abs_lo` + `*out_dir` if the
    /// ring has at least one entry to consume; returns `false` (and leaves
    /// the out-params untouched) on:
    ///   - null `rt`, null out-params, or `motor_idx >= 4`;
    ///   - `INIT_DONE == false`;
    ///   - the ring is empty (producer hasn't caught up).
    ///
    /// SAFETY (caller): the per-stepper consumer Klipper timer (Task 7) is
    /// the single serialised consumer of motor `motor_idx`'s ring; concurrent
    /// `peek_*` / `advance` against the same motor from another context is
    /// outside the SPSC contract.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_step_ring_peek_head(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
        out_cycles_abs_lo: *mut u32,
        out_dir: *mut i8,
    ) -> bool {
        if rt.is_null()
            || (motor_idx as usize) >= 4
            || out_cycles_abs_lo.is_null()
            || out_dir.is_null()
        {
            return false;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return false;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only ring access — `StepRing::peek_head` takes `&self`
        // and only loads atomics. We project to `&IsrState` (shared, not
        // mutable) and call the const accessor.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let isr: &IsrState = &*isr_ptr;
            let Some(ring) = isr.engine.step_ring(motor_idx as usize) else {
                return false;
            };
            let Some((t, dir)) = ring.peek_head() else {
                return false;
            };
            *out_cycles_abs_lo = t;
            *out_dir = dir;
            true
        }
    }

    /// Per-motor consumer: peek the second entry (the one after the cursor's
    /// head). Used by the per-stepper consumer ISR to compute its next
    /// reschedule time and decide whether the next pulse already has a
    /// known direction (no DIR flip required).
    ///
    /// Same return / safety contract as `kalico_runtime_step_ring_peek_head`,
    /// but reports `false` if fewer than two entries are available.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_step_ring_peek_next(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
        out_cycles_abs_lo: *mut u32,
        out_dir: *mut i8,
    ) -> bool {
        if rt.is_null()
            || (motor_idx as usize) >= 4
            || out_cycles_abs_lo.is_null()
            || out_dir.is_null()
        {
            return false;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return false;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only ring access through atomics (same as
        // `peek_head`); `StepRing::peek_next` is `&self`.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let isr: &IsrState = &*isr_ptr;
            let Some(ring) = isr.engine.step_ring(motor_idx as usize) else {
                return false;
            };
            let Some((t, dir)) = ring.peek_next() else {
                return false;
            };
            *out_cycles_abs_lo = t;
            *out_dir = dir;
            true
        }
    }

    /// Per-motor consumer: advance the cursor past `n` entries. Called
    /// after the per-stepper consumer Klipper timer has fired the step
    /// pulse(s) corresponding to the entries up to (but not including) the
    /// new cursor.
    ///
    /// `Release` ordering on the cursor (via `StepRing::advance`) pairs
    /// with the producer's `Acquire` load in `StepRing::space`, publishing
    /// that the consumed slots are free for the producer to overwrite.
    ///
    /// No-op on null `rt`, `motor_idx >= 4`, or before init completes.
    ///
    /// SAFETY (caller): see `kalico_runtime_step_ring_peek_head`. The
    /// consumer Klipper timer is the single serialised caller of `advance`
    /// for its motor.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_step_ring_advance(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
        n: u32,
    ) {
        if rt.is_null() || (motor_idx as usize) >= 4 {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `StepRing::advance` takes `&self` (the cursor atomic
        // does the synchronization). We project read-only into IsrState.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let isr: &IsrState = &*isr_ptr;
            let Some(ring) = isr.engine.step_ring(motor_idx as usize) else {
                return;
            };
            ring.advance(n);
        }
    }

    /// Per-motor consumer: read the count of entries currently available
    /// to consume (i.e. `head - cursor` with wrap-aware arithmetic).
    ///
    /// Used by the consumer Klipper timer's low-water hook to decide if it
    /// should call `kalico_runtime_kick_producer` (e.g. when the ring is
    /// below half full and the producer is currently idle waiting on a
    /// kick).
    ///
    /// Returns `0` on null `rt`, `motor_idx >= 4`, or before init completes.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_step_ring_available(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
    ) -> u32 {
        if rt.is_null() || (motor_idx as usize) >= 4 {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `StepRing::available` is `&self` and only loads atomics.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let isr: &IsrState = &*isr_ptr;
            isr.engine
                .step_ring(motor_idx as usize)
                .map(|r| r.available())
                .unwrap_or(0)
        }
    }

    /// Wake the producer: CAS-set `shared.producer_pending` from `false`
    /// to `true`. Returns `true` iff this call won the CAS — in which case
    /// the caller is responsible for actually scheduling the producer
    /// Klipper timer (`sched_add_timer` on the C side, Task 8). Returns
    /// `false` if `producer_pending` was already `true` (another kicker
    /// got there first; their scheduled producer run will see this
    /// caller's wake reason).
    ///
    /// Wake sources covered by this entry point:
    ///   - the per-motor consumer's low-water hook (Task 7);
    ///   - any host-side reason to force a producer fill before the next
    ///     `push_segment` arrives.
    ///
    /// `push_segment` already CAS-sets `producer_pending` inside
    /// `Engine::push_segment` (rust/runtime/src/engine.rs); call sites
    /// invoking `runtime_handle_push_segment` do NOT need to also call
    /// this — the engine wakes itself.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_kick_producer(rt: *mut KalicoRuntime) -> bool {
        if rt.is_null() {
            return false;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return false;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState atomics-only access; no `&mut` forms. The CAS
        // ordering matches `Engine::push_segment`'s own kick (Release on
        // success / Acquire on failure) so a winning kick publishes any
        // prior writes the kicker performed (e.g. the consumer's cursor
        // advance preceding the low-water decision).
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared
                .producer_pending
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
        }
    }

    /// Diagnostic: read the high-water mark for motor `motor_idx`'s step
    /// ring. Returns the maximum `available()` value observed across the
    /// runtime's lifetime. Used by the C-side fault_detail rotation (tag
    /// 0xB2) to localise whether the producer has actually pushed entries
    /// — `0` means "ring was never written" (Cardano returned no roots, or
    /// `fetch_segment_for_motor` never returned a curve).
    ///
    /// Returns `0` on null `rt`, `motor_idx >= 4`, or before init completes.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_ring_high_water(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
    ) -> u32 {
        if rt.is_null() || (motor_idx as usize) >= 4 {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared
                .ring_high_water
                .get(motor_idx as usize)
                .map(|a| a.load(Ordering::Acquire))
                .unwrap_or(0)
        }
    }

    /// Diagnostic: read the low 32 bits of `producer_steps_pushed_total`.
    /// Number of successful `ring.push` calls across all motors.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_steps_pushed_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_steps_pushed_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the low 32 bits of
    /// `producer_motor_finished_curve_total`. Number of times Cardano
    /// returned SegmentExhausted in a `producer_step` loop (per motor,
    /// summed). Distinguishes "Cardano can't find roots" from "fetch
    /// returns None".
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_motor_finished_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_motor_finished_curve_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the low 32 bits of `producer_segment_retired_total`.
    /// Number of segments fully retired by `producer_step`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_segments_retired_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_segment_retired_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the low 32 bits of `producer_segment_dequeued_total`.
    /// Number of segments pulled off the queue by `producer_step`.
    /// If host sent N PushSegment but this stays at 0, segments aren't
    /// reaching the engine queue (kalico_dispatch or runtime_handle_push_segment
    /// dropping silently).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_segments_dequeued_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_segment_dequeued_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the low 32 bits of `producer_runs_total`. Tells
    /// how many `Engine::producer_step` invocations have completed since
    /// boot. If `step_time_producer_kicks` (C side) is incrementing but
    /// `producer_runs_total` stays at 0, the kick path is broken between
    /// the CAS and `sched_add_timer`.
    ///
    /// Returns `0` on null `rt` or before init completes.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_producer_runs_lo(
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
            shared.producer_runs_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the low 32 bits of `producer_fetch_attempts_total`.
    /// Bumps unconditionally at function entry — if 0 while
    /// `producer_runs_total` is non-zero, the per-motor loop is filtering
    /// every motor at the step_mode / step_distance / is_idle gates.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_fetch_attempts_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_fetch_attempts_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the low 32 bits of `producer_enqueue_success_total`.
    /// Bumps AFTER `fg.queue_producer.enqueue(seg)` returns Ok in
    /// `push_segment_impl`. If non-zero while
    /// `producer_segment_dequeued_total` is 0, the queue split is broken
    /// (producer and consumer ends not sharing the backing buffer).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_enqueue_success_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_enqueue_success_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read low 32 bits of `producer_primary_resolved_total`.
    /// Number of times pool.resolve(primary) returned Some in producer_step.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_primary_resolved_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_primary_resolved_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: low 32 bits of `producer_primary_stale_total`.
    /// Counted when handle != UNUSED but pool.resolve returned None.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_primary_stale_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_primary_stale_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: low 32 bits of `producer_primary_unused_total`.
    /// Counted when handle is the UNUSED sentinel (stationary axis).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_primary_unused_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.producer_primary_unused_total.load(Ordering::Acquire) as u32
        }
    }

    /// Diagnostic: read the last result code from `push_segment_impl`.
    /// 0 = KALICO_OK, negative = an error code (see error.rs). Updated on
    /// every call regardless of outcome.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_push_segment_result(
        rt: *mut KalicoRuntime,
    ) -> i32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_segment_result.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis: read the low 32 bits of
    /// `push_segment_all_unused_total`. If this counter advances during a
    /// jog, the bridge sent push_segment frames with every handle set to
    /// the UNUSED sentinel — the segment retires on producer dequeue
    /// without ever invoking motor processing, which matches the
    /// "energized but no motion" symptom.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_push_seg_all_unused_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
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
    pub unsafe extern "C" fn kalico_runtime_last_push_x_handle(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_x_handle_packed.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis: read the packed last `y_handle` from
    /// `push_segment_impl`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_push_y_handle(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
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
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_push_consumers_remaining.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis (CP capture). Raw f32 bits of cps[0]
    /// (first control point, position at u=0) of piece 0 of the most
    /// recently resolved primary X curve. For a 0.5mm pure-X jog
    /// starting at X=125.0, expect 0x42FA0000 (= 125.0f). Captured only
    /// for motor 0.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_resolved_primary_cps_0(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_resolved_primary_cps_0.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis (CP capture). Raw f32 bits of cps[3]
    /// (last control point, position at u=1) of piece 0 of the most
    /// recently resolved primary X curve. For a 0.5mm pure-X jog from
    /// X=125.0, expect 0x42FB0000 (= 125.5f). If `cps_0 == cps_3`, the
    /// curve has zero displacement — `SegmentExhausted` from
    /// `compute_next_step_time` is then expected, indicating a planner
    /// bug (or wire-corruption that zeroed the displacement).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_resolved_primary_cps_3(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_resolved_primary_cps_3.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis (CP capture). Raw f32 bits of cps[0]
    /// of the COMBINED (post-CoreXY-mix) curve for motor 0 (A = X + Y).
    /// For a 0.5mm pure-X jog from X=125, Y=100 (Y curve constant), the
    /// combined motor-A position at u=0 is X+Y = 225.0 (0x43610000).
    /// Compare with the raw primary cps_0: if raw is correct but
    /// combined is unexpected, the kinematic mix or the Y constant
    /// value is wrong.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_combined_motor_a_cps_0(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_combined_motor_a_cps_0.load(Ordering::Acquire)
        }
    }

    /// 2026-05-15 live diagnosis (CP capture). Raw f32 bits of cps[3]
    /// of the COMBINED (post-CoreXY-mix) curve for motor 0. For a 0.5mm
    /// pure-X jog from X=125, Y=100, expect 225.5 (0x43618000). If
    /// `combined_cps_0 == combined_cps_3` while raw primary cps_0 !=
    /// cps_3, the kinematic combination is cancelling the displacement
    /// (wrong sign in `kine.combine`, or follower curve carrying the
    /// negative of the driver curve's delta).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_last_combined_motor_a_cps_3(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            shared.last_combined_motor_a_cps_3.load(Ordering::Acquire)
        }
    }

    /// Synchronous foreground flush. Spec §3.10 (Task 11).
    ///
    /// Drains the segment queue, retires every in-flight curve-pool slot,
    /// resets every `StepRing` (head + cursor → 0), clears every
    /// `ProducerState`, zeroes per-motor `StepAccumulator` residuals, and
    /// clears the engine's `producer_current` + legacy `current` slots.
    /// After this returns, the runtime is in the "fresh, no work" state:
    /// subsequent `kalico_runtime_producer_step` / TIM5
    /// `runtime_modulated_tick` invocations return immediately.
    ///
    /// **Caller contract:** no concurrent `kalico_runtime_producer_step`,
    /// TIM5 ISR, or per-stepper consumer Klipper-timer access may be in
    /// flight. The C side enforces this by either (a) calling under
    /// `irq_save` (the Klipper-timer / ISR paths gate on IRQs), or
    /// (b) serialising through the bridge command channel (which is
    /// single-threaded on the Klipper foreground task). The host's flush
    /// path satisfies (b).
    ///
    /// Returns `true` on success, `false` on null `rt` or pre-init.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_force_idle(rt: *mut KalicoRuntime) -> bool {
        if rt.is_null() {
            return false;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return false;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null + INIT_DONE=true. We project `&mut IsrState`
        // via the UnsafeCell raw pointer — same pattern as
        // `kalico_runtime_producer_step`. The §11.1 ownership discipline
        // requires the producer + consumer timers and the TIM5 ISR to be
        // quiesced at the moment of call; the caller's docstring above
        // documents that contract.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let pool: &CurvePool = &*core::ptr::addr_of!((*ctx).curve_pool);
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            let isr: &mut IsrState = &mut *isr_ptr;
            let IsrState {
                engine,
                queue_consumer,
                ..
            } = isr;
            engine.runtime_force_idle(pool, queue_consumer, shared);
        }
        true
    }

    /// Apply `delta_steps` to `shared.stepper_counts[stepper_idx]` atomically.
    ///
    /// Called by the C-side `step_time_event` ISR after `runtime_emit_step_pulses`
    /// to commit the just-emitted step into the engine's step counter. Without
    /// this, `arm_step_timer_for_stepper`'s `current_step` read stays at 0
    /// forever and the Newton solver always solves for the FIRST step within
    /// the active segment — the timer fires, emits a step, then computes the
    /// same first-step time again and never advances along the curve (bench
    /// wedge 2026-05-12: engine retired segments by virtue of `Engine::tick`'s
    /// `now` advancing, but step pulses-per-second was capped at the
    /// 1 kHz NO_STEP poll rate rather than the true Newton-iterated step rate).
    ///
    /// Mirrors `engine.rs:1067`'s `counter.fetch_add(step_result.n_steps, ...)`
    /// for the polled-tick / Modulated path.
    ///
    /// Returns `KALICO_OK` or an error.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_apply_step(
        rt: *mut KalicoRuntime,
        stepper_idx: u8,
        delta_steps: i32,
    ) -> i32 {
        use runtime::state::MAX_STEPPER_OIDS;
        if rt.is_null() || (stepper_idx as usize) >= MAX_STEPPER_OIDS {
            return KALICO_ERR_INVALID_HANDLE;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null + INIT_DONE=true. `stepper_counts` are
        // `AtomicI32` entries in `SharedState`; the AcqRel fetch_add is
        // ISR-safe (TIM5 path uses the same Ordering for its own emissions).
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            if let Some(counter) = shared.stepper_counts.get(stepper_idx as usize) {
                counter.fetch_add(delta_steps, Ordering::AcqRel);
                KALICO_OK
            } else {
                KALICO_ERR_INVALID_HANDLE
            }
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

    /// Count how many steppers are currently in `StepMode::Modulated`.
    ///
    /// Used by `runtime_tick_enable` (C-side, spec §6.3) to decide whether
    /// TIM5 is needed: if the count is zero, TIM5 has no work and is left
    /// disabled. F4 (no `PHASE_STEPPING` capability) always hits this path;
    /// H7 in an all-StepTime config also leaves TIM5 idle.
    ///
    /// Returns `0` for a null `rt` or uninitialised runtime.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_count_modulated_steppers(
        rt: *mut KalicoRuntime,
    ) -> u8 {
        use runtime::state::MAX_STEPPER_OIDS;
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` is the published RT_CELL pointer (non-null, INIT_DONE=true).
        // `step_modes` are `AtomicU8` fields; we read via a shared `&SharedState`
        // reference — no `&mut` is formed. Acquire ordering ensures we see the
        // latest `set_step_mode` write from any foreground caller.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            let mut count = 0u8;
            for i in 0..MAX_STEPPER_OIDS {
                if shared.step_modes[i].load(Ordering::Acquire)
                    == runtime::state::StepMode::Modulated as u8
                {
                    count = count.saturating_add(1);
                }
            }
            count
        }
    }

    /// Returns 1 if motor `stepper_idx` is configured (has step_distance > 0
    /// in its `ProducerState`), 0 otherwise. Used by C-side
    /// `init_step_time_timers` to avoid enabling consumer Klipper timers
    /// for unconfigured motors — without this gate every motor with the
    /// default `StepMode::StepTime` gets a timer regardless of
    /// configuration, and on Renode (1 µs quantum) the resulting
    /// scheduler load drowns LoadCurve byte processing in the USART RX
    /// FIFO.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_motor_is_configured(
        rt: *mut KalicoRuntime,
        stepper_idx: u8,
    ) -> u8 {
        use runtime::state::MAX_STEPPER_OIDS;
        if rt.is_null() || (stepper_idx as usize) >= MAX_STEPPER_OIDS {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: rt is the published RT_CELL pointer; producer_states are
        // in the ISR-side state half but we only read step_distance (set
        // once at Engine::configure() and never mutated thereafter), so a
        // shared-borrow read is safe.
        unsafe {
            let isr_ptr: *const runtime::state::IsrState =
                core::cell::UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let isr: &runtime::state::IsrState = &*isr_ptr;
            let sd = isr
                .engine
                .producer_step_distance(stepper_idx as usize)
                .unwrap_or(0.0);
            if sd > 0.0 { 1 } else { 0 }
        }
    }
}
