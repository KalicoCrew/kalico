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

    use runtime::curve_pool::{CURVE_POOL_N, CurveHandle, CurvePool};
    use runtime::engine::RuntimeStatus;
    use runtime::error::{
        KALICO_ERR_CAPABILITY_MISSING, KALICO_ERR_FAULT_LATCHED, KALICO_ERR_INVALID_ARG,
        KALICO_ERR_INVALID_CURVE, KALICO_ERR_INVALID_DURATION, KALICO_ERR_INVALID_HANDLE,
        KALICO_ERR_INVALID_KINEMATICS,
        KALICO_ERR_NOT_INIT, KALICO_ERR_NULL_PTR, KALICO_ERR_PHASE_MODE_NOT_AVAILABLE,
        KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED, KALICO_ERR_QUEUE_FULL,
        KALICO_ERR_SEGMENT_ID_NON_MONOTONIC, KALICO_ERR_ZERO_DURATION_SEGMENT, KALICO_OK,
    };
    use runtime::segment::{KinematicTag, Segment};
    use runtime::state::{FgState, IsrState, RuntimeContext, SharedState};
    use runtime::trace::TraceSample;
    use runtime::RT_STORAGE_SIZE;

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
    #[repr(transparent)]
    struct HostRtStorage(UnsafeCell<[u8; RT_STORAGE_SIZE]>);
    // SAFETY: see runtime_handle_create's SAFETY comment.
    #[cfg(not(target_os = "none"))]
    unsafe impl Sync for HostRtStorage {}
    // Lowercase to match the C-side `uint8_t rt_storage[N]` symbol that
    // the MCU extern declaration above resolves to. The same identifier
    // must work in both modes.
    #[cfg(not(target_os = "none"))]
    #[allow(non_upper_case_globals)]
    static rt_storage: HostRtStorage =
        HostRtStorage(UnsafeCell::new([0u8; RT_STORAGE_SIZE]));

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
        // Boot-relative widened MCU clock from src/runtime_tick.c —
        // timer_read_time() widened with stats_send_time_high. Same time
        // domain the host uses for seg.t_start.
        fn runtime_widened_host_clock() -> u64;
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
                // Seed widening with the C-side boot-relative widened
                // clock (timer_read_time widened with stats_send_time_high).
                // Earlier seed from `read_widened_now(shared)` returned 0
                // (the seqlock cell stays 0 until the first ISR publish),
                // making widening start at zero while seg.t_start is
                // billions — segments parked forever in pending_segment.
                // Re-attempting after the 01f4090a5 revert; if this trips
                // the previous 4-second-IRQ freeze, the per-stage cycle
                // counters added in this commit (shared.isr_*_cycles_max +
                // isr_overrun_count, emitted as fault_detail tags
                // 0xE6-0xE9) will pinpoint which stage of isr_sample_tick
                // is the spike.
                let last_widened = super::exports::runtime_widened_host_clock();
                widen_state.reinit(raw, last_widened);
                runtime::clock::publish_widened_now(shared, last_widened);
                super::exports::runtime_tick_enable();
            }
        }
        KALICO_OK
    }

    /// Atomic one-shot load of a cubic-piece curve. Spec §3.2 (2026-05-20
    /// stepping-redesign).
    ///
    /// Wire frame body:
    ///   slot_idx u16 (LE), axis_idx u8, piece_count u8,
    ///   pieces: piece_count * 20 bytes, each = 5 × u32 (LE):
    ///     bp0_bits, bp1_bits, bp2_bits, bp3_bits, duration_bits
    ///
    /// Returns `KALICO_OK` on success and writes `(gen << 16) | slot_idx`
    /// into `out_handle_packed`. Validation rejections (zero / oversized
    /// `piece_count`, non-finite control points, slot already in flight)
    /// return `KALICO_ERR_INVALID_CURVE` without mutating the slot.
    ///
    /// `axis_idx` is reserved for future per-axis validation; ignored for
    /// now (the curve pool is a flat slot pool).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_load_curve_cubic(
        rt: *mut KalicoRuntime,
        slot_idx: u16,
        axis_idx: u8,
        piece_count: u8,
        pieces_blob: *const u8,
        out_handle_packed: *mut u32,
    ) -> i32 {
        if rt.is_null() || pieces_blob.is_null() || out_handle_packed.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        use runtime::cubic_curve::WirePiece;
        use runtime::curve_pool::MAX_PIECES_PER_CURVE;
        if piece_count == 0 || piece_count as usize > MAX_PIECES_PER_CURVE {
            return KALICO_ERR_INVALID_CURVE;
        }
        let _ = axis_idx; // reserved for future per-axis validation
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null and INIT_DONE=true above. The CurvePool lives
        // at the top of RuntimeContext; foreground writes the slot via
        // `try_alloc_and_load`'s atomic gen-bump under the existing
        // foreground-sole-writer discipline (§10.2 / curve_pool.rs).
        // `pieces_blob` must be valid for `piece_count * 20` bytes per the
        // caller's contract (the C dispatch handler reads the same bounds
        // from the wire frame).
        unsafe {
            let pool_ptr: *const CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
            let pool: &CurvePool = &*pool_ptr;

            // Decode the wire blob into a stack-local WirePiece array. Each
            // piece is 5 little-endian u32 words.
            let mut wire: [WirePiece; MAX_PIECES_PER_CURVE] = [WirePiece {
                bp0_bits: 0,
                bp1_bits: 0,
                bp2_bits: 0,
                bp3_bits: 0,
                duration_bits: 0,
            }; MAX_PIECES_PER_CURVE];
            for i in 0..piece_count as usize {
                let base = pieces_blob.add(i * 20);
                let mut buf = [0u8; 20];
                core::ptr::copy_nonoverlapping(base, buf.as_mut_ptr(), 20);
                wire[i].bp0_bits = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                wire[i].bp1_bits = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                wire[i].bp2_bits = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
                wire[i].bp3_bits =
                    u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
                wire[i].duration_bits =
                    u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]);
            }
            let wire_slice = &wire[..piece_count as usize];
            match pool.try_alloc_and_load(slot_idx as usize, wire_slice) {
                Some(handle) => {
                    *out_handle_packed = handle.pack();
                    KALICO_OK
                }
                None => KALICO_ERR_INVALID_CURVE,
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

    /// Stepping-redesign Task 17 — TIM5 ISR body.
    ///
    /// Drives the unified per-sample evaluator
    /// [`runtime::tick::runtime_tick_sample`] over the engine's
    /// `stepping_axes` / `tick_caches` and the runtime's shared state.
    /// Projects a `&mut IsrState` (engine half) and a `&SharedState`
    /// (cross-half) out of `RuntimeContext` exactly once per ISR fire.
    ///
    /// Called from `TIM5_IRQHandler` in `src/stm32/runtime_tick_{h7,f4}.c`
    /// at the rate configured by `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`.
    /// Replaces the prior `kalico_runtime_modulated_tick` entry point
    /// removed in the 2026-05-20 stepping redesign.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick_sample(rt: *mut KalicoRuntime) {
        if rt.is_null() {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `rt` non-null and INIT_DONE=true. TIM5 is the SOLE
        // writer of `IsrState`, so the half-split borrow (engine + queue
        // consumer + widen state via IsrState; shared state via `&`) is
        // sound under §11.1. Codex M1+M2 (2026-05-20) moved the dequeue +
        // arm + widen+publish steps INTO this ISR body — see
        // `runtime::tick::isr_sample_tick` for the unified per-sample
        // entry. Reading `runtime_cyccnt_read` here keeps the raw-DWT read
        // adjacent to the widen-then-publish step the engine's
        // `widened_now_lo.load()` depends on.
        unsafe {
            let raw = runtime_cyccnt_read();
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState =
                core::ptr::addr_of!((*ctx).shared);
            let pool_ptr: *const CurvePool =
                core::ptr::addr_of!((*ctx).curve_pool);
            let isr: &mut IsrState = &mut *isr_ptr;
            let shared: &SharedState = &*shared_ptr;
            let curve_pool: &CurvePool = &*pool_ptr;
            runtime::tick::isr_sample_tick(isr, shared, curve_pool, raw);
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

    // ---- Bench bring-up diagnostic (2026-05-21) ---------------------------
    //
    // 2026-05-21 bench reported `queue_depth=2, engine_status=Idle,
    // current_segment_id=0` after a jog — segments arrived on the MCU but
    // the engine never armed. Diagnosis collapses to four hypotheses,
    // distinguished by three independent observables exposed here:
    //
    //   * `kalico_runtime_get_tick_counter` — `Engine::tick_counter` snapshot.
    //     Now actually increments per ISR call (see
    //     `runtime::tick::isr_sample_tick`); zero means TIM5 is not firing
    //     or `kalico_runtime_tick_sample` early-exits at the null/INIT_DONE
    //     guards.
    //   * `kalico_runtime_pending_segment_is_some` — whether the ISR
    //     dequeued a segment but parked it because `seg.t_start > now`.
    //     Non-zero with `current_segment_id == 0` means widened_now is
    //     stale or the host's arm_lead is enormous.
    //   * `kalico_runtime_queue_consumer_dequeues_lo` — low 32 bits of
    //     `SharedState::producer_segment_dequeued_total`. Bumped in
    //     `isr_sample_tick` after each successful `queue_consumer.dequeue()`.
    //     Zero with tick_counter > 0 means the ISR fires but
    //     `queue_consumer.dequeue()` returns None despite C-side
    //     `kalico_native_queue_len() > 0` — the C/Rust queue sync bug
    //     pattern.
    //
    // These three accessors compose into the C-side diag tag 0xE3 added to
    // `src/runtime_tick.c`'s `runtime_status_drain` rotation.

    /// Read whether the ISR has a parked-and-waiting segment in
    /// `IsrState::pending_segment` (returns 1) or not (returns 0). Used by
    /// the 0xE3 diag tag to disambiguate "queue stuck" vs "park stuck"
    /// failure modes on the bench. Read-only access through the §11.1
    /// shared-borrow discipline — `pending_segment` is mutated only by
    /// `isr_sample_tick`, but the foreground status_drain reads the
    /// `Option` discriminant byte directly under the same precondition
    /// `runtime_handle_tick_counter` uses (no concurrent mutation while
    /// status_drain executes from the host-task context).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_pending_segment_is_some(
        rt: *mut KalicoRuntime,
    ) -> u8 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: same shared-borrow contract as `runtime_handle_tick_counter`
        // above — read-only access to ISR-owned state. The `is_some()` read
        // is a single byte load of the `Option` discriminant; non-atomic
        // but tolerable for a diagnostic that may race the ISR by one tick.
        unsafe {
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            u8::from((*isr_ptr).pending_segment.is_some())
        }
    }

    /// Low 32 bits of `SharedState::producer_segment_dequeued_total`. The
    /// counter is bumped Acq/Rel in `isr_sample_tick` after every successful
    /// `queue_consumer.dequeue()`. Pair with `kalico_runtime_get_tick_counter`
    /// to distinguish "ISR not firing" from "ISR fires but never dequeues"
    /// in the 0xE3 diag tag.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_queue_consumer_dequeues_lo(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: `SharedState` atomics — Acquire load synchronizes with
        // the Release fetch_add in `isr_sample_tick`.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let full = (*shared_ptr)
                .producer_segment_dequeued_total
                .load(Ordering::Acquire);
            #[allow(clippy::cast_possible_truncation)]
            let lo = full as u32;
            lo
        }
    }

    /// Alias for `runtime_handle_tick_counter` with the
    /// `kalico_runtime_get_*` naming the bench diag rotation uses. Same
    /// underlying read — `Engine::tick_counter.snapshot()`. Returns 0 on a
    /// null handle or before `INIT_DONE`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_tick_counter(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        // SAFETY: delegates to the existing tick-counter accessor; same
        // shared-borrow contract.
        unsafe { runtime_handle_tick_counter(rt) }
    }

    // 2026-05-21 bench diag — per-stage isr_sample_tick cycle counters.
    // Each reads a SharedState atomic populated by `runtime::tick::isr_sample_tick`.
    macro_rules! shared_u32_reader {
        ($fn_name:ident, $field:ident) => {
            #[unsafe(no_mangle)]
            pub unsafe extern "C" fn $fn_name(rt: *mut KalicoRuntime) -> u32 {
                if rt.is_null() {
                    return 0;
                }
                if !INIT_DONE.load(Ordering::Acquire) {
                    return 0;
                }
                let ctx = rt.cast::<RuntimeContext>();
                unsafe {
                    let shared_ptr: *const SharedState =
                        core::ptr::addr_of!((*ctx).shared);
                    (*shared_ptr).$field.load(Ordering::Relaxed)
                }
            }
        };
    }
    shared_u32_reader!(kalico_runtime_get_isr_widen_cycles_max, isr_widen_cycles_max);
    shared_u32_reader!(kalico_runtime_get_isr_arm_cycles_max, isr_arm_cycles_max);
    shared_u32_reader!(kalico_runtime_get_isr_eval_cycles_max, isr_eval_cycles_max);
    shared_u32_reader!(kalico_runtime_get_isr_overrun_count, isr_overrun_count);
    shared_u32_reader!(kalico_runtime_get_isr_deq_some_count, isr_deq_some_count);
    shared_u32_reader!(kalico_runtime_get_isr_deq_none_count, isr_deq_none_count);
    shared_u32_reader!(kalico_runtime_get_isr_parked_count, isr_parked_count);
    shared_u32_reader!(kalico_runtime_get_isr_armed_count, isr_armed_count);
    shared_u32_reader!(kalico_runtime_get_isr_last_t_start_lo, isr_last_t_start_lo);
    shared_u32_reader!(kalico_runtime_get_isr_last_widened_lo, isr_last_widened_lo);
    shared_u32_reader!(kalico_runtime_get_isr_last_p_end_bits, isr_last_p_end_bits);
    shared_u32_reader!(kalico_runtime_get_isr_last_microstep_bits, isr_last_microstep_bits);

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

    /// Extended blob layout (25 bytes) and phase-stepping blob layout
    /// (33 bytes — Task 4 / spec §4.1):
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
    ///             -- present only in phase-stepping (26+3N-byte) format --
    /// byte 25                 phase_motor_count = N (1..=MAX_STEPPER_OIDS)
    /// bytes 26+3i..26+3i+2    motor i: (bus_id, cs_pin_id, slot_idx)
    /// ```
    ///
    /// Legacy hosts emit the 20-byte format; the MCU defaults all steppers to
    /// `StepTime` in that case. Any `blob_len` not in
    /// `{20, 25, 26 + 3·N for 0 <= N <= MAX_STEPPER_OIDS}` is rejected. The
    /// variable-length format requires `step_mode[slot_idx] == Modulated`
    /// for every motor entry (spec §4.1). N is bounded by MAX_STEPPER_OIDS
    /// (16); the earlier audible-band ≤2 cap is no longer enforced here —
    /// per-shared-SPI-bus bandwidth derating is a separate future change.
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
        // Accept 20-byte (legacy), 25-byte (extended with StepMode array),
        // or 26+3·N-byte (variable-length per-motor phase-stepping SPI
        // config; 0 <= N <= MAX_STEPPER_OIDS). Any other length is
        // rejected. The N=0 case (blob_len == 26) is accepted as a
        // "clear all phase config" signal, but practically the host emits
        // a 25-byte body in that case — see motion_toolhead.py.
        let blob_len_usize = blob_len as usize;
        let phase_len_ok = blob_len_usize >= 26
            && (blob_len_usize - 26) % 3 == 0
            && ((blob_len_usize - 26) / 3) <= runtime::state::MAX_STEPPER_OIDS;
        if blob_len != 20 && blob_len != 25 && !phase_len_ok {
            diag_progress(0xCA, 10, blob_len);
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
        // `kalico_configure_axis` commands populates a fresh slate.
        // Without this, the table accumulates across klippy
        // reconnects (the MCU stays powered) and motor 0 / motor 1 hit
        // RUNTIME_MAX_STEPPERS_PER_MOTOR=4 after two reconnects —
        // the third shutdowns with "too many steppers per motor".
        // Bench-observed 2026-05-11 (pre-redesign, where the same
        // failure mode applied to the legacy `config_runtime_stepper`
        // path that `kalico_configure_axis` replaced in Task 21).
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
        // further action is needed for the legacy path. Both the 25-byte
        // extended and 33-byte phase-config blobs carry the StepMode array.
        if blob_len >= 25 {
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

        // --- Variable-length format: parse per-motor phase-stepping SPI
        // config (one entry per phase-stepped motor; topology-agnostic).
        //
        // Layout (3 bytes per motor starting at offset 26):
        //   byte 25                 phase_motor_count = N (0..=MAX_STEPPER_OIDS=16)
        //   bytes 26+3i..26+3i+2    motor i: (bus_id, cs_pin_id, slot_idx)
        //
        // `bus_id == 0xFF` is a sentinel marking "motor entry intentionally
        // empty" (rare — the host emits dense entries). Each non-sentinel
        // entry's slot_idx must be < 4 (kinematic-slot count) and
        // `step_mode[slot_idx]` must be `Modulated` (validated against the
        // step_modes already-installed above).
        //
        // Errors do NOT clear previously-stored slots — the runtime is
        // single-shot configured before TIM5 runs, so a rejected blob
        // leaves the runtime in its pre-call state for the host to inspect
        // and retry with a corrected blob.
        if blob_len_usize >= 26 {
            let count = blob[25] as usize;
            let expected_len = 26 + 3 * count;
            if blob_len_usize != expected_len
                || count > runtime::state::MAX_STEPPER_OIDS
            {
                diag_progress(0xCA, 10, blob_len);
                return KALICO_ERR_INVALID_KINEMATICS;
            }
            // Pre-validate per-entry before mutating SharedState.
            let mut parsed: [Option<(runtime::phase_config::PhaseConfig, u8)>;
                runtime::state::MAX_STEPPER_OIDS] =
                [None; runtime::state::MAX_STEPPER_OIDS];
            for i in 0..count {
                let off = 26 + i * 3;
                let bus = blob[off];
                let cs = blob[off + 1];
                let slot_idx = blob[off + 2];
                if bus == 0xFF {
                    // Sentinel entries inside count are allowed but skipped.
                    parsed[i] = None;
                    continue;
                }
                if slot_idx >= 4 {
                    diag_progress(0xCA, 11, slot_idx as u32);
                    return KALICO_ERR_INVALID_KINEMATICS;
                }
                // step_mode[slot_idx] must be Modulated. The step_modes
                // bytes from the same blob were just installed above;
                // check the source byte directly so the rejection is
                // independent of SharedState mutation order.
                let mode_byte = blob[21 + slot_idx as usize];
                if mode_byte != (runtime::state::StepMode::Modulated as u8) {
                    diag_progress(0xCA, 11, slot_idx as u32);
                    return KALICO_ERR_INVALID_KINEMATICS;
                }
                parsed[i] = Some((
                    runtime::phase_config::PhaseConfig {
                        spi_bus_id: bus,
                        cs_pin_id: cs,
                    },
                    slot_idx,
                ));
            }
            // Commit. The per-motor count is also stored so the ISR can
            // loop 0..count rather than scanning all MAX_STEPPER_OIDS
            // entries.
            unsafe {
                let shared_ptr: *const runtime::state::SharedState =
                    core::ptr::addr_of!((*ctx).shared);
                let shared: &runtime::state::SharedState = &*shared_ptr;
                for i in 0..runtime::state::MAX_STEPPER_OIDS {
                    if i < count {
                        if let Some(slot) = shared.phase_config.get(i) {
                            runtime::phase_config::store(
                                slot,
                                parsed[i].map(|(c, _)| c),
                            );
                        }
                        if let Some(s) = shared.phase_slot_idx.get(i) {
                            s.store(
                                parsed[i].map(|(_, idx)| idx).unwrap_or(0xFF),
                                Ordering::Release,
                            );
                        }
                    } else {
                        // Clear unused motor entries past `count` so a
                        // shrinking re-configure can't leave stale slots.
                        if let Some(slot) = shared.phase_config.get(i) {
                            runtime::phase_config::store(slot, None);
                        }
                        if let Some(s) = shared.phase_slot_idx.get(i) {
                            s.store(0xFF, Ordering::Release);
                        }
                    }
                }
                shared.phase_motor_count.store(count as u8, Ordering::Release);
            }
            diag_progress(0xCA, 13, count as u32);
        } else if blob_len == 25 {
            // step_modes only, no phase config — clear any prior phase
            // state so a "configure axes without phase stepping" call
            // doesn't leave stale phase config behind.
            unsafe {
                let shared_ptr: *const runtime::state::SharedState =
                    core::ptr::addr_of!((*ctx).shared);
                let shared: &runtime::state::SharedState = &*shared_ptr;
                for i in 0..runtime::state::MAX_STEPPER_OIDS {
                    if let Some(slot) = shared.phase_config.get(i) {
                        runtime::phase_config::store(slot, None);
                    }
                    if let Some(s) = shared.phase_slot_idx.get(i) {
                        s.store(0xFF, Ordering::Release);
                    }
                }
                shared.phase_motor_count.store(0, Ordering::Release);
            }
        }
        // blob_len == 20 (legacy) doesn't touch phase config; left as-is.

        // 2026-05-19: TIM5 is no longer armed here. configure_axes_blob now
        // sets up state but leaves the phase-stepping ISR off; the first
        // successful `push_segment_impl` arms TIM5 (it's idempotent on the
        // C side after the 2026-05-19 guard). Pre-fix, arming at
        // configure_axes meant the ISR fired at 40 kHz writing zero-delta
        // XDIRECT for the entire idle period between config and first
        // motion, starving USB CDC and causing "No such device"
        // disconnects within ~1 s of any sustained load.

        KALICO_OK
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
    /// single-threaded-foreground precondition as `configure_axes_blob`.
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
        // we project `&mut IsrState` under the same single-threaded-foreground
        // precondition documented for `configure_axes_blob` — no ISR is running
        // when the host sends this command (it arrives before the first
        // PushSegment). No other `&mut IsrState` or `&mut FgState` may be live
        // on this call path.
        unsafe {
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.seed_position([x, y, z]);
        }
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

    /// Flip the `phase_trace_enabled` gate (2026-05-18 plan Task 5).
    ///
    /// When enabled, `runtime_tick_sample` pushes one
    /// `TRACE_FLAG_PHASE_STEP`-flagged `TraceSample` per
    /// phase-stepping tick per motor (Task 6 wiring). Default is `false`;
    /// production builds leave it off so the trace ring isn't burned by
    /// the 80 kHz per-motor PhaseStep stream when no diagnostic is active.
    ///
    /// `enabled`: non-zero → true, zero → false. The store uses `Release`
    /// ordering; the ISR-side load pairs with `Acquire`.
    ///
    /// Returns:
    /// - `KALICO_OK` on success.
    /// - `KALICO_ERR_NULL_PTR` if `rt` is null.
    /// - `KALICO_ERR_NOT_INIT` if the runtime has not been initialised.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_set_phase_trace_enabled(
        rt: *mut KalicoRuntime,
        enabled: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: SharedState is atomics-only; no `&mut` is formed. The
        // `phase_trace_enabled` atomic is designed for foreground writes
        // and ISR reads, same discipline as `step_modes` / `phase_config`.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            shared
                .phase_trace_enabled
                .store(enabled != 0, Ordering::Release);
        }
        KALICO_OK
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


    /// 2026-05-18 wedge diag: live snapshot of `queue_consumer.len()` —
    /// the SPSC's view of how many segments are queued and visible to the
    /// Consumer. Cross-check against `accepted_segment_id -
    /// retired_through_segment_id` (host's view of queue depth):
    ///   - queue.len() == queue_depth → SPSC is consistent; bug elsewhere.
    ///   - queue.len() < queue_depth  → SPSC's Consumer can't see all the
    ///                                   Producer's enqueued segments
    ///                                   (memory visibility / write-buffer
    ///                                   / cache issue, or queue corrupted).
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_queue_len_diag(
        rt: *mut KalicoRuntime,
    ) -> u32 {
        if rt.is_null() { return 0; }
        if !INIT_DONE.load(Ordering::Acquire) { return 0; }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: §11.1 — foreground sole access to IsrState here.
        // `Consumer::len()` reads atomics via &self, no mutation.
        unsafe {
            let isr_ptr: *const IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let isr: &IsrState = &*isr_ptr;
            isr.queue_consumer.len() as u32
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
            unsafe { rt_storage.get().cast::<RuntimeContext>() }
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
            return 0;
        };
        // SAFETY: `rt_ptr` is the published rt_storage projection, valid
        // for the lifetime of the program once INIT_DONE is set. Read-only
        // access to `SharedState` atomics via a shared `&` projection — no
        // `&mut` reaches this path.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr).dispatcher_floor_cycles.load(Ordering::Acquire)
        }
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
            return 0;
        };
        // SAFETY: see `kalico_runtime_get_dispatcher_floor_cycles`.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr).sample_period_cycles.load(Ordering::Acquire)
        }
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
    // `kalico_runtime_seed_position` and `kalico_runtime_configure_axes_blob`
    // rely on.

    /// Stepping-redesign Task 14. Publish per-axis configuration with
    /// explicit stepper bindings. `microstep_distance_f32_bits` is
    /// `f32::to_bits` of the per-step distance (Klipper carries f32 as u32
    /// on the wire). `mode` is `0` for Pulse; `1` for Phase — Phase is
    /// currently rejected with `KALICO_ERR_PHASE_MODE_NOT_AVAILABLE` (the
    /// SPI dispatch path is a follow-up task). Other mode values return
    /// `KALICO_ERR_INVALID_ARG`. `bindings_ptr` points to an array of
    /// `stepper_count` [`runtime::stepping_state::StepperBindingRust`]
    /// entries; a null pointer with `stepper_count == 0` is legal (axis
    /// with no steppers, e.g. virtual / logical-only). Returns `0` on
    /// success, negative on validation failure. The C handler treats any
    /// non-zero return as a hard error and shuts the MCU down.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_configure_axis(
        rt: *mut KalicoRuntime,
        axis_idx: u8,
        mode: u8,
        microstep_distance_f32_bits: u32,
        bindings_ptr: *const runtime::stepping_state::StepperBindingRust,
        stepper_count: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        // Phase mode is not yet available — reject at the FFI boundary so
        // the C host sees KALICO_ERR_PHASE_MODE_NOT_AVAILABLE rather than
        // a generic invalid-arg code. The engine method also guards this,
        // but checking here avoids decoding the bindings slice for a mode
        // the engine will reject regardless.
        if mode == 1 {
            return KALICO_ERR_PHASE_MODE_NOT_AVAILABLE;
        }
        let mode_enum = match mode {
            0 => runtime::stepping_state::StepMode::Pulse,
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
            unsafe {
                core::slice::from_raw_parts(bindings_ptr, stepper_count as usize)
            }
        };
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only entry; spec §11.2 raw-pointer projection.
        // Engine state is on `IsrState`; foreground may form `&mut
        // IsrState` here under the precondition that TIM5 is not
        // concurrently ticking the same per-axis state (Klipper command
        // dispatch is single-threaded and serialised against the modulated
        // tick by priority arbitration during configuration windows).
        unsafe {
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr)
                .engine
                .configure_axis(axis_idx, mode_enum, mstep_dist, bindings)
        }
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
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
            let isr_ptr: *mut IsrState =
                UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared_ptr: *const SharedState =
                core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            (*isr_ptr).engine.set_stepper_offset(
                shared,
                stepper_idx,
                delta_microsteps,
                max_microsteps_per_sample,
            )
        }
    }

}
