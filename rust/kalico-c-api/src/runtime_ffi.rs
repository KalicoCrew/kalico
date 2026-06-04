// Every FFI entry projects to &mut FgState or &mut IsrState (disjoint memory) via
// core::ptr::addr_of! + UnsafeCell::raw_get; no &mut RuntimeContext is ever materialised.
// See docs/kalico-rewrite/mcu-c-rust-boundary.md.

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

    #[allow(missing_debug_implementations)]
    #[repr(C)]
    pub struct KalicoRuntime {
        _private: [u8; 0],
    }

    #[cfg(target_os = "none")]
    unsafe extern "C" {
        static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>;
    }

    #[cfg(not(target_os = "none"))]
    #[repr(C, align(16))]
    struct HostRtStorage(UnsafeCell<[u8; RT_STORAGE_SIZE]>);
    // SAFETY: half-split aliasing + INIT_DONE guard ensure no concurrent &mut; UnsafeCell::raw_get is the only access path.
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

    pub(super) static INIT_DONE: AtomicBool = AtomicBool::new(false);

    unsafe extern "C" {
        fn runtime_cyccnt_read() -> u32;
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn runtime_handle_create() -> *mut KalicoRuntime {
        // Plain store not compare_exchange: Renode H7 v1.16 silently drops STREXB, leaving INIT_DONE=0 after CAS succeeds in code.
        if INIT_DONE.load(Ordering::Relaxed) {
            return core::ptr::null_mut();
        }
        // SAFETY: single-threaded init; no other context observes rt_storage until INIT_DONE is published. rt_storage provenance covers the full buffer; RuntimeContext fits (const_assert above).
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
            INIT_DONE.store(true, Ordering::Release);
            rt_ptr.cast::<KalicoRuntime>()
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_check_blob_version(
        payload_ptr: *const u8,
        payload_len: u32,
    ) -> i32 {
        if payload_ptr.is_null() || payload_len == 0 {
            return KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED;
        }
        // SAFETY: caller contracts payload_ptr is valid for payload_len bytes.
        let blob = unsafe { core::slice::from_raw_parts(payload_ptr, payload_len as usize) };
        match runtime::wire::check_version(blob) {
            Ok(()) => KALICO_OK,
            Err(_) => KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED,
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_tick_sample(rt: *mut KalicoRuntime) {
        if rt.is_null() {
            return;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: rt non-null, INIT_DONE=true. TIM5 is the sole writer of IsrState; ISR owns ring tail, foreground writes only HEAD positions not yet seen by ISR. UnsafeCell::raw_get yields provenance without a shared ref.
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
        // SAFETY: read-only SharedState atomics; no &mut.
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
        // SAFETY: read-only SharedState atomics; no &mut.
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
        // SAFETY: ISR is sole writer of IsrState; atomic read is safe from foreground.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.tick_counter()
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_tick_counter(rt: *mut KalicoRuntime) -> u32 {
        unsafe { runtime_handle_tick_counter(rt) }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_widened_now(rt: *mut KalicoRuntime) -> u64 {
        // rt is unused (widening reads Klipper globals), but kept for ABI stability.
        let _ = rt;
        unsafe extern "C" {
            fn timer_read_time() -> u32;
            static stats_send_time: u32;
            static stats_send_time_high: u32;
        }
        // SAFETY: timer_read_time is a u32 MMIO read, safe from non-ISR context. stats_send_time* are u32 globals; torn reads self-correct within ~5 s.
        unsafe {
            let low = timer_read_time();
            let high = stats_send_time_high + ((low < stats_send_time) as u32);
            ((high as u64) << 32) | (low as u64)
        }
    }

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

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_fault_detail(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).fault_detail.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_func.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_pc(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_pc.load(Ordering::Acquire)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn runtime_handle_tick_blocker_exc(rt: *mut KalicoRuntime) -> u32 {
        if rt.is_null() {
            return 0;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return 0;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: read-only SharedState atomics; no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            (*shared_ptr).tick_blocker_exc.load(Ordering::Acquire)
        }
    }

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
        // SAFETY: foreground-only; single-threaded command dispatch, no concurrent TIM5.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.seed_position([x, y, z]);
        }
        KALICO_OK
    }

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
        // SAFETY: rt non-null + INIT_DONE verified; flush() performs its own half-split projections.
        unsafe { runtime::stream::flush(rt.cast::<RuntimeContext>(), out_credit_epoch) }
    }

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
        // SAFETY: single u32 reads of Klipper globals, safe from non-ISR context.
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

    use runtime::endstop::{
        ArmMsg, ArmPolicy, ArmStatus, DisarmStatus, MAX_SOURCES, MAX_STEPPERS, SourceConfig,
        SourceKind, VelocityAxis,
    };

    const SOURCE_RECORD_LEN: usize = 11;
    const STEPPER_RECORD_LEN: usize = 1;

    pub const KALICO_TRIP_EVENT_V1_HEADER_LEN: usize = 15;
    pub const KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN: usize = 5;
    pub const KALICO_TRIP_EVENT_V1_FMT_VERSION: u8 = 1;
    pub const KALICO_TRIP_EVENT_V1_MAX_LEN: usize =
        KALICO_TRIP_EVENT_V1_HEADER_LEN + MAX_STEPPERS * KALICO_TRIP_EVENT_V1_PER_STEPPER_LEN;

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
        let result = runtime::endstop::software_trip(arm_id, clock, &[]);
        let status = match result {
            runtime::endstop::TripResult::Tripped => 0u8,
            runtime::endstop::TripResult::NotArmed => 1u8,
            runtime::endstop::TripResult::WrongArmId => 2u8,
        };
        unsafe { *out_status = status };
        KALICO_OK
    }

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

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_endstop_set_pin_level(gpio: u16, level: u8) -> i32 {
        if runtime::endstop::set_pin_level(gpio, level != 0) {
            KALICO_OK
        } else {
            KALICO_ERR_NULL_PTR
        }
    }

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
        // SAFETY: step_modes are AtomicU8 in SharedState; shared &SharedState, no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match runtime::state::set_step_mode(shared, stepper_idx, mode, mcu_supports_phase != 0)
            {
                Ok(()) => KALICO_OK,
                Err(runtime::state::SetStepModeError::CapabilityMissing) => {
                    KALICO_ERR_CAPABILITY_MISSING
                }
                Err(runtime::state::SetStepModeError::OutOfRange) => KALICO_ERR_INVALID_HANDLE,
            }
        }
    }

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
        // SAFETY: step_modes are AtomicU8 in SharedState; shared &SharedState, no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            shared.step_modes[stepper_idx as usize].load(Ordering::Acquire)
        }
    }

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
        // SAFETY: phase_config is AtomicU16 in SharedState; shared &SharedState, no &mut.
        unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
            let shared: &SharedState = &*shared_ptr;
            match shared.phase_config.get(motor_idx as usize) {
                Some(slot) => slot.load(Ordering::Acquire),
                None => 0xFFFF,
            }
        }
    }

    fn runtime_handle_or_null() -> Option<*const RuntimeContext> {
        if !INIT_DONE.load(Ordering::Acquire) {
            return None;
        }
        #[cfg(target_os = "none")]
        let rt_ptr: *const RuntimeContext = {
            // SAFETY: .get() on C extern static yields a raw pointer without an aliasing ref; gated by INIT_DONE.
            #[allow(unsafe_code)]
            unsafe {
                rt_storage.get().cast::<RuntimeContext>()
            }
        };
        #[cfg(not(target_os = "none"))]
        let rt_ptr: *const RuntimeContext = rt_storage.0.get().cast::<RuntimeContext>();
        Some(rt_ptr)
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_dispatcher_floor_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: rt_storage projection; read-only SharedState atomics, no &mut.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr)
                .dispatcher_floor_cycles
                .load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn kalico_runtime_get_sample_period_cycles() -> u32 {
        let Some(rt_ptr) = runtime_handle_or_null() else {
            return 5_000_000;
        };
        // SAFETY: rt_storage projection; read-only SharedState atomics, no &mut.
        let v = unsafe {
            let shared_ptr: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ptr).sample_period_cycles.load(Ordering::Acquire)
        };
        if v == 0 { 5_000_000 } else { v }
    }

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
        let bindings: &[runtime::stepping_state::StepperBindingRust] = if stepper_count == 0 {
            &[]
        } else if bindings_ptr.is_null() {
            return KALICO_ERR_NULL_PTR;
        } else {
            // SAFETY: caller guarantees bindings_ptr is valid for stepper_count elements; slice borrow does not escape.
            unsafe { core::slice::from_raw_parts(bindings_ptr, stepper_count as usize) }
        };
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; §11.2 raw-pointer projection; command dispatch serialised against TIM5.
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

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_reset(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground under C-side IRQ guard; §11.2 raw-pointer projection.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.reset();
        }
        #[cfg(not(any(test, feature = "host")))]
        runtime::step_queue::reset_all_queues();
        KALICO_OK
    }

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
        // SAFETY: §11.2 foreground-only. ISR pops ring tail; foreground writes slots only, never advances head here. piece_ptr is unaligned (protocol frame offset); PieceEntry has no invalid bit patterns.
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
        // SAFETY: §11.2 foreground-only. ring.head is a plain u32 written only by foreground; on single-core ARMv7E-M exception entry/return are memory barriers — no explicit fence needed.
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
        // SAFETY: foreground-only; §11.2 raw-pointer projection.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.set_axis_mode(axis_idx, new_mode)
        }
    }

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
        // SAFETY: foreground-only; &SharedState borrow is independent of &mut IsrState — SharedState is atomics-only.
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
        // SAFETY: §11.2 shared-borrow on IsrState; retired_counts() reads u32 fields written only by ISR — aligned u32 read is single-instruction on Cortex-M.
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
