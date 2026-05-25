use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use kalico_host_rt::host_io::{InterceptorId, KalicoHostIo};
use kalico_host_rt::transport::TransportError;

use crate::homing::HomingSegmentState;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProbeHomingResult {
    ProbeTriggered = 0,
    SegmentRetired = 1,
    SensorFault = 2,
    DeadlineExpired = 3,
}

const TICK_INTERVAL: Duration = Duration::from_millis(25);

/// Opaque handle returned by [`prepare_probe_homing`].  Holds the
/// interceptor registration and the shared trigger flag.  Must be
/// passed to [`run_probe_homing`] to enter the homing loop, and
/// cleaned up by [`cleanup_probe_homing`] afterwards.
pub struct ProbeHomingHandle {
    pub(crate) triggered: Arc<AtomicBool>,
    pub(crate) interceptor_id: InterceptorId,
    pub(crate) beacon_io: Arc<KalicoHostIo>,
    pub(crate) stepper_io: Arc<KalicoHostIo>,
    pub(crate) arm_id: u32,
    pub(crate) sensor_fault_timeout: Duration,
}

/// Phase 1: register the interceptor on the Beacon reactor.
///
/// Call this BEFORE `home_start()` sends `beacon_home` to the Beacon
/// MCU, so the interceptor is in place when the probe triggers.
pub fn prepare_probe_homing(
    beacon_io: Arc<KalicoHostIo>,
    stepper_io: Arc<KalicoHostIo>,
    beacon_trsync_oid: u8,
    arm_id: u32,
    sensor_fault_timeout: Duration,
) -> Result<ProbeHomingHandle, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));

    let interceptor_id = {
        let triggered_clone = Arc::clone(&triggered);
        let stepper_io_clone = Arc::clone(&stepper_io);

        beacon_io.register_frame_interceptor(
            "trsync_state",
            Some(u32::from(beacon_trsync_oid)),
            Box::new(move |msg_params| {
                let can_trigger = msg_params.get_u32("can_trigger");
                if can_trigger != 0 {
                    return;
                }
                // Only fire on ENDSTOP_HIT (reason 1). Ignore HOST_REQUEST
                // (reason 2) which is the stale trsync_trigger cleanup from
                // a previous homing pass.
                let reason = msg_params.get_u32("trigger_reason");
                if reason != 1 {
                    return;
                }
                let cmd = format!("runtime_software_trip arm_id={arm_id}");
                let _ = stepper_io_clone.send_fire_and_forget(&cmd);
                triggered_clone.store(true, Ordering::Release);
            }),
        )?
    };

    Ok(ProbeHomingHandle {
        triggered,
        interceptor_id,
        beacon_io,
        stepper_io,
        arm_id,
        sensor_fault_timeout,
    })
}

/// Phase 2: enter the blocking homing loop.
///
/// Call AFTER the homing move has been submitted.  Sends an immediate
/// deadline extension, then loops at 25 ms checking for the trigger
/// flag, segment retirement, or sensor-fault timeout.
pub fn run_probe_homing(
    handle: &ProbeHomingHandle,
    homing: &crate::homing::HomingState,
) -> Result<ProbeHomingResult, TransportError> {
    let extend_cmd = format!(
        "runtime_extend_homing_deadline arm_id={}",
        handle.arm_id
    );
    handle.stepper_io.send_fire_and_forget(&extend_cmd)?;

    run_loop(handle, homing, &extend_cmd)
}

/// Phase 3: unregister the interceptor.  Always call this, even on
/// error (the handle borrows the Beacon I/O, so it must be cleaned up
/// before the next homing cycle can register a new interceptor for the
/// same OID).
pub fn cleanup_probe_homing(handle: ProbeHomingHandle) {
    let _ = handle.beacon_io.unregister_frame_interceptor(handle.interceptor_id);
}

fn run_loop(
    handle: &ProbeHomingHandle,
    homing: &crate::homing::HomingState,
    extend_cmd: &str,
) -> Result<ProbeHomingResult, TransportError> {
    let start = Instant::now();

    loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();

        if handle.triggered.load(Ordering::Acquire) {
            log::info!(
                "[probe-homing] probe triggered elapsed={:.3}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::ProbeTriggered);
        }

        homing.refresh_after_wait();
        let state = homing.state();

        if matches!(
            state,
            HomingSegmentState::Tripped | HomingSegmentState::DeadlineExpired
        ) {
            log::info!(
                "[probe-homing] segment terminal state={:?} elapsed={:.3}s",
                state,
                elapsed.as_secs_f64(),
            );
            return Ok(match state {
                HomingSegmentState::DeadlineExpired => ProbeHomingResult::DeadlineExpired,
                _ => ProbeHomingResult::ProbeTriggered,
            });
        }

        if state == HomingSegmentState::Completed {
            log::info!(
                "[probe-homing] segment retired (no trigger) elapsed={:.3}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::SegmentRetired);
        }

        if elapsed > handle.sensor_fault_timeout {
            log::error!(
                "[probe-homing] SENSOR FAULT: no trigger after {:.1}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::SensorFault);
        }

        handle.stepper_io.send_fire_and_forget(extend_cmd)?;
    }
}
