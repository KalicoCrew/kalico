use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use kalico_host_rt::host_io::{InterceptorId, KalicoHostIo};
use kalico_host_rt::transport::TransportError;

use crate::homing::HomingSegmentState;

/// Outcome reported by [`run_probe_homing`] on termination.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProbeHomingResult {
    /// The Beacon probe triggered and `software_trip` was fired to the stepper
    /// MCU in the interceptor callback.
    ProbeTriggered = 0,
    /// The homing segment retired (moved past the active segment) without any
    /// probe trigger — the toolhead passed the expected probe position with no
    /// contact.
    SegmentRetired = 1,
    /// No trigger was received within `sensor_fault_timeout`. Indicates a
    /// sensor connectivity or configuration problem.
    SensorFault = 2,
    /// The MCU's internal software deadline expired before the host detected a
    /// trigger. The segment was frozen autonomously by the MCU.
    DeadlineExpired = 3,
}

/// Parameters consumed by [`run_probe_homing`].
#[derive(Debug)]
pub struct ProbeHomingParams {
    /// I/O handle to the Beacon MCU (source of `trsync_state` frames).
    pub beacon_io: Arc<KalicoHostIo>,
    /// I/O handle to the stepper MCU (destination for `runtime_software_trip`
    /// and `runtime_extend_homing_deadline` commands).
    pub stepper_io: Arc<KalicoHostIo>,
    /// OID of the Beacon's trsync object as assigned during identify.
    pub beacon_trsync_oid: u8,
    /// Arm ID that identifies this homing operation to the stepper MCU.
    pub arm_id: u32,
    /// Duration after which, if no probe trigger has been observed, the loop
    /// returns [`ProbeHomingResult::SensorFault`].
    pub sensor_fault_timeout: Duration,
}

/// How often the loop wakes to extend the MCU deadline and poll segment state.
const TICK_INTERVAL: Duration = Duration::from_millis(25);

/// Run the probe homing loop.
///
/// Registers a frame interceptor on `params.beacon_io` for `trsync_state`
/// frames whose OID matches `params.beacon_trsync_oid`. When the interceptor
/// observes `can_trigger == 0` it fires `runtime_software_trip` to the stepper
/// MCU at wire speed and sets an `AtomicBool` flag.
///
/// A blocking loop on the calling thread wakes every 25 ms to:
/// - check the triggered flag,
/// - call [`crate::homing::HomingState::refresh_after_wait`] and inspect the
///   resulting [`HomingSegmentState`],
/// - extend the stepper MCU's homing deadline,
/// - check the sensor-fault timeout.
///
/// The interceptor is unregistered before returning regardless of outcome.
///
/// # Errors
///
/// Returns `Err(TransportError)` if the initial `register_frame_interceptor`
/// call or any subsequent `send_fire_and_forget` fails due to a closed or
/// broken transport.
pub fn run_probe_homing(
    params: &ProbeHomingParams,
    homing: &crate::homing::HomingState,
) -> Result<ProbeHomingResult, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));

    let interceptor_id: InterceptorId = {
        let triggered_clone = Arc::clone(&triggered);
        let stepper_io_clone = Arc::clone(&params.stepper_io);
        let arm_id = params.arm_id;

        params.beacon_io.register_frame_interceptor(
            "trsync_state",
            Some(u32::from(params.beacon_trsync_oid)),
            Box::new(move |msg_params| {
                let can_trigger = msg_params.get_u32("can_trigger");
                if can_trigger == 0 {
                    let cmd = format!("runtime_software_trip arm_id={arm_id}");
                    let _ = stepper_io_clone.send_fire_and_forget(&cmd);
                    triggered_clone.store(true, Ordering::Release);
                }
            }),
        )?
    };

    // Send the first deadline extension immediately so the MCU does not expire
    // during the first tick interval.
    let extend_cmd = format!("runtime_extend_homing_deadline arm_id={}", params.arm_id);
    params.stepper_io.send_fire_and_forget(&extend_cmd)?;

    let result = run_loop(params, homing, &triggered, &extend_cmd);

    // Always unregister, ignoring errors (transport may already be closed).
    let _ = params.beacon_io.unregister_frame_interceptor(interceptor_id);

    result
}

/// Inner blocking loop; separated from [`run_probe_homing`] so the interceptor
/// is always unregistered after the loop exits regardless of how it exits.
fn run_loop(
    params: &ProbeHomingParams,
    homing: &crate::homing::HomingState,
    triggered: &AtomicBool,
    extend_cmd: &str,
) -> Result<ProbeHomingResult, TransportError> {
    let start = Instant::now();

    loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();

        // Check interceptor flag first — highest-priority path.
        if triggered.load(Ordering::Acquire) {
            log::info!(
                "[probe-homing] probe triggered elapsed={:.3}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::ProbeTriggered);
        }

        // Poll the homing segment state for MCU-side terminals.
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
                // Tripped or any other terminal variant maps to ProbeTriggered
                // because the MCU-side trip was caused by the software_trip we
                // sent from the interceptor callback.
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

        // Sensor-fault guard: no trigger after the configured timeout.
        if elapsed > params.sensor_fault_timeout {
            log::error!(
                "[probe-homing] SENSOR FAULT: no trigger after {:.1}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::SensorFault);
        }

        // Still active — extend the MCU deadline for the next tick.
        params.stepper_io.send_fire_and_forget(extend_cmd)?;
    }
}
