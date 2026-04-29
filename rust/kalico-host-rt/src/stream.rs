//! Host-side stream lifecycle. Spec §6.3, §6.4 + Plan-decision B.
//!
//! Implements [`arm_all_mcus`]: the multi-MCU ARMING sequence that
//! 1. issues an explicit `kalico_clock_sync_request` per MCU (spec
//!    §12.3 / Plan-decision B);
//! 2. updates each MCU's [`ClockSyncEstimator`] with the RTT-aware
//!    sample;
//! 3. runs the §12.4 quality gate (which Plan-decision B extends with
//!    the dedicated-sample-fresh check);
//! 4. computes per-MCU `t_start_local` from the regression anchor (B11
//!    fix);
//! 5. issues `kalico_stream_arm` and waits for the ack with an arming
//!    deadline (Round-1 ARMING-race protection: total budget is
//!    `arm_lead_time / 2`).
//!
//! On any failure we abort the in-progress arm and surface the error;
//! the caller is responsible for calling `kalico_stream_flush` to wind
//! the MCU back to IDLE.

use std::time::{Duration, Instant};

use crate::clock_sync::ClockSyncEstimator;
use crate::transport::{Transport, TransportError};

/// Default timeout for the dedicated `kalico_clock_sync_request`
/// round-trip during ARMING. Sized loosely vs the expected µs RTT so a
/// stalled link surfaces as `ArmError::Transport(Timeout)` rather than
/// a quality-gate failure.
pub const CLOCK_SYNC_REQUEST_TIMEOUT: Duration = Duration::from_millis(50);

/// ARMING flow per spec §6.3 + §6.4 + Plan-decision B.
///
/// `mcus` is `&mut [(T, ClockSyncEstimator)]` — each tuple is a single
/// MCU's transport + estimator. The function takes a mutable slice
/// (rather than a `Vec<&mut T>`) so callers can keep the estimators
/// owned alongside their transports without lifetime gymnastics.
///
/// `t_start_wall_clock` is the absolute host-wall instant at which the
/// stream should begin streaming. `arm_lead_time` is the total ARMING
/// budget (spec §6.4); the function uses `arm_lead_time / 2` as its
/// deadline so half the budget remains for in-flight wire latency.
///
/// `arm_lead_cycles` is forwarded verbatim to the MCU's
/// `kalico_stream_arm` (spec §6.4 — number of MCU cycles between arm
/// and `t_start` to absorb any host-side jitter on the arm command).
///
/// `baseline_freq` is the per-MCU `CONFIG_CLOCK_FREQ` used by the
/// quality gate's drift-ppm threshold.
pub fn arm_all_mcus<T: Transport>(
    mcus: &mut [(T, ClockSyncEstimator)],
    t_start_wall_clock: Instant,
    arm_lead_time: Duration,
    arm_lead_cycles: u32,
    baseline_freq: f64,
) -> Result<(), ArmError> {
    let arming_deadline = Instant::now() + arm_lead_time / 2;

    // Step 1+2+3: dedicated sync + quality gate per MCU.
    for (io, est) in mcus.iter_mut() {
        if Instant::now() >= arming_deadline {
            return Err(ArmError::DeadlineMissed);
        }
        let host_send = Instant::now();
        // Round-2 B04 carry-over: the `host_send_time_*` args are
        // back-trace request_id values; the MCU echoes them. We send
        // zero here because the estimator independently records
        // `host_send` and `host_recv` instants.
        io.send(
            "kalico_clock_sync_request request_id=1 host_send_time_lo=0 host_send_time_hi=0",
        )?;
        let resp = io.wait_for_response(
            "kalico_clock_sync_response",
            CLOCK_SYNC_REQUEST_TIMEOUT,
        )?;
        let host_recv = Instant::now();
        let mcu_clock = (u64::from(resp.get_u32("mcu_clock_hi")) << 32)
            | u64::from(resp.get_u32("mcu_clock_lo"));
        est.add_dedicated_sample(host_send, host_recv, mcu_clock);

        if !est.is_quality_gate_passed(baseline_freq) {
            return Err(ArmError::QualityGate);
        }
    }

    // Step 4+5: arm each MCU with deadline.
    //
    // Round-2 fix B11-real: per-MCU `t_start_local` MUST be the
    // absolute MCU-clock value at wall-time `t_start_wall_clock`,
    // NOT just `delta_secs * freq`. We compute via the estimator's
    // anchor: t_start_local =
    // mcu_time_at_host(host_time_secs(t_start_wall_clock)).
    for (io, est) in mcus.iter_mut() {
        if Instant::now() >= arming_deadline {
            return Err(ArmError::DeadlineMissed);
        }
        let t_start_host_secs = est.host_time_at(t_start_wall_clock);
        let t_start_local = est.mcu_time_at_host(t_start_host_secs);
        let cmd = format!(
            "kalico_stream_arm t_start_t0_lo={lo} t_start_t0_hi={hi} arm_lead_cycles={alc}",
            lo = t_start_local as u32,
            hi = (t_start_local >> 32) as u32,
            alc = arm_lead_cycles,
        );
        io.send(&cmd)?;
        let now = Instant::now();
        if now >= arming_deadline {
            return Err(ArmError::DeadlineMissed);
        }
        let remaining = arming_deadline - now;
        let resp = io.wait_for_response("kalico_stream_arm_response", remaining)?;
        if resp.get_i32("result") != 0 {
            return Err(ArmError::McuRejected(resp.get_i32("result")));
        }
    }

    Ok(())
}

#[derive(Debug)]
pub enum ArmError {
    /// `arm_lead_time / 2` elapsed before all MCUs were armed.
    DeadlineMissed,
    /// At least one MCU's [`ClockSyncEstimator`] failed
    /// `is_quality_gate_passed` after the dedicated sync.
    QualityGate,
    /// `kalico_stream_arm_response.result != 0`.
    McuRejected(i32),
    /// Transport-layer failure during ARMING.
    Transport(TransportError),
}

impl From<TransportError> for ArmError {
    fn from(e: TransportError) -> Self {
        ArmError::Transport(e)
    }
}

impl std::fmt::Display for ArmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ArmError::DeadlineMissed => write!(f, "ARMING deadline missed"),
            ArmError::QualityGate => write!(
                f,
                "ARMING aborted: clock-sync quality gate failed (Plan-decision B)"
            ),
            ArmError::McuRejected(r) => {
                write!(f, "ARMING aborted: MCU rejected stream_arm (result={r})")
            }
            ArmError::Transport(e) => write!(f, "ARMING transport error: {e}"),
        }
    }
}

impl std::error::Error for ArmError {}
