use std::time::{Duration, Instant};

use crate::clock_sync::ClockSyncEstimator;
use crate::transport::{Transport, TransportError};

pub const MAX_CROSS_MCU_FREQ_RATIO_OFFSET: f64 = 1e-3;

pub fn check_cross_mcu_desync(freqs: &[f64]) -> Option<(usize, usize, f64)> {
    for i in 0..freqs.len() {
        for j in (i + 1)..freqs.len() {
            let fa = freqs[i];
            let fb = freqs[j];
            if fa <= 0.0 || fb <= 0.0 {
                continue;
            }
            let ratio_offset = (fa / fb - 1.0).abs();
            if ratio_offset > MAX_CROSS_MCU_FREQ_RATIO_OFFSET {
                return Some((i, j, ratio_offset));
            }
        }
    }
    None
}

pub const CLOCK_SYNC_REQUEST_TIMEOUT: Duration = Duration::from_millis(50);

pub fn arm_all_mcus<T: Transport>(
    mcus: &mut [(T, ClockSyncEstimator)],
    t_start_wall_clock: Instant,
    arm_lead_time: Duration,
    arm_lead_cycles: u32,
    baseline_freq: f64,
) -> Result<(), ArmFailure> {
    let arming_deadline = Instant::now() + arm_lead_time / 2;

    let mut armed_indices: Vec<usize> = Vec::with_capacity(mcus.len());
    let fail = |error: ArmError, armed: &[usize]| ArmFailure {
        error,
        armed_indices: armed.to_vec(),
    };

    for (idx, (io, est)) in mcus.iter_mut().enumerate() {
        if Instant::now() >= arming_deadline {
            return Err(fail(ArmError::DeadlineMissed, &armed_indices));
        }
        let host_send = Instant::now();
        let request_id = est.next_clock_sync_request_id();
        let resp = io
            .call(
                &format!("runtime_clock_sync_request request_id={request_id} host_send_time_lo=0 host_send_time_hi=0"),
                "kalico_clock_sync_response",
                CLOCK_SYNC_REQUEST_TIMEOUT,
            )
            .map_err(|e| fail(ArmError::Transport(e), &armed_indices))?;
        let host_recv = Instant::now();

        let echoed = resp.try_get_u32("request_id").ok_or_else(|| {
            fail(
                ArmError::Transport(TransportError::Parse(
                    "kalico_clock_sync_response missing request_id field".into(),
                )),
                &armed_indices,
            )
        })?;
        if echoed != request_id {
            return Err(fail(
                ArmError::Transport(TransportError::Parse(format!(
                    "clock_sync request_id mismatch: sent {request_id}, got {echoed}"
                ))),
                &armed_indices,
            ));
        }

        let mcu_clock = (u64::from(resp.get_u32("mcu_clock_hi")) << 32)
            | u64::from(resp.get_u32("mcu_clock_lo"));
        est.add_dedicated_sample(host_send, host_recv, mcu_clock);

        est.is_quality_gate_passed(baseline_freq)
            .map_err(|subgate| {
                fail(
                    ArmError::QualityGate {
                        mcu_index: idx,
                        subgate,
                    },
                    &armed_indices,
                )
            })?;
    }

    let freqs: Vec<f64> = mcus
        .iter()
        .map(|(_, est)| est.clock_freq_estimate)
        .collect();
    if let Some((i, j, ratio_offset)) = check_cross_mcu_desync(&freqs) {
        return Err(fail(
            ArmError::CrossMcuDesync {
                mcu_a: i,
                mcu_b: j,
                ratio_offset,
            },
            &armed_indices,
        ));
    }

    for (idx, (io, est)) in mcus.iter_mut().enumerate() {
        if Instant::now() >= arming_deadline {
            return Err(fail(ArmError::DeadlineMissed, &armed_indices));
        }
        let t_start_host_secs = est.host_time_at(t_start_wall_clock);
        let t_start_local = est.mcu_time_at_host(t_start_host_secs);
        let cmd = format!(
            "runtime_stream_arm t_start_t0_lo={lo} t_start_t0_hi={hi} arm_lead_cycles={alc}",
            lo = t_start_local as u32,
            hi = (t_start_local >> 32) as u32,
            alc = arm_lead_cycles,
        );
        let now = Instant::now();
        if now >= arming_deadline {
            return Err(fail(ArmError::DeadlineMissed, &armed_indices));
        }
        let remaining = arming_deadline - now;
        let resp = io
            .call(&cmd, "kalico_stream_arm_response", remaining)
            .map_err(|e| fail(ArmError::Transport(e), &armed_indices))?;
        let Some(result) = resp.try_get_i32("result") else {
            return Err(fail(
                ArmError::Transport(TransportError::Parse(
                    "kalico_stream_arm_response missing 'result' field".to_string(),
                )),
                &armed_indices,
            ));
        };
        if result != 0 {
            return Err(fail(ArmError::McuRejected(result), &armed_indices));
        }
        armed_indices.push(idx);
    }

    Ok(())
}

#[derive(Debug)]
pub enum ArmError {
    DeadlineMissed,
    QualityGate {
        mcu_index: usize,
        subgate: crate::clock_sync::QualityGateFailure,
    },
    CrossMcuDesync {
        mcu_a: usize,
        mcu_b: usize,
        ratio_offset: f64,
    },
    McuRejected(i32),
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
            ArmError::QualityGate { mcu_index, subgate } => write!(
                f,
                "ARMING aborted: clock-sync quality gate failed on MCU {mcu_index}: {subgate}"
            ),
            ArmError::CrossMcuDesync {
                mcu_a,
                mcu_b,
                ratio_offset,
            } => {
                let max = MAX_CROSS_MCU_FREQ_RATIO_OFFSET;
                write!(
                    f,
                    "ARMING aborted: cross-MCU clock desync between MCU {mcu_a} \
                     and MCU {mcu_b} (|fA/fB - 1| = {ratio_offset:.6}, max = \
                     {max:.6})"
                )
            }
            ArmError::McuRejected(r) => {
                write!(f, "ARMING aborted: MCU rejected stream_arm (result={r})")
            }
            ArmError::Transport(e) => write!(f, "ARMING transport error: {e}"),
        }
    }
}

impl std::error::Error for ArmError {}

#[derive(Debug)]
pub struct ArmFailure {
    pub error: ArmError,
    pub armed_indices: Vec<usize>,
}

impl std::fmt::Display for ArmFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} (armed MCUs requiring flush: {:?})",
            self.error, self.armed_indices
        )
    }
}

impl std::error::Error for ArmFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.error)
    }
}

impl From<TransportError> for ArmFailure {
    fn from(e: TransportError) -> Self {
        ArmFailure {
            error: ArmError::Transport(e),
            armed_indices: Vec::new(),
        }
    }
}
