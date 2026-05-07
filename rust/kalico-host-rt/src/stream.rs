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
//! On any failure we abort the in-progress arm and surface an
//! [`ArmFailure`] carrying both the originating [`ArmError`] and the
//! list of MCU indices that DID complete the arm step before the
//! failure (i.e. received a successful `kalico_stream_arm_response`).
//! The caller iterates `armed_indices` and issues `kalico_stream_flush`
//! to wind those MCUs back to IDLE — leaving the others alone, since
//! they never armed.

use std::time::{Duration, Instant};

use crate::clock_sync::ClockSyncEstimator;
use crate::transport::{Transport, TransportError};

/// Spec §6.3 + §12.4: cross-MCU clock-frequency drift sanity check at
/// arm time. The wire-rate scheduler relies on every MCU's local clock
/// running at the same rate to within ~1e-3 (1000 ppm); anything wider
/// is a `KALICO_FAULT_CROSS_MCU_DESYNC` and we refuse to arm.
pub const MAX_CROSS_MCU_FREQ_RATIO_OFFSET: f64 = 1e-3;

/// Pure-data form of the cross-MCU drift check (spec §6.3 + §12.4).
/// Returns `Some((i, j, ratio_offset))` for the first pair (lexicographic
/// `i < j`) where `|fA / fB - 1|` exceeds
/// `MAX_CROSS_MCU_FREQ_RATIO_OFFSET`. Returns `None` if every pair is
/// within tolerance, or if there are fewer than two MCUs. Non-positive
/// freqs are skipped (an upstream quality gate is responsible for
/// rejecting them).
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
) -> Result<(), ArmFailure> {
    let arming_deadline = Instant::now() + arm_lead_time / 2;

    // Track which MCUs successfully completed the arm-issuance step so
    // a partial-failure caller can flush exactly those MCUs (I5 fix).
    // Pre-arm-ack failures leave `armed_indices` empty.
    let mut armed_indices: Vec<usize> = Vec::with_capacity(mcus.len());
    let fail = |error: ArmError, armed: &[usize]| ArmFailure {
        error,
        armed_indices: armed.to_vec(),
    };

    // Step 1+2+3: dedicated sync + quality gate per MCU.
    for (idx, (io, est)) in mcus.iter_mut().enumerate() {
        if Instant::now() >= arming_deadline {
            return Err(fail(ArmError::DeadlineMissed, &armed_indices));
        }
        let host_send = Instant::now();
        // Per spec §5.9: monotonic request_id per MCU so stale or
        // reordered responses are detected. The counter lives on the
        // estimator so it persists across arm attempts — a delayed
        // response from a prior arm cannot reuse a fresh request_id.
        // `host_send_time_*` are sent as zero because the estimator
        // independently records `host_send` and `host_recv` wall-clock
        // instants.
        let request_id = est.next_clock_sync_request_id();
        let resp = io
            .call(
                &format!("runtime_clock_sync_request request_id={request_id} host_send_time_lo=0 host_send_time_hi=0"),
                "kalico_clock_sync_response",
                CLOCK_SYNC_REQUEST_TIMEOUT,
            )
            .map_err(|e| fail(ArmError::Transport(e), &armed_indices))?;
        let host_recv = Instant::now();

        let echoed = resp.try_get_u32("request_id").ok_or_else(|| fail(
            ArmError::Transport(TransportError::Parse(
                "kalico_clock_sync_response missing request_id field".into(),
            )),
            &armed_indices,
        ))?;
        if echoed != request_id {
            return Err(fail(
                ArmError::Transport(TransportError::Parse(
                    format!("clock_sync request_id mismatch: sent {request_id}, got {echoed}"),
                )),
                &armed_indices,
            ));
        }

        let mcu_clock = (u64::from(resp.get_u32("mcu_clock_hi")) << 32)
            | u64::from(resp.get_u32("mcu_clock_lo"));
        est.add_dedicated_sample(host_send, host_recv, mcu_clock);

        est.is_quality_gate_passed(baseline_freq)
            .map_err(|subgate| fail(ArmError::QualityGate { mcu_index: idx, subgate }, &armed_indices))?;
    }

    // Spec §6.3 + §12.4 (GAP-1 fix): cross-MCU drift sanity check
    // BEFORE issuing any arm. Refuse to arm if any pair of MCUs has
    // |fA / fB - 1| > 1e-3 — this is `KALICO_FAULT_CROSS_MCU_DESYNC`.
    // Done after every estimator has a fresh dedicated sample so the
    // freq estimates are current.
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

    // Step 4+5: arm each MCU with deadline.
    //
    // Round-2 fix B11-real: per-MCU `t_start_local` MUST be the
    // absolute MCU-clock value at wall-time `t_start_wall_clock`,
    // NOT just `delta_secs * freq`. We compute via the estimator's
    // anchor: t_start_local =
    // mcu_time_at_host(host_time_secs(t_start_wall_clock)).
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
        // I1 fix: `result` is load-bearing (0 = success); a missing
        // field must surface as a Parse error rather than silently
        // succeed.
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
        // This MCU acknowledged the arm; record so the caller can flush
        // it on a later-MCU partial failure.
        armed_indices.push(idx);
    }

    Ok(())
}

#[derive(Debug)]
pub enum ArmError {
    /// `arm_lead_time / 2` elapsed before all MCUs were armed.
    DeadlineMissed,
    /// At least one MCU's [`ClockSyncEstimator`] failed
    /// `is_quality_gate_passed` after the dedicated sync. Carries the
    /// index of the failing MCU and the specific subgate that tripped.
    QualityGate {
        mcu_index: usize,
        subgate:   crate::clock_sync::QualityGateFailure,
    },
    /// Spec §6.3 + §12.4: cross-MCU clock-frequency drift exceeds
    /// `MAX_CROSS_MCU_FREQ_RATIO_OFFSET`. Carries the indices of the
    /// offending pair plus the actual `|fA / fB - 1|` value so the
    /// caller can log a useful diagnostic.
    CrossMcuDesync {
        mcu_a: usize,
        mcu_b: usize,
        ratio_offset: f64,
    },
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

/// I5 fix: partial-arm failure surface. When `arm_all_mcus` fails
/// mid-stream, MCUs that already received a successful
/// `kalico_stream_arm_response` are armed and need to be flushed back
/// to IDLE (`kalico_stream_flush`); MCUs not in `armed_indices` were
/// never armed and require no rollback.
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
