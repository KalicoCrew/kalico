//! Segment producer. Layer-1/2/3 → wire encoder → [`Transport::send`].
//!
//! Spec §4.2 + §3.2: each push acquires credit from a local
//! [`CreditCounter`], encodes the wire command, sends via the
//! transport, then waits on the named `kalico_push_response` reply.
//! Failures roll back the credit acquisition.

use std::time::Duration;

use crate::credit::CreditCounter;
use crate::transport::{Transport, TransportError};

/// Default timeout for `kalico_push_response`. The MCU should reply
/// within microseconds; 100 ms is loose by ~3 orders of magnitude and
/// only triggers on a host-side stall or a wire fault.
pub const DEFAULT_PUSH_RESPONSE_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy)]
pub struct PushedSegmentInfo {
    pub accepted_segment_id: u32,
    pub credit_epoch: u32,
}

#[derive(Debug)]
pub enum ProducerError {
    /// `try_acquire` returned `None` — caller should back off until the
    /// next `kalico_credit_freed` event.
    NoCredit,
    /// Transport-layer failure (timeout, I/O, parse).
    Transport(TransportError),
    /// MCU rejected the push (`result != 0` in `kalico_push_response`).
    /// The negative `i32` is the spec §9 fault-code mapping.
    McuRejected(i32),
}

impl From<TransportError> for ProducerError {
    fn from(e: TransportError) -> Self {
        ProducerError::Transport(e)
    }
}

impl std::fmt::Display for ProducerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProducerError::NoCredit => write!(f, "producer: no credit (MCU queue full)"),
            ProducerError::Transport(e) => write!(f, "producer transport: {e}"),
            ProducerError::McuRejected(r) => {
                write!(f, "producer: MCU rejected push (result={r})")
            }
        }
    }
}

impl std::error::Error for ProducerError {}

/// Push a single segment to the MCU.
///
/// `curve_handle_packed` is the u32 packed (slot, gen) handle returned
/// by a prior `kalico_load_curve_response`; `t_start`/`t_end` are
/// 64-bit MCU-clock values produced by [`crate::stream::arm_all_mcus`]
/// or by a downstream Layer-2/3 scheduler.
pub fn push_segment<T: Transport>(
    io: &mut T,
    credit: &CreditCounter,
    id: u32,
    curve_handle_packed: u32,
    t_start: u64,
    t_end: u64,
    kinematics: u8,
) -> Result<PushedSegmentInfo, ProducerError> {
    push_segment_with_timeout(
        io,
        credit,
        id,
        curve_handle_packed,
        t_start,
        t_end,
        kinematics,
        DEFAULT_PUSH_RESPONSE_TIMEOUT,
    )
}

// Eight args is the full Layer-1/2/3 push surface (id, curve, t_start,
// t_end, kinematics, timeout, plus io+credit). Splitting into a struct
// would obscure the call-site readability without removing any
// arguments — the test suite calls this directly with concrete values
// per Step-6 plan, so we lint-allow.
#[allow(clippy::too_many_arguments)]
pub fn push_segment_with_timeout<T: Transport>(
    io: &mut T,
    credit: &CreditCounter,
    id: u32,
    curve_handle_packed: u32,
    t_start: u64,
    t_end: u64,
    kinematics: u8,
    timeout: Duration,
) -> Result<PushedSegmentInfo, ProducerError> {
    credit.try_acquire().ok_or(ProducerError::NoCredit)?;

    let cmd = format!(
        "kalico_push_segment id={id} curve_handle_packed={curve_handle_packed} \
         t_start_lo={t_start_lo} t_start_hi={t_start_hi} \
         t_end_lo={t_end_lo} t_end_hi={t_end_hi} kin={kinematics}",
        t_start_lo = t_start as u32,
        t_start_hi = (t_start >> 32) as u32,
        t_end_lo = t_end as u32,
        t_end_hi = (t_end >> 32) as u32,
    );

    if let Err(e) = io.send(&cmd) {
        credit.release();
        return Err(ProducerError::Transport(e));
    }

    let resp = match io.wait_for_response("kalico_push_response", timeout) {
        Ok(r) => r,
        Err(e) => {
            credit.release();
            return Err(ProducerError::Transport(e));
        }
    };

    // I1 fix: `result` is load-bearing — `result == 0` means success, so
    // a missing field on a malformed reply would be silently treated as
    // success. Use the fallible accessor and surface a Parse error if
    // the MCU response is missing the field.
    let Some(result) = resp.try_get_i32("result") else {
        credit.release();
        return Err(ProducerError::Transport(TransportError::Parse(
            "kalico_push_response missing 'result' field".to_string(),
        )));
    };
    if result != 0 {
        credit.release();
        return Err(ProducerError::McuRejected(result));
    }

    Ok(PushedSegmentInfo {
        accepted_segment_id: resp.get_u32("accepted_segment_id"),
        credit_epoch: resp.get_u32("credit_epoch"),
    })
}
