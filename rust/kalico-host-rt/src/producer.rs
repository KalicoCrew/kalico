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

/// Per-segment push parameters for the 4-handle wire format.
#[derive(Debug, Clone, Copy)]
pub struct SegmentPushParams {
    pub id: u32,
    pub x_handle_packed: u32,
    pub y_handle_packed: u32,
    pub z_handle_packed: u32,
    pub e_handle_packed: u32,
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: u8,
    pub e_mode: u8,
    pub extrusion_ratio: f32,
}

/// Push a single segment to the MCU.
///
/// The 4 packed handles are `(generation << 16) | slot_idx` returned
/// by prior `kalico_load_curve_response` calls; `t_start`/`t_end` are
/// 64-bit MCU-clock values produced by [`crate::stream::arm_all_mcus`]
/// or by a downstream Layer-2/3 scheduler.
pub fn push_segment<T: Transport>(
    io: &T,
    credit: &CreditCounter,
    params: &SegmentPushParams,
) -> Result<PushedSegmentInfo, ProducerError> {
    push_segment_with_timeout(io, credit, params, DEFAULT_PUSH_RESPONSE_TIMEOUT)
}

pub fn push_segment_with_timeout<T: Transport>(
    io: &T,
    credit: &CreditCounter,
    params: &SegmentPushParams,
    timeout: Duration,
) -> Result<PushedSegmentInfo, ProducerError> {
    credit.try_acquire().ok_or(ProducerError::NoCredit)?;

    // Field names + ordering MUST match the firmware's DECL_COMMAND format
    // string in `src/runtime_tick.c`:
    //   "kalico_push_segment id=%u x_handle=%u y_handle=%u z_handle=%u
    //    e_handle=%u t_start_hi=%u t_start_lo=%u t_end_hi=%u t_end_lo=%u
    //    kinematics=%c e_mode=%c extrusion_ratio=%u"
    let cmd = format!(
        "kalico_push_segment id={id} x_handle={x_handle} \
         y_handle={y_handle} z_handle={z_handle} e_handle={e_handle} \
         t_start_hi={t_start_hi} t_start_lo={t_start_lo} \
         t_end_hi={t_end_hi} t_end_lo={t_end_lo} \
         kinematics={kin} e_mode={e_mode} extrusion_ratio={extrusion_ratio}",
        id = params.id,
        x_handle = params.x_handle_packed,
        y_handle = params.y_handle_packed,
        z_handle = params.z_handle_packed,
        e_handle = params.e_handle_packed,
        t_start_lo = params.t_start as u32,
        t_start_hi = (params.t_start >> 32) as u32,
        t_end_lo = params.t_end as u32,
        t_end_hi = (params.t_end >> 32) as u32,
        kin = params.kinematics,
        e_mode = params.e_mode,
        extrusion_ratio = params.extrusion_ratio.to_bits(),
    );

    let resp = match io.call(&cmd, "kalico_push_response", timeout) {
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

/// Default timeout for `kalico_load_curve_response`.
pub const DEFAULT_LOAD_CURVE_TIMEOUT: Duration = Duration::from_millis(100);

/// Parameters for loading a single scalar curve into an MCU's curve pool.
///
/// Lays out the V1 wire blob format defined in
/// [`crate::wire::encode_load_curve_scalar`]. The actual `kalico_load_curve`
/// command sends `degree` and the `knots_f32` / `cps_f32` byte buffers as
/// separate msgproto fields; [`CurveLoadParams::encode`] returns the
/// aggregated blob form for offline diagnostics and tests.
#[derive(Debug, Clone)]
pub struct CurveLoadParams {
    pub degree: u8,
    pub knots_f32: Vec<f32>,
    pub cps_f32: Vec<f32>,
}

impl CurveLoadParams {
    /// Encode this curve as a V1 scalar wire blob.
    pub fn encode(&self) -> Vec<u8> {
        crate::wire::encode_load_curve_scalar(self.degree, &self.knots_f32, &self.cps_f32)
    }

    /// Construct from a `nurbs::ScalarNurbs<f64>`, truncating to f32.
    pub fn from_scalar_nurbs(curve: &nurbs::ScalarNurbs<f64>) -> Self {
        Self {
            degree: nurbs::NurbsView::degree(curve),
            knots_f32: curve.knots().iter().map(|&k| k as f32).collect(),
            cps_f32: curve.control_points().iter().map(|&v| v as f32).collect(),
        }
    }

    /// Pack `cps_f32` into a little-endian byte buffer suitable for the
    /// `cps=%*s` wire arg.
    pub fn cps_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.cps_f32.len() * 4);
        for &v in &self.cps_f32 {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }

    /// Pack `knots_f32` into a little-endian byte buffer.
    pub fn knots_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.knots_f32.len() * 4);
        for &k in &self.knots_f32 {
            out.extend_from_slice(&k.to_le_bytes());
        }
        out
    }
}

/// Load a scalar curve into the MCU's curve pool at the caller-specified slot.
///
/// Sends `kalico_load_curve` and waits for `kalico_load_curve_response`.
/// On success, returns the packed handle `(generation << 16) | slot_idx`
/// reported by the firmware (`curve_handle_packed`).
pub fn load_curve<T: Transport>(
    io: &T,
    slot: u16,
    params: &CurveLoadParams,
    timeout: Duration,
) -> Result<u32, ProducerError> {
    use crate::host_io::parser::FieldValue;
    use crate::wire::FORMAT_VERSION_V1;

    let cps_buf = params.cps_bytes();
    let knots_buf = params.knots_bytes();

    let resp = io.call_typed(
        "kalico_load_curve",
        &[
            ("version", FieldValue::Byte(FORMAT_VERSION_V1)),
            ("slot", FieldValue::U16(slot)),
            ("degree", FieldValue::Byte(params.degree)),
            ("cps", FieldValue::Buffer(&cps_buf)),
            ("knots", FieldValue::Buffer(&knots_buf)),
        ],
        "kalico_load_curve_response",
        timeout,
    )?;

    let Some(result) = resp.try_get_i32("result") else {
        return Err(ProducerError::Transport(TransportError::Parse(
            "kalico_load_curve_response missing 'result' field".to_string(),
        )));
    };
    if result != 0 {
        return Err(ProducerError::McuRejected(result));
    }

    let Some(handle) = resp.try_get_u32("curve_handle_packed") else {
        return Err(ProducerError::Transport(TransportError::Parse(
            "kalico_load_curve_response missing 'curve_handle_packed' field".to_string(),
        )));
    };
    Ok(handle)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn curve_load_params_encodes_correctly() {
        let params = CurveLoadParams {
            degree: 3,
            knots_f32: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            cps_f32: vec![0.0, 3.33, 6.67, 10.0],
        };
        let blob = params.encode();
        assert_eq!(blob[0], crate::wire::FORMAT_VERSION_V1);
        assert_eq!(blob[1], 3);
        assert_eq!(blob[2], 4); // num_cps
        assert_eq!(blob[3], 8); // num_knots
    }

    #[test]
    fn curve_load_params_byte_buffers_are_le() {
        let params = CurveLoadParams {
            degree: 0,
            knots_f32: vec![1.0, 2.0],
            cps_f32: vec![1.5],
        };
        assert_eq!(params.cps_bytes().len(), 4);
        assert_eq!(params.knots_bytes().len(), 8);
        let cp_bytes: [u8; 4] = params.cps_bytes()[..4].try_into().unwrap();
        assert_eq!(f32::from_le_bytes(cp_bytes), 1.5);
    }
}
