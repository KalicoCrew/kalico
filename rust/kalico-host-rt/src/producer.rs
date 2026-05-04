//! Segment producer (kalico-native, Phase C-B).
//!
//! Spec §7.3 / §7.4 / §15: each push acquires credit from a local
//! [`CreditCounter`], encodes the kalico-native command, sends via
//! [`KalicoHostIo::kalico_call`], and waits on the matching response.
//! Failures roll back the credit acquisition.
//!
//! The pre-Phase-C `producer::load_curve` (begin/chunk/finalize over
//! Klipper msgproto) and `producer::push_segment` (Klipper command
//! `kalico_push_segment`) retire — the MCU no longer accepts those
//! commands (commit `0b263982d`).

use std::time::Duration;

use kalico_protocol::{Encode, LoadCurve, LoadCurveResponse, MessageKind, PushSegment, PushSegmentResponse, Decode};

use crate::credit::CreditCounter;
use crate::host_io::KalicoHostIo;
use crate::transport::TransportError;

/// Default timeout for `LoadCurveResponse` (spec §7.4). The MCU should
/// reply within microseconds; 100 ms is loose by ~3 orders of magnitude
/// and only triggers on a host-side stall or a wire fault.
pub const DEFAULT_LOAD_CURVE_TIMEOUT: Duration = Duration::from_millis(100);

/// Default timeout for `PushSegmentResponse`.
pub const DEFAULT_PUSH_RESPONSE_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy)]
pub struct PushedSegmentInfo {
    pub accepted_segment_id: u32,
    pub credit_epoch: u32,
}

#[derive(Debug)]
pub enum ProducerError {
    /// `try_acquire` returned `None` — caller should back off until the
    /// next CreditFreed event.
    NoCredit,
    /// Transport-layer failure (timeout, I/O, parse).
    Transport(TransportError),
    /// MCU rejected the command (`result != 0`).
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
                write!(f, "producer: MCU rejected command (result={r})")
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

/// Push a single segment to the MCU via kalico-native PushSegment.
pub fn push_segment(
    io: &KalicoHostIo,
    credit: &CreditCounter,
    params: &SegmentPushParams,
) -> Result<PushedSegmentInfo, ProducerError> {
    push_segment_with_timeout(io, credit, params, DEFAULT_PUSH_RESPONSE_TIMEOUT)
}

pub fn push_segment_with_timeout(
    io: &KalicoHostIo,
    credit: &CreditCounter,
    params: &SegmentPushParams,
    timeout: Duration,
) -> Result<PushedSegmentInfo, ProducerError> {
    credit.try_acquire().ok_or(ProducerError::NoCredit)?;

    let body = PushSegment {
        id: params.id,
        handle_x: params.x_handle_packed,
        handle_y: params.y_handle_packed,
        handle_z: params.z_handle_packed,
        handle_e: params.e_handle_packed,
        t_start: params.t_start,
        t_end: params.t_end,
        kinematics: params.kinematics,
        e_mode: params.e_mode,
        extrusion_ratio: params.extrusion_ratio,
    }
    .encoded_to_vec();

    let (kind, resp_body) = match io.kalico_call(MessageKind::PushSegment, body, timeout) {
        Ok(r) => r,
        Err(e) => {
            credit.release();
            return Err(ProducerError::Transport(e));
        }
    };
    if kind != MessageKind::PushSegmentResponse {
        credit.release();
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "expected PushSegmentResponse, got 0x{:04x}",
            kind.as_u16()
        ))));
    }
    let resp = match PushSegmentResponse::decode(&resp_body) {
        Ok(r) => r,
        Err(e) => {
            credit.release();
            return Err(ProducerError::Transport(TransportError::Parse(format!(
                "PushSegmentResponse decode failed: {e:?}"
            ))));
        }
    };
    if resp.result != 0 {
        credit.release();
        return Err(ProducerError::McuRejected(resp.result));
    }
    Ok(PushedSegmentInfo {
        accepted_segment_id: resp.accepted_segment_id,
        credit_epoch: resp.credit_epoch,
    })
}

/// Parameters for loading a single scalar curve into an MCU's curve pool.
///
/// Lays out the post-Phase-C kalico-native LoadCurve body (spec §7.3):
/// `slot: u16, degree: u8, n_cps: u32, n_knots: u32, cps[..]: f32 LE,
/// knots[..]: f32 LE`.
#[derive(Debug, Clone)]
pub struct CurveLoadParams {
    pub degree: u8,
    pub knots_f32: Vec<f32>,
    pub cps_f32: Vec<f32>,
}

impl CurveLoadParams {
    /// Construct from a `nurbs::ScalarNurbs<f64>`, truncating to f32.
    pub fn from_scalar_nurbs(curve: &nurbs::ScalarNurbs<f64>) -> Self {
        Self {
            degree: nurbs::NurbsView::degree(curve),
            knots_f32: curve.knots().iter().map(|&k| k as f32).collect(),
            cps_f32: curve.control_points().iter().map(|&v| v as f32).collect(),
        }
    }

    /// Construct from a time-domain `ScalarNurbs<f64>` for the MCU evaluator.
    ///
    /// Firmware evaluates loaded curves at normalized segment progress
    /// `u = elapsed / duration`, not at absolute host time. Keep the control
    /// points as positions, but map the curve knot domain from
    /// `[t_start_s, t_end_s]` onto `[0, 1]` before f32 truncation.
    pub fn from_scalar_nurbs_normalized(
        curve: &nurbs::ScalarNurbs<f64>,
        t_start_s: f64,
        t_end_s: f64,
    ) -> Self {
        let duration = t_end_s - t_start_s;
        debug_assert!(duration > 0.0);
        let knots_f32 = curve
            .knots()
            .iter()
            .map(|&k| {
                let u = if duration > 0.0 {
                    (k - t_start_s) / duration
                } else {
                    k
                };
                u.clamp(0.0, 1.0) as f32
            })
            .collect();
        Self {
            degree: nurbs::NurbsView::degree(curve),
            knots_f32,
            cps_f32: curve.control_points().iter().map(|&v| v as f32).collect(),
        }
    }
}

/// Load a scalar curve into the MCU's curve pool at the caller-specified slot.
///
/// Single kalico-native LoadCurve frame (spec §7.3) — the Phase-A through
/// Phase-B `kalico-native-transport` plumbing replaces the legacy
/// `begin/chunk/finalize` Klipper-msgproto sequence. On success returns the
/// packed handle `(generation << 16) | slot_idx`. On `result != 0` returns
/// [`ProducerError::McuRejected`].
pub fn load_curve(
    io: &KalicoHostIo,
    slot: u16,
    params: &CurveLoadParams,
    timeout: Duration,
) -> Result<u32, ProducerError> {
    let body = LoadCurve {
        slot,
        degree: params.degree,
        cps: params.cps_f32.clone(),
        knots: params.knots_f32.clone(),
    }
    .encoded_to_vec();

    eprintln!("[host] producer::load_curve calling kalico_call (slot={slot}, body_len={})", body.len());
    let (kind, resp_body) = io.kalico_call(MessageKind::LoadCurve, body, timeout)?;
    eprintln!("[host] producer::load_curve got response kind=0x{:04x}", kind.as_u16());
    if kind != MessageKind::LoadCurveResponse {
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "expected LoadCurveResponse, got 0x{:04x}",
            kind.as_u16()
        ))));
    }
    let resp = LoadCurveResponse::decode(&resp_body).map_err(|e| {
        ProducerError::Transport(TransportError::Parse(format!(
            "LoadCurveResponse decode failed: {e:?}"
        )))
    })?;
    if resp.result != 0 {
        return Err(ProducerError::McuRejected(resp.result));
    }
    Ok(resp.curve_handle_packed)
}
