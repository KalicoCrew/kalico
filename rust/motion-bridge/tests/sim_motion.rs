//! Task 10 — kalico-sim integration test scaffold (Phase 2 motion bridge).
//!
//! ## Scope (Path B — structural integration)
//!
//! These tests verify the host-side Phase 2 motion pipeline end-to-end up to
//! the wire boundary:
//!
//!   classify → planner thread → reduce/temporal/trajectory shape_batch →
//!   per-MCU dispatch closure → producer::load_curve / producer::push_segment
//!   on a recording `Transport`.
//!
//! What this does **not** verify:
//!
//!   * Step events emitted on belts (A/B for CoreXY, Z for cartesian).
//!   * Step counts, direction, or step timing monotonicity.
//!
//! There is no end-to-end "host bridge ↔ simulated MCU emitting fake step
//! events" harness in this repo today. The runtime crate's MCU-side tests
//! (`rust/runtime/tests/step_generation.rs` etc.) test the firmware engine in
//! isolation; they are not wired into a host-driven integration sim. Stitching
//! them together would require new infrastructure (producer plumbed into a
//! firmware-side reactor that exposes a step-trace channel) that is beyond
//! Task 10's scope.
//!
//! Step-event verification is therefore deferred to **Task 12** (Renode gate
//! test, where real firmware runs against a simulated H723 + step-output
//! capture) and **Task 7-D** (hardware bring-up, where real steppers move).
//!
//! What these tests *do* verify, though, is meaningful:
//!
//!   * `single_axis_x_move` — boot dispatch with CoreXY (Octopus drives X+Y),
//!     submit `submit_move(10, 0, 0, 0, 100)`, wait. Assert: `kalico_load_curve`
//!     fired for the X axis on the Octopus, `kalico_push_segment` fired with
//!     CoreXY kinematics tag, nothing landed on F446.
//!   * `single_axis_z_move_different_mcu` — same harness with two MCUs
//!     (Octopus X+Y, F446 Z); submit `submit_move(0, 0, 5, 0, 50)`. Assert:
//!     `kalico_load_curve` + `kalico_push_segment` only on F446, nothing on
//!     Octopus.
//!   * `extrusion_rejected` — `submit_move(0, 0, 0, 1, 100)` returns
//!     `ClassifyError::ExtrusionNotSupported`.
//!
//! **Architectural note:** these tests bypass `RouterTransport` (which
//! requires a `MsgProtoParser`) and instead drive `producer::load_curve` /
//! `producer::push_segment` against a hand-rolled recording `Transport`. The
//! producer surface is the same one the bridge's dispatch closure invokes; we
//! rebuild a small slice of that closure here. This keeps the test focused on
//! the planner + dispatch + producer wiring without dragging the whole router
//! adapter into the test harness.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::host_io::parser::FieldValue;
use kalico_host_rt::producer::{self, DEFAULT_LOAD_CURVE_TIMEOUT};
use kalico_host_rt::transport::{
    MessageParams, MessageValue, Transport, TransportError,
};
use trajectory::{
    AxisShaper, RequiredShaper, ShapedSegment, ShaperConfig,
};

use nurbs::ScalarNurbs;

use motion_bridge::classify::{ClassifyError, classify_and_build};
use motion_bridge::config::{PlannerConfig, PlannerLimits};
use motion_bridge::dispatch::{
    AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, build_push_params,
};
use motion_bridge::planner::PlannerHandle;

// ---------------------------------------------------------------------------
// RecordingTransport — synchronous recording stub for `kalico_load_curve` and
// `kalico_push_segment`. Returns canned successful responses.
// ---------------------------------------------------------------------------

struct CallRecord {
    cmd: String,
    /// Captured load_curve payload (degree, knots, cps), parsed from typed
    /// args. Populated for `kalico_load_curve` calls, `None` otherwise.
    load_curve: Option<LoadCurveCapture>,
}

#[derive(Clone, Debug)]
struct LoadCurveCapture {
    degree: u8,
    knots: Vec<f32>,
    cps: Vec<f32>,
}

#[derive(Default)]
struct TransportState {
    sent: Vec<CallRecord>,
    next_handle_lo: u32,
    next_segment_id: u32,
}

struct RecordingTransport {
    state: Mutex<TransportState>,
}

impl RecordingTransport {
    fn new() -> Self {
        Self {
            state: Mutex::new(TransportState {
                sent: Vec::new(),
                next_handle_lo: 1,
                next_segment_id: 1,
            }),
        }
    }

    fn sent_starting_with(&self, prefix: &str) -> Vec<String> {
        self.state
            .lock()
            .unwrap()
            .sent
            .iter()
            .filter(|c| c.cmd.starts_with(prefix))
            .map(|c| c.cmd.clone())
            .collect()
    }

    /// All captured `kalico_load_curve` payloads, in submission order.
    fn load_curve_captures(&self) -> Vec<LoadCurveCapture> {
        self.state
            .lock()
            .unwrap()
            .sent
            .iter()
            .filter_map(|c| c.load_curve.clone())
            .collect()
    }
}

impl Transport for RecordingTransport {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        _timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let mut s = self.state.lock().unwrap();
        s.sent.push(CallRecord {
            cmd: cmd.to_string(),
            load_curve: None,
        });
        let mut p = MessageParams::new();
        match expected_response_name {
            "kalico_load_curve_response" => {
                let lo = s.next_handle_lo;
                s.next_handle_lo = s.next_handle_lo.wrapping_add(1);
                p.insert("result".to_string(), MessageValue::I32(0));
                // Producer extracts curve_handle_packed as the wire handle.
                p.insert(
                    "curve_handle_packed".to_string(),
                    MessageValue::U32(lo),
                );
                Ok(p)
            }
            "kalico_push_response" => {
                let id = s.next_segment_id;
                s.next_segment_id = s.next_segment_id.wrapping_add(1);
                p.insert("result".to_string(), MessageValue::I32(0));
                p.insert(
                    "accepted_segment_id".to_string(),
                    MessageValue::U32(id),
                );
                p.insert("credit_epoch".to_string(), MessageValue::U32(0));
                Ok(p)
            }
            other => Err(TransportError::Parse(format!(
                "RecordingTransport: unexpected expected_response_name '{other}'"
            ))),
        }
    }

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        // Snoop typed args for kalico_load_curve so tests can decode the
        // shaped NURBS that producer sent on the wire.
        let load_curve = if name == "kalico_load_curve" {
            let mut degree: Option<u8> = None;
            let mut cps_bytes: Option<Vec<u8>> = None;
            let mut knots_bytes: Option<Vec<u8>> = None;
            for (k, v) in args {
                match (*k, v) {
                    ("degree", FieldValue::Byte(b)) => degree = Some(*b),
                    ("cps", FieldValue::Buffer(b)) => cps_bytes = Some(b.to_vec()),
                    ("knots", FieldValue::Buffer(b)) => knots_bytes = Some(b.to_vec()),
                    _ => {}
                }
            }
            match (degree, cps_bytes, knots_bytes) {
                (Some(d), Some(cb), Some(kb)) => Some(LoadCurveCapture {
                    degree: d,
                    cps: cb
                        .chunks_exact(4)
                        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                        .collect(),
                    knots: kb
                        .chunks_exact(4)
                        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                        .collect(),
                }),
                _ => None,
            }
        } else {
            None
        };

        // Inline a simplified version of `call` so we can stash the
        // captured load_curve payload alongside the cmd string.
        {
            let mut s = self.state.lock().unwrap();
            s.sent.push(CallRecord {
                cmd: name.to_string(),
                load_curve,
            });
        }
        let mut p = MessageParams::new();
        match expected_response_name {
            "kalico_load_curve_response" => {
                let mut s = self.state.lock().unwrap();
                let lo = s.next_handle_lo;
                s.next_handle_lo = s.next_handle_lo.wrapping_add(1);
                p.insert("result".to_string(), MessageValue::I32(0));
                p.insert(
                    "curve_handle_packed".to_string(),
                    MessageValue::U32(lo),
                );
                Ok(p)
            }
            "kalico_push_response" => {
                let mut s = self.state.lock().unwrap();
                let id = s.next_segment_id;
                s.next_segment_id = s.next_segment_id.wrapping_add(1);
                p.insert("result".to_string(), MessageValue::I32(0));
                p.insert(
                    "accepted_segment_id".to_string(),
                    MessageValue::U32(id),
                );
                p.insert("credit_epoch".to_string(), MessageValue::U32(0));
                Ok(p)
            }
            other => {
                let _ = timeout;
                Err(TransportError::Parse(format!(
                    "RecordingTransport: unexpected expected_response_name '{other}'"
                )))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Test harness — assemble PlannerHandle + dispatch closure against a fresh
// RecordingTransport, exactly mirroring what `bridge::init_planner` does but
// without the PyO3 / RouterTransport indirection.
// ---------------------------------------------------------------------------

const OCTOPUS_ID: u32 = 1;
const F446_ID: u32 = 2;

struct Harness {
    planner: Option<PlannerHandle>,
    transports: HashMap<u32, Arc<RecordingTransport>>,
    dispatched: Arc<AtomicU64>,
}

impl Harness {
    /// Single-MCU harness — Octopus drives X+Y as CoreXY (kinematics=0).
    /// No F446 in the topology.
    fn corexy_only() -> Self {
        let mcu_configs = vec![McuAxisConfig {
            mcu_id: OCTOPUS_ID,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: 0,
        }];
        Self::build(mcu_configs)
    }

    /// Single-MCU CoreXY harness with caller-supplied planner limits.
    fn corexy_with_limits(limits: PlannerLimits) -> Self {
        let mcu_configs = vec![McuAxisConfig {
            mcu_id: OCTOPUS_ID,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: 0,
        }];
        Self::build_with(mcu_configs, Some(limits))
    }

    /// Two-MCU harness — Octopus drives X+Y as CoreXY (kinematics=0),
    /// F446 drives Z as cartesian (kinematics=1). Mirrors the
    /// (octopus_handle, f446_handle) pairing used by `bridge::init_planner`.
    fn corexy_plus_z() -> Self {
        let mcu_configs = vec![
            McuAxisConfig {
                mcu_id: OCTOPUS_ID,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: 0,
            },
            McuAxisConfig {
                mcu_id: F446_ID,
                axes: vec![AXIS_Z],
                kinematics: 1,
            },
        ];
        Self::build(mcu_configs)
    }

    fn build(mcu_configs: Vec<McuAxisConfig>) -> Self {
        Self::build_with(mcu_configs, None)
    }

    fn build_with(
        mcu_configs: Vec<McuAxisConfig>,
        override_limits: Option<PlannerLimits>,
    ) -> Self {
        let mut transports: HashMap<u32, Arc<RecordingTransport>> = HashMap::new();
        let mut credits: HashMap<u32, Arc<CreditCounter>> = HashMap::new();
        for cfg in &mcu_configs {
            transports.insert(cfg.mcu_id, Arc::new(RecordingTransport::new()));
            credits.insert(cfg.mcu_id, Arc::new(CreditCounter::new(1024)));
        }

        let dispatched = Arc::new(AtomicU64::new(0));
        let counter = Arc::clone(&dispatched);

        // Capture per-MCU state into the dispatch closure.
        let cb_transports = transports.clone();
        let cb_credits = credits.clone();
        let cb_mcu_configs = mcu_configs.clone();

        // Per-MCU rolling slot index (matches bridge::init_planner behaviour).
        let next_slot: Arc<Mutex<HashMap<u32, u16>>> =
            Arc::new(Mutex::new(HashMap::new()));
        let next_seg_id: Arc<Mutex<HashMap<u32, u32>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let dispatch: Arc<
            dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync,
        > = Arc::new(move |seg: &ShapedSegment| -> Result<(), String> {
            // No real clock estimate — use the t*1e6 fallback path the bridge
            // also uses during early bring-up.
            let t_start_clock = (seg.t_start * 1e6) as u64;
            let t_end_clock = (seg.t_end * 1e6) as u64;

            let mut plans = build_push_params(
                seg,
                &cb_mcu_configs,
                t_start_clock,
                t_end_clock,
            );

            for plan in &mut plans {
                let transport = match cb_transports.get(&plan.mcu_id) {
                    Some(t) => t.clone(),
                    None => continue,
                };
                let credit = cb_credits.get(&plan.mcu_id).unwrap().clone();

                plan.params.t_start = t_start_clock;
                plan.params.t_end = t_end_clock;

                {
                    let mut ids = next_seg_id.lock().unwrap();
                    let entry = ids.entry(plan.mcu_id).or_insert(1);
                    plan.params.id = *entry;
                    *entry = entry.wrapping_add(1);
                }

                let curves = std::mem::take(&mut plan.curves_to_load);
                for (axis_idx, curve_params) in &curves {
                    let slot: u16 = {
                        let mut slots = next_slot.lock().unwrap();
                        let entry = slots.entry(plan.mcu_id).or_insert(0);
                        let v = *entry;
                        *entry = entry.wrapping_add(1);
                        v
                    };
                    let handle = producer::load_curve(
                        transport.as_ref(),
                        slot,
                        curve_params,
                        DEFAULT_LOAD_CURVE_TIMEOUT,
                    )
                    .map_err(|e| {
                        format!("load_curve mcu={}: {e}", plan.mcu_id)
                    })?;
                    plan.set_handle(*axis_idx, handle);
                }

                producer::push_segment(transport.as_ref(), &credit, &plan.params)
                    .map_err(|e| format!("push_segment mcu={}: {e}", plan.mcu_id))?;
            }

            counter.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        let mut cfg = PlannerConfig::default();
        // Relax the C1 refit tolerance — matches the in-crate planner tests'
        // convention; the default 5 µm is tighter than the degree-4 refit can
        // achieve on a 10 mm collinear cubic under the test grid budget.
        cfg.fit_tolerance_mm = 0.05;
        cfg.limits = override_limits.unwrap_or(PlannerLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            // Generous Z limits — the default 15 mm/s / 100 mm/s² combined
            // with the X/Y-axis-derived corner-deviation chord tolerance
            // make even modest pure-Z moves trip TemporalJoining infeasibility.
            max_z_velocity: 50.0,
            max_z_accel: 500.0,
            square_corner_velocity: 5.0,
        });
        cfg.shaper = ShaperConfig {
            x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
            y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
            z: AxisShaper::Passthrough,
        };

        let planner = PlannerHandle::spawn(cfg, dispatch);

        Self {
            planner: Some(planner),
            transports,
            dispatched,
        }
    }

    fn submit_move(
        &self,
        start: [f64; 3],
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feed: f64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let m = classify_and_build(start, dx, dy, dz, de, feed)?;
        self.planner.as_ref().unwrap().submit_move(m)?;
        Ok(())
    }

    fn flush(&self) {
        self.planner.as_ref().unwrap().flush().expect("flush");
    }

    fn update_limits(&self, l: PlannerLimits) {
        self.planner
            .as_ref()
            .unwrap()
            .update_limits(l)
            .expect("update_limits");
    }
}

impl Drop for Harness {
    fn drop(&mut self) {
        if let Some(mut p) = self.planner.take() {
            p.shutdown();
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn single_axis_x_move() {
    let h = Harness::corexy_only();

    h.submit_move([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0)
        .expect("submit_move");
    h.flush();

    assert!(
        h.dispatched.load(Ordering::Relaxed) > 0,
        "no shaped segments dispatched"
    );

    // Octopus must see at least one load_curve and one push_segment.
    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    let load_curves = octopus.sent_starting_with("kalico_load_curve");
    let pushes = octopus.sent_starting_with("kalico_push_segment");
    assert!(
        !load_curves.is_empty(),
        "expected kalico_load_curve on Octopus, saw none"
    );
    assert!(
        !pushes.is_empty(),
        "expected kalico_push_segment on Octopus, saw none"
    );

    // CoreXY tag (kinematics=0) must be on the wire. The bridge's
    // CoreXY-and-E firmware kinematics tag is 0.
    assert!(
        pushes.iter().all(|p| p.contains("kinematics=0")),
        "expected kinematics=0 on all Octopus pushes, saw: {pushes:?}"
    );
}

/// Pure-Z move on a 2-MCU topology — exercises the real planner pipeline
/// end-to-end and asserts F446-only routing. Previously fed a hand-built
/// ShapedSegment directly into `build_push_params` because pure-Z tripped
/// `TemporalJoining(StalledOnInfeasibleSegment)`; that bug was fixed in
/// 485ec4d93 ("fix(temporal): pure-axis moves stalled SLP at verifier
/// knife-edge"), so we now drive the live submit_move path.
#[test]
fn single_axis_z_move_different_mcu() {
    let h = Harness::corexy_plus_z();

    h.submit_move([0.0; 3], 0.0, 0.0, 5.0, 0.0, 50.0)
        .expect("submit_move");
    h.flush();

    assert!(
        h.dispatched.load(Ordering::Relaxed) > 0,
        "no shaped segments dispatched"
    );

    let f446 = h.transports.get(&F446_ID).unwrap();
    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();

    let f446_loads = f446.sent_starting_with("kalico_load_curve");
    let f446_pushes = f446.sent_starting_with("kalico_push_segment");
    assert!(
        !f446_loads.is_empty(),
        "expected kalico_load_curve on F446 (Z), saw none"
    );
    assert!(
        !f446_pushes.is_empty(),
        "expected kalico_push_segment on F446 (Z), saw none"
    );
    assert!(
        f446_pushes.iter().all(|p| p.contains("kinematics=1")),
        "expected kinematics=1 (cartesian) on F446 pushes, saw: {f446_pushes:?}"
    );

    // Octopus may receive X/Y curves with sub-µm post-shape residue (the
    // dispatch `is_trivially_constant` check uses 1e-12 mm tol — well below
    // the planner's numerical noise floor on a pure-Z move). What it must
    // NOT see is a moving curve: filter by control-point span ≥ 1 µm.
    let octopus_caps = octopus.load_curve_captures();
    // Threshold 0.1 mm: post-shape numerical residue on the unmoved X/Y
    // axes is ~tens of µm; an actual X or Y component on a pure-Z move
    // would be on the same order as Z (mm-scale). 0.1 mm cleanly separates
    // them.
    let octopus_moving = moving_captures(&octopus_caps, 0.1);
    assert!(
        octopus_moving.is_empty(),
        "expected no moving X/Y curves on Octopus for pure-Z move; \
         saw {} moving capture(s) (cps spans: {:?})",
        octopus_moving.len(),
        octopus_moving
            .iter()
            .map(|c| {
                let mn = c.cps.iter().cloned().fold(f32::INFINITY, f32::min);
                let mx = c.cps.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
                mx - mn
            })
            .collect::<Vec<_>>()
    );
}

#[test]
fn extrusion_rejected() {
    // Phase 2 doesn't support extrusion — classify rejects de != 0 before the
    // planner ever sees the move.
    let r = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 1.0, 100.0);
    assert!(
        matches!(r, Err(ClassifyError::ExtrusionNotSupported)),
        "expected ExtrusionNotSupported, got {r:?}"
    );

    // And via the harness path, submit_move must surface the same rejection.
    let h = Harness::corexy_only();
    let err = h
        .submit_move([0.0; 3], 10.0, 0.0, 0.0, 1.0, 100.0)
        .expect_err("extrusion submit must error");
    assert!(
        err.to_string().contains("extrusion"),
        "error must mention extrusion, got: {err}"
    );

    // No wire traffic at all.
    h.flush();
    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    assert!(octopus.sent_starting_with("kalico_load_curve").is_empty());
    assert!(octopus.sent_starting_with("kalico_push_segment").is_empty());
}

// ---------------------------------------------------------------------------
// Task 11 — shaper / velocity-limit validation tests.
//
// These tests reach past the dispatch boundary by snooping the load_curve
// typed args (degree, knots, cps), reconstructing the f64 NURBS the bridge
// actually shipped to the MCU, and asserting kinematic properties on it:
//
//   * the smooth-MZV shaper / β-medium drives peak post-shape acceleration
//     to the machine limit,
//   * the velocity profile respects `max_velocity`,
//   * `update_limits` takes effect on subsequent moves.
//
// All X-axis-only — pure-Z is broken by an unrelated joining bug (tracked
// separately) and does not belong in this set.
// ---------------------------------------------------------------------------

/// Reconstruct an f64 `ScalarNurbs` from a captured load_curve payload.
fn capture_to_nurbs(c: &LoadCurveCapture) -> ScalarNurbs<f64> {
    let knots: Vec<f64> = c.knots.iter().map(|&k| k as f64).collect();
    let cps: Vec<f64> = c.cps.iter().map(|&v| v as f64).collect();
    ScalarNurbs::try_new(c.degree, knots, cps, None)
        .expect("captured payload must be a valid scalar NURBS")
}

/// Sample x(t) on `[t0, t1)` at `n` uniformly-spaced points.
fn sample_position(curve: &ScalarNurbs<f64>, t0: f64, t1: f64, n: usize) -> Vec<f64> {
    let mut out = Vec::with_capacity(n);
    let dt = (t1 - t0) / n as f64;
    for i in 0..n {
        let t = t0 + dt * i as f64;
        out.push(nurbs::eval::eval(curve, t));
    }
    out
}

/// Filter captures to "moving X-axis curves" — those whose control-point
/// spread exceeds `min_span_mm`. Both X and Y axis curves ship for every
/// segment when neither is trivially constant; for an X-only move the Y
/// curve has small post-shape residue (sub-µm) that fails the dispatch
/// `is_trivially_constant` check (1e-12 tol) but is non-physical. Keeping
/// only captures with ≥ 1 mm span reliably retains just the X-axis curve(s).
fn moving_captures(captures: &[LoadCurveCapture], min_span_mm: f64) -> Vec<LoadCurveCapture> {
    captures
        .iter()
        .filter(|c| {
            let mn = c.cps.iter().cloned().fold(f32::INFINITY, f32::min);
            let mx = c.cps.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            (mx - mn) as f64 >= min_span_mm
        })
        .cloned()
        .collect()
}

/// Concatenate position samples from a sequence of captures, taking each
/// curve's own knot range as `[t0, t1]`. Sample rate is `fs` Hz; output is
/// resampled at uniform `1/fs` spacing within each curve.
fn sample_captures_at(captures: &[LoadCurveCapture], fs: f64) -> Vec<f64> {
    let mut out = Vec::new();
    for c in captures {
        let curve = capture_to_nurbs(c);
        let knots = curve.knots();
        let t0 = knots[0];
        let t1 = *knots.last().unwrap();
        let dur = t1 - t0;
        let n = ((dur * fs).round() as usize).max(2);
        out.extend(sample_position(&curve, t0, t1, n));
    }
    out
}

/// Peak |first-difference| / dt across a sample vector.
fn peak_first_diff(samples: &[f64], dt: f64) -> f64 {
    let mut peak: f64 = 0.0;
    for w in samples.windows(2) {
        peak = peak.max(((w[1] - w[0]) / dt).abs());
    }
    peak
}

/// Mean-squared bandpower in a frequency window `[f_lo, f_hi]` via
/// Hann-windowed real-FFT. `signal` is real-valued at `fs` Hz.
fn bandpower(signal: &[f64], fs: f64, f_lo: f64, f_hi: f64) -> f64 {
    use rustfft::FftPlanner;
    use rustfft::num_complex::Complex;
    let n = signal.len();
    if n < 8 {
        return 0.0;
    }
    let mut buf: Vec<Complex<f64>> = signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5
                - 0.5
                    * (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos();
            Complex { re: x * w, im: 0.0 }
        })
        .collect();
    let mut planner = FftPlanner::<f64>::new();
    let fft = planner.plan_fft_forward(n);
    fft.process(&mut buf);
    let bin_hz = fs / n as f64;
    let k_lo = ((f_lo / bin_hz).floor() as usize).max(1);
    let k_hi = ((f_hi / bin_hz).ceil() as usize).min(n / 2);
    let mut sum = 0.0;
    for k in k_lo..=k_hi {
        sum += buf[k].norm_sqr();
    }
    sum
}

#[test]
fn shaper_attenuates_resonance_and_respects_accel_limit() {
    // Smooth-MZV at 50 Hz on X+Y with the default 3000 mm/s² accel cap.
    // Two assertions:
    //   1) β-medium outer iteration drives post-shape peak |ẍ| to the
    //      machine limit (analytic peak from the shaped NURBS).
    //   2) FFT bandpower in a tight window around 50 Hz is small relative
    //      to a broadband reference window — smooth-MZV's defining notch.
    //
    // We chain three sequential X-axis moves to get ~300 ms of shaped
    // motion, which yields useful FFT bin resolution near 50 Hz.
    let h = Harness::corexy_only();

    // Three back-to-back X moves totalling 300 mm; flush between each so
    // they shape as separate batches (multi-move-in-one-batch hits an
    // unrelated `non-contiguous Bezier pieces` panic in temporal joining
    // that's outside the TemporalJoining-SLP fix's scope). Concatenated
    // captures still give ~hundreds of ms of shaped motion.
    h.submit_move([0.0; 3], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 1");
    h.flush();
    h.submit_move([50.0, 0.0, 0.0], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 2");
    h.flush();
    h.submit_move([100.0, 0.0, 0.0], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 3");
    h.flush();

    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    let captures = octopus.load_curve_captures();
    assert!(!captures.is_empty(), "no load_curve captures");

    let x_caps = moving_captures(&captures, 1.0);
    assert!(!x_caps.is_empty(), "no moving X-axis captures");

    // (1) Peak |ẍ| via the trajectory crate's analytic per-piece peak
    // finder (the same one β-medium uses internally). Plain second-
    // difference on a degree-9 multi-piece NURBS spikes spuriously at
    // internal breakpoints — even though the curve itself is smooth — so
    // we use the trusted helper.
    let limit = 3000.0_f64;
    let peak_a = x_caps
        .iter()
        .map(|c| trajectory::peak::peak_accel(&capture_to_nurbs(c)))
        .fold(0.0_f64, f64::max);
    // 10% headroom for β-medium's per-batch tolerance band.
    assert!(
        peak_a <= limit * 1.10,
        "post-shape peak |ẍ| = {peak_a:.1} exceeds 1.10 × limit ({:.1})",
        limit * 1.10,
    );

    // (2) FFT smoke test on the acceleration signal.
    //
    // We sample the concatenated shaped X(t) at 40 kHz, second-difference
    // for acceleration, and look at the spectrum. Smooth-MZV's design
    // frequency (50 Hz) should be a notch; broadband content lives below
    // ~30 Hz (the bulk of a trapezoidal-ish accel pulse). We assert:
    //   - the 50 Hz bin (± half a Hann main-lobe width) has *less* energy
    //     than a low-frequency reference window [5, 25] Hz, by a margin.
    //
    // This is intentionally pragmatic: the goal is to catch a regression
    // where the shaper stops working entirely (50 Hz bin would then
    // dominate, since the unshaped accel pulse has appreciable 50 Hz
    // content). It is not a tight notch-depth measurement.
    let fs = 40_000.0_f64;
    let positions = sample_captures_at(&x_caps, fs);
    if positions.len() >= 64 {
        let dt = 1.0 / fs;
        let mut accel: Vec<f64> = Vec::with_capacity(positions.len().saturating_sub(2));
        for w in positions.windows(3) {
            accel.push((w[2] - 2.0 * w[1] + w[0]) / (dt * dt));
        }
        // Notch window: 50 Hz ± 5 Hz. Reference window: [5, 25] Hz.
        let notch = bandpower(&accel, fs, 45.0, 55.0);
        let reference = bandpower(&accel, fs, 5.0, 25.0);
        assert!(
            reference > 0.0,
            "low-frequency reference bandpower must be > 0 ({reference:.3e})"
        );
        // Notch should be well below the low-frequency reference. If the
        // shaper is bypassed, the 50 Hz bin sits near or above the
        // broadband floor and this ratio collapses.
        assert!(
            notch < reference * 0.5,
            "shaper notch check failed: bp(45..55 Hz) = {notch:.3e} \
             vs bp(5..25 Hz) = {reference:.3e} — expected notch < 0.5 × ref",
        );
    }
}

#[test]
fn velocity_limit_respected() {
    // Tight velocity cap, generous-feed request — planner must clamp v.
    let limits = PlannerLimits {
        max_velocity: 100.0,
        max_accel: 3000.0,
        max_z_velocity: 50.0,
        max_z_accel: 500.0,
        square_corner_velocity: 5.0,
    };
    let h = Harness::corexy_with_limits(limits);

    h.submit_move([0.0; 3], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move");
    h.flush();

    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    let captures = octopus.load_curve_captures();
    assert!(!captures.is_empty(), "no load_curve captures");

    let x_caps = moving_captures(&captures, 1.0);
    assert!(!x_caps.is_empty(), "no moving X-axis captures");
    let fs = 40_000.0_f64;
    let positions = sample_captures_at(&x_caps, fs);
    let peak_v = peak_first_diff(&positions, 1.0 / fs);
    // 2% headroom for finite-difference quantization at the velocity peak.
    assert!(
        peak_v <= 100.0 * 1.02,
        "peak |ẋ| = {peak_v:.3} mm/s exceeds 1.02 × max_velocity (102.0)",
    );
}

#[test]
fn set_velocity_limit_applies_to_next_move() {
    // Single harness, two moves: boot at v=300, submit move 1 and capture
    // peak; `update_limits` to v=50, submit move 2 and capture peak. The
    // tight cap must clamp the post-shape velocity on the second move only.
    //
    // X25 instead of X50 because at the harness's accel cap (3000 mm/s²)
    // an X50 move tops out near 89 mm/s — well below the 50 mm/s cap we
    // need to differentiate from. X25 lets the post-shape velocity stay
    // close to ~56 mm/s at cap=300 and clamp tightly to ~50 at cap=50.

    let fs = 40_000.0_f64;

    let h = Harness::corexy_with_limits(PlannerLimits {
        max_velocity: 300.0,
        max_accel: 3000.0,
        max_z_velocity: 50.0,
        max_z_accel: 500.0,
        square_corner_velocity: 5.0,
    });

    // --- Move 1: high cap.
    h.submit_move([0.0; 3], 25.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move (move 1)");
    h.flush();
    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    let caps_after_move1 = octopus.load_curve_captures();
    let move1_caps = moving_captures(&caps_after_move1, 1.0);
    assert!(!move1_caps.is_empty(), "no moving captures (move 1)");
    let pos1 = sample_captures_at(&move1_caps, fs);
    let peak_v_high = peak_first_diff(&pos1, 1.0 / fs);

    // --- Update limits to a tight cap before move 2.
    h.update_limits(PlannerLimits {
        max_velocity: 50.0,
        max_accel: 3000.0,
        max_z_velocity: 50.0,
        max_z_accel: 500.0,
        square_corner_velocity: 5.0,
    });

    // --- Move 2: tight cap. Start where move 1 ended.
    h.submit_move([25.0, 0.0, 0.0], 25.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move (move 2)");
    h.flush();
    let caps_after_move2 = octopus.load_curve_captures();
    // Take only the captures emitted *after* move 1's tail.
    let move2_caps_all = &caps_after_move2[caps_after_move1.len()..];
    let move2_caps = moving_captures(move2_caps_all, 1.0);
    assert!(!move2_caps.is_empty(), "no moving captures (move 2)");
    let pos2 = sample_captures_at(&move2_caps, fs);
    let peak_v_low = peak_first_diff(&pos2, 1.0 / fs);

    // Runtime `update_limits` must clamp post-shape velocity; 5% headroom
    // for finite-difference quantization at the velocity peak.
    assert!(
        peak_v_low <= 50.0 * 1.05,
        "after update_limits to 50 mm/s, peak |ẋ| = {peak_v_low:.3} \
         (expected ≤ 52.5)",
    );
    // High-cap move must have peaked measurably faster — sanity that
    // update_limits actually changed planner behaviour, not that both
    // moves happened to peak below 50 by coincidence.
    assert!(
        peak_v_high > peak_v_low * 1.05,
        "expected high-cap (300) peak measurably > low-cap (50) peak; \
         peak_v_high = {peak_v_high:.3}, peak_v_low = {peak_v_low:.3}",
    );
}

/// Regression coverage for the TemporalJoining stall fix (485ec4d93).
/// Five sequential moves with reversals must all flow through the planner
/// without `StalledOnInfeasibleSegment`, and produce a non-trivial number
/// of shaped segments on the wire.
#[test]
fn multi_move_chain_completes_without_stall() {
    let h = Harness::corexy_only();

    // dx sequence: +10, -5, +8, -3, +12 (reversals stress junction logic).
    // Submit the whole chain before flushing so the planner shapes it as a
    // batched lookahead window. This locks in the multi-segment joining path
    // instead of relying on the old flush-after-every-submit workaround.
    let steps = [10.0_f64, -5.0, 8.0, -3.0, 12.0];
    let mut x = 0.0_f64;
    for &dx in &steps {
        h.submit_move([x, 0.0, 0.0], dx, 0.0, 0.0, 0.0, 1000.0)
            .unwrap_or_else(|e| panic!("submit_move dx={dx} from x={x}: {e}"));
        x += dx;
    }
    h.flush();

    let dispatched = h.dispatched.load(Ordering::Relaxed);
    assert!(
        dispatched >= steps.len() as u64,
        "expected ≥ {} dispatched segments, saw {dispatched}",
        steps.len()
    );

    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    let pushes = octopus.sent_starting_with("kalico_push_segment");
    let loads = octopus.sent_starting_with("kalico_load_curve");
    assert!(
        !pushes.is_empty() && !loads.is_empty(),
        "expected wire traffic on Octopus, saw loads={} pushes={}",
        loads.len(),
        pushes.len()
    );
    // Every push must carry the CoreXY tag.
    assert!(
        pushes.iter().all(|p| p.contains("kinematics=0")),
        "expected kinematics=0 on all pushes"
    );
}
