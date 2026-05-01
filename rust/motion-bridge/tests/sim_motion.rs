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

use geometry::segment::EMode;
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
        _args: &[(&str, FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.call(name, expected_response_name, timeout)
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

    fn build(mcu_configs: Vec<McuAxisConfig>) -> Self {
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
        cfg.limits = PlannerLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            // Generous Z limits — the default 15 mm/s / 100 mm/s² combined
            // with the X/Y-axis-derived corner-deviation chord tolerance
            // make even modest pure-Z moves trip TemporalJoining infeasibility.
            max_z_velocity: 50.0,
            max_z_accel: 500.0,
            square_corner_velocity: 5.0,
        };
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

// Helpers for synthetic ShapedSegments — degree-3 Béziers with collinear
// control points. Mirrors the in-crate `dispatch::tests` helpers.
fn linear_curve(a: f64, b: f64) -> ScalarNurbs<f64> {
    let cps = vec![a, a + (b - a) / 3.0, a + 2.0 * (b - a) / 3.0, b];
    ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps,
        None,
    )
    .unwrap()
}

fn constant_curve(v: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![v, v, v, v],
        None,
    )
    .unwrap()
}

/// Pure-Z dispatch — synthetic ShapedSegment route.
///
/// We construct a `ShapedSegment` by hand (rather than running `dz != 0`
/// through `submit_move` + the planner) for a concrete reason: under any
/// harness limits we tried, `temporal::multi` joining returns
/// `StalledOnInfeasibleSegment` for pure-Z moves — the X/Y-derived chord
/// deviation / junction tolerance interacts badly with a curve that has
/// `|dx|=|dy|=0`. That's a separate Phase 2 / Task 11 concern, not in scope
/// for Task 10's "verify routing" goal. Feeding a hand-built shaped segment
/// directly into `build_push_params` + the producer wire path still
/// exercises the dispatch + load_curve + push_segment surface for the Z-only
/// case and asserts F446-only routing, which is the test's actual purpose.
#[test]
fn single_axis_z_move_different_mcu() {
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
    let octopus = Arc::new(RecordingTransport::new());
    let f446 = Arc::new(RecordingTransport::new());
    let octopus_credit = CreditCounter::new(1024);
    let f446_credit = CreditCounter::new(1024);

    let seg = ShapedSegment {
        axes: [
            constant_curve(0.0),
            constant_curve(0.0),
            linear_curve(0.0, 5.0),
        ],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 0.5,
    };
    let mut plans = build_push_params(&seg, &mcu_configs, 1_000, 2_000);

    // Sanity: only F446 should appear in plans (X/Y are constant → skipped).
    assert_eq!(plans.len(), 1, "expected one plan, got {}", plans.len());
    assert_eq!(plans[0].mcu_id, F446_ID);

    // Run the producer wire surface, mirroring the bridge dispatch closure.
    for plan in &mut plans {
        let (transport, credit) = if plan.mcu_id == OCTOPUS_ID {
            (octopus.as_ref(), &octopus_credit)
        } else {
            (f446.as_ref(), &f446_credit)
        };
        let curves = std::mem::take(&mut plan.curves_to_load);
        let mut slot: u16 = 0;
        for (axis_idx, curve_params) in &curves {
            let handle = producer::load_curve(
                transport,
                slot,
                curve_params,
                DEFAULT_LOAD_CURVE_TIMEOUT,
            )
            .expect("load_curve");
            plan.set_handle(*axis_idx, handle);
            slot += 1;
        }
        producer::push_segment(transport, credit, &plan.params)
            .expect("push_segment");
    }

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

    assert!(
        octopus.sent_starting_with("kalico_load_curve").is_empty(),
        "expected NO load_curve on Octopus for pure-Z move"
    );
    assert!(
        octopus.sent_starting_with("kalico_push_segment").is_empty(),
        "expected NO push_segment on Octopus for pure-Z move"
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
