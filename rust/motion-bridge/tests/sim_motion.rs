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
use std::collections::VecDeque;
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
use motion_bridge::homing::{HomingSegmentState, HomingState};
use motion_bridge::planner::PlannerHandle;
use motion_bridge::slot_pool::{SlotPool, CURVE_POOL_N};
use runtime::endstop::{self, ArmMsg, ArmPolicy, SourceConfig, SourceKind, VelocityAxis};

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
    axis_idx: Option<usize>,
    degree: u8,
    knots: Vec<f32>,
    cps: Vec<f32>,
}

#[derive(Default)]
struct TransportState {
    sent: Vec<CallRecord>,
    next_handle_lo: u32,
    next_segment_id: u32,
    pending_load_axes: VecDeque<usize>,
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
                pending_load_axes: VecDeque::new(),
            }),
        }
    }

    fn note_next_load_axis(&self, axis_idx: usize) {
        self.state
            .lock()
            .unwrap()
            .pending_load_axes
            .push_back(axis_idx);
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

    fn moving_capture_duration_pairs(
        &self,
        axis_idx: Option<usize>,
        min_span_mm: f64,
    ) -> Vec<(LoadCurveCapture, f64)> {
        let records = self.state.lock().unwrap();
        let mut pending_loads: Vec<LoadCurveCapture> = Vec::new();
        let mut pairs = Vec::new();
        for record in &records.sent {
            if let Some(c) = &record.load_curve {
                pending_loads.push(c.clone());
                continue;
            }
            if record.cmd.starts_with("kalico_push_segment") {
                if let Some(duration) = parse_push_duration_s(&record.cmd) {
                    for c in pending_loads.drain(..) {
                        if axis_idx.is_none_or(|axis| c.axis_idx == Some(axis))
                            && capture_motion_mm(&c) >= min_span_mm
                        {
                            pairs.push((c, duration));
                        }
                    }
                } else {
                    pending_loads.clear();
                }
            }
        }
        pairs
    }
}

fn parse_cmd_field_u64(cmd: &str, field: &str) -> Option<u64> {
    let prefix = format!("{field}=");
    cmd.split_whitespace()
        .find_map(|token| token.strip_prefix(&prefix))
        .and_then(|v| v.parse::<u64>().ok())
}

fn parse_push_duration_s(cmd: &str) -> Option<f64> {
    let t_start_hi = parse_cmd_field_u64(cmd, "t_start_hi")?;
    let t_start_lo = parse_cmd_field_u64(cmd, "t_start_lo")?;
    let t_end_hi = parse_cmd_field_u64(cmd, "t_end_hi")?;
    let t_end_lo = parse_cmd_field_u64(cmd, "t_end_lo")?;
    let t_start = (t_start_hi << 32) | t_start_lo;
    let t_end = (t_end_hi << 32) | t_end_lo;
    Some((t_end.saturating_sub(t_start)) as f64 / 1_000_000.0)
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
            let axis_idx = self.state.lock().unwrap().pending_load_axes.pop_front();
            match (degree, cps_bytes, knots_bytes) {
                (Some(d), Some(cb), Some(kb)) => Some(LoadCurveCapture {
                    axis_idx,
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
static ENDSTOP_TEST_MUTEX: Mutex<()> = Mutex::new(());

struct Harness {
    planner: Option<PlannerHandle>,
    transports: HashMap<u32, Arc<RecordingTransport>>,
    dispatched: Arc<AtomicU64>,
    /// Per-MCU slot pool — same data structure the bridge uses, exposed
    /// to tests so they can simulate `kalico_credit_freed` retirement
    /// events.
    slot_pools: HashMap<u32, Arc<Mutex<SlotPool>>>,
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
        let mut slot_pools: HashMap<u32, Arc<Mutex<SlotPool>>> = HashMap::new();
        for cfg in &mcu_configs {
            transports.insert(cfg.mcu_id, Arc::new(RecordingTransport::new()));
            credits.insert(cfg.mcu_id, Arc::new(CreditCounter::new(1024)));
            slot_pools.insert(cfg.mcu_id, Arc::new(Mutex::new(SlotPool::new())));
        }

        let dispatched = Arc::new(AtomicU64::new(0));
        let counter = Arc::clone(&dispatched);

        // Capture per-MCU state into the dispatch closure.
        let cb_transports = transports.clone();
        let cb_credits = credits.clone();
        let cb_slot_pools = slot_pools.clone();
        let cb_mcu_configs = mcu_configs.clone();

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

                let pool = cb_slot_pools.get(&plan.mcu_id).unwrap().clone();
                let curves = std::mem::take(&mut plan.curves_to_load);
                let mut allocated_slots: Vec<u16> = Vec::with_capacity(curves.len());
                for (axis_idx, curve_params) in &curves {
                    let (slot, _gen) = pool
                        .lock()
                        .unwrap()
                        .try_alloc()
                        .ok_or_else(|| {
                            format!(
                                "slot pool exhausted for mcu={}",
                                plan.mcu_id
                            )
                        })?;
                    allocated_slots.push(slot);
                    transport.note_next_load_axis(*axis_idx);
                    let handle = producer::load_curve(
                        transport.as_ref(),
                        slot,
                        curve_params,
                        DEFAULT_LOAD_CURVE_TIMEOUT,
                    )
                    .map_err(|e| {
                        pool.lock().unwrap().release(slot);
                        format!("load_curve mcu={}: {e}", plan.mcu_id)
                    })?;
                    plan.set_handle(*axis_idx, handle);
                }
                {
                    let mut p = pool.lock().unwrap();
                    for slot in &allocated_slots {
                        p.register_segment(*slot, plan.params.id);
                    }
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
            // Generous Z limits — the default 15 mm/s / 100 mm/s² were
            // originally chosen to avoid a now-fixed TemporalJoining
            // infeasibility (485ec4d93); kept generous so the harness default
            // doesn't constrain Z-axis test scenarios.
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
            slot_pools,
        }
    }

    /// Simulate a `kalico_credit_freed` event for the given MCU. Releases
    /// curve slots whose owning segment id is `<= retired_through`.
    /// Mirrors what `PyMotionBridge::on_credit_freed` does at runtime.
    fn simulate_credit_freed(&self, mcu: u32, retired_through: u32) -> usize {
        self.slot_pools
            .get(&mcu)
            .map(|p| {
                p.lock()
                    .unwrap()
                    .retire_through_segment(retired_through)
            })
            .unwrap_or(0)
    }

    fn slot_pool_in_flight(&self, mcu: u32) -> usize {
        self.slot_pools
            .get(&mcu)
            .map(|p| p.lock().unwrap().in_flight_count())
            .unwrap_or(0)
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

fn arm_test_endstop(arm_id: u32, gpio: u16) {
    let mut sources = [SourceConfig::EMPTY; endstop::MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    let _ = endstop::arm(ArmMsg {
        arm_id,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
    });
}

#[test]
fn homing_move_trip_wait_returns_and_exposes_trip() {
    let _guard = ENDSTOP_TEST_MUTEX.lock().unwrap();
    let _ = endstop::disarm(9001);
    let _ = endstop::disarm(9002);
    let _ = endstop::poll_trip();
    endstop::set_pin_level(21, false);
    endstop::set_pin_level(22, false);

    let homing = HomingState::new();
    arm_test_endstop(9001, 21);
    homing.begin(9001);
    homing.mark_dispatched_segment(1);
    assert_eq!(homing.state(), HomingSegmentState::Active);

    endstop::set_pin_level(21, true);
    assert_eq!(
        endstop::tick(42, [0, 0, 0], &[123]),
        endstop::TripAction::AbortNow
    );
    homing.refresh_after_wait();
    assert_eq!(homing.state(), HomingSegmentState::Tripped);
    let trip = homing.take_trip_event().expect("trip event");
    assert_eq!(trip.arm_id, 9001);
    assert_eq!(trip.steppers[0].step_count, 123);
}

#[test]
fn homing_move_without_trip_completes() {
    let _guard = ENDSTOP_TEST_MUTEX.lock().unwrap();
    let _ = endstop::disarm(9001);
    let _ = endstop::disarm(9002);
    let _ = endstop::poll_trip();
    endstop::set_pin_level(21, false);
    endstop::set_pin_level(22, false);

    let homing = HomingState::new();
    arm_test_endstop(9002, 22);
    homing.begin(9002);
    homing.mark_dispatched_segment(1);
    homing.refresh_after_wait();
    assert_eq!(homing.state(), HomingSegmentState::Completed);
    assert!(homing.take_trip_event().is_none());
    let _ = endstop::disarm(9002);
}

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
// All X-axis-only — pure-Z joining is now sound (fixed in the same batch as
// the TemporalJoining-SLP fix), but these tests are intentionally scoped to
// X-axis kinematics.
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
        .filter(|c| capture_motion_mm(c) >= min_span_mm)
        .cloned()
        .collect()
}

fn capture_motion_mm(c: &LoadCurveCapture) -> f64 {
    if c.cps.is_empty() {
        return 0.0;
    }
    (f64::from(*c.cps.last().unwrap()) - f64::from(c.cps[0])).abs()
}

fn sample_capture_duration_pairs_at(pairs: &[(LoadCurveCapture, f64)], fs: f64) -> Vec<f64> {
    let mut out = Vec::new();
    for (c, duration) in pairs {
        let curve = capture_to_nurbs(c);
        let n = ((duration * fs).round() as usize).max(2);
        out.extend(sample_position(&curve, 0.0, 1.0, n));
    }
    out
}

/// Peak physical |x''(t)| for a wire curve whose knot domain is normalized
/// to segment progress u in [0, 1].
fn physical_peak_accel_from_normalized_capture(c: &LoadCurveCapture, duration_s: f64) -> f64 {
    assert!(duration_s > 0.0, "segment duration must be positive");
    trajectory::peak::peak_accel(&capture_to_nurbs(c)) / (duration_s * duration_s)
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

    // Three back-to-back X moves totalling 300 mm submitted in one batch;
    // batched multi-segment dispatch is sound now that convolve final-piece
    // corruption (bug #18, fixed in 4812ac647) and all its cascade follow-ups
    // (f4dcafaf8, 0b23ecca4, 79c5a5047, e02deb0cf) are resolved.
    h.submit_move([0.0; 3], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 1");
    h.submit_move([50.0, 0.0, 0.0], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 2");
    h.submit_move([100.0, 0.0, 0.0], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 3");
    h.flush();

    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    let captures = octopus.load_curve_captures();
    assert!(!captures.is_empty(), "no load_curve captures");

    let x_pairs = octopus.moving_capture_duration_pairs(Some(AXIS_X), 1.0);
    assert!(!x_pairs.is_empty(), "no moving X-axis captures");

    // (1) Peak |ẍ| via the trajectory crate's analytic per-piece peak
    // finder (the same one β-medium uses internally). Plain second-
    // difference on a degree-9 multi-piece NURBS spikes spuriously at
    // internal breakpoints — even though the curve itself is smooth — so
    // we use the trusted helper.
    let limit = 3000.0_f64;
    let peak_a = x_pairs
        .iter()
        .map(|(c, duration)| physical_peak_accel_from_normalized_capture(c, *duration))
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
    let positions = sample_capture_duration_pairs_at(&x_pairs, fs);
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

    let x_pairs = octopus.moving_capture_duration_pairs(Some(AXIS_X), 1.0);
    assert!(!x_pairs.is_empty(), "no moving X-axis captures");
    let fs = 40_000.0_f64;
    let positions = sample_capture_duration_pairs_at(&x_pairs, fs);
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
    let move1_pairs = octopus.moving_capture_duration_pairs(Some(AXIS_X), 1.0);
    assert!(!move1_pairs.is_empty(), "no moving captures (move 1)");
    let pos1 = sample_capture_duration_pairs_at(&move1_pairs, fs);
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
    let pairs_after_move2 = octopus.moving_capture_duration_pairs(Some(AXIS_X), 1.0);
    let move2_pairs = &pairs_after_move2[move1_pairs.len()..];
    assert!(!move2_pairs.is_empty(), "no moving captures (move 2)");
    let pos2 = sample_capture_duration_pairs_at(move2_pairs, fs);
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

/// Regression: batched two-move dispatch — the second segment in a single
/// planner window must ship a curve in the right rough order of magnitude:
/// finite duration, finite span, control-point count not blown up by ~10x.
///
/// History:
/// - Bug #17 (derate cascade): seg[1] shipped as `span = 8.07e22 mm`,
///   `duration = 5000 s`, `ncps = 1792` — total numerical corruption.
///   Mechanism: a derate cascade in `trajectory::beta`. Pure-X submit gives Y
///   axis a sub-mm pre-shape span; post-shape Y `peak_accel` is dominated by
///   shaper-boundary numerical transients (the kernel's `c = 15/(16 h^5)`
///   constant amplifies short-piece coefficients through double
///   differentiation). β-medium derates Y `planning_a_max` toward zero based
///   on this fake peak; the next iteration's seg[1] re-plans with the clamped
///   Y limit and explodes. Fixed by two guards in `beta.rs`:
///   `MIN_AXIS_SPAN_FOR_DERATE = 0.5 mm` and `BETA_ACCEL_MIN_RATIO = 0.02`.
///
/// - Bug #18 (convolve final-piece corruption): after fixing #17, seg[1]
///   shipped a curve evaluating correctly at u<99% then jumping ~14 mm at
///   the final knot — `cp[last] ≈ 114` instead of ~100. Mechanism:
///   `nurbs::algebra::convolve`'s `integrate_product_piece` did its
///   monomial arithmetic in absolute-u basis and re-shifted to Pascal-at-α
///   at the end. With α ≈ 2 (second segment of a batch starts at t ≈ 0.7s
///   and ends at ≈ 2s) and a degree-9 output (degree-4 input × degree-4
///   smooth-MZV kernel), the absolute-u coefficients reached u^9 ≈ 512×,
///   then `absolute_to_pascal_shift` summed alternating-sign binomial
///   products — catastrophic cancellation killed ~10 digits on the trailing
///   tiny piece (width = kernel half-support). Fixed by doing all integrand
///   arithmetic in the (u−α, s−α) frame so every intermediate coefficient is
///   O(width^k), eliminating the lossy re-shift.
///
/// With both fixes, post-shape span ≈ 50 mm, duration ≈ 0.86 s, and
/// `cp[last]` matches the physical endpoint within sub-µm.
#[test]
fn batched_two_move_curves_are_sane() {
    let h = Harness::corexy_only();
    // Two X-axis moves into one batch, single trailing flush.
    h.submit_move([0.0; 3], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 1");
    h.submit_move([50.0, 0.0, 0.0], 50.0, 0.0, 0.0, 0.0, 1000.0)
        .expect("submit_move 2");
    h.flush();

    let octopus = h.transports.get(&OCTOPUS_ID).unwrap();
    // Both segments must have dispatched.
    assert!(
        h.dispatched.load(Ordering::Relaxed) >= 2,
        "expected ≥2 dispatched segments"
    );

    // Collect (curve, duration) pairs for the X axis only, minimum 1 mm span.
    let pairs = octopus.moving_capture_duration_pairs(Some(AXIS_X), 1.0);
    assert_eq!(
        pairs.len(),
        2,
        "expected exactly 2 moving X-axis captures (one per segment), got {}",
        pairs.len()
    );

    let (c0, dur0) = &pairs[0];
    let (c1, dur1) = &pairs[1];

    // ---- sanity: segment 0 ----
    // Seg[0] is the leading segment in the batch and is unaffected by the
    // batched-tail-piece artifact, so we can pin it tightly.
    assert!(
        *dur0 > 0.0 && *dur0 < 10.0,
        "[0] duration={dur0:.4}s out of range [0, 10]"
    );
    let span0 = capture_motion_mm(c0);
    assert!(
        (span0 - 50.0).abs() < 1.0,
        "[0] span={span0:.4}mm, expected ~50mm"
    );

    // ---- sanity: segment 1 (the previously-corrupt one) ----
    // With both #17 (derate cascade) and #18 (convolve final-piece) fixed,
    // seg[1] should be physically tight: 50 mm of motion, ≤ 1 s under the
    // 3000 mm/s² limit + smooth-MZV @ 50 Hz, and post-shape peak accel
    // bounded by the 10 % shaper-aware derate margin.
    assert!(
        *dur1 > 0.0 && *dur1 < 1.0,
        "[1] duration={dur1:.4}s out of physical range — likely a derate cascade (pre-fix was 5000s)"
    );
    let span1 = capture_motion_mm(c1);
    assert!(
        (span1 - 50.0).abs() < 1.0,
        "[1] span={span1:.4}mm — expected ~50mm (pre-fix bug #18 was ~64mm with corrupted final piece)"
    );
    // Peak accel must respect the shaper-aware β-medium target.
    let peak1 = physical_peak_accel_from_normalized_capture(c1, *dur1);
    assert!(
        peak1 < 3300.0,
        "[1] peak_accel={peak1:.1} mm/s² exceeds shaper-aware ceiling 3300 (machine 3000 + 10%)"
    );
    // CP count must be in the same ballpark as segment 0 (not 1792 vs ~136).
    assert!(
        c1.cps.len() < c0.cps.len() * 4,
        "[1] ncps={} wildly exceeds [0] ncps={} — control-point explosion",
        c1.cps.len(),
        c0.cps.len()
    );
}

/// Regression: dispatch must not blow up the firmware curve-pool capacity
/// (`CURVE_POOL_N = 64`). Pre-fix the dispatch closure rolled a u16 slot
/// counter so any 65th-and-onward slot would be rejected by firmware
/// bounds-check. Post-fix, the bridge owns a real `SlotPool` and slot
/// reuse is gated on `kalico_credit_freed` retirement events.
///
/// This test:
///   1. Submits N moves (alternating direction) and flushes between
///      submissions so segments enter the dispatch closure faster than
///      they're retired.
///   2. After each flush, simulates a `kalico_credit_freed` event with
///      the latest known segment id, releasing every in-flight slot.
///   3. Asserts no exhaustion occurred (the dispatch closure would have
///      surfaced "slot pool exhausted" via PlannerError::Dispatch
///      otherwise) AND that the in-flight count is bounded.
#[test]
fn slot_pool_recycles_via_credit_freed_events() {
    let h = Harness::corexy_only();

    // 100 short X moves — well past the 64-slot pool capacity. Without
    // recycling this would either error or wedge the planner.
    let n = 100usize;
    let mut x = 0.0_f64;
    for i in 0..n {
        let dx = if i % 2 == 0 { 5.0 } else { -5.0 };
        h.submit_move([x, 0.0, 0.0], dx, 0.0, 0.0, 0.0, 1000.0)
            .unwrap_or_else(|e| panic!("submit_move {i}: {e}"));
        x += dx;
        h.flush();

        // Simulate the firmware retiring everything up through the most
        // recent dispatched segment. The harness allocates segment ids
        // monotonically per MCU starting at 1, so passing u32::MAX
        // retires the lot. (Equivalent to a real `kalico_credit_freed`
        // arriving with `retired_through_segment_id` advanced past every
        // in-flight segment.)
        h.simulate_credit_freed(OCTOPUS_ID, u32::MAX);
    }

    // After full retirement the pool must have zero slots in flight.
    assert_eq!(
        h.slot_pool_in_flight(OCTOPUS_ID),
        0,
        "all slots should have been released after retire-all event"
    );
    // And we should have dispatched at least n segments.
    assert!(
        h.dispatched.load(Ordering::Relaxed) >= n as u64,
        "expected ≥{n} dispatched, saw {}",
        h.dispatched.load(Ordering::Relaxed)
    );

    // The CURVE_POOL_N constant must be the same one the bridge uses.
    assert_eq!(CURVE_POOL_N, 64);
}

/// Regression: without retirement events, the slot pool exhausts after
/// `CURVE_POOL_N` allocations and the dispatch closure surfaces a clean
/// `PlannerError::Dispatch`. (Pre-fix this manifested as a firmware-side
/// `kalico_load_curve_response { result != 0 }` once slot >= 64.)
#[test]
fn slot_pool_exhaustion_surfaces_as_dispatch_error() {
    let h = Harness::corexy_only();

    // Each X move yields ≤ 2 curve allocs on the Octopus (X + Y, with Y
    // possibly elided as trivially-constant). 80 moves is comfortably
    // past the 64-slot capacity even in the best-case 1-slot-per-move
    // scenario. Submit and flush WITHOUT retirement events.
    let mut x = 0.0_f64;
    let mut first_err: Option<String> = None;
    for i in 0..80 {
        let dx = if i % 2 == 0 { 5.0 } else { -5.0 };
        let r = h.submit_move([x, 0.0, 0.0], dx, 0.0, 0.0, 0.0, 1000.0);
        if let Err(e) = r {
            first_err = Some(e.to_string());
            break;
        }
        x += dx;
        if let Err(e) = h.planner.as_ref().unwrap().flush() {
            first_err = Some(e.to_string());
            break;
        }
    }

    let msg = first_err.expect(
        "expected slot-pool exhaustion within 80 moves without retirement events",
    );
    assert!(
        msg.contains("slot pool exhausted") || msg.contains("Dispatch"),
        "expected slot-pool-exhaustion error, got: {msg}"
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
