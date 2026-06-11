use geometry::segment::{CubicSegment, EMode, SourceRange};
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::VectorNurbs;
use trajectory::plan_velocity::{PlanShaper, SafetyMode};
use trajectory::streaming::{EmitContext, ReplanContext, ShaperState};
use trajectory::{AxisShaper, ELimits, ShapedSegment};

fn live_shapers() -> [Option<AxisShaper>; 4] {
    [
        Some(AxisShaper::SmoothMzv {
            frequency_hz: 186.0,
        }),
        Some(AxisShaper::SmoothMzv {
            frequency_hz: 122.0,
        }),
        Some(AxisShaper::Passthrough),
        None,
    ]
}

fn live_ctx() -> ReplanContext {
    ReplanContext {
        limits: temporal::Limits::new(
            [1000.0, 1000.0, 5.0],
            [70000.0, 70000.0, 100.0],
            [140000.0, 140000.0, 200.0],
            70000.0,
        ),
        kernels: [
            Some(PlanShaper::SmoothMzv {
                frequency_hz: 186.0,
            }),
            Some(PlanShaper::SmoothMzv {
                frequency_hz: 122.0,
            }),
            Some(PlanShaper::Passthrough),
            None,
        ],
        fit_tolerance_mm: 0.005,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        e_limits: ELimits {
            v_max: 100.0,
            a_max: 5000.0,
        },
        junction_chord_tolerance_mm: 0.05,
        worker_threads: 1,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        fallback_initial_v: 0.0,
        safety_mode: SafetyMode::WorstCaseFuture,
    }
}

fn emit_kernels() -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [
        AxisShaper::SmoothMzv {
            frequency_hz: 186.0,
        }
        .to_kernel(),
        AxisShaper::SmoothMzv {
            frequency_hz: 122.0,
        }
        .to_kernel(),
        None,
        None,
    ]
}

fn passthrough_shapers() -> [Option<AxisShaper>; 4] {
    [
        Some(AxisShaper::Passthrough),
        Some(AxisShaper::Passthrough),
        Some(AxisShaper::Passthrough),
        None,
    ]
}

fn passthrough_ctx() -> ReplanContext {
    let mut ctx = live_ctx();
    ctx.kernels = [
        Some(PlanShaper::Passthrough),
        Some(PlanShaper::Passthrough),
        Some(PlanShaper::Passthrough),
        None,
    ];
    ctx
}

fn passthrough_emit_kernels() -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [None, None, None, None]
}

fn run_jog_experiment(
    label: &str,
    shapers: &[Option<AxisShaper>; 4],
    ctx: &ReplanContext,
    emit: &[Option<PiecewisePolynomialKernel<f64>>; 4],
    strokes: &[(f64, f64)],
    feedrate: f64,
) -> Vec<bool> {
    let mut state = ShaperState::new([0.0; 4], shapers);
    let halos = Vec::new();
    let ctx_emit = EmitContext {
        kernels: emit,
        e_halos: &halos,
    };
    let mut converged = Vec::with_capacity(strokes.len());
    eprintln!("--- {label} ---");
    for (i, (from, to)) in strokes.iter().enumerate() {
        let report = state
            .append_and_replan(linear_x_segment(*from, *to, feedrate), ctx)
            .unwrap_or_else(|e| panic!("{label} stroke {i} ({from}->{to}) failed: {e:?}"));
        eprintln!(
            "[{label}] stroke {i} {from:>4}->{to:<4} window_segs={} beta_iters={} converged={} solve_us={}",
            report.window_segments,
            report.plan.beta_iterations,
            report.plan.beta_converged,
            report.solve_us,
        );
        converged.push(report.plan.beta_converged);
        state
            .emit_committed(&ctx_emit)
            .unwrap_or_else(|e| panic!("{label} stroke {i} emit failed: {e:?}"));
    }
    converged
}

/// Mid-flight replan of back-to-back fast moves used to drive the jerk-relaxation
/// derate loop to its iteration cap without converging: the feasibility emit padded
/// the first window segment's left edge against empty shaper history, fabricating a
/// velocity step at the dispatch seam whose convolution looked like a ~2x acceleration
/// spike. Derating planning accel could not move a boundary artifact, so the loop spun
/// to the cap (~294 ms on the Pi) and starved playback. With the left pad extrapolated
/// at the entry velocity instead, the phantom is gone and every regime converges in one
/// iteration — regardless of direction reversal, segment length, or feedrate-to-vmax ratio.
#[test]
fn mid_flight_fast_jog_replan_converges_across_regimes() {
    let feedrate = 1000.0;
    let reversal = [(0.0, 30.0), (30.0, 0.0), (0.0, 30.0), (30.0, 0.0)];
    let continuation = [(0.0, 30.0), (30.0, 60.0), (60.0, 90.0), (90.0, 120.0)];
    let long_continuation = [(0.0, 200.0), (200.0, 400.0), (400.0, 600.0), (600.0, 800.0)];

    let mut ctx_vmax2000 = live_ctx();
    ctx_vmax2000.limits = temporal::Limits::new(
        [2000.0, 2000.0, 5.0],
        [70000.0, 70000.0, 100.0],
        [140000.0, 140000.0, 200.0],
        70000.0,
    );

    let scenarios: [(
        &str,
        &[Option<AxisShaper>; 4],
        &ReplanContext,
        &[Option<PiecewisePolynomialKernel<f64>>; 4],
        &[(f64, f64)],
        f64,
    ); 6] = [
        (
            "shaped+reversal",
            &live_shapers(),
            &live_ctx(),
            &emit_kernels(),
            &reversal,
            feedrate,
        ),
        (
            "shaped+continuation",
            &live_shapers(),
            &live_ctx(),
            &emit_kernels(),
            &continuation,
            feedrate,
        ),
        (
            "shaped+continuation_long_200mm",
            &live_shapers(),
            &live_ctx(),
            &emit_kernels(),
            &long_continuation,
            feedrate,
        ),
        (
            "shaped+continuation_500mms",
            &live_shapers(),
            &live_ctx(),
            &emit_kernels(),
            &continuation,
            500.0,
        ),
        (
            "shaped+continuation_1000mms_of_vmax2000",
            &live_shapers(),
            &ctx_vmax2000,
            &emit_kernels(),
            &continuation,
            feedrate,
        ),
        (
            "passthrough+reversal",
            &passthrough_shapers(),
            &passthrough_ctx(),
            &passthrough_emit_kernels(),
            &reversal,
            feedrate,
        ),
    ];

    for (label, shapers, ctx, emit, strokes, fr) in scenarios {
        let converged = run_jog_experiment(label, shapers, ctx, emit, strokes, fr);
        assert!(
            converged.iter().all(|&c| c),
            "{label}: a mid-flight replan failed to converge — the empty-history boundary \
             phantom has regressed (see {label} trace above)",
        );
    }
}

fn linear_x_segment(start_x: f64, end_x: f64, feedrate: f64) -> CubicSegment {
    let p0 = [start_x, 0.0, 0.0];
    let p3 = [end_x, 0.0, 0.0];
    let lerp = |t: f64| -> [f64; 3] {
        [
            p0[0] + (p3[0] - p0[0]) * t,
            p0[1] + (p3[1] - p0[1]) * t,
            p0[2] + (p3[2] - p0[2]) * t,
        ]
    };
    let cps = vec![p0, lerp(1.0 / 3.0), lerp(2.0 / 3.0), p3];
    let xyz = VectorNurbs::<f64, 3>::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps)
        .unwrap();
    CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        feedrate,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .unwrap()
}

fn cross_axis_contamination(label: &str, segments: &[ShapedSegment], moving_axis: usize) {
    let dt = 2e-4;
    for axis in 0..3 {
        if axis == moving_axis {
            continue;
        }
        let mut max_v: f64 = 0.0;
        let mut max_v_t = 0.0;
        let mut max_excursion: f64 = 0.0;
        let mut p_ref: Option<f64> = None;
        for seg in segments {
            let view = seg.axes[axis].as_view();
            let mut t = seg.t_start;
            while t < seg.t_end - 1e-12 {
                let p0 = nurbs::eval::eval(&view, t);
                let p1 = nurbs::eval::eval(&view, (t + dt).min(seg.t_end));
                let v = ((p1 - p0) / ((t + dt).min(seg.t_end) - t)).abs();
                if v > max_v {
                    max_v = v;
                    max_v_t = t;
                }
                let r = *p_ref.get_or_insert(p0);
                max_excursion = max_excursion.max((p0 - r).abs());
                t += dt;
            }
        }
        eprintln!(
            "[probe] {label}: axis{axis} (should be still) max_v={max_v:.6} at t={max_v_t:.5} max_excursion={max_excursion:.6}",
        );
    }
}

fn probe_stream(label: &str, segments: &[ShapedSegment]) {
    let dt = 2e-4;
    let mut prev_v: Option<f64> = None;
    let mut max_v: f64 = 0.0;
    let mut min_cruise_v = f64::MAX;
    let mut max_dv: f64 = 0.0;
    let mut max_dv_t: f64 = 0.0;
    let mut samples: Vec<(f64, f64)> = Vec::new();

    for seg in segments {
        let view = seg.axes[0].as_view();
        let mut t = seg.t_start;
        while t < seg.t_end - 1e-12 {
            let p0 = nurbs::eval::eval(&view, t);
            let p1 = nurbs::eval::eval(&view, (t + dt).min(seg.t_end));
            let v = (p1 - p0) / ((t + dt).min(seg.t_end) - t);
            samples.push((t, v));
            if let Some(pv) = prev_v {
                let dv = (v - pv).abs();
                if dv > max_dv {
                    max_dv = dv;
                    max_dv_t = t;
                }
            }
            prev_v = Some(v);
            max_v = max_v.max(v.abs());
            t += dt;
        }
    }

    let cruise_threshold = 0.5 * max_v;
    let mut in_cruise = false;
    for &(_, v) in &samples {
        if v.abs() >= cruise_threshold {
            in_cruise = true;
        }
        if in_cruise && v.abs() > 1.0 {
            min_cruise_v = min_cruise_v.min(v.abs());
        }
    }

    let t_total =
        segments.last().map_or(0.0, |s| s.t_end) - segments.first().map_or(0.0, |s| s.t_start);
    eprintln!(
        "[probe] {label}: segs={} t_total={t_total:.5} max_v={max_v:.3} \
         min_cruise_v={min_cruise_v:.3} max_step_dv={max_dv:.4} at t={max_dv_t:.5}",
        segments.len(),
    );

    let mut dips: Vec<(f64, f64)> = Vec::new();
    for w in samples.windows(3) {
        let (t1, v1) = w[1];
        if v1.abs() > 1.0
            && v1.abs() < 0.6 * max_v
            && w[0].1.abs() >= v1.abs()
            && w[2].1.abs() >= v1.abs()
        {
            dips.push((t1, v1));
        }
    }
    if !dips.is_empty() {
        let worst = dips
            .iter()
            .min_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap();
        eprintln!(
            "[probe] {label}: {} interior velocity dips below 60% of peak; worst v={:.3} at t={:.5}",
            dips.len(),
            worst.1,
            worst.0,
        );
    }
}

/// Regression for the 2026-06-11 Trident crash: back-to-back +30/-30 mm jogs at the
/// 1000 mm/s axis limit. Each mid-flight replan must converge within the iteration cap;
/// non-convergence is what blew the real-time budget (294 ms solve for 30 ms of motion)
/// and tripped the SegmentLate fatal abort.
#[test]
fn back_to_back_fast_jogs_replan_stays_realtime() {
    let mut state = ShaperState::new([0.0; 4], &live_shapers());
    let ctx = live_ctx();
    let kernels = emit_kernels();
    let halos = Vec::new();
    let ctx_emit = EmitContext {
        kernels: &kernels,
        e_halos: &halos,
    };

    let feedrate = 1000.0;
    let jog_mm = 30.0;
    let strokes = [
        (0.0, jog_mm),
        (jog_mm, 0.0),
        (0.0, jog_mm),
        (jog_mm, 0.0),
        (0.0, jog_mm),
        (jog_mm, 0.0),
    ];

    for (i, (from, to)) in strokes.iter().enumerate() {
        let report = state
            .append_and_replan(linear_x_segment(*from, *to, feedrate), &ctx)
            .unwrap_or_else(|e| panic!("stroke {i} ({from}->{to}) replan failed: {e:?}"));
        let plan = report.plan;
        eprintln!(
            "stroke {i} {from:>4}->{to:<4} window_segs={} beta_iters={} converged={} solve_us={}",
            report.window_segments, plan.beta_iterations, plan.beta_converged, report.solve_us,
        );
        assert!(
            plan.beta_converged,
            "stroke {i} ({from}->{to}): mid-flight replan failed to converge in {} iters — \
             the empty-history boundary phantom that starved playback on Trident has regressed",
            plan.beta_iterations,
        );
        assert!(
            plan.beta_iterations < ctx.beta_max_iters,
            "stroke {i}: converged only by exhausting the iteration cap ({} iters)",
            plan.beta_iterations,
        );
        state
            .emit_committed(&ctx_emit)
            .unwrap_or_else(|e| panic!("stroke {i} emit_committed failed: {e:?}"));
    }
}

#[test]
fn live_jog_sequence_velocity_probe() {
    let mut state = ShaperState::new([0.0; 4], &live_shapers());
    let ctx = live_ctx();
    let kernels = emit_kernels();
    let halos = Vec::new();
    let ctx_emit = EmitContext {
        kernels: &kernels,
        e_halos: &halos,
    };

    let mut all: Vec<ShapedSegment> = Vec::new();

    state
        .append_and_replan(linear_x_segment(0.0, 5.0, 100.0), &ctx)
        .expect("retract 5mm");
    all.extend(state.emit_committed(&ctx_emit).expect("emit 1"));

    state
        .append_and_replan(linear_x_segment(5.0, 100.0, 100.0), &ctx)
        .expect("jog 95mm");
    all.extend(state.emit_committed(&ctx_emit).expect("emit 2"));

    state
        .append_and_replan(linear_x_segment(100.0, 200.0, 100.0), &ctx)
        .expect("jog 100mm spliced");
    all.extend(state.emit_committed(&ctx_emit).expect("emit 3"));

    all.extend(state.commit_decel_to_zero(&ctx_emit).expect("final commit"));

    probe_stream("jog-sequence", &all);
    cross_axis_contamination("jog-sequence", &all, 0);

    let mut state2 = ShaperState::new([0.0; 4], &live_shapers());
    let mut homing: Vec<ShapedSegment> = Vec::new();
    state2
        .append_and_replan(linear_x_segment(0.0, -258.0, 40.0), &ctx)
        .expect("homing drip move");
    homing.extend(state2.emit_committed(&ctx_emit).expect("emit drip"));
    homing.extend(
        state2
            .commit_decel_to_zero(&ctx_emit)
            .expect("drip decel commit"),
    );
    probe_stream("homing-drip", &homing);
    cross_axis_contamination("homing-drip", &homing, 0);
}
