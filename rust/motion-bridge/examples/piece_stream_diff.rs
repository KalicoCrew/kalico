/// Piece-stream comparison harness for diagnosing the "grindy motion" regression
/// introduced by commit 6ca2d685a ("fix(trajectory): emit constant axes as cubic").
///
/// Run this on the current branch (6ca2d685a) and on the parent commit (387534eec)
/// worktree to compare the dispatched PieceEntry streams.
///
/// Usage (from rust/):
///   cargo run --example piece_stream_diff 2>/dev/null
///   # in /tmp/kalico-387534eec/rust/:
///   cargo run --example piece_stream_diff 2>/dev/null
use std::sync::{Arc, Mutex};

use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::dispatch::{KINEMATICS_COREXY, McuAxisConfig, McuCaps};
use motion_bridge_native::enqueue::enqueue_segment;
use motion_bridge_native::planner::{DispatchError, PlannerHandle};
use motion_bridge_native::pump::MAX_LEAD_SECS;
use trajectory::{AxisShaper, ShapedSegment, ShaperConfig};

type Recorded = Arc<Mutex<Vec<ShapedSegment>>>;

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Recorded,
) {
    let recorded: Recorded = Arc::new(Mutex::new(Vec::new()));
    let rec_for_closure = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec_for_closure.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

fn smooth_zv_186hz_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.shaper = ShaperConfig {
        x: AxisShaper::SmoothZv {
            frequency_hz: 186.0,
        },
        y: AxisShaper::SmoothZv {
            frequency_hz: 186.0,
        },
        z: AxisShaper::Passthrough,
    };
    c.fit_tolerance_mm = 0.05;
    c
}

fn bench_limits() -> PlannerLimits {
    PlannerLimits {
        max_velocity: 300.0,
        max_accel: 5000.0,
        max_z_velocity: 10.0,
        max_z_accel: 80.0,
        square_corner_velocity: 5.0,
    }
}

fn corexy_cfg() -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: 0,
        axes: vec![0, 1],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps {
            total_piece_memory: 1 << 20,
        },
    }
}

fn wait_for_commits(h: &PlannerHandle, target: u32) {
    let start = std::time::Instant::now();
    while h.commit_fire_count() < target {
        assert!(
            start.elapsed() < std::time::Duration::from_secs(10),
            "timeout waiting for {} commits (got {})",
            target,
            h.commit_fire_count()
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

struct PieceStreamEntry {
    scenario: &'static str,
    seg_idx: usize,
    axis_name: &'static str,
    piece_idx: usize,
    host_secs: f64,
    duration_secs: f64,
    coeffs: [f32; 4],
}

fn collect_pieces(
    segs: &[ShapedSegment],
    scenario: &'static str,
    motor_a_label: &'static str,
    motor_b_label: &'static str,
) -> Vec<PieceStreamEntry> {
    let cfg = corexy_cfg();
    let mut out = Vec::new();

    for (seg_idx, seg) in segs.iter().enumerate() {
        let msgs = enqueue_segment(
            seg,
            std::slice::from_ref(&cfg),
            0.0,
            seg_idx == 0,
            0.0,
            MAX_LEAD_SECS,
            |_mcu, host_secs| (host_secs.max(0.0) * 1e6) as u64,
            None,
        );

        for msg in &msgs {
            let axis_name = if msg.key.axis == 0 {
                motor_a_label
            } else {
                motor_b_label
            };
            for (piece_idx, (entry, host_secs)) in msg.pieces.iter().enumerate() {
                out.push(PieceStreamEntry {
                    scenario,
                    seg_idx,
                    axis_name,
                    piece_idx,
                    host_secs: *host_secs,
                    duration_secs: entry.duration as f64,
                    coeffs: entry.coeffs,
                });
            }
        }
    }
    out
}

fn bernstein_eval(coeffs: &[f32; 4], t_norm: f64) -> f64 {
    let u = t_norm;
    let v = 1.0 - u;
    let c0 = coeffs[0] as f64;
    let c1 = coeffs[1] as f64;
    let c2 = coeffs[2] as f64;
    let c3 = coeffs[3] as f64;
    v * v * v * c0 + 3.0 * v * v * u * c1 + 3.0 * v * u * u * c2 + u * u * u * c3
}

fn analyze_pieces(label: &str, pieces: &[PieceStreamEntry]) {
    println!("\n=== {} ===", label);

    if pieces.is_empty() {
        println!("  (no pieces)");
        return;
    }

    let axes: Vec<&'static str> = {
        let mut v: Vec<&'static str> = pieces.iter().map(|p| p.axis_name).collect();
        v.dedup();
        v.sort_unstable();
        v.dedup();
        v
    };

    for &axis in &axes {
        let axis_pieces: Vec<&PieceStreamEntry> =
            pieces.iter().filter(|p| p.axis_name == axis).collect();

        println!("\n  -- axis: {} --", axis);
        println!("  piece_count: {}", axis_pieces.len());

        let total_dur: f64 = axis_pieces.iter().map(|p| p.duration_secs).sum();
        println!("  total_duration_s: {:.6}", total_dur);

        let min_dur = axis_pieces
            .iter()
            .map(|p| p.duration_secs)
            .fold(f64::INFINITY, f64::min);
        let max_dur = axis_pieces
            .iter()
            .map(|p| p.duration_secs)
            .fold(0.0_f64, f64::max);
        println!("  piece_dur_min_s: {:.6}  max_s: {:.6}", min_dur, max_dur);

        let start_pos = bernstein_eval(&axis_pieces[0].coeffs, 0.0);
        let end_pos = bernstein_eval(&axis_pieces.last().unwrap().coeffs, 1.0);
        println!("  position_start: {:.6}  end: {:.6}", start_pos, end_pos);

        let mut v_jumps: Vec<f64> = Vec::new();
        for i in 0..axis_pieces.len().saturating_sub(1) {
            let a = axis_pieces[i];
            let b = axis_pieces[i + 1];
            let d_a = a.duration_secs;
            let d_b = b.duration_secs;
            if d_a < 1e-12 || d_b < 1e-12 {
                continue;
            }
            let v_end = (bernstein_eval(&a.coeffs, 1.0) - bernstein_eval(&a.coeffs, 0.999_999))
                / (d_a * 0.000_001);
            let v_start = (bernstein_eval(&b.coeffs, 0.000_001) - bernstein_eval(&b.coeffs, 0.0))
                / (d_b * 0.000_001);
            v_jumps.push((v_end - v_start).abs());
        }
        if !v_jumps.is_empty() {
            let max_vjump = v_jumps.iter().copied().fold(0.0_f64, f64::max);
            println!(
                "  max_velocity_jump_at_piece_boundary_mm_s: {:.4}",
                max_vjump
            );
        }

        compute_step_stats(axis, &axis_pieces);
    }
}

fn compute_step_stats(axis_name: &str, pieces: &[&PieceStreamEntry]) {
    const STEPS_PER_MM: f64 = 160.0;
    const DT: f64 = 25.0e-6;

    let mut positions: Vec<f64> = Vec::new();

    for piece in pieces.iter() {
        let dur = piece.duration_secs;
        if dur < 1e-12 {
            continue;
        }
        let n = ((dur / DT).ceil() as usize).max(2);
        for k in 0..=n {
            let t_norm = k as f64 / n as f64;
            let pos = bernstein_eval(&piece.coeffs, t_norm);
            positions.push(pos);
        }
    }

    if positions.len() < 2 {
        return;
    }

    let mut step_times: Vec<f64> = Vec::new();
    let mut cur_pos = positions[0];
    let mut cur_step = (cur_pos * STEPS_PER_MM).round();
    let mut t_cursor = 0.0_f64;

    for (i, &next_pos) in positions[1..].iter().enumerate() {
        let next_step = (next_pos * STEPS_PER_MM).round();
        let dt_sample = DT;
        let n_steps = (next_step - cur_step).abs() as usize;
        if n_steps > 0 {
            let dt_per_step = dt_sample / n_steps as f64;
            for j in 0..n_steps {
                step_times.push(t_cursor + (j as f64 + 0.5) * dt_per_step);
            }
        }
        cur_step = next_step;
        cur_pos = next_pos;
        t_cursor += dt_sample;
        let _ = (i, cur_pos);
    }

    if step_times.len() < 2 {
        println!(
            "  step_analysis_{}: too few steps ({})",
            axis_name,
            step_times.len()
        );
        return;
    }

    let intervals: Vec<f64> = step_times.windows(2).map(|w| w[1] - w[0]).collect();
    let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
    let max_interval = intervals.iter().copied().fold(0.0_f64, f64::max);
    let min_interval = intervals.iter().copied().fold(f64::INFINITY, f64::min);
    let variance =
        intervals.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / intervals.len() as f64;
    let stddev = variance.sqrt();

    let max_ratio = if min_interval > 1e-12 {
        max_interval / min_interval
    } else {
        f64::INFINITY
    };

    let jerk_in_intervals: Vec<f64> = intervals.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    let max_jerk = jerk_in_intervals.iter().copied().fold(0.0_f64, f64::max);

    println!("  step_count_{}: {}", axis_name, step_times.len());
    println!(
        "  interval_mean_{}: {:.3} us  stddev: {:.3} us",
        axis_name,
        mean * 1e6,
        stddev * 1e6
    );
    println!(
        "  interval_max_ratio_{}:  {:.4}  (max {:.3} us / min {:.3} us)",
        axis_name,
        max_ratio,
        max_interval * 1e6,
        min_interval * 1e6
    );
    println!(
        "  max_adjacent_interval_jerk_{}: {:.3} us",
        axis_name,
        max_jerk * 1e6
    );
}

fn scenario_single_x_jog(label: &'static str) -> Vec<PieceStreamEntry> {
    println!(
        "\n--- scenario: single pure-X jog 25mm at 100mm/s ({})",
        label
    );

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(bench_limits()).expect("update_limits");

    h.submit_move(classify_and_build([0.0; 3], 25.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit jog");
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    println!("  dispatched_segments: {}", segs.len());
    for (i, seg) in segs.iter().enumerate() {
        println!(
            "  seg[{}]: t=[{:.6}, {:.6}]  X-deg={}  Y-deg={}",
            i,
            seg.t_start,
            seg.t_end,
            seg.axes[0].degree(),
            seg.axes[1].degree(),
        );
    }

    h.shutdown();
    collect_pieces(&segs, label, "motor_A", "motor_B")
}

fn scenario_three_x_jogs_in_flight(label: &'static str) -> Vec<PieceStreamEntry> {
    println!(
        "\n--- scenario: three pure-X jogs ~50ms apart, in-flight ({})",
        label
    );
    use std::time::Duration;

    let (dispatch, recorded) = recording_dispatch();
    let mut h = PlannerHandle::spawn(smooth_zv_186hz_config(), dispatch);
    h.update_limits(bench_limits()).expect("update_limits");
    h.kalico_stream_open([295.0, 0.0, 0.0, 0.0])
        .expect("kalico_stream_open");

    h.submit_move(classify_and_build([295.0, 0.0, 0.0], -20.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit jog 1");

    std::thread::sleep(Duration::from_millis(50));

    h.submit_move(classify_and_build([275.0, 0.0, 0.0], -25.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit jog 2");

    std::thread::sleep(Duration::from_millis(50));

    h.submit_move(classify_and_build([250.0, 0.0, 0.0], -25.0, 0.0, 0.0, 0.0, 100.0).unwrap())
        .expect("submit jog 3");

    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    println!("  dispatched_segments: {}", segs.len());
    for (i, seg) in segs.iter().enumerate() {
        println!(
            "  seg[{}]: t=[{:.6}, {:.6}]  X-deg={}  Y-deg={}",
            i,
            seg.t_start,
            seg.t_end,
            seg.axes[0].degree(),
            seg.axes[1].degree(),
        );
    }

    h.shutdown();

    let cfg = corexy_cfg();
    let mut out = Vec::new();
    for (seg_idx, seg) in segs.iter().enumerate() {
        let result = std::panic::catch_unwind(|| {
            enqueue_segment(
                seg,
                std::slice::from_ref(&cfg),
                0.0,
                seg_idx == 0,
                0.0,
                MAX_LEAD_SECS,
                |_mcu, host_secs| (host_secs.max(0.0) * 1e6) as u64,
                None,
            )
        });

        match result {
            Ok(msgs) => {
                for msg in &msgs {
                    let axis_name = if msg.key.axis == 0 {
                        "motor_A"
                    } else {
                        "motor_B"
                    };
                    for (piece_idx, (entry, host_secs)) in msg.pieces.iter().enumerate() {
                        out.push(PieceStreamEntry {
                            scenario: label,
                            seg_idx,
                            axis_name,
                            piece_idx,
                            host_secs: *host_secs,
                            duration_secs: entry.duration as f64,
                            coeffs: entry.coeffs,
                        });
                    }
                }
            }
            Err(_) => {
                println!(
                    "  seg[{}] enqueue panicked (expected on parent for degree-mismatch segments)",
                    seg_idx
                );
            }
        }
    }

    out
}

fn print_piece_table(label: &str, pieces: &[PieceStreamEntry]) {
    println!("\n=== piece table: {} ===", label);
    println!(
        "{:>5} {:>6} {:>8} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "seg", "pidx", "axis", "host_s", "dur_ms", "c0", "c1", "c2", "c3"
    );
    for p in pieces {
        println!(
            "{:>5} {:>6} {:>8} {:>10.6} {:>10.4} {:>10.5} {:>10.5} {:>10.5} {:>10.5}",
            p.seg_idx,
            p.piece_idx,
            p.axis_name,
            p.host_secs,
            p.duration_secs * 1e3,
            p.coeffs[0],
            p.coeffs[1],
            p.coeffs[2],
            p.coeffs[3],
        );
    }
}

fn compare_piece_streams(label: &str, new_pieces: &[PieceStreamEntry], old_label: &str) {
    let motor_a_new: Vec<&PieceStreamEntry> = new_pieces
        .iter()
        .filter(|p| p.axis_name == "motor_A")
        .collect();
    let motor_b_new: Vec<&PieceStreamEntry> = new_pieces
        .iter()
        .filter(|p| p.axis_name == "motor_B")
        .collect();

    println!("\n=== {} ===", label);
    println!(
        "  Note: run identical harness on {} for comparison",
        old_label
    );
    println!(
        "  motor_A piece count: {}  motor_B piece count: {}",
        motor_a_new.len(),
        motor_b_new.len()
    );

    println!("\n  --- motor_A analysis ---");
    compute_step_stats("motor_A", &motor_a_new);

    println!("\n  --- motor_B analysis ---");
    compute_step_stats("motor_B", &motor_b_new);

    println!("\n  --- velocity continuity at piece boundaries (motor_A) ---");
    for i in 0..motor_a_new.len().saturating_sub(1) {
        let a = motor_a_new[i];
        let b = motor_a_new[i + 1];
        let d_a = a.duration_secs;
        let d_b = b.duration_secs;
        if d_a < 1e-12 || d_b < 1e-12 {
            continue;
        }
        let v_end =
            (bernstein_eval(&a.coeffs, 1.0) - bernstein_eval(&a.coeffs, 0.99999)) / (d_a * 0.00001);
        let v_start =
            (bernstein_eval(&b.coeffs, 0.00001) - bernstein_eval(&b.coeffs, 0.0)) / (d_b * 0.00001);
        let jump = (v_end - v_start).abs();
        if jump > 0.1 {
            println!(
                "  piece {}: v_end={:.3} v_start={:.3} jump={:.4} mm/s",
                i, v_end, v_start, jump
            );
        }
    }

    for (name, stream) in [("motor_A", &motor_a_new), ("motor_B", &motor_b_new)] {
        println!(
            "\n  --- position continuity at piece boundaries ({}) ---",
            name
        );
        let mut t_cursor = 0.0_f64;
        let mut worst = (0.0_f64, 0.0_f64, 0usize);
        for i in 0..stream.len().saturating_sub(1) {
            let a = stream[i];
            let b = stream[i + 1];
            t_cursor += a.duration_secs;
            let p_end = bernstein_eval(&a.coeffs, 1.0);
            let p_start = bernstein_eval(&b.coeffs, 0.0);
            let jump = (p_end - p_start).abs();
            if jump > worst.0 {
                worst = (jump, t_cursor, i);
            }
            if jump > 0.001 {
                println!(
                    "  piece {} -> {} at t={:.6}s: p_end={:.6} p_start={:.6} JUMP={:.6} mm ({:.2} steps)",
                    i,
                    i + 1,
                    t_cursor,
                    p_end,
                    p_start,
                    jump,
                    jump * 160.0
                );
            }
        }
        println!(
            "  worst position jump: {:.9} mm at t={:.6}s (piece {})",
            worst.0, worst.1, worst.2
        );

        println!("  --- max intra-piece speed scan ({}) ---", name);
        let mut worst_v = (0.0_f64, 0usize);
        for (i, p) in stream.iter().enumerate() {
            if p.duration_secs < 1e-12 {
                println!("  piece {i}: ZERO-DURATION piece, coeffs={:?}", p.coeffs);
                continue;
            }
            for k in 0..=100 {
                let u = k as f64 / 100.0;
                let c = &p.coeffs;
                let dv = 3.0
                    * ((c[1] - c[0]) as f64 * (1.0 - u) * (1.0 - u)
                        + 2.0 * (c[2] - c[1]) as f64 * (1.0 - u) * u
                        + (c[3] - c[2]) as f64 * u * u)
                    / p.duration_secs;
                if dv.abs() > worst_v.0 {
                    worst_v = (dv.abs(), i);
                }
            }
        }
        println!(
            "  max |velocity| anywhere: {:.3} mm/s (piece {}, duration {:.6}s)",
            worst_v.0,
            worst_v.1,
            stream.get(worst_v.1).map_or(0.0, |p| p.duration_secs)
        );
    }

    println!("\n  --- constant-value piece detection (motor_B, Y=0 for pure-X jog) ---");
    let mut const_count = 0;
    let mut nonconst_count = 0;
    for piece in &motor_b_new {
        let c0 = piece.coeffs[0] as f64;
        let c1 = piece.coeffs[1] as f64;
        let c2 = piece.coeffs[2] as f64;
        let c3 = piece.coeffs[3] as f64;
        let span = (c0 - c1).abs() + (c1 - c2).abs() + (c2 - c3).abs();
        if span < 1e-6 {
            const_count += 1;
        } else {
            nonconst_count += 1;
        }
    }
    println!(
        "  motor_B: {} constant pieces, {} non-constant pieces",
        const_count, nonconst_count
    );

    println!("\n  --- add_with_knot_union knot-grid inspection (first 5 segs) ---");
}

fn main() {
    let jog_pieces = scenario_single_x_jog("current");
    analyze_pieces("single_jog/current", &jog_pieces);
    print_piece_table(
        "single_jog_motor_A",
        &jog_pieces
            .iter()
            .filter(|p| p.axis_name == "motor_A")
            .cloned()
            .collect::<Vec<_>>(),
    );
    print_piece_table(
        "single_jog_motor_B",
        &jog_pieces
            .iter()
            .filter(|p| p.axis_name == "motor_B")
            .cloned()
            .collect::<Vec<_>>(),
    );
    compare_piece_streams("single_jog/current vs parent", &jog_pieces, "387534eec");

    let three_jog_pieces = scenario_three_x_jogs_in_flight("current");
    analyze_pieces("three_jogs_in_flight/current", &three_jog_pieces);
    print_piece_table(
        "three_jogs_motor_A",
        &three_jog_pieces
            .iter()
            .filter(|p| p.axis_name == "motor_A")
            .cloned()
            .collect::<Vec<_>>(),
    );
    compare_piece_streams(
        "three_jogs_in_flight/current vs parent",
        &three_jog_pieces,
        "387534eec",
    );

    println!("\n=== DIAGNOSIS SUMMARY ===");
    println!("On the CURRENT branch (6ca2d685a):");
    println!("  - A constant axis (Y=0 for pure-X jog) is emitted as a single-piece cubic");
    println!("    knot vector [t_start x4, t_end x4], 4 identical control points.");
    println!("  - On the PARENT (387534eec):");
    println!("    a constant axis was cloned directly (native degree of the FittedSegment,");
    println!("    usually also cubic, but with the SAME dense multi-piece knot grid as X).");
    println!();
    println!("  - In enqueue.rs:add_with_knot_union(X, Y):");
    println!("    PARENT: both X and Y had the same dense knot grid → no refinement needed");
    println!("    CURRENT: Y is single-piece [t_start x4, t_end x4], X has many interior knots");
    println!("    → refine_pieces_to_breakpoints splits Y's single piece at every X breakpoint");
    println!("    → result has the same piece count as X, and the sum is numerically correct");
    println!();
    println!("  - The MOVING axis (X) knot grid should be unchanged by this operation.");
    println!("    The motor A/B = X±Y sums should have the same values as before.");
    println!("  - Checking step stats above to verify whether any irregularity is introduced...");
}

impl Clone for PieceStreamEntry {
    fn clone(&self) -> Self {
        PieceStreamEntry {
            scenario: self.scenario,
            seg_idx: self.seg_idx,
            axis_name: self.axis_name,
            piece_idx: self.piece_idx,
            host_secs: self.host_secs,
            duration_secs: self.duration_secs,
            coeffs: self.coeffs,
        }
    }
}
