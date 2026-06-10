use std::env;
use std::fs;
use std::process;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use motion_bridge_native::classify::{ClassifyError, classify_and_build};
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::planner::{DispatchError, PlannerHandle};
use nurbs::bezier::extract_bezier_pieces;
use trajectory::{AxisShaper, RequiredShaper, ShapedSegment, ShaperConfig};

fn trident_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.shaper = ShaperConfig {
        x: RequiredShaper::SmoothZv { frequency_hz: 55.4 },
        y: RequiredShaper::SmoothZv { frequency_hz: 39.2 },
        z: AxisShaper::Passthrough,
    };
    c
}

fn trident_limits() -> PlannerLimits {
    PlannerLimits {
        max_velocity: 300.0,
        max_accel: 5000.0,
        max_z_velocity: 15.0,
        max_z_accel: 350.0,
        square_corner_velocity: 5.0,
    }
}

// Three-element array: segment count, total pieces summed per axis [x, y, z], max pieces per
// segment per axis — all accessed by index so a single atomic array suffices.
struct DispatchStats {
    segments: AtomicU64,
    // pieces_total[ax] and pieces_max[ax] each hold per-axis values.
    pieces_total: [AtomicU64; 3],
    pieces_max: [AtomicU64; 3],
}

impl DispatchStats {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            segments: AtomicU64::new(0),
            pieces_total: [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)],
            pieces_max: [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)],
        })
    }

    fn record(&self, seg: &ShapedSegment) {
        self.segments.fetch_add(1, Ordering::Relaxed);
        for ax in 0..3 {
            let count = extract_bezier_pieces(&seg.axes[ax]).len() as u64;
            self.pieces_total[ax].fetch_add(count, Ordering::Relaxed);
            let mut cur = self.pieces_max[ax].load(Ordering::Relaxed);
            while count > cur {
                match self.pieces_max[ax].compare_exchange_weak(
                    cur,
                    count,
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => break,
                    Err(updated) => cur = updated,
                }
            }
        }
    }
}

struct PositionState {
    pos: [f64; 3],
    feedrate_mm_s: f64,
    absolute: bool,
    established: bool,
}

impl PositionState {
    fn new() -> Self {
        Self {
            pos: [0.0; 3],
            feedrate_mm_s: 0.0,
            absolute: true,
            established: false,
        }
    }

    fn apply_move(
        &mut self,
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        f: Option<f64>,
    ) -> Option<([f64; 3], f64, f64, f64)> {
        if let Some(f_val) = f {
            self.feedrate_mm_s = f_val / 60.0;
        }

        let target = if self.absolute {
            [
                x.unwrap_or(self.pos[0]),
                y.unwrap_or(self.pos[1]),
                z.unwrap_or(self.pos[2]),
            ]
        } else {
            [
                self.pos[0] + x.unwrap_or(0.0),
                self.pos[1] + y.unwrap_or(0.0),
                self.pos[2] + z.unwrap_or(0.0),
            ]
        };

        let dx = target[0] - self.pos[0];
        let dy = target[1] - self.pos[1];
        let dz = target[2] - self.pos[2];
        let start = self.pos;
        self.pos = target;

        if !self.established {
            self.established = true;
            return None;
        }

        Some((start, dx, dy, dz))
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    let (path, max_moves) = parse_args(&args);

    let source = fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("error: cannot read {path}: {e}");
        process::exit(1);
    });

    let stats = DispatchStats::new();
    let stats_cb = Arc::clone(&stats);
    let dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            stats_cb.record(seg);
            Ok(())
        });

    let mut h = PlannerHandle::spawn(trident_config(), dispatch);
    h.update_limits(trident_limits())
        .unwrap_or_else(|e| fatal_planner(e, 0, 0));

    let mut pos = PositionState::new();
    let mut submitted: u64 = 0;
    let mut skipped_zero: u64 = 0;
    let mut skipped_other: u64 = 0;
    let mut unknown_g_counts: std::collections::BTreeMap<u32, u64> = std::collections::BTreeMap::new();

    let wall_start = Instant::now();

    for token_result in gcode::lex(&source) {
        let token = match token_result {
            Ok(t) => t,
            Err(e) => {
                eprintln!("warning: lex error, skipping line: {e}");
                skipped_other += 1;
                continue;
            }
        };

        let (letter, major, params, line_no) = match token {
            gcode::Token::Command {
                letter,
                major,
                params,
                line_no,
                minor: _,
            } => (letter, major, params, line_no),
            gcode::Token::Comment { .. } | gcode::Token::Marker { .. } => continue,
            _ => continue,
        };

        match letter {
            b'G' => match major {
                0 | 1 => {
                    let x = params.x();
                    let y = params.y();
                    let z = params.z();
                    let f = params.f();

                    if x.is_none() && y.is_none() && z.is_none() && f.is_none() {
                        skipped_other += 1;
                        continue;
                    }

                    let Some((start, dx, dy, dz)) = pos.apply_move(x, y, z, f) else {
                        continue;
                    };

                    if dx.abs() < 1e-9 && dy.abs() < 1e-9 && dz.abs() < 1e-9 {
                        skipped_zero += 1;
                        continue;
                    }

                    let classified = match classify_and_build(start, dx, dy, dz, 0.0, pos.feedrate_mm_s) {
                        Ok(m) => m,
                        Err(ClassifyError::ZeroDisplacement) => {
                            skipped_zero += 1;
                            continue;
                        }
                        Err(e) => {
                            eprintln!(
                                "error: move {submitted} (line {line_no}): classify failed: {e}"
                            );
                            process::exit(1);
                        }
                    };

                    if let Err(e) = h.submit_move(classified) {
                        fatal_planner(e, submitted, line_no);
                    }
                    submitted += 1;

                    if submitted % 10_000 == 0 {
                        println!(
                            "progress: {submitted} moves submitted, {:.1}s elapsed",
                            wall_start.elapsed().as_secs_f64()
                        );
                    }

                    if let Some(limit) = max_moves {
                        if submitted >= limit {
                            println!("--max-moves {limit} reached, stopping early");
                            break;
                        }
                    }
                }

                90 => pos.absolute = true,
                91 => pos.absolute = false,

                28 | 92 => {
                    skipped_other += 1;
                }

                other => {
                    *unknown_g_counts.entry(other).or_insert(0) += 1;
                    skipped_other += 1;
                }
            },
            b'M' => {
                skipped_other += 1;
            }
            _ => {
                skipped_other += 1;
            }
        }
    }

    h.flush().unwrap_or_else(|e| fatal_planner(e, submitted, 0));

    let wall_s = wall_start.elapsed().as_secs_f64();
    let planner_time_s = h.last_move_time();

    h.shutdown();

    for (g, count) in &unknown_g_counts {
        println!("warning: G{g} encountered {count} times (skipped)");
    }

    let seg_count = stats.segments.load(Ordering::Relaxed);
    let moves_per_s = if wall_s > 0.0 {
        submitted as f64 / wall_s
    } else {
        f64::INFINITY
    };

    println!();
    println!("=== plan_gcode report ===");
    println!("moves submitted:          {submitted}");
    println!("zero-displacement skipped:{skipped_zero}");
    println!("other commands skipped:   {skipped_other}");
    println!("planner print time:       {planner_time_s:.3} s");
    println!("wall time:                {wall_s:.3} s");
    println!("throughput:               {moves_per_s:.0} moves/s");
    println!("dispatched segments:      {seg_count}");

    if seg_count > 0 {
        for (ax, label) in [(0, "X"), (1, "Y"), (2, "Z")] {
            let total = stats.pieces_total[ax].load(Ordering::Relaxed);
            let max = stats.pieces_max[ax].load(Ordering::Relaxed);
            let mean = total as f64 / seg_count as f64;
            println!("  axis {label}: mean {mean:.1} pieces/seg, max {max} pieces/seg, total {total}");
        }
    }
}

fn parse_args(args: &[String]) -> (String, Option<u64>) {
    if args.len() < 2 {
        eprintln!("usage: plan_gcode <file.gcode> [--max-moves N]");
        process::exit(1);
    }

    let path = args[1].clone();
    let mut max_moves: Option<u64> = None;

    let mut i = 2;
    while i < args.len() {
        if args[i] == "--max-moves" {
            i += 1;
            if i >= args.len() {
                eprintln!("error: --max-moves requires a value");
                process::exit(1);
            }
            max_moves = Some(args[i].parse::<u64>().unwrap_or_else(|_| {
                eprintln!("error: --max-moves value must be a positive integer");
                process::exit(1);
            }));
        }
        i += 1;
    }

    (path, max_moves)
}

fn fatal_planner(e: motion_bridge_native::planner::PlannerError, move_idx: u64, line_no: u32) -> ! {
    eprintln!(
        "error: planner error after move {move_idx} (line {line_no}): {e}"
    );
    process::exit(1);
}
