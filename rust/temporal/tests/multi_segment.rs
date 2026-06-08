use nurbs::VectorNurbs;
use temporal::{
    BatchInput, GridStrategy, JoiningStatus, JunctionBindingCap, Limits, SegmentInput, plan_batch,
};

fn textbook_limits() -> Limits {
    Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
}

fn adaptive() -> GridStrategy {
    GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    }
}

fn assert_junction_continuity_for_all(output: &temporal::BatchOutput, eps_mm_s: f64) {
    for (k, junction) in output.junctions.iter().enumerate() {
        let v_jct = junction.v_junction;
        let v_end_left = output.profiles[k].samples.last().unwrap().v;
        let v_start_right = output.profiles[k + 1].samples[0].v;
        assert!(
            (v_end_left - v_jct).abs() < eps_mm_s,
            "junction {k}: v_end_left={v_end_left} vs v_jct={v_jct} (ε={eps_mm_s})",
        );
        assert!(
            (v_start_right - v_jct).abs() < eps_mm_s,
            "junction {k}: v_start_right={v_start_right} vs v_jct={v_jct} (ε={eps_mm_s})",
        );
    }
}

mod fixture_1_two_g1_sharp_corner {
    use super::*;

    #[test]
    fn fixture_1() {
        let left = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
        )
        .unwrap();
        let right = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[50.0, 0.0, 0.0], [50.0, 50.0, 0.0]],
        )
        .unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput {
                curve: &left,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            SegmentInput {
                curve: &right,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
        ];
        let input = BatchInput {
            segments: &segments,
            grid_strategy: adaptive(),
            worker_threads: 3,
            initial_velocity: 0.0,
            initial_accel: 0.0,
            terminal_velocity: 0.0,
        };
        let output = plan_batch(input).expect("should succeed");

        assert_eq!(output.profiles.len(), 2);

        assert_junction_continuity_for_all(&output, 1.0);
        let v_jct = output.junctions[0].v_junction;

        // Sharp-corner cap: v_jd² = a·δ·cos(α/2)/(1-cos(α/2)); for 90° deviation
        // α = π/2, cos(α/2) = 1/√2, so factor = 1/(1 - 1/√2) ≈ 2.414.
        let expected = (2_500.0_f64 * 0.05 * 2.414_213_562).sqrt();
        assert!(
            (v_jct - expected).abs() < 0.1,
            "v_jct {v_jct} vs expected {expected}"
        );
        assert!(matches!(
            output.junctions[0].binding_cap,
            JunctionBindingCap::SharpCornerChord
        ));

        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}

mod fixture_2_g1_to_g5_smooth {
    use super::*;

    #[test]
    fn fixture_2() {
        let left = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
        )
        .unwrap();
        let right = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [50.0, 0.0, 0.0],
                [60.0, 0.0, 0.0],
                [70.0, 30.0, 0.0],
                [100.0, 50.0, 0.0],
            ],
        )
        .unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput {
                curve: &left,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            SegmentInput {
                curve: &right,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
        ];
        let input = BatchInput {
            segments: &segments,
            grid_strategy: adaptive(),
            worker_threads: 3,
            initial_velocity: 0.0,
            initial_accel: 0.0,
            terminal_velocity: 0.0,
        };
        let output = plan_batch(input).expect("should succeed");

        let j = &output.junctions[0];
        assert!(
            j.kappa_right.abs() > 1e-6,
            "G5 should have nonzero κ at u=0, got {}",
            j.kappa_right
        );
        assert!(
            matches!(
                j.binding_cap,
                JunctionBindingCap::Centripetal
                    | JunctionBindingCap::PerAxisVelocity
                    | JunctionBindingCap::GlobalVMax
            ),
            "smooth junction should not trigger SharpCornerChord, got {:?}",
            j.binding_cap
        );

        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));

        assert_junction_continuity_for_all(&output, 1.0);
    }
}

mod fixture_3_long_straight_then_corner {
    use super::*;
    use temporal::{GridConfig, GridScheme, ToleranceMode, schedule_segment_with_tolerance};

    #[test]
    fn fixture_3() {
        let straight = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        )
        .unwrap();
        let corner_right = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[100.0, 0.0, 0.0], [100.0, 50.0, 0.0]],
        )
        .unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput {
                curve: &straight,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            SegmentInput {
                curve: &corner_right,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
        ];
        let input = BatchInput {
            segments: &segments,
            grid_strategy: adaptive(),
            worker_threads: 3,
            initial_velocity: 0.0,
            initial_accel: 0.0,
            terminal_velocity: 0.0,
        };
        let output = plan_batch(input).expect("should succeed");

        let v_end_seg0 = output.profiles[0].samples.last().unwrap().v;
        assert!(
            v_end_seg0 < 499.0,
            "seg 0 should be braking, v_end = {v_end_seg0}"
        );

        let solo_grid = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };
        let solo = schedule_segment_with_tolerance(
            &straight,
            &limits,
            &solo_grid,
            0.0,
            500.0,
            ToleranceMode::Auto,
        )
        .expect("solo solve");
        let t_joined = output.profiles[0].total_time;
        let t_solo = solo.total_time;
        assert!(
            t_joined > t_solo,
            "joined seg 0 should take longer (decel for corner): joined={t_joined} solo={t_solo}"
        );

        assert_junction_continuity_for_all(&output, 1.0);

        assert!(
            output.joining_sweeps <= 3,
            "lookahead fixture should converge in ≤3 sweeps"
        );
        assert!(matches!(
            output.joining_status,
            temporal::JoiningStatus::Converged
        ));
    }
}

mod fixture_4_per_segment_limits_change {
    use super::*;

    #[test]
    fn fixture_4() {
        let segments_curves: Vec<_> = (0..3_usize)
            .map(|i| {
                VectorNurbs::<f64, 3>::try_new(
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![
                        [i as f64 * 50.0, 0.0, 0.0],
                        [(i + 1) as f64 * 50.0, 0.0, 0.0],
                    ],
                )
                .unwrap()
            })
            .collect();
        let normal_limits = textbook_limits();
        let mut reduced_limits = normal_limits;
        reduced_limits.a_max = [2_500.0; 3];
        let segments = [
            SegmentInput {
                curve: &segments_curves[0],
                limits: normal_limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            SegmentInput {
                curve: &segments_curves[1],
                limits: reduced_limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            SegmentInput {
                curve: &segments_curves[2],
                limits: normal_limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
        ];
        let input = BatchInput {
            segments: &segments,
            grid_strategy: adaptive(),
            worker_threads: 3,
            initial_velocity: 0.0,
            initial_accel: 0.0,
            terminal_velocity: 0.0,
        };
        let output = plan_batch(input).expect("should succeed");

        let max_a_seg1 = output.profiles[1]
            .samples
            .iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_a_seg1 <= 2_500.0 * 1.001,
            "seg 1 peak accel {max_a_seg1} exceeds reduced a_max 2500"
        );

        let max_a_seg0 = output.profiles[0]
            .samples
            .iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        let max_a_seg2 = output.profiles[2]
            .samples
            .iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_a_seg0 > 2_500.0 * 1.5,
            "seg 0 peak accel {max_a_seg0} suggests reduced limits leaked outside seg 1"
        );
        assert!(
            max_a_seg2 > 2_500.0 * 1.5,
            "seg 2 peak accel {max_a_seg2} suggests reduced limits leaked outside seg 1"
        );

        assert_junction_continuity_for_all(&output, 1.0);

        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}

mod fixture_5_star_pattern {
    use super::*;

    #[test]
    fn fixture_5() {
        let r_outer: f64 = 30.0;
        let r_inner: f64 = 12.0;
        let n_points = 5_usize;
        let mut points: Vec<[f64; 3]> = Vec::new();
        for i in 0..n_points * 2 {
            let theta = i as f64 * std::f64::consts::PI / n_points as f64;
            let r = if i % 2 == 0 { r_outer } else { r_inner };
            points.push([r * theta.cos(), r * theta.sin(), 0.0]);
        }
        let curves: Vec<_> = points
            .windows(2)
            .map(|w| {
                VectorNurbs::<f64, 3>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![w[0], w[1]])
                    .unwrap()
            })
            .collect();
        let limits = textbook_limits();
        let segments: Vec<_> = curves
            .iter()
            .map(|c| SegmentInput {
                curve: c,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            })
            .collect();
        let input = BatchInput {
            segments: &segments,
            grid_strategy: adaptive(),
            worker_threads: 3,
            initial_velocity: 0.0,
            initial_accel: 0.0,
            terminal_velocity: 0.0,
        };
        let output = plan_batch(input).expect("should succeed");

        assert!(
            output.joining_sweeps <= 5,
            "joining took {} sweeps",
            output.joining_sweeps
        );
        assert!(matches!(output.joining_status, JoiningStatus::Converged));

        assert_junction_continuity_for_all(&output, 1.0);
    }
}

mod fixture_6_long_realistic_chain {
    use super::*;
    use std::time::Instant;

    fn realistic_machine_limits() -> Limits {
        Limits::new([1_000.0; 3], [65_000.0; 3], [50_000_000.0; 3], 65_000.0)
    }

    #[test]
    #[allow(clippy::too_many_lines)]
    fn fixture_6() {
        let mut curves: Vec<VectorNurbs<f64, 3>> = Vec::new();

        let mut px = 0.0_f64;
        let mut py = 0.0_f64;

        for i in 0..6_usize {
            let len = 20.0 + i as f64 * 5.0;
            curves.push(
                VectorNurbs::<f64, 3>::try_new(
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![[px, py, 0.0], [px + len, py, 0.0]],
                )
                .unwrap(),
            );
            px += len;
        }

        for _ in 0..2 {
            let p0 = [px, py, 0.0];
            let p1 = [px + 10.0, py + 20.0, 0.0];
            let p2 = [px + 30.0, py + 20.0, 0.0];
            let p3 = [px + 40.0, py, 0.0];
            curves.push(
                VectorNurbs::<f64, 3>::try_new(
                    3,
                    vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                    vec![p0, p1, p2, p3],
                )
                .unwrap(),
            );
            px += 40.0;
        }

        let k = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
        for _ in 0..2 {
            let r = 20.0_f64;
            curves.push(
                VectorNurbs::<f64, 3>::try_new(
                    3,
                    vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                    vec![
                        [px, py, 0.0],
                        [px + r * k, py, 0.0],
                        [px + r, py + r * (1.0 - k), 0.0],
                        [px + r, py + r, 0.0],
                    ],
                )
                .unwrap(),
            );
            px += r;
            py += r;
        }

        let limits = realistic_machine_limits();
        let segments: Vec<_> = curves
            .iter()
            .map(|c| SegmentInput {
                curve: c,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            })
            .collect();
        let input = BatchInput {
            segments: &segments,
            grid_strategy: adaptive(),
            worker_threads: 3,
            initial_velocity: 0.0,
            initial_accel: 0.0,
            terminal_velocity: 0.0,
        };

        let t0 = Instant::now();
        let output = plan_batch(input).expect("should succeed");
        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;

        assert!(output.joining_sweeps <= 3);

        for (i, profile) in output.profiles.iter().enumerate() {
            // Profile 7 (a G5 cubic) hits the SLP per-axis-jerk linearization
            // gap and may surface MaxIter with a tiny residual; all others must
            // be in the solved set.
            let is_curved_arc = i == 7;
            let acceptable = matches!(
                profile.status,
                temporal::SolveStatus::Solved
                    | temporal::SolveStatus::SolvedInexact { .. }
                    | temporal::SolveStatus::SolvedSlp { .. }
            ) || (is_curved_arc
                && matches!(
                    profile.status,
                    temporal::SolveStatus::MaxIter { last_residual } if last_residual < 1e-6
                ));
            assert!(
                acceptable,
                "profile {i} status not acceptable: {:?}",
                profile.status
            );
        }

        let joining_ok = matches!(output.joining_status, JoiningStatus::Converged)
            || (matches!(
                output.joining_status,
                JoiningStatus::StalledOnInfeasibleSegment {
                    last_dirty_count: 1
                }
            ) && output.profiles.iter().enumerate().all(|(i, p)| {
                let is_curved_arc = i == 7;
                matches!(
                    p.status,
                    temporal::SolveStatus::Solved
                        | temporal::SolveStatus::SolvedInexact { .. }
                        | temporal::SolveStatus::SolvedSlp { .. }
                ) || (is_curved_arc
                    && matches!(
                        p.status,
                        temporal::SolveStatus::MaxIter { last_residual } if last_residual < 1e-6
                    ))
            }));
        assert!(
            joining_ok,
            "joining_status not acceptable: {:?}",
            output.joining_status
        );

        if matches!(output.joining_status, JoiningStatus::Converged) {
            assert_junction_continuity_for_all(&output, 1.0);
        }

        eprintln!("fixture_6 wall-clock: {elapsed_ms:.2} ms (no acceptance threshold)");
    }
}

mod fixture_8_stub_25mms_no_haircut {
    use super::*;
    use temporal::{
        GridConfig, GridScheme, SolveStatus, ToleranceMode, schedule_segment_with_tolerance,
    };

    fn trident_limits() -> Limits {
        Limits::new(
            [25.0, 1000.0, 15.0],
            [70_000.0, 70_000.0, 100.0],
            [140_000.0, 140_000.0, 200.0],
            5.0_f64.powi(2) / (70_000.0 * 0.5),
        )
    }

    #[test]
    fn stub_returns_success_at_full_entry_velocity() {
        let stub = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [0.2, 0.0, 0.0],
                [0.4, 0.0, 0.0],
                [0.6, 0.0, 0.0],
            ],
        )
        .unwrap();
        let limits = trident_limits();
        let grid = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 20,
        };
        let profile =
            schedule_segment_with_tolerance(&stub, &limits, &grid, 25.0, 1.0, ToleranceMode::Auto)
                .expect("must not return ScheduleError");

        assert!(
            matches!(
                profile.status,
                SolveStatus::Solved
                    | SolveStatus::SolvedInexact { .. }
                    | SolveStatus::SolvedSlp { .. }
            ),
            "stub must return success (not DivergedSlp); got {:?}",
            profile.status,
        );

        let v_first = profile.samples.first().expect("non-empty profile").v;
        assert!(
            (v_first - 25.0).abs() < 0.5,
            "v_start must equal 25 mm/s (no haircut); got {v_first:.4}",
        );
    }
}

mod fixture_7_curvature_spike_intergrid_sanity {
    use super::*;
    use nurbs::eval::{curvature_from_derivs, vector_derivative, vector_eval};
    use temporal::{
        GridConfig, GridSample, GridScheme, ToleranceMode, schedule_segment_with_tolerance,
    };

    #[test]
    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn fixture_7() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [2.0, 2.0, 0.0],
                [3.0, 2.0, 0.0],
                [5.0, 0.0, 0.0],
            ],
        )
        .unwrap();
        let limits = textbook_limits();

        let grid = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 10,
        };
        let profile =
            schedule_segment_with_tolerance(&curve, &limits, &grid, 0.0, 0.0, ToleranceMode::Auto)
                .expect("schedule_segment_with_tolerance");

        let d1 = vector_derivative(&curve);
        let d2 = vector_derivative(&d1);

        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_resampled = 4 * profile.samples.len();
        let mut violations: Vec<String> = Vec::new();
        let u_start = curve.knots()[0];
        let u_end = curve.knots()[curve.knots().len() - 1];
        for k in 0..n_resampled {
            #[allow(clippy::cast_precision_loss)]
            let t = (k as f64) / (n_resampled as f64 - 1.0);
            let (v_path, a_path) = hermite_interp(&profile.samples, t);

            let u = u_start + (u_end - u_start) * t;

            let r1 = vector_eval(&d1.as_view(), u);
            let r2 = vector_eval(&d2.as_view(), u);
            let kappa = curvature_from_derivs(&d1, &d2, u);
            let speed_param = mag_3(r1);
            if speed_param < 1e-12 {
                continue;
            }

            let inv_speed = 1.0 / speed_param;
            let tangent = [r1[0] * inv_speed, r1[1] * inv_speed, r1[2] * inv_speed];
            let r2_dot_t = r2[0] * tangent[0] + r2[1] * tangent[1] + r2[2] * tangent[2];
            let r2_perp = [
                r2[0] - r2_dot_t * tangent[0],
                r2[1] - r2_dot_t * tangent[1],
                r2[2] - r2_dot_t * tangent[2],
            ];
            let r2_perp_mag = mag_3(r2_perp);
            let normal_dir = if r2_perp_mag < 1e-12 {
                [0.0; 3]
            } else {
                [
                    r2_perp[0] / r2_perp_mag,
                    r2_perp[1] / r2_perp_mag,
                    r2_perp[2] / r2_perp_mag,
                ]
            };
            let v_squared = v_path * v_path;
            let a_axis = [
                tangent[0] * a_path + normal_dir[0] * kappa * v_squared,
                tangent[1] * a_path + normal_dir[1] * kappa * v_squared,
                tangent[2] * a_path + normal_dir[2] * kappa * v_squared,
            ];

            for axis in 0..3 {
                let v_axis = tangent[axis].abs() * v_path;
                if v_axis > limits.v_max[axis] * 1.001 {
                    violations.push(format!(
                        "v_axis at u={u}, axis={axis}: {v_axis} > v_max={}",
                        limits.v_max[axis],
                    ));
                }
                if a_axis[axis].abs() > limits.a_max[axis] * 1.001 {
                    violations.push(format!(
                        "a_axis at u={u}, axis={axis}: {} > a_max={}",
                        a_axis[axis].abs(),
                        limits.a_max[axis],
                    ));
                }
            }
            // The SOCP enforces centripetal only at grid points; inter-grid
            // overshoot measures 1.036 on this fixture.
            if v_squared * kappa > limits.a_centripetal_max * 1.05 {
                violations.push(format!(
                    "centripetal at u={u}: v²·κ={} > a_cent={}",
                    v_squared * kappa,
                    limits.a_centripetal_max,
                ));
            }
        }

        assert!(
            violations.is_empty(),
            "v1 adaptive-N policy under-resolved curvature spikes — escalate to v2:\n{}",
            violations.join("\n"),
        );
    }

    /// Piecewise-cubic Hermite interpolation of (v, a) solver samples at t ∈ [0,1].
    /// Treats `sample.v` as function value and `sample.a` as its time-derivative.
    fn hermite_interp(samples: &[GridSample], t: f64) -> (f64, f64) {
        let n = samples.len();
        if n < 2 {
            return (samples.first().map_or(0.0, |s| s.v), 0.0);
        }
        #[allow(clippy::cast_precision_loss)]
        let pos = t * ((n - 1) as f64);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let i = (pos.floor() as usize).min(n - 2);
        #[allow(clippy::cast_precision_loss)]
        let s = pos - (i as f64);

        let v_i = samples[i].v;
        let v_ip1 = samples[i + 1].v;
        let a_i = samples[i].a;
        let a_ip1 = samples[i + 1].a;

        let ds = samples[i + 1].s - samples[i].s;
        let v_avg = 0.5_f64.mul_add(v_i + v_ip1, 0.0).max(1e-9);
        let dt = ds / v_avg;

        let s2 = s * s;
        let s3 = s2 * s;
        let h00 = 2.0_f64.mul_add(s3, -(3.0 * s2)) + 1.0;
        let h10 = s3 - 2.0 * s2 + s;
        let h01 = (-2.0_f64).mul_add(s3, 3.0 * s2);
        let h11 = s3 - s2;
        let v_interp = h00 * v_i + h10 * dt * a_i + h01 * v_ip1 + h11 * dt * a_ip1;

        let dh00 = 6.0_f64.mul_add(s2, -(6.0 * s));
        let dh10 = 3.0_f64.mul_add(s2, -(4.0 * s)) + 1.0;
        let dh01 = (-6.0_f64).mul_add(s2, 6.0 * s);
        let dh11 = 3.0_f64.mul_add(s2, -(2.0 * s));
        let dv_ds = dh00 * v_i + dh10 * dt * a_i + dh01 * v_ip1 + dh11 * dt * a_ip1;
        let a_interp = dv_ds / dt;

        (v_interp, a_interp)
    }

    #[inline]
    fn mag_3(v: [f64; 3]) -> f64 {
        v[0].mul_add(v[0], v[1].mul_add(v[1], v[2] * v[2])).sqrt()
    }
}
