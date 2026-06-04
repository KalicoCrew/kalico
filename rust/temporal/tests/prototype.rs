#![allow(clippy::doc_markdown)]
#![allow(clippy::uninlined_format_args)]

mod biagiotti_melchiorri {
    pub fn total_time_double_s(l: f64, v_max: f64, a_max: f64, j_max: f64) -> f64 {
        let t_j = a_max / j_max;
        let v_after_jerk_pair = a_max * a_max / j_max;

        if v_after_jerk_pair > v_max {
            return bisect_v_peak_for_short_move(l, v_max, a_max, j_max);
        }

        let t_a = ((v_max - a_max * a_max / j_max) / a_max).max(0.0);
        let v_peak = v_max;

        let d_accel = v_peak * (2.0 * t_j + t_a) / 2.0;

        let d_cruise_required = l - 2.0 * d_accel;
        if d_cruise_required <= 0.0 {
            return bisect_v_peak_for_short_move(l, v_max, a_max, j_max);
        }
        let t_cruise = d_cruise_required / v_peak;

        2.0 * (2.0 * t_j + t_a) + t_cruise
    }

    fn bisect_v_peak_for_short_move(l: f64, v_max: f64, a_max: f64, j_max: f64) -> f64 {
        let mut lo = 1e-6_f64;
        let mut hi = v_max;
        for _ in 0..80 {
            let mid = 0.5 * (lo + hi);
            let t_j = a_max / j_max;
            let t_a = ((mid - a_max * a_max / j_max) / a_max).max(0.0);
            let d_accel = mid * (2.0 * t_j + t_a) / 2.0;
            if 2.0 * d_accel > l {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        let v_peak = 0.5 * (lo + hi);
        let t_j = a_max / j_max;
        let t_a = ((v_peak - a_max * a_max / j_max) / a_max).max(0.0);
        2.0 * (2.0 * t_j + t_a)
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        #[test]
        fn cruise_dominated_move_total_time_known() {
            let t = total_time_double_s(100.0, 500.0, 5_000.0, 100_000.0);
            assert!((t - 0.35).abs() < 1e-6, "got T = {t}, expected 0.35");
        }
    }
}

mod fixture_1_straight_line_x_aligned {
    use super::biagiotti_melchiorri::total_time_double_s;
    use nurbs::VectorNurbs;
    use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

    fn textbook_limits() -> Limits {
        Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    #[test]
    fn fixture_1() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        )
        .unwrap();

        let limits = textbook_limits();
        let cfg = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };
        let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule");

        match profile.status {
            SolveStatus::Solved
            | SolveStatus::SolvedInexact { .. }
            | SolveStatus::SolvedSlp { .. } => {}
            ref other => panic!("fixture 1 status: {:?}", other),
        }

        let t_closed =
            total_time_double_s(100.0, limits.v_max[0], limits.a_max[0], limits.j_max[0]);
        let rel_err = (profile.total_time - t_closed).abs() / t_closed;
        assert!(
            rel_err <= 0.08,
            "fixture 1 §6.3: T_topp = {} vs T_closed = {} (rel_err = {:.4})",
            profile.total_time,
            t_closed,
            rel_err,
        );

        eprintln!(
            "fixture 1: T_topp = {:.6}, T_closed = {:.6}",
            profile.total_time, t_closed
        );
    }
}

mod fixture_2_diagonal {
    use super::biagiotti_melchiorri::total_time_double_s;
    use nurbs::VectorNurbs;
    use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

    fn textbook_limits() -> Limits {
        Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    #[test]
    fn fixture_2() {
        let h = 100.0 / std::f64::consts::SQRT_2;
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [h, h, 0.0]],
        )
        .unwrap();

        let limits = textbook_limits();
        let cfg = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };
        let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule");

        assert!(matches!(
            profile.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. }
        ));

        let sqrt2 = std::f64::consts::SQRT_2;
        let v_eff = limits.v_max[0] * sqrt2;
        let a_eff = limits.a_max[0] * sqrt2;
        let j_eff = limits.j_max[0] * sqrt2;
        let t_closed = total_time_double_s(100.0, v_eff, a_eff, j_eff);
        let rel_err = (profile.total_time - t_closed).abs() / t_closed;
        assert!(
            rel_err <= 0.05,
            "fixture 2 §6.3: T_topp = {} vs T_closed = {} (rel = {:.4})",
            profile.total_time,
            t_closed,
            rel_err
        );

        eprintln!(
            "fixture 2: T_topp = {:.6}, T_closed = {:.6}",
            profile.total_time, t_closed
        );
    }
}

mod fixture_4_g5_cubic {
    use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

    fn textbook_limits() -> Limits {
        Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    #[test]
    fn fixture_4() {
        let curve = build_g5_via_geometry();

        let limits = textbook_limits();
        let cfg = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };

        let (mvc_b_start, mvc_b_end) = mvc_endpoints(&curve, &limits);
        let v_start = 0.5 * mvc_b_start.sqrt();
        let v_end = 0.5 * mvc_b_end.sqrt();

        eprintln!(
            "fixture 4: mvc_b_start = {:.4}, mvc_b_end = {:.4}, v_start = {:.4}, v_end = {:.4}",
            mvc_b_start, mvc_b_end, v_start, v_end,
        );

        let profile = schedule_segment(&curve, &limits, &cfg, v_start, v_end).expect("schedule");

        assert!(
            matches!(
                profile.status,
                SolveStatus::Solved
                    | SolveStatus::SolvedInexact { .. }
                    | SolveStatus::SolvedSlp { .. }
            ),
            "fixture 4 status: {:?}",
            profile.status,
        );

        eprintln!(
            "fixture 4: status = {:?}, total_time = {:.6}",
            profile.status, profile.total_time
        );
    }

    fn build_g5_via_geometry() -> nurbs::VectorNurbs<f64, 3> {
        use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

        let src = "G5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
        let mut pipeline = GeometryPipeline::new(FitterParams::default());
        let mut events: Vec<TelemetryEvent> = vec![];
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            pipeline.process(src, &mut sink).collect()
        };

        items
            .into_iter()
            .find_map(|it| match it {
                Item::Segment(Segment::Cubic(c)) => Some(c.xyz),
                _ => None,
            })
            .expect("G5 reduction must emit exactly one Segment::Cubic")
    }

    fn mvc_endpoints(curve: &nurbs::VectorNurbs<f64, 3>, limits: &Limits) -> (f64, f64) {
        use temporal::topp::path::sample_arclength_grid;

        let grid = sample_arclength_grid(curve, 3)
            .expect("arclength grid must succeed for a valid G5 NURBS");

        let kappa_start = grid.kappa[0];
        let kappa_end = *grid.kappa.last().expect("grid has ≥ 2 points");

        eprintln!(
            "fixture 4 mvc_endpoints: κ_start = {:.6e}, κ_end = {:.6e}",
            kappa_start, kappa_end,
        );

        let b_start = (limits.a_centripetal_max / kappa_start.max(1e-12)).min(1e8);
        let b_end = (limits.a_centripetal_max / kappa_end.max(1e-12)).min(1e8);

        (b_start, b_end)
    }
}

mod fixture_5_curvature_spike {
    use nurbs::VectorNurbs;
    use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

    fn textbook_limits() -> Limits {
        Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    #[test]
    fn fixture_5() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [29.0, 5.0, 0.0],
                [31.0, 5.0, 0.0],
                [60.0, 0.0, 0.0],
            ],
        )
        .unwrap();

        let limits = textbook_limits();
        let cfg = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };
        let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule");

        assert!(
            matches!(
                profile.status,
                SolveStatus::Solved
                    | SolveStatus::SolvedInexact { .. }
                    | SolveStatus::SolvedSlp { .. }
            ),
            "fixture 5 status: {:?} (relaxation tightness gap or numerical pathology, see spec §7.1, §7.2)",
            profile.status
        );

        eprintln!(
            "fixture 5: status = {:?}, total_time = {:.6}",
            profile.status, profile.total_time
        );
    }
}

mod fixture_6_mixed_feature {
    use nurbs::VectorNurbs;
    use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

    fn textbook_limits() -> Limits {
        Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    pub(super) fn build_mixed_curve() -> nurbs::VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [60.0, 0.0, 0.0],
                [145.0, 0.0, 0.0],
                [155.0, 0.0, 0.0],
                [161.0, 12.0, 0.0],
                [169.0, 0.0, 0.0],
                [180.0, 0.0, 0.0],
                [305.0, 0.0, 0.0],
            ],
        )
        .unwrap()
    }

    #[test]
    fn fixture_6() {
        let curve = build_mixed_curve();
        let limits = textbook_limits();
        let cfg = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };
        let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule");

        assert!(
            matches!(
                profile.status,
                SolveStatus::Solved
                    | SolveStatus::SolvedInexact { .. }
                    | SolveStatus::SolvedSlp { .. }
            ),
            "got {:?}",
            profile.status
        );

        let n = profile.samples.len();
        let (min_idx, _) = profile
            .samples
            .iter()
            .enumerate()
            .skip(1)
            .take(n - 2)
            .min_by(|(_, a), (_, b)| a.v.partial_cmp(&b.v).unwrap())
            .unwrap();
        assert!(
            min_idx > n / 4 && min_idx < 3 * n / 4,
            "fixture 6: min-v at idx {} not in middle half (n = {})",
            min_idx,
            n
        );

        for i in 1..(n / 4) {
            assert!(
                profile.samples[i].v >= profile.samples[i - 1].v - 1e-3,
                "fixture 6: lead-in not monotone at i={}: v[{}]={} v[{}]={}",
                i,
                i - 1,
                profile.samples[i - 1].v,
                i,
                profile.samples[i].v
            );
        }
        for i in (3 * n / 4)..n {
            assert!(
                profile.samples[i].v <= profile.samples[i - 1].v + 1e-3,
                "fixture 6: lead-out not monotone at i={}",
                i
            );
        }

        eprintln!(
            "fixture 6: status = {:?}, total_time = {:.6}, min_v_idx = {}",
            profile.status, profile.total_time, min_idx
        );
    }
}

mod fixture_7_convergence {
    use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

    fn realistic_limits() -> Limits {
        Limits::new(
            [1_000.0, 1_000.0, 1_000.0],
            [65_000.0, 65_000.0, 65_000.0],
            [50_000_000.0, 50_000_000.0, 50_000_000.0],
            65_000.0,
        )
    }

    #[test]
    fn fixture_7_convergence() {
        let curve = super::fixture_6_mixed_feature::build_mixed_curve();
        let limits = realistic_limits();

        let mut times = std::collections::BTreeMap::new();
        for &n in &[50_usize, 100, 200, 400] {
            let cfg = GridConfig {
                scheme: GridScheme::UniformArclength,
                n,
            };
            let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0)
                .unwrap_or_else(|e| panic!("fixture 7 N={n} schedule error: {e}"));
            assert!(
                matches!(
                    profile.status,
                    SolveStatus::Solved
                        | SolveStatus::SolvedInexact { .. }
                        | SolveStatus::SolvedSlp { .. }
                ),
                "fixture 7 N={n} status: {:?}",
                profile.status
            );
            eprintln!("fixture 7 N={n}: total_time = {:.6}", profile.total_time);
            times.insert(n, profile.total_time);
        }

        let t100 = times[&100];
        let t200 = times[&200];
        let t400 = times[&400];

        let rel_400_200 = (t400 - t200).abs() / t400;
        let rel_200_100 = (t200 - t100).abs() / t200;

        assert!(
            rel_400_200 < 0.05,
            "§6.4: |T(400)-T(200)|/T(400) = {:.5} > 5.0%",
            rel_400_200
        );
        assert!(
            rel_200_100 < 0.05,
            "§6.4: |T(200)-T(100)|/T(200) = {:.5} > 5.0%",
            rel_200_100
        );
    }
}

mod fixture_3_constant_curvature_arc {
    use temporal::{
        BindingConstraint, GridConfig, GridScheme, Limits, SolveStatus, schedule_segment,
    };

    fn textbook_limits() -> Limits {
        Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    #[test]
    fn fixture_3() {
        let curve = build_g2_arc_via_geometry();

        let limits = textbook_limits();
        let cfg = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        };
        let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule");

        match profile.status {
            SolveStatus::Solved
            | SolveStatus::SolvedInexact { .. }
            | SolveStatus::SolvedSlp { .. } => {}
            SolveStatus::DivergedSlp { last_max_ratio, .. } => {
                assert!(
                    last_max_ratio < 1.02,
                    "DivergedSlp accepted only with last_max_ratio < 1.02, got {}",
                    last_max_ratio,
                );
            }
            ref other => panic!(
                "expected Solved/SolvedInexact/SolvedSlp or DivergedSlp(<1.02), got {:?}",
                other,
            ),
        }

        let mid = profile.samples.len() / 2;
        let v_cruise_expected = (2_500.0_f64 / 0.05).sqrt();
        let v_mid = profile.samples[mid].v;
        assert!(
            (v_mid - v_cruise_expected).abs() / v_cruise_expected < 0.05,
            "fixture 3 cruise: v_mid = {}, expected ~{} (5% tolerance)",
            v_mid,
            v_cruise_expected,
        );
        assert!(
            matches!(profile.samples[mid].binding, BindingConstraint::Centripetal),
            "fixture 3: binding at mid should be Centripetal, got {:?}",
            profile.samples[mid].binding,
        );

        eprintln!(
            "fixture 3: v_mid = {:.4}, v_cruise_expected = {:.4}, status = {:?}",
            v_mid, v_cruise_expected, profile.status,
        );
    }

    fn build_g2_arc_via_geometry() -> nurbs::VectorNurbs<f64, 3> {
        let r = 20.0_f64;
        let k = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
        nurbs::VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [r * k, 0.0, 0.0],
                [r, r * (1.0 - k), 0.0],
                [r, r, 0.0],
            ],
        )
        .expect("cubic arc approximation NURBS always valid")
    }
}
