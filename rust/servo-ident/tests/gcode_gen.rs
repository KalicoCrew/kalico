use servo_ident::gcode_gen::{generate, Excitation};

#[test]
fn strokes_stay_in_bounds_and_reach_peak_speed() {
    let e = Excitation {
        axis: "X".into(),
        min_mm: 10.0,
        max_mm: 210.0,
        accels_mm_s2: vec![1000.0, 3000.0],
        speeds_mm_s: vec![100.0, 300.0],
        reps: 3,
    };
    let g = generate(&e).unwrap();
    assert!(g
        .lines()
        .any(|l| l == "SET_VELOCITY_LIMIT ACCEL=1000 ACCEL_TO_DECEL=1000"));
    assert!(g
        .lines()
        .any(|l| l == "SET_VELOCITY_LIMIT ACCEL=3000 ACCEL_TO_DECEL=3000"));
    assert!(g.lines().any(|l| l == "G1 X210 F18000"));
    assert!(g.lines().any(|l| l == "M400"));
    for line in g.lines().filter(|l| l.starts_with("G1 X")) {
        let x: f64 = line[4..]
            .split_whitespace()
            .next()
            .unwrap()
            .parse()
            .unwrap();
        assert!((10.0..=210.0).contains(&x), "{line}");
    }
    // 2 accels × 2 speeds × 3 reps × 2 directions = 24 G1 lines
    assert_eq!(g.lines().filter(|l| l.starts_with("G1 ")).count(), 24);
    // 2 accels × 2 speeds = 4 M400 lines
    assert_eq!(g.lines().filter(|l| *l == "M400").count(), 4);
}

#[test]
fn refuses_stroke_too_short_for_peak_speed() {
    let e = Excitation {
        axis: "X".into(),
        min_mm: 0.0,
        max_mm: 20.0,
        accels_mm_s2: vec![500.0],
        speeds_mm_s: vec![300.0],
        reps: 1,
    };
    assert!(generate(&e).is_err());
}

#[test]
fn refuses_empty_or_nonpositive_inputs() {
    let base = Excitation {
        axis: "X".into(),
        min_mm: 0.0,
        max_mm: 100.0,
        accels_mm_s2: vec![1000.0],
        speeds_mm_s: vec![100.0],
        reps: 1,
    };

    assert!(generate(&Excitation {
        accels_mm_s2: vec![],
        ..base.clone()
    })
    .is_err());
    assert!(generate(&Excitation {
        speeds_mm_s: vec![],
        ..base.clone()
    })
    .is_err());
    assert!(generate(&Excitation {
        reps: 0,
        ..base.clone()
    })
    .is_err());
    assert!(generate(&Excitation {
        accels_mm_s2: vec![-1.0],
        ..base.clone()
    })
    .is_err());
    assert!(generate(&Excitation {
        speeds_mm_s: vec![0.0],
        ..base.clone()
    })
    .is_err());
    assert!(generate(&Excitation {
        min_mm: 100.0,
        max_mm: 0.0,
        ..base.clone()
    })
    .is_err());
}
