use std::time::Instant;

fn gen_straight_line_raster() -> String {
    const MOVE_COUNT: u32 = 160_080;
    let mut out = String::with_capacity(MOVE_COUNT as usize * 40 + 256);

    out.push_str("; Synthetic straight-line corpus — generated at test time\n");
    out.push_str("; Contains > 150k G1 command tokens\n");
    out.push_str("G21\n");
    out.push_str("G90\n");
    out.push_str("M82\n");
    out.push_str("G92 E0\n");
    out.push_str(";LAYER_CHANGE\n");
    out.push_str("G1 Z0.200 F600\n");

    for i in 0..MOVE_COUNT {
        let x = if i % 2 == 0 { 220.0_f64 } else { 0.0_f64 };
        let y = f64::from(i) * 0.300;
        let e = f64::from(i + 1) * 0.0500;

        use std::fmt::Write as _;
        let _ = writeln!(out, "G1 X{x:.3} Y{y:.3} E{e:.4} F3000");
    }

    out
}

const ARC_CYCLE: &[([f64; 2], [f64; 2]); 4] = &[
    ([110.0, 160.0], [-50.0, 0.0]),
    ([60.0, 110.0], [0.0, -50.0]),
    ([110.0, 60.0], [50.0, 0.0]),
    ([160.0, 110.0], [0.0, 50.0]),
];

fn gen_arc_circle() -> String {
    const CYCLE_COUNT: u32 = 25_000;
    const ARCS_PER_CYCLE: u32 = 4;
    let total_arcs = CYCLE_COUNT * ARCS_PER_CYCLE;
    let capacity = (total_arcs as usize) * 55 + (CYCLE_COUNT as usize) * 30 + 256;
    let mut out = String::with_capacity(capacity);

    out.push_str("; Synthetic arc-fitted corpus — generated at test time\n");
    out.push_str("; Contains > 100k G2 command tokens\n");
    out.push_str("G21\n");
    out.push_str("G90\n");
    out.push_str("M82\n");
    out.push_str("G92 E0\n");
    out.push_str(";LAYER_CHANGE\n");
    out.push_str("G1 Z0.200 F600\n");

    use std::fmt::Write as _;
    let mut arc_index: u32 = 0;

    for _ in 0..CYCLE_COUNT {
        out.push_str("G1 X160.000 Y110.000 F3000\n");

        for &([ex, ey], [i, j]) in ARC_CYCLE {
            arc_index += 1;
            let e = f64::from(arc_index) * 0.1;
            let _ = writeln!(out, "G2 X{ex:.3} Y{ey:.3} I{i:.3} J{j:.3} E{e:.4} F3000");
        }
    }

    out
}

fn assert_no_legacy_gcode(output: &str) {
    for line in output.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with(';') {
            continue;
        }
        assert!(
            !trimmed.starts_with("G0 ")
                && !trimmed.starts_with("G1 ")
                && !trimmed.starts_with("G2 ")
                && !trimmed.starts_with("G3 ")
                && !trimmed.starts_with("G5.1 "),
            "Legacy G-code found in output: {trimmed}"
        );
    }
}

fn convert_generated(gcode: &str, label: &str) -> String {
    compat::converter::convert(gcode, label, 5.0)
        .unwrap_or_else(|e| panic!("conversion of {label} failed: {e}"))
}

#[test]
fn generated_straight_line_converts() {
    let input = gen_straight_line_raster();
    let output = convert_generated(&input, "<generated:straight_line>");
    assert_no_legacy_gcode(&output);
}

#[test]
fn generated_arc_circle_converts() {
    let input = gen_arc_circle();
    let output = convert_generated(&input, "<generated:arc_circle>");
    assert_no_legacy_gcode(&output);
}

#[test]
fn output_relexes_cleanly() {
    let input = gen_straight_line_raster();
    let output = convert_generated(&input, "<generated:straight_line>");
    let errors: Vec<_> = gcode::lex(&output)
        .filter_map(std::result::Result::err)
        .collect();
    assert!(errors.is_empty(), "Lexer errors in output: {errors:?}");
}

#[test]
fn straight_line_corpus_under_30_seconds() {
    let input = gen_straight_line_raster();
    let t0 = Instant::now();
    let _output = convert_generated(&input, "<generated:straight_line>");
    let elapsed = t0.elapsed();
    assert!(
        elapsed.as_secs() < 30,
        "straight-line corpus conversion took {}s — must complete in < 30s",
        elapsed.as_secs()
    );
}
