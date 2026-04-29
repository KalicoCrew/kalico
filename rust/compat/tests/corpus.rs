use std::fs;

fn convert_file(path: &str) -> String {
    let input = fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("failed to read {path}: {e}"));
    compat::converter::convert(&input, path, 5.0)
        .unwrap_or_else(|e| panic!("conversion of {path} failed: {e}"))
}

#[test]
fn voron_cube_straight_line_converts() {
    let output = convert_file("../../scripts/fitter_prototype/corpus/voron_cube_straight_line.gcode");
    // No legacy G-code in output
    for line in output.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with(';') {
            continue;
        }
        // Allow G5, G17, G90, M82, G92, M-codes, T-codes
        // Reject G0, G1, G2, G3, G5.1
        assert!(
            !trimmed.starts_with("G0 ") && !trimmed.starts_with("G1 ")
                && !trimmed.starts_with("G2 ") && !trimmed.starts_with("G3 ")
                && !trimmed.starts_with("G5.1 "),
            "Legacy G-code found in output: {trimmed}"
        );
    }
}

#[test]
fn voron_cube_arc_fitted_converts() {
    let output = convert_file("../../scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode");
    for line in output.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with(';') {
            continue;
        }
        assert!(
            !trimmed.starts_with("G0 ") && !trimmed.starts_with("G1 ")
                && !trimmed.starts_with("G2 ") && !trimmed.starts_with("G3 ")
                && !trimmed.starts_with("G5.1 "),
            "Legacy G-code found in output: {trimmed}"
        );
    }
}

#[test]
fn output_relexes_cleanly() {
    let output = convert_file("../../scripts/fitter_prototype/corpus/voron_cube_straight_line.gcode");
    let errors: Vec<_> = gcode::lex(&output)
        .filter_map(|r| r.err())
        .collect();
    assert!(errors.is_empty(), "Lexer errors in output: {errors:?}");
}

#[test]
fn straight_line_corpus_under_30_seconds() {
    let start = std::time::Instant::now();
    let _ = convert_file("../../scripts/fitter_prototype/corpus/voron_cube_straight_line.gcode");
    let elapsed = start.elapsed();
    assert!(
        elapsed.as_secs() < 30,
        "Conversion took {elapsed:?}, expected under 30 seconds"
    );
}
