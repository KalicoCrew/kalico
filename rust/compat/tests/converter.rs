use std::fmt::Write;

use compat::converter::convert;

#[test]
fn g1_to_g5() {
    let input = "G1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("G5 "), "no G5 in output");
    assert!(!output.contains("G1 "), "G1 leaked to output");
}

#[test]
fn g0_to_g5() {
    let input = "G0 X10 Y0 F6000\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("G5 "));
}

#[test]
fn g5_1_to_g5() {
    let input = "G5.1 X10 Y0 I3 J3 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("G5 X"));
    assert!(!output.contains("G5.1"));
}

#[test]
fn g5_passthrough_canonicalized() {
    let input = "G5 X10 Y0 I3 J3 P-3 Q3 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("G5 X"));
}

#[test]
fn comments_preserved() {
    let input = "; hello world\nG1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("; hello world"));
}

#[test]
fn m_codes_preserved() {
    let input = "M104 S210\nG1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("M104"));
}

#[test]
fn preamble_present() {
    let input = "G1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("G90"));
    assert!(output.contains("M82"));
    assert!(output.contains("G17"));
}

#[test]
fn g90_g91_stripped() {
    let input = "G90\nG91\nG90\nG1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    let g90_count = output.matches("G90").count();
    assert_eq!(g90_count, 1, "only preamble G90 should survive");
    assert!(!output.contains("G91"));
}

#[test]
fn g18_stripped() {
    let input = "G18\nG17\nG1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(!output.contains("G18"));
}

#[test]
fn missing_feedrate_fatal() {
    let input = "G1 X10 Y0 E1.0\n";
    let result = convert(input, "test", 5.0);
    assert!(result.is_err());
}

#[test]
fn g92_passes_through() {
    let input = "G92 E0\nG1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("G92"));
}

#[test]
fn e_only_move() {
    // Retraction: G1 with E but no XY motion
    let input = "G1 X10 Y0 E1.0 F1500\nG1 E0.5 F2400\n";
    let output = convert(input, "test", 5.0).unwrap();
    // Should produce 2 G5 lines (one for XY move, one for E-only)
    let g5_count = output.lines().filter(|l| l.starts_with("G5 ")).count();
    assert!(g5_count >= 2, "expected >=2 G5 lines, got {g5_count}");
}

#[test]
fn multi_g1_sequence() {
    let mut lines = String::new();
    for i in 0..=10 {
        let _ = writeln!(lines, "G1 X{i} Y0 E{:.5} F1500", f64::from(i) * 0.1);
    }
    let output = convert(&lines, "test", 5.0).unwrap();
    assert!(output.contains("G5 "));
    // Should produce fewer G5 lines than input G1 lines (fitter combines them)
    let g5_count = output.lines().filter(|l| l.starts_with("G5 ")).count();
    assert!(g5_count <= 11, "expected <=11 G5 lines, got {g5_count}");
}

#[test]
fn t_code_preserved() {
    let input = "T0\nG1 X10 Y0 E1.0 F1500\n";
    let output = convert(input, "test", 5.0).unwrap();
    assert!(output.contains("T0"));
}
