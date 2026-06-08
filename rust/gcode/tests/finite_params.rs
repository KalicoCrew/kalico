use gcode::{ParseError, lex};

fn assert_lex_rejects_as_malformed(src: &str) {
    let results: Vec<_> = lex(src).collect();
    assert!(
        results
            .iter()
            .any(|r| matches!(r, Err(ParseError::MalformedNumber { .. }))),
        "expected MalformedNumber for {src:?}, got {results:#?}"
    );
}

#[test]
fn rejects_nan() {
    assert_lex_rejects_as_malformed("G5 XNaN Y0 I0 J3 P0 Q-3 F1000\n");
}

#[test]
fn rejects_lower_nan() {
    assert_lex_rejects_as_malformed("G5 X1 Ynan I0 J3 P0 Q-3 F1000\n");
}

#[test]
fn rejects_signed_nan() {
    assert_lex_rejects_as_malformed("G5 X1 Y-NaN I0 J3 P0 Q-3 F1000\n");
}

#[test]
fn rejects_inf() {
    assert_lex_rejects_as_malformed("G5 X1 Y0 Iinf J3 P0 Q-3 F1000\n");
}

#[test]
fn rejects_signed_inf() {
    assert_lex_rejects_as_malformed("G5 X1 Y0 I+inf J3 P0 Q-3 F1000\n");
}

#[test]
fn rejects_negative_inf() {
    assert_lex_rejects_as_malformed("G5 X1 Y0 I3 J-inf P0 Q-3 F1000\n");
}

#[test]
fn rejects_infinity() {
    assert_lex_rejects_as_malformed("G5 X1 Y0 Iinfinity J3 P0 Q-3 F1000\n");
}

#[test]
fn rejects_inf_feedrate() {
    assert_lex_rejects_as_malformed("G5 X1 Y0 I0 J3 P0 Q-3 Finf\n");
}

#[test]
fn rejects_nan_e() {
    assert_lex_rejects_as_malformed("G5 X1 Y0 I0 J3 P0 Q-3 ENaN F1000\n");
}

#[test]
fn accepts_finite_floats() {
    let results: Vec<_> = lex("G5 X1.5 Y-2.7 I0 J3 P0 Q-3 F1000\n").collect();
    assert!(
        results.iter().all(Result::is_ok),
        "expected all-Ok for finite input, got {results:#?}"
    );
}
