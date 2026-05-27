use super::*;

#[test]
fn parse_error_displays_line_no() {
    let e = ParseError::MalformedNumber {
        line_no: 7,
        text: "G1 X1.2.3".into(),
    };
    let s = format!("{e}");
    assert!(s.contains("line 7"));
    assert!(s.contains("malformed number"));
}

#[test]
fn parse_error_unrecognized_head() {
    let e = ParseError::UnrecognizedHead {
        line_no: 12,
        head: "X1".into(),
    };
    let s = format!("{e}");
    assert!(s.contains("line 12"));
}
