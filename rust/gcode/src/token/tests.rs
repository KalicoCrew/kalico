use super::*;

#[test]
fn params_indexed_by_letter() {
    let mut p = Params::default();
    p.set(b'X', 1.5);
    p.set(b'Y', -2.0);
    assert_eq!(p.get(b'X'), Some(1.5));
    assert_eq!(p.get(b'Y'), Some(-2.0));
    assert_eq!(p.get(b'Z'), None);
}

#[test]
fn params_get_returns_none_for_non_letter_bytes() {
    let p = Params::default();
    assert_eq!(p.get(b'1'), None);
    assert_eq!(p.get(b'a'), None);
    assert_eq!(p.get(b' '), None);
    assert_eq!(p.get(0), None);
}

#[test]
fn params_set_is_no_op_for_non_uppercase() {
    let mut p = Params::default();
    p.set(b'a', 1.0);
    p.set(b'1', 2.0);
    assert_eq!(p.get(b'A'), None);
    assert_eq!(p.get(b'a'), None);
}

#[test]
fn token_command_round_trip() {
    let mut params = Params::default();
    params.set(b'X', 10.0);
    let t = Token::Command {
        letter: b'G',
        major: 1,
        minor: None,
        params,
        line_no: 42,
    };
    match t {
        Token::Command {
            letter,
            major,
            minor,
            params,
            line_no,
        } => {
            assert_eq!(letter, b'G');
            assert_eq!(major, 1);
            assert_eq!(minor, None);
            assert_eq!(params.x(), Some(10.0));
            assert_eq!(line_no, 42);
        }
        _ => panic!("expected Command"),
    }
}

#[test]
fn marker_kind_layer_change() {
    let m = MarkerKind::LayerChange { layer: Some(5) };
    match m {
        MarkerKind::LayerChange { layer } => assert_eq!(layer, Some(5)),
        _ => panic!("expected LayerChange"),
    }
}
