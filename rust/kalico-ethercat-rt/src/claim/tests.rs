use super::{eval_wkc, parse_fail_bringup, WkcDecision};

fn args(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

#[test]
fn wkc_good_good_stays_zero() {
    let mut c = 0u8;
    assert_eq!(eval_wkc(3, 3, &mut c), WkcDecision::Good);
    assert_eq!(c, 0);
    assert_eq!(eval_wkc(3, 3, &mut c), WkcDecision::Good);
    assert_eq!(c, 0);
}

#[test]
fn wkc_bad_then_good_resets_counter() {
    let mut c = 0u8;
    assert_eq!(eval_wkc(-1, 3, &mut c), WkcDecision::Warn(1));
    assert_eq!(c, 1);
    assert_eq!(eval_wkc(3, 3, &mut c), WkcDecision::Good);
    assert_eq!(c, 0);
}

#[test]
fn wkc_two_consecutive_bad_halts() {
    let mut c = 0u8;
    assert_eq!(eval_wkc(-1, 3, &mut c), WkcDecision::Warn(1));
    assert_eq!(eval_wkc(-1, 3, &mut c), WkcDecision::Halt);
}

#[test]
fn wkc_interleaved_bad_good_bad_good() {
    let mut c = 0u8;
    assert_eq!(eval_wkc(-1, 3, &mut c), WkcDecision::Warn(1));
    assert_eq!(eval_wkc(3, 3, &mut c), WkcDecision::Good);
    assert_eq!(c, 0, "counter must reset on good cycle");
    assert_eq!(eval_wkc(-1, 3, &mut c), WkcDecision::Warn(1));
    assert_eq!(eval_wkc(3, 3, &mut c), WkcDecision::Good);
    assert_eq!(c, 0, "counter must reset again");
}

#[test]
fn absent_returns_none() {
    assert_eq!(
        parse_fail_bringup(&args(&["--socket", "/tmp/s.sock"])),
        Ok(None)
    );
}

#[test]
fn present_valid() {
    assert_eq!(
        parse_fail_bringup(&args(&["--fail-bringup", "slave=3"])),
        Ok(Some(3))
    );
}

#[test]
fn present_slave_zero() {
    assert_eq!(
        parse_fail_bringup(&args(&["--fail-bringup", "slave=0"])),
        Ok(Some(0))
    );
}

#[test]
fn present_slave_max_u8() {
    assert_eq!(
        parse_fail_bringup(&args(&["--fail-bringup", "slave=255"])),
        Ok(Some(255))
    );
}

#[test]
fn malformed_not_slave_prefix() {
    assert!(parse_fail_bringup(&args(&["--fail-bringup", "banana"])).is_err());
}

#[test]
fn malformed_overflow() {
    assert!(parse_fail_bringup(&args(&["--fail-bringup", "slave=256"])).is_err());
}

#[test]
fn malformed_non_numeric() {
    assert!(parse_fail_bringup(&args(&["--fail-bringup", "slave=abc"])).is_err());
}

#[test]
fn missing_value_after_flag() {
    assert!(parse_fail_bringup(&args(&["--fail-bringup"])).is_err());
}

#[test]
fn flag_not_last_other_args_after() {
    assert_eq!(
        parse_fail_bringup(&args(&["--fail-bringup", "slave=7", "--socket", "/tmp/s"])),
        Ok(Some(7))
    );
}
