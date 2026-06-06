use super::*;

#[test]
fn monotonic_ns_is_monotone() {
    let t0 = monotonic_ns();
    let t1 = monotonic_ns();
    assert!(
        t1 >= t0,
        "monotonic clock must not go backwards: t0={t0} t1={t1}"
    );
}
