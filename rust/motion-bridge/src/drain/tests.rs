use super::*;

#[test]
fn drained_when_retired_equals_sent() {
    let d = DrainSync::new();
    d.add_sent(1, 0, 3);
    d.add_sent(1, 1, 2);
    assert!(d.wait_drained(Duration::from_millis(20)).is_err());
    d.set_retired(1, 0, 3);
    d.set_retired(1, 1, 2);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn no_sent_is_trivially_drained() {
    let d = DrainSync::new();
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn reset_clears_sent_cumulative_retired_does_not_break_drain() {
    let d = DrainSync::new();
    d.add_sent(1, 0, 5);
    d.set_retired(1, 0, 5);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    d.reset();
    d.set_retired(1, 0, 5);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn regression_cumulative_retired_across_two_streams() {
    let d = DrainSync::new();

    d.add_sent(0, 0, 8737);
    d.set_retired(0, 0, 8737);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "stream 1 should be drained"
    );

    d.reset();

    d.set_retired(0, 0, 8737);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "after reset with no new sent, trivially drained"
    );

    d.add_sent(0, 0, 8737);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "stream 2 not yet drained before MCU catches up"
    );
    d.set_retired(0, 0, 17474);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "stream 2 should be drained at cumulative 17474"
    );
}

#[test]
fn post_reset_partial_retired_not_drained() {
    let d = DrainSync::new();

    d.add_sent(0, 1, 86);
    d.set_retired(0, 1, 86);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());

    d.reset();

    d.add_sent(0, 1, 100);
    d.set_retired(0, 1, 86);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "delta 0 against sent 100 must not be drained"
    );
    d.set_retired(0, 1, 136);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "delta 50 against sent 100 must not be drained"
    );
    d.set_retired(0, 1, 186);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "delta 100 against sent 100 must be drained"
    );
}

#[test]
fn multi_axis_all_must_drain() {
    let d = DrainSync::new();
    d.add_sent(0, 0, 17474);
    d.add_sent(1, 2, 86);

    d.set_retired(0, 0, 17474);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "axis (1,2) still pending"
    );
    d.set_retired(1, 2, 86);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "all axes drained"
    );
}
