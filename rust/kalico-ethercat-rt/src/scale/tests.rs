use super::*;

#[test]
fn origin_maps_to_itself() {
    let m = CountMap::new(3276.8, 14578, 5.0);
    assert_eq!(m.target_counts(5.0), 14578);
}

#[test]
fn positive_delta_rounds_and_adds() {
    let m = CountMap::new(1000.0, 0, 0.0);
    assert_eq!(m.target_counts(1.0004), 1000); // 1000.4 -> 1000
    assert_eq!(m.target_counts(1.0006), 1001); // 1000.6 -> 1001
}

#[test]
fn negative_delta() {
    let m = CountMap::new(1000.0, 5000, 10.0);
    assert_eq!(m.target_counts(9.0), 4000);
}

#[test]
fn origin_no_jump() {
    let counts_per_mm = 3276.8_f64;
    let actual_counts = 14578_i32;
    let origin_mm = 7.5_f64;

    let m = CountMap::new(counts_per_mm, actual_counts, origin_mm);

    assert_eq!(
        m.target_counts(origin_mm),
        actual_counts,
        "origin_mm must map to actual_counts exactly; a mismatch is a startup jump"
    );

    let one_count_fwd = origin_mm + 1.0 / counts_per_mm;
    assert_eq!(
        m.target_counts(one_count_fwd),
        actual_counts + 1,
        "one count forward must be actual_counts + 1"
    );

    let one_count_back = origin_mm - 1.0 / counts_per_mm;
    assert_eq!(
        m.target_counts(one_count_back),
        actual_counts - 1,
        "one count back must be actual_counts - 1"
    );
}
