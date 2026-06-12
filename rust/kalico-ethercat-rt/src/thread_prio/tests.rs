use super::parse_cpu_list;

#[test]
fn parses_single_range() {
    assert_eq!(parse_cpu_list("2-3"), vec![2, 3]);
}

#[test]
fn parses_mixed_singles_and_ranges() {
    assert_eq!(parse_cpu_list("1,3-5,7"), vec![1, 3, 4, 5, 7]);
}

#[test]
fn empty_string_yields_no_cpus() {
    assert_eq!(parse_cpu_list(""), Vec::<usize>::new());
}

#[test]
fn garbage_is_ignored() {
    assert_eq!(parse_cpu_list("x,2,?-3"), vec![2]);
}
