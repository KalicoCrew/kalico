use super::*;

#[test]
fn fan_out_indexed_empty() {
    let results: Vec<(usize, i32)> = fan_out_indexed(0, 4, |_| 42);
    assert!(results.is_empty());
}

#[test]
fn fan_out_indexed_identity() {
    let results = fan_out_indexed(5, 2, |i| i * 10);
    assert_eq!(results.len(), 5);
    for (idx, val) in &results {
        assert_eq!(*val, idx * 10);
    }
}

#[test]
fn fan_out_indexed_preserves_order() {
    let results = fan_out_indexed(10, 3, |i| i);
    let indices: Vec<usize> = results.iter().map(|(i, _)| *i).collect();
    assert_eq!(indices, (0..10).collect::<Vec<_>>());
}
