use super::*;

#[test]
fn from_dict_expands_range() {
    let mut d = IndexMap::new();
    d.insert(
        "PA0".to_string(),
        EnumValue::Range {
            start: 0,
            count: 16,
        },
    );
    let table = EnumTable::from_dict(&d);
    assert_eq!(table.by_name.get("PA0"), Some(&0));
    assert_eq!(table.by_name.get("PA15"), Some(&15));
    assert_eq!(table.by_int.get(&15), Some(&"PA15".to_string()));
    assert_eq!(table.by_name.len(), 16);
}
