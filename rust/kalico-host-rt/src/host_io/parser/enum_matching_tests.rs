use super::*;

#[test]
fn matches_exact_name() {
    let mut enums = IndexMap::new();
    let mut pin_table = IndexMap::new();
    pin_table.insert("PA0".to_string(), EnumValue::Single(0));
    enums.insert("pin".to_string(), pin_table);

    let fields = vec![("pin".to_string(), FieldType::U32)];
    let wrapped = apply_enumeration_wrapping(fields, &enums);
    match &wrapped[0].1 {
        WrappedField::Enumerated { enum_name, .. } => assert_eq!(enum_name, "pin"),
        other => panic!("expected Enumerated, got {:?}", other),
    }
}

#[test]
fn matches_underscore_suffix() {
    let mut enums = IndexMap::new();
    let mut pin_table = IndexMap::new();
    pin_table.insert("PA0".to_string(), EnumValue::Single(0));
    enums.insert("pin".to_string(), pin_table);

    let fields = vec![("step_pin".to_string(), FieldType::U32)];
    let wrapped = apply_enumeration_wrapping(fields, &enums);
    match &wrapped[0].1 {
        WrappedField::Enumerated { enum_name, .. } => assert_eq!(enum_name, "pin"),
        other => panic!(
            "expected Enumerated (matched via _pin suffix), got {:?}",
            other
        ),
    }
}

#[test]
fn first_match_in_insertion_order_wins() {
    let mut enums = IndexMap::new();

    let mut pin_table = IndexMap::new();
    pin_table.insert("PA0".to_string(), EnumValue::Single(0));
    enums.insert("pin".to_string(), pin_table);

    let mut step_pin_table = IndexMap::new();
    step_pin_table.insert("X_step".to_string(), EnumValue::Single(99));
    enums.insert("step_pin".to_string(), step_pin_table);

    let fields = vec![("step_pin".to_string(), FieldType::U32)];
    let wrapped = apply_enumeration_wrapping(fields, &enums);
    match &wrapped[0].1 {
        WrappedField::Enumerated { enum_name, .. } => {
            assert_eq!(
                enum_name, "pin",
                "first-match (pin via _pin suffix) wins, NOT longest-suffix (step_pin)"
            );
        }
        other => panic!("expected Enumerated, got {:?}", other),
    }
}
