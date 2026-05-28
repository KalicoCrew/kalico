use super::*;

#[test]
fn parses_single_int_enum() {
    let json = r#"{"ADC_TEMPERATURE": 254}"#;
    let table: IndexMap<String, EnumValue> = serde_json::from_str(json).unwrap();
    match table.get("ADC_TEMPERATURE") {
        Some(EnumValue::Single(254)) => {}
        other => panic!("expected Single(254), got {:?}", other),
    }
}

#[test]
fn parses_range_enum() {
    let json = r#"{"PA0": [0, 16]}"#;
    let table: IndexMap<String, EnumValue> = serde_json::from_str(json).unwrap();
    match table.get("PA0") {
        Some(EnumValue::Range {
            start: 0,
            count: 16,
        }) => {}
        other => panic!("expected Range {{0, 16}}, got {:?}", other),
    }
}

#[test]
fn parses_negative_msgids() {
    let json = r#"{
        "commands": {"kalico_load_curve x": -7},
        "responses": {},
        "output": {},
        "enumerations": {},
        "config": {},
        "version": "test",
        "app": "kalico"
    }"#;
    let dict: DataDictionary = serde_json::from_str(json).unwrap();
    assert_eq!(*dict.commands.get("kalico_load_curve x").unwrap(), -7);
}

#[test]
fn enumerations_preserve_insertion_order() {
    let json = r#"{
        "commands": {}, "responses": {}, "output": {},
        "enumerations": {
            "pin": {"PA0": 0},
            "step_pin": {"X_step": 5}
        },
        "config": {}, "version": "v", "app": "kalico"
    }"#;
    let dict: DataDictionary = serde_json::from_str(json).unwrap();
    let order: Vec<&String> = dict.enumerations.keys().collect();
    assert_eq!(
        order,
        vec![&"pin".to_string(), &"step_pin".to_string()],
        "IndexMap must preserve JSON insertion order"
    );
}
