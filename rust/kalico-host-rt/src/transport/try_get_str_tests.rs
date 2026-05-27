use super::*;

#[test]
fn returns_string_directly() {
    let mut p = MessageParams::new();
    p.insert("name", MessageValue::String("PA0".into()));
    assert_eq!(p.try_get_str("name"), Some("PA0"));
}

#[test]
fn falls_back_to_utf8_bytes() {
    let mut p = MessageParams::new();
    p.insert("data", MessageValue::Bytes(b"hello".to_vec()));
    assert_eq!(p.try_get_str("data"), Some("hello"));
}

#[test]
fn returns_none_for_int_field() {
    let mut p = MessageParams::new();
    p.insert("count", MessageValue::U32(42));
    assert_eq!(p.try_get_str("count"), None);
}
