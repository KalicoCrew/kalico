//! Production MsgProtoParser. Spec §4.

use indexmap::IndexMap;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct DataDictionary {
    pub commands:     IndexMap<String, i32>,
    pub responses:    IndexMap<String, i32>,
    pub output:       IndexMap<String, i32>,
    pub enumerations: IndexMap<String, IndexMap<String, EnumValue>>,
    pub config:       serde_json::Value,
    pub version:      String,
    pub app:          String,
    #[serde(default)]
    pub build_versions: Option<String>,
    #[serde(default)]
    pub license:      Option<String>,
}

#[derive(Debug)]
pub enum EnumValue {
    Single(i32),
    Range { start: i32, count: i32 },
}

impl<'de> serde::Deserialize<'de> for EnumValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where D: serde::Deserializer<'de>,
    {
        use serde::de::Error;
        let value = serde_json::Value::deserialize(deserializer)?;
        if let Some(i) = value.as_i64() {
            return Ok(EnumValue::Single(i as i32));
        }
        if let Some(arr) = value.as_array() {
            if arr.len() == 2 {
                let start = arr[0].as_i64().ok_or_else(|| D::Error::custom("EnumValue range[0] not int"))? as i32;
                let count = arr[1].as_i64().ok_or_else(|| D::Error::custom("EnumValue range[1] not int"))? as i32;
                return Ok(EnumValue::Range { start, count });
            }
        }
        Err(D::Error::custom("EnumValue: expected int or [start, count]"))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldType {
    U32,           // %u
    I32,           // %i
    U16,           // %hu
    I16,           // %hi
    Byte,          // %c
    String,        // %s
    ProgmemBuffer, // %.*s
    Buffer,        // %*s
}

#[derive(Debug)]
pub enum ParseError {
    UnknownFormatCode(String),
    EmptyFormat,
    MalformedField,
}

impl FieldType {
    pub fn from_format_code(code: &str) -> Result<Self, ParseError> {
        match code {
            "%u"   => Ok(Self::U32),
            "%i"   => Ok(Self::I32),
            "%hu"  => Ok(Self::U16),
            "%hi"  => Ok(Self::I16),
            "%c"   => Ok(Self::Byte),
            "%s"   => Ok(Self::String),
            "%.*s" => Ok(Self::ProgmemBuffer),
            "%*s"  => Ok(Self::Buffer),
            other  => Err(ParseError::UnknownFormatCode(other.to_string())),
        }
    }
}

#[cfg(test)]
mod data_dictionary_tests {
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
            Some(EnumValue::Range { start: 0, count: 16 }) => {}
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
        assert_eq!(order, vec![&"pin".to_string(), &"step_pin".to_string()],
                   "IndexMap must preserve JSON insertion order");
    }
}
