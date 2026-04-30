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
