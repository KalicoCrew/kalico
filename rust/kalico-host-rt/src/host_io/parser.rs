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
