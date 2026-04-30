//! Production MsgProtoParser. Spec §4.

use std::collections::{HashMap, HashSet};

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
    EmptyCommand,
    EmptyBody,
    MalformedField,
    MalformedArg,
    UnknownCommand(String),
    UnknownMsgid(i32),
    BadMsgid,
    UnknownEnumName(String),
    UnknownEnumValue { enum_name: String, value: String },
    MissingField(String),
    OutOfRange { value: i64, range: &'static str },
    ShortFrame,
    Truncated,
    BadVlq,
    BadHex(String),
    DuplicateMsgid(i32),
    DuplicateFormatString(String),
    DuplicateMessageName(String),
    Zlib(String),
    Json(String),
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl std::error::Error for ParseError {}

impl From<ParseError> for crate::transport::TransportError {
    fn from(e: ParseError) -> Self {
        crate::transport::TransportError::Parse(e.to_string())
    }
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

#[derive(Debug, Clone)]
pub enum WrappedField {
    Plain(FieldType),
    Enumerated { inner: FieldType, enum_name: String },
}

pub fn parse_format_string(s: &str) -> Result<(String, Vec<(String, FieldType)>), ParseError> {
    let mut tokens = s.split_whitespace();
    let name = tokens.next().ok_or(ParseError::EmptyFormat)?.to_string();
    let mut fields = Vec::new();
    for token in tokens {
        let (k, v) = token.split_once('=').ok_or(ParseError::MalformedField)?;
        let ty = FieldType::from_format_code(v)?;
        fields.push((k.to_string(), ty));
    }
    Ok((name, fields))
}

#[derive(Debug, Clone)]
pub struct EnumTable {
    pub by_name: HashMap<String, i32>,
    pub by_int:  HashMap<i32, String>,
}

impl EnumTable {
    pub fn from_dict(d: &IndexMap<String, EnumValue>) -> Self {
        let mut by_name = HashMap::new();
        let mut by_int  = HashMap::new();
        for (name, value) in d {
            match value {
                EnumValue::Single(i) => {
                    by_name.insert(name.clone(), *i);
                    by_int.insert(*i, name.clone());
                }
                EnumValue::Range { start, count } => {
                    let root: String = name.trim_end_matches(|c: char| c.is_ascii_digit()).to_string();
                    let prefix_num: i32 = name[root.len()..].parse().unwrap_or(0);
                    for i in 0..*count {
                        let key = format!("{}{}", root, prefix_num + i);
                        let val = start + i;
                        by_name.insert(key.clone(), val);
                        by_int.insert(val, key);
                    }
                }
            }
        }
        Self { by_name, by_int }
    }
}

pub fn apply_enumeration_wrapping(
    fields: Vec<(String, FieldType)>,
    enums: &IndexMap<String, IndexMap<String, EnumValue>>,
) -> Vec<(String, WrappedField)> {
    fields
        .into_iter()
        .map(|(field_name, ty)| {
            for enum_name in enums.keys() {
                if field_name == *enum_name
                    || field_name.ends_with(&format!("_{}", enum_name))
                {
                    return (
                        field_name,
                        WrappedField::Enumerated {
                            inner: ty,
                            enum_name: enum_name.clone(),
                        },
                    );
                }
            }
            (field_name, WrappedField::Plain(ty))
        })
        .collect()
}

#[derive(Debug)]
pub struct MsgProtoParser {
    pub(crate) by_msgid:        HashMap<i32, DispatchSpec>,
    pub(crate) by_command_name: IndexMap<String, OutboundSpec>,
    pub(crate) enumerations:    IndexMap<String, EnumTable>,
    pub(crate) static_strings:  HashMap<i32, String>,
    pub(crate) config:          serde_json::Value,
    pub(crate) version:         String,
}

#[derive(Debug)]
pub enum DispatchSpec {
    Response(ResponseSpec),
    Output(OutputSpec),
}

#[derive(Debug)]
pub struct ResponseSpec {
    pub name:   String,
    pub fields: Vec<(String, WrappedField)>,
}

#[derive(Debug)]
pub struct OutputSpec {
    pub format:      String,
    pub fields:      Vec<WrappedField>,
    pub field_names: Vec<String>,
}

#[derive(Debug)]
pub struct OutboundSpec {
    pub msgid:  i32,
    pub fields: Vec<(String, WrappedField)>,
}

impl MsgProtoParser {
    pub fn from_dictionary(dict: DataDictionary) -> Result<Self, ParseError> {
        let mut seen_msgids:   HashSet<i32>    = HashSet::new();
        let mut seen_formats:  HashSet<String> = HashSet::new();
        let mut seen_msgnames: HashSet<String> = HashSet::new();

        // Cross-section msgid + format-string collision check.
        for (format, msgid) in dict.commands.iter()
                                .chain(dict.responses.iter())
                                .chain(dict.output.iter()) {
            if !seen_msgids.insert(*msgid) {
                return Err(ParseError::DuplicateMsgid(*msgid));
            }
            if !seen_formats.insert(format.clone()) {
                return Err(ParseError::DuplicateFormatString(format.clone()));
            }
        }

        // Message-name collision check (commands + responses only).
        for format in dict.commands.keys().chain(dict.responses.keys()) {
            let name = format.split_whitespace().next().unwrap_or("").to_string();
            if !seen_msgnames.insert(name.clone()) {
                return Err(ParseError::DuplicateMessageName(name));
            }
        }

        // Build enumerations.
        let mut enumerations: IndexMap<String, EnumTable> = IndexMap::new();
        for (enum_name, table) in &dict.enumerations {
            enumerations.insert(enum_name.clone(), EnumTable::from_dict(table));
        }

        let mut by_msgid: HashMap<i32, DispatchSpec> = HashMap::new();
        let mut by_command_name: IndexMap<String, OutboundSpec> = IndexMap::new();

        for (format, msgid) in &dict.commands {
            let (name, fields) = parse_format_string(format)?;
            let wrapped = apply_enumeration_wrapping(fields, &dict.enumerations);
            by_command_name.insert(
                name.clone(),
                OutboundSpec { msgid: *msgid, fields: wrapped.clone() },
            );
            by_msgid.insert(*msgid, DispatchSpec::Response(ResponseSpec { name, fields: wrapped }));
        }

        for (format, msgid) in &dict.responses {
            let (name, fields) = parse_format_string(format)?;
            let wrapped = apply_enumeration_wrapping(fields, &dict.enumerations);
            by_msgid.insert(*msgid, DispatchSpec::Response(ResponseSpec { name, fields: wrapped }));
        }

        for (format, msgid) in &dict.output {
            let (_name, named_fields) = parse_format_string(format)?;
            let wrapped = apply_enumeration_wrapping(named_fields, &dict.enumerations);
            let (field_names, positional_fields): (Vec<String>, Vec<WrappedField>) =
                wrapped.into_iter().unzip();
            by_msgid.insert(*msgid, DispatchSpec::Output(OutputSpec {
                format: format.clone(),
                fields: positional_fields,
                field_names,
            }));
        }

        let static_strings: HashMap<i32, String> = enumerations
            .get("static_string_id")
            .map(|t| t.by_int.clone())
            .unwrap_or_default();

        Ok(Self {
            by_msgid,
            by_command_name,
            enumerations,
            static_strings,
            config: dict.config,
            version: dict.version,
        })
    }
}

pub fn decode_vlq(buf: &[u8]) -> Result<(i64, usize), ParseError> {
    let mut value: i64 = 0;
    let mut consumed = 0;
    for &b in buf.iter().take(5) {
        consumed += 1;
        value = (value << 7) | i64::from(b & 0x7F);
        if (b & 0x80) == 0 {
            // Sign-extend from 32-bit.
            if (value & (1 << 31)) != 0 {
                value -= 1 << 32;
            }
            return Ok((value, consumed));
        }
    }
    Err(ParseError::BadVlq)
}

pub fn encode_vlq(out: &mut Vec<u8>, value: i64) -> Result<(), ParseError> {
    if !(i64::from(i32::MIN)..=i64::from(u32::MAX)).contains(&value) {
        return Err(ParseError::OutOfRange {
            value,
            range: "[i32::MIN, u32::MAX]",
        });
    }
    let mut v = value;
    if value < 0 {
        v += 1 << 32;
    }
    let mut bytes: [u8; 5] = [0; 5];
    let mut idx = 5usize;
    loop {
        idx -= 1;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        {
            bytes[idx] = (v as u8) & 0x7F;
        }
        v >>= 7;
        if v == 0 || v == -1 { break; }
        if idx == 0 { break; }
    }
    let last = bytes.len() - 1;
    for b in &mut bytes[idx..last] {
        *b |= 0x80;
    }
    out.extend_from_slice(&bytes[idx..]);
    Ok(())
}

pub fn encode_field_int(out: &mut Vec<u8>, ty: FieldType, value: i64) -> Result<(), ParseError> {
    match ty {
        FieldType::U32 | FieldType::I32 |
        FieldType::U16 | FieldType::I16 |
        FieldType::Byte => encode_vlq(out, value),
        _ => Err(ParseError::MalformedField),
    }
}

pub fn encode_field_value<'a>(out: &mut Vec<u8>, ty: FieldType, value: &FieldValue<'a>) -> Result<(), ParseError> {
    match (ty, value) {
        (FieldType::U32,   FieldValue::U32(v))  => encode_vlq(out, i64::from(*v)),
        (FieldType::I32,   FieldValue::I32(v))  => encode_vlq(out, i64::from(*v)),
        (FieldType::U16,   FieldValue::U16(v))  => encode_vlq(out, i64::from(*v)),
        (FieldType::I16,   FieldValue::I16(v))  => encode_vlq(out, i64::from(*v)),
        (FieldType::Byte,  FieldValue::Byte(v)) => encode_vlq(out, i64::from(*v)),
        (FieldType::String, FieldValue::String(s)) => {
            let bytes = s.as_bytes();
            if bytes.len() > u8::MAX as usize {
                return Err(ParseError::OutOfRange { value: bytes.len() as i64, range: "string len 0..=255" });
            }
            out.push(bytes.len() as u8);
            out.extend_from_slice(bytes);
            Ok(())
        }
        (FieldType::Buffer, FieldValue::Buffer(b)) |
        (FieldType::ProgmemBuffer, FieldValue::Buffer(b)) => {
            if b.len() > u8::MAX as usize {
                return Err(ParseError::OutOfRange { value: b.len() as i64, range: "buffer len 0..=255" });
            }
            out.push(b.len() as u8);
            out.extend_from_slice(b);
            Ok(())
        }
        _ => Err(ParseError::MalformedField),
    }
}

pub fn parse_hex_buffer(s: &str) -> Result<Vec<u8>, ParseError> {
    if s.len() % 2 != 0 {
        return Err(ParseError::BadHex(s.to_string()));
    }
    let mut out = Vec::with_capacity(s.len() / 2);
    for pair in s.as_bytes().chunks(2) {
        let pair_str = std::str::from_utf8(pair).map_err(|_| ParseError::BadHex(s.to_string()))?;
        let byte = u8::from_str_radix(pair_str, 16).map_err(|_| ParseError::BadHex(s.to_string()))?;
        out.push(byte);
    }
    Ok(out)
}

pub fn encode_field_str<'a>(
    out: &mut Vec<u8>,
    wrapped: &WrappedField,
    value_str: &str,
    enums: &IndexMap<String, EnumTable>,
) -> Result<(), ParseError> {
    match wrapped {
        WrappedField::Plain(ty) => match ty {
            FieldType::Byte | FieldType::U16 | FieldType::U32 |
            FieldType::I16 | FieldType::I32 => {
                let v: i64 = value_str.parse().map_err(|_| ParseError::MalformedField)?;
                range_check(*ty, v)?;
                encode_vlq(out, v)
            }
            FieldType::String => {
                let bytes = value_str.as_bytes();
                if bytes.len() > u8::MAX as usize {
                    return Err(ParseError::OutOfRange { value: bytes.len() as i64, range: "string len 0..=255" });
                }
                out.push(bytes.len() as u8);
                out.extend_from_slice(bytes);
                Ok(())
            }
            FieldType::Buffer | FieldType::ProgmemBuffer => {
                let bytes = parse_hex_buffer(value_str)?;
                if bytes.len() > u8::MAX as usize {
                    return Err(ParseError::OutOfRange { value: bytes.len() as i64, range: "buffer len 0..=255" });
                }
                out.push(bytes.len() as u8);
                out.extend_from_slice(&bytes);
                Ok(())
            }
        },
        WrappedField::Enumerated { inner, enum_name } => {
            let table = enums.get(enum_name)
                .ok_or_else(|| ParseError::UnknownEnumName(enum_name.clone()))?;
            let int = table.by_name.get(value_str)
                .ok_or_else(|| ParseError::UnknownEnumValue {
                    enum_name: enum_name.clone(),
                    value:     value_str.to_string(),
                })?;
            encode_field_int(out, *inner, i64::from(*int))
        }
    }
}

pub fn encode_wrapped_field_typed<'a>(
    out: &mut Vec<u8>,
    wrapped: &WrappedField,
    value: &FieldValue<'a>,
    enums: &IndexMap<String, EnumTable>,
) -> Result<(), ParseError> {
    match wrapped {
        WrappedField::Plain(ty) => encode_field_value(out, *ty, value),
        WrappedField::Enumerated { inner, enum_name } => match value {
            FieldValue::EnumName(name) => {
                let table = enums.get(enum_name)
                    .ok_or_else(|| ParseError::UnknownEnumName(enum_name.clone()))?;
                let int = table.by_name.get(*name).ok_or_else(|| ParseError::UnknownEnumValue {
                    enum_name: enum_name.clone(),
                    value: (*name).to_string(),
                })?;
                encode_field_int(out, *inner, i64::from(*int))
            }
            FieldValue::EnumIntOverride(i) => encode_field_int(out, *inner, i64::from(*i)),
            _ => Err(ParseError::MalformedField),
        },
    }
}

fn range_check(ty: FieldType, v: i64) -> Result<(), ParseError> {
    let in_range = match ty {
        FieldType::Byte => (0..=255).contains(&v),
        FieldType::U16  => (0..=65535).contains(&v),
        FieldType::I16  => (-32768..=32767).contains(&v),
        FieldType::U32  => (0..=i64::from(u32::MAX)).contains(&v),
        FieldType::I32  => (i64::from(i32::MIN)..=i64::from(i32::MAX)).contains(&v),
        _ => return Ok(()),
    };
    if !in_range {
        return Err(ParseError::OutOfRange { value: v, range: "FieldType range" });
    }
    Ok(())
}

#[derive(Debug, Clone)]
pub enum FieldValue<'a> {
    U32(u32),
    I32(i32),
    U16(u16),
    I16(i16),
    Byte(u8),
    String(&'a str),
    Buffer(&'a [u8]),
    EnumName(&'a str),
    EnumIntOverride(i32),
}

impl MsgProtoParser {
    pub fn encode(&self, cmd: &str) -> Result<Vec<u8>, ParseError> {
        let mut tokens = cmd.split_whitespace();
        let name = tokens.next().ok_or(ParseError::EmptyCommand)?;
        let spec = self.by_command_name.get(name)
            .ok_or_else(|| ParseError::UnknownCommand(name.to_string()))?;

        let mut provided: HashMap<&str, &str> = HashMap::new();
        for token in tokens {
            let (k, v) = token.split_once('=').ok_or(ParseError::MalformedArg)?;
            provided.insert(k, v);
        }

        let mut payload = Vec::new();
        encode_vlq(&mut payload, i64::from(spec.msgid))?;
        for (field_name, wrapped) in &spec.fields {
            let value_str = provided.get(field_name.as_str())
                .ok_or_else(|| ParseError::MissingField(field_name.clone()))?;
            encode_field_str(&mut payload, wrapped, value_str, &self.enumerations)?;
        }
        Ok(payload)
    }

    pub fn encode_typed<'a>(&self, name: &str, args: &[(&str, FieldValue<'a>)]) -> Result<Vec<u8>, ParseError> {
        let spec = self.by_command_name.get(name)
            .ok_or_else(|| ParseError::UnknownCommand(name.to_string()))?;
        let provided: HashMap<&str, &FieldValue> =
            args.iter().map(|(k, v)| (*k, v)).collect();

        let mut payload = Vec::new();
        encode_vlq(&mut payload, i64::from(spec.msgid))?;
        for (field_name, wrapped) in &spec.fields {
            let value = provided.get(field_name.as_str())
                .ok_or_else(|| ParseError::MissingField(field_name.clone()))?;
            encode_wrapped_field_typed(&mut payload, wrapped, value, &self.enumerations)?;
        }
        Ok(payload)
    }
}

#[cfg(test)]
mod from_dictionary_tests {
    use super::*;

    fn empty_dict() -> DataDictionary {
        DataDictionary {
            commands: IndexMap::new(),
            responses: IndexMap::new(),
            output: IndexMap::new(),
            enumerations: IndexMap::new(),
            config: serde_json::json!({}),
            version: "v".into(),
            app: "kalico".into(),
            build_versions: None,
            license: None,
        }
    }

    #[test]
    fn rejects_duplicate_msgid_across_sections() {
        let mut d = empty_dict();
        d.commands.insert("cmd_a arg=%u".into(), 5);
        d.responses.insert("rsp_b arg=%u".into(), 5);
        match MsgProtoParser::from_dictionary(d) {
            Err(ParseError::DuplicateMsgid(5)) => {}
            other => panic!("expected DuplicateMsgid(5), got {:?}", other),
        }
    }

    #[test]
    fn rejects_duplicate_format_string() {
        let mut d = empty_dict();
        d.commands.insert("cmd arg=%u".into(), 5);
        d.responses.insert("cmd arg=%u".into(), 6);
        match MsgProtoParser::from_dictionary(d) {
            Err(ParseError::DuplicateFormatString(_)) => {}
            other => panic!("expected DuplicateFormatString, got {:?}", other),
        }
    }

    #[test]
    fn accepts_disjoint_categories() {
        let mut d = empty_dict();
        d.commands.insert("cmd_a arg=%u".into(), 1);
        d.responses.insert("rsp_a arg=%u".into(), 2);
        d.output.insert("evt_a arg=%u".into(), 3);
        let p = MsgProtoParser::from_dictionary(d).unwrap();
        assert!(matches!(p.by_msgid.get(&1), Some(DispatchSpec::Response(_))));
        assert!(matches!(p.by_msgid.get(&2), Some(DispatchSpec::Response(_))));
        assert!(matches!(p.by_msgid.get(&3), Some(DispatchSpec::Output(_))));
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

#[cfg(test)]
mod format_string_tests {
    use super::*;

    #[test]
    fn parses_kalico_push_segment_format() {
        let s = "kalico_push_segment id=%u x_handle=%u y_handle=%u z_handle=%u e_handle=%u kinematics=%c";
        let (name, fields) = parse_format_string(s).unwrap();
        assert_eq!(name, "kalico_push_segment");
        assert_eq!(fields.len(), 6);
        assert_eq!(fields[0], ("id".to_string(), FieldType::U32));
        assert_eq!(fields[5], ("kinematics".to_string(), FieldType::Byte));
    }

    #[test]
    fn parses_progmem_buffer_in_identify_response() {
        let s = "identify_response offset=%u data=%.*s";
        let (name, fields) = parse_format_string(s).unwrap();
        assert_eq!(name, "identify_response");
        assert_eq!(fields[1].1, FieldType::ProgmemBuffer);
    }

    #[test]
    fn rejects_unknown_format_code_hc() {
        let s = "bad_cmd val=%hc";
        match parse_format_string(s) {
            Err(ParseError::UnknownFormatCode(c)) if c == "%hc" => {}
            other => panic!("expected UnknownFormatCode(%hc), got {:?}", other),
        }
    }
}

#[cfg(test)]
mod enum_table_tests {
    use super::*;

    #[test]
    fn from_dict_expands_range() {
        let mut d = IndexMap::new();
        d.insert("PA0".to_string(), EnumValue::Range { start: 0, count: 16 });
        let table = EnumTable::from_dict(&d);
        assert_eq!(table.by_name.get("PA0"), Some(&0));
        assert_eq!(table.by_name.get("PA15"), Some(&15));
        assert_eq!(table.by_int.get(&15), Some(&"PA15".to_string()));
        assert_eq!(table.by_name.len(), 16);
    }
}

#[cfg(test)]
mod enum_matching_tests {
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
            other => panic!("expected Enumerated (matched via _pin suffix), got {:?}", other),
        }
    }

    #[test]
    fn first_match_in_insertion_order_wins() {
        let mut enums = IndexMap::new();

        let mut pin_table = IndexMap::new();
        pin_table.insert("PA0".to_string(), EnumValue::Single(0));
        enums.insert("pin".to_string(), pin_table);    // declared FIRST

        let mut step_pin_table = IndexMap::new();
        step_pin_table.insert("X_step".to_string(), EnumValue::Single(99));
        enums.insert("step_pin".to_string(), step_pin_table);    // declared SECOND

        let fields = vec![("step_pin".to_string(), FieldType::U32)];
        let wrapped = apply_enumeration_wrapping(fields, &enums);
        match &wrapped[0].1 {
            WrappedField::Enumerated { enum_name, .. } => {
                assert_eq!(enum_name, "pin",
                    "first-match (pin via _pin suffix) wins, NOT longest-suffix (step_pin)");
            }
            other => panic!("expected Enumerated, got {:?}", other),
        }
    }
}

#[cfg(test)]
mod vlq_tests {
    use super::*;

    #[test]
    fn round_trips_representative_values() {
        for v in [0i64, 1, -1, 100, 100_000, i64::from(i32::MIN), i64::from(u32::MAX)] {
            let mut buf = Vec::new();
            encode_vlq(&mut buf, v).unwrap();
            let (decoded, consumed) = decode_vlq(&buf).unwrap();
            assert_eq!(consumed, buf.len(), "consumed != encoded length for {}", v);
            assert_eq!(decoded, v as i32 as i64, "round-trip for {} produced {}", v, decoded);
        }
    }

    #[test]
    fn encode_vlq_rejects_out_of_range() {
        let mut buf = Vec::new();
        match encode_vlq(&mut buf, i64::from(u32::MAX) + 1) {
            Err(ParseError::OutOfRange { .. }) => {}
            other => panic!("expected OutOfRange, got {:?}", other),
        }
        match encode_vlq(&mut buf, i64::from(i32::MIN) - 1) {
            Err(ParseError::OutOfRange { .. }) => {}
            other => panic!("expected OutOfRange, got {:?}", other),
        }
    }
}

#[cfg(test)]
mod encode_field_tests {
    use super::*;

    #[test]
    fn encodes_string_length_prefixed() {
        let mut buf = Vec::new();
        encode_field_value(&mut buf, FieldType::String, &FieldValue::String("hi")).unwrap();
        assert_eq!(buf, vec![2, b'h', b'i']);
    }

    #[test]
    fn encodes_byte_via_vlq() {
        let mut buf = Vec::new();
        encode_field_value(&mut buf, FieldType::Byte, &FieldValue::Byte(0xFF)).unwrap();
        assert_eq!(buf, vec![0x81, 0x7F]);
    }

    #[test]
    fn rejects_string_too_long() {
        let s = "x".repeat(300);
        let mut buf = Vec::new();
        match encode_field_value(&mut buf, FieldType::String, &FieldValue::String(&s)) {
            Err(ParseError::OutOfRange { .. }) => {}
            other => panic!("expected OutOfRange, got {:?}", other),
        }
    }

    #[test]
    fn parse_hex_buffer_round_trips() {
        assert_eq!(parse_hex_buffer("0123abcd").unwrap(), vec![0x01, 0x23, 0xAB, 0xCD]);
        assert_eq!(parse_hex_buffer("").unwrap(), Vec::<u8>::new());
        assert!(matches!(parse_hex_buffer("0z"), Err(ParseError::BadHex(_))));
        assert!(matches!(parse_hex_buffer("1"), Err(ParseError::BadHex(_))));
    }
}

#[cfg(test)]
mod encode_method_tests {
    use super::*;

    fn parser_with_one_command() -> MsgProtoParser {
        let mut d = DataDictionary {
            commands: IndexMap::new(), responses: IndexMap::new(), output: IndexMap::new(),
            enumerations: IndexMap::new(), config: serde_json::json!({}),
            version: "v".into(), app: "kalico".into(),
            build_versions: None, license: None,
        };
        d.commands.insert("ping val=%u".into(), 42);
        MsgProtoParser::from_dictionary(d).unwrap()
    }

    #[test]
    fn string_and_typed_encode_to_same_bytes() {
        let p = parser_with_one_command();
        let bytes_str = p.encode("ping val=100").unwrap();
        let bytes_typed = p.encode_typed("ping", &[("val", FieldValue::U32(100))]).unwrap();
        assert_eq!(bytes_str, bytes_typed);
    }

    #[test]
    fn encode_rejects_unknown_command() {
        let p = parser_with_one_command();
        match p.encode("unknown_cmd") {
            Err(ParseError::UnknownCommand(_)) => {}
            other => panic!("expected UnknownCommand, got {:?}", other),
        }
    }

    #[test]
    fn encode_rejects_missing_field() {
        let p = parser_with_one_command();
        match p.encode("ping") {
            Err(ParseError::MissingField(_)) => {}
            other => panic!("expected MissingField, got {:?}", other),
        }
    }
}
