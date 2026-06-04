use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;
use serde::Deserialize;

use crate::transport::MessageParams;
use crate::transport::MessageValue;

#[derive(Debug, Deserialize)]
pub struct DataDictionary {
    pub commands: IndexMap<String, i32>,
    pub responses: IndexMap<String, i32>,
    #[serde(default)]
    pub output: IndexMap<String, i32>,
    #[serde(default)]
    pub enumerations: IndexMap<String, IndexMap<String, EnumValue>>,
    pub config: serde_json::Value,
    #[serde(default)]
    pub version: String,
    #[serde(default)]
    pub app: String,
    #[serde(default)]
    pub build_versions: Option<String>,
    #[serde(default)]
    pub license: Option<String>,
}

#[derive(Debug)]
pub enum EnumValue {
    Single(i32),
    Range { start: i32, count: i32 },
}

impl<'de> serde::Deserialize<'de> for EnumValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de::Error;
        let value = serde_json::Value::deserialize(deserializer)?;
        if let Some(i) = value.as_i64() {
            return Ok(EnumValue::Single(i as i32));
        }
        if let Some(arr) = value.as_array() {
            if arr.len() == 2 {
                let start = arr[0]
                    .as_i64()
                    .ok_or_else(|| D::Error::custom("EnumValue range[0] not int"))?
                    as i32;
                let count = arr[1]
                    .as_i64()
                    .ok_or_else(|| D::Error::custom("EnumValue range[1] not int"))?
                    as i32;
                return Ok(EnumValue::Range { start, count });
            }
        }
        Err(D::Error::custom(
            "EnumValue: expected int or [start, count]",
        ))
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
            "%u" => Ok(Self::U32),
            "%i" => Ok(Self::I32),
            "%hu" => Ok(Self::U16),
            "%hi" => Ok(Self::I16),
            "%c" => Ok(Self::Byte),
            "%s" => Ok(Self::String),
            "%.*s" => Ok(Self::ProgmemBuffer),
            "%*s" => Ok(Self::Buffer),
            other => Err(ParseError::UnknownFormatCode(other.to_string())),
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

pub fn extract_free_form_field_types(s: &str) -> Result<Vec<FieldType>, ParseError> {
    let bytes = s.as_bytes();
    let mut codes = Vec::new();
    let mut i = 0;
    // Longer prefixes first: %hu before %h, %.*s/%*s before %s.
    const CANDIDATES: &[&str] = &["%hu", "%hi", "%.*s", "%*s", "%u", "%i", "%c", "%s"];
    while i < bytes.len() {
        if bytes[i] != b'%' {
            i += 1;
            continue;
        }
        if i + 1 < bytes.len() && bytes[i + 1] == b'%' {
            i += 2;
            continue;
        }
        let rest = &s[i..];
        let mut matched = None;
        for cand in CANDIDATES {
            if rest.starts_with(cand) {
                matched = Some(*cand);
                break;
            }
        }
        match matched {
            Some(cand) => {
                codes.push(FieldType::from_format_code(cand)?);
                i += cand.len();
            }
            None => {
                let next = rest
                    .chars()
                    .nth(1)
                    .map(|c| format!("%{c}"))
                    .unwrap_or_else(|| "%".into());
                return Err(ParseError::UnknownFormatCode(next));
            }
        }
    }
    Ok(codes)
}

#[derive(Debug, Clone)]
pub struct EnumTable {
    pub by_name: HashMap<String, i32>,
    pub by_int: HashMap<i32, String>,
}

impl EnumTable {
    pub fn from_dict(d: &IndexMap<String, EnumValue>) -> Self {
        let mut by_name = HashMap::new();
        let mut by_int = HashMap::new();
        for (name, value) in d {
            match value {
                EnumValue::Single(i) => {
                    by_name.insert(name.clone(), *i);
                    by_int.insert(*i, name.clone());
                }
                EnumValue::Range { start, count } => {
                    let root: String = name
                        .trim_end_matches(|c: char| c.is_ascii_digit())
                        .to_string();
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
                if field_name == *enum_name || field_name.ends_with(&format!("_{}", enum_name)) {
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
    pub(crate) by_msgid: HashMap<i32, DispatchSpec>,
    pub(crate) by_command_name: IndexMap<String, OutboundSpec>,
    pub(crate) enumerations: IndexMap<String, EnumTable>,
    pub(crate) static_strings: HashMap<i32, String>,
    pub(crate) config: serde_json::Value,
    pub(crate) version: String,
}

#[derive(Debug)]
pub enum DispatchSpec {
    Response(ResponseSpec),
    Output(OutputSpec),
}

#[derive(Debug)]
pub struct ResponseSpec {
    pub name: String,
    pub fields: Vec<(String, WrappedField)>,
}

#[derive(Debug)]
pub struct OutputSpec {
    pub format: String,
    pub fields: Vec<WrappedField>,
    pub field_names: Vec<String>,
    pub is_free_form: bool,
}

#[derive(Debug)]
pub struct OutboundSpec {
    pub msgid: i32,
    pub fields: Vec<(String, WrappedField)>,
}

impl MsgProtoParser {
    #[cfg(any(test, feature = "test-harness"))]
    pub fn new_empty() -> Self {
        use indexmap::IndexMap;
        Self {
            by_msgid: std::collections::HashMap::new(),
            by_command_name: IndexMap::new(),
            enumerations: IndexMap::new(),
            static_strings: std::collections::HashMap::new(),
            config: serde_json::json!({}),
            version: "empty".into(),
        }
    }

    pub fn from_dictionary(dict: DataDictionary) -> Result<Self, ParseError> {
        let mut seen_msgids: HashSet<i32> = HashSet::new();
        let mut seen_formats: HashSet<String> = HashSet::new();
        let mut seen_msgnames: HashSet<String> = HashSet::new();

        for (format, msgid) in dict
            .commands
            .iter()
            .chain(dict.responses.iter())
            .chain(dict.output.iter())
        {
            if !seen_msgids.insert(*msgid) {
                return Err(ParseError::DuplicateMsgid(*msgid));
            }
            if !seen_formats.insert(format.clone()) {
                return Err(ParseError::DuplicateFormatString(format.clone()));
            }
        }

        for format in dict.commands.keys().chain(dict.responses.keys()) {
            let name = format.split_whitespace().next().unwrap_or("").to_string();
            if !seen_msgnames.insert(name.clone()) {
                return Err(ParseError::DuplicateMessageName(name));
            }
        }

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
                OutboundSpec {
                    msgid: *msgid,
                    fields: wrapped.clone(),
                },
            );
            by_msgid.insert(
                *msgid,
                DispatchSpec::Response(ResponseSpec {
                    name,
                    fields: wrapped,
                }),
            );
        }

        for (format, msgid) in &dict.responses {
            let (name, fields) = parse_format_string(format)?;
            let wrapped = apply_enumeration_wrapping(fields, &dict.enumerations);
            by_msgid.insert(
                *msgid,
                DispatchSpec::Response(ResponseSpec {
                    name,
                    fields: wrapped,
                }),
            );
        }

        for (format, msgid) in &dict.output {
            let spec = match parse_format_string(format) {
                Ok((_name, named_fields)) => {
                    let wrapped = apply_enumeration_wrapping(named_fields, &dict.enumerations);
                    let (field_names, positional_fields): (Vec<String>, Vec<WrappedField>) =
                        wrapped.into_iter().unzip();
                    OutputSpec {
                        format: format.clone(),
                        fields: positional_fields,
                        field_names,
                        is_free_form: false,
                    }
                }
                Err(ParseError::MalformedField) | Err(ParseError::UnknownFormatCode(_)) => {
                    let codes = extract_free_form_field_types(format)?;
                    let positional_fields: Vec<WrappedField> =
                        codes.into_iter().map(WrappedField::Plain).collect();
                    OutputSpec {
                        format: format.clone(),
                        fields: positional_fields,
                        field_names: Vec::new(),
                        is_free_form: true,
                    }
                }
                Err(other) => return Err(other),
            };
            by_msgid.insert(*msgid, DispatchSpec::Output(spec));
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
    let first = *buf.first().ok_or(ParseError::BadVlq)?;
    let mut value = i64::from(first & 0x7F);
    if (first & 0x60) == 0x60 {
        value |= -0x20_i64;
    }
    if (first & 0x80) == 0 {
        return Ok((value, 1));
    }
    let mut consumed = 1;
    for &b in buf[1..].iter().take(4) {
        consumed += 1;
        value = (value << 7) | i64::from(b & 0x7F);
        if (b & 0x80) == 0 {
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
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    {
        if value >= 0x0C00_0000 || value < -0x0400_0000 {
            out.push(((value >> 28) as u8 & 0x7F) | 0x80);
        }
        if value >= 0x0018_0000 || value < -0x0008_0000 {
            out.push(((value >> 21) as u8 & 0x7F) | 0x80);
        }
        if value >= 0x0000_3000 || value < -0x0000_1000 {
            out.push(((value >> 14) as u8 & 0x7F) | 0x80);
        }
        if value >= 0x60 || value < -0x20 {
            out.push(((value >> 7) as u8 & 0x7F) | 0x80);
        }
        out.push(value as u8 & 0x7F);
    }
    Ok(())
}

pub fn encode_field_int(out: &mut Vec<u8>, ty: FieldType, value: i64) -> Result<(), ParseError> {
    match ty {
        FieldType::U32 | FieldType::I32 | FieldType::U16 | FieldType::I16 | FieldType::Byte => {
            encode_vlq(out, value)
        }
        _ => Err(ParseError::MalformedField),
    }
}

pub fn encode_field_value<'a>(
    out: &mut Vec<u8>,
    ty: FieldType,
    value: &FieldValue<'a>,
) -> Result<(), ParseError> {
    match (ty, value) {
        (FieldType::U32, FieldValue::U32(v)) => encode_vlq(out, i64::from(*v)),
        (FieldType::I32, FieldValue::I32(v)) => encode_vlq(out, i64::from(*v)),
        (FieldType::U16, FieldValue::U16(v)) => encode_vlq(out, i64::from(*v)),
        (FieldType::I16, FieldValue::I16(v)) => encode_vlq(out, i64::from(*v)),
        (FieldType::Byte, FieldValue::Byte(v)) => encode_vlq(out, i64::from(*v)),
        (FieldType::String, FieldValue::String(s)) => {
            let bytes = s.as_bytes();
            if bytes.len() > u8::MAX as usize {
                return Err(ParseError::OutOfRange {
                    value: bytes.len() as i64,
                    range: "string len 0..=255",
                });
            }
            out.push(bytes.len() as u8);
            out.extend_from_slice(bytes);
            Ok(())
        }
        (FieldType::Buffer, FieldValue::Buffer(b))
        | (FieldType::ProgmemBuffer, FieldValue::Buffer(b)) => {
            if b.len() > u8::MAX as usize {
                return Err(ParseError::OutOfRange {
                    value: b.len() as i64,
                    range: "buffer len 0..=255",
                });
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
        let byte =
            u8::from_str_radix(pair_str, 16).map_err(|_| ParseError::BadHex(s.to_string()))?;
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
            FieldType::Byte | FieldType::U16 | FieldType::U32 | FieldType::I16 | FieldType::I32 => {
                let v: i64 = value_str.parse().map_err(|_| ParseError::MalformedField)?;
                range_check(*ty, v)?;
                let v_for_vlq = match ty {
                    FieldType::U32 => i64::from((v as u64 & 0xFFFF_FFFF) as u32),
                    FieldType::I32 => i64::from(v as i32),
                    _ => v,
                };
                encode_vlq(out, v_for_vlq)
            }
            FieldType::String => {
                let bytes = value_str.as_bytes();
                if bytes.len() > u8::MAX as usize {
                    return Err(ParseError::OutOfRange {
                        value: bytes.len() as i64,
                        range: "string len 0..=255",
                    });
                }
                out.push(bytes.len() as u8);
                out.extend_from_slice(bytes);
                Ok(())
            }
            FieldType::Buffer | FieldType::ProgmemBuffer => {
                let bytes = parse_hex_buffer(value_str)?;
                if bytes.len() > u8::MAX as usize {
                    return Err(ParseError::OutOfRange {
                        value: bytes.len() as i64,
                        range: "buffer len 0..=255",
                    });
                }
                out.push(bytes.len() as u8);
                out.extend_from_slice(&bytes);
                Ok(())
            }
        },
        WrappedField::Enumerated { inner, enum_name } => {
            let table = enums
                .get(enum_name)
                .ok_or_else(|| ParseError::UnknownEnumName(enum_name.clone()))?;
            let int = table
                .by_name
                .get(value_str)
                .ok_or_else(|| ParseError::UnknownEnumValue {
                    enum_name: enum_name.clone(),
                    value: value_str.to_string(),
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
                let table = enums
                    .get(enum_name)
                    .ok_or_else(|| ParseError::UnknownEnumName(enum_name.clone()))?;
                let int = table
                    .by_name
                    .get(*name)
                    .ok_or_else(|| ParseError::UnknownEnumValue {
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
        // invert_step=-1 (SF_SINGLE_SCHED) uses signed byte range; accept [-128..=255].
        FieldType::Byte => (-128..=255).contains(&v),
        FieldType::U16 => (0..=65535).contains(&v),
        FieldType::I16 => (-32768..=32767).contains(&v),
        FieldType::U32 | FieldType::I32 => return Ok(()),
        _ => return Ok(()),
    };
    if !in_range {
        return Err(ParseError::OutOfRange {
            value: v,
            range: "FieldType range",
        });
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
        let spec = self
            .by_command_name
            .get(name)
            .ok_or_else(|| ParseError::UnknownCommand(name.to_string()))?;

        let mut provided: HashMap<&str, &str> = HashMap::new();
        for token in tokens {
            let (k, v) = token.split_once('=').ok_or(ParseError::MalformedArg)?;
            provided.insert(k, v);
        }

        let mut payload = Vec::new();
        encode_vlq(&mut payload, i64::from(spec.msgid))?;
        for (field_name, wrapped) in &spec.fields {
            let value_str = provided
                .get(field_name.as_str())
                .ok_or_else(|| ParseError::MissingField(field_name.clone()))?;
            encode_field_str(&mut payload, wrapped, value_str, &self.enumerations)?;
        }
        Ok(payload)
    }

    pub fn encode_typed<'a>(
        &self,
        name: &str,
        args: &[(&str, FieldValue<'a>)],
    ) -> Result<Vec<u8>, ParseError> {
        let spec = self
            .by_command_name
            .get(name)
            .ok_or_else(|| ParseError::UnknownCommand(name.to_string()))?;
        let provided: HashMap<&str, &FieldValue> = args.iter().map(|(k, v)| (*k, v)).collect();

        let mut payload = Vec::new();
        encode_vlq(&mut payload, i64::from(spec.msgid))?;
        for (field_name, wrapped) in &spec.fields {
            let value = provided
                .get(field_name.as_str())
                .ok_or_else(|| ParseError::MissingField(field_name.clone()))?;
            encode_wrapped_field_typed(&mut payload, wrapped, value, &self.enumerations)?;
        }
        Ok(payload)
    }

    pub fn decode_wrapped_field(
        &self,
        body: &[u8],
        wrapped: &WrappedField,
    ) -> Result<(MessageValue, usize), ParseError> {
        match wrapped {
            WrappedField::Plain(ty) => decode_field_plain(body, *ty),
            WrappedField::Enumerated { inner, enum_name } => {
                let (raw, consumed) = decode_field_plain(body, *inner)?;
                let int = match raw {
                    MessageValue::U32(v) => v as i32,
                    MessageValue::I32(v) => v,
                    _ => return Err(ParseError::MalformedField),
                };
                let table = self
                    .enumerations
                    .get(enum_name)
                    .ok_or_else(|| ParseError::UnknownEnumName(enum_name.clone()))?;
                let resolved = table
                    .by_int
                    .get(&int)
                    .cloned()
                    .unwrap_or_else(|| format!("?{}", int));
                Ok((MessageValue::String(resolved), consumed))
            }
        }
    }

    pub fn decode_body(&self, body: &[u8]) -> Result<(String, MessageParams), ParseError> {
        let (msgid_signed, n) = decode_vlq(body)?;
        let msgid = msgid_signed as i32;
        let dispatch = self
            .by_msgid
            .get(&msgid)
            .ok_or(ParseError::UnknownMsgid(msgid))?;
        match dispatch {
            DispatchSpec::Response(spec) => {
                let params = self.decode_response(&body[n..], &spec.fields)?;
                Ok((spec.name.clone(), params))
            }
            DispatchSpec::Output(spec) => {
                let (name, params) = self.decode_output(&body[n..], spec)?;
                Ok((name, params))
            }
        }
    }

    pub fn decode_response(
        &self,
        mut body: &[u8],
        fields: &[(String, WrappedField)],
    ) -> Result<MessageParams, ParseError> {
        let mut params = MessageParams::new();
        for (field_name, wrapped) in fields {
            let (value, consumed) = self.decode_wrapped_field(body, wrapped)?;
            params.insert(field_name, value);
            body = &body[consumed..];
        }
        Ok(params)
    }

    pub fn decode_output(
        &self,
        body: &[u8],
        spec: &OutputSpec,
    ) -> Result<(String, MessageParams), ParseError> {
        if spec.is_free_form {
            let mut cur = body;
            let mut values: Vec<MessageValue> = Vec::with_capacity(spec.fields.len());
            for wrapped in &spec.fields {
                let (value, consumed) = self.decode_wrapped_field(cur, wrapped)?;
                values.push(value);
                cur = &cur[consumed..];
            }
            let formatted = format_output_message(&spec.format, &values);
            let mut params = MessageParams::new();
            params.insert("#msg", MessageValue::String(formatted));
            params.insert("#format", MessageValue::String(spec.format.clone()));
            return Ok(("#output".to_string(), params));
        }

        let mut params = MessageParams::new();
        let mut cur = body;
        for (field_name, wrapped) in spec.field_names.iter().zip(spec.fields.iter()) {
            let (value, consumed) = self.decode_wrapped_field(cur, wrapped)?;
            params.insert(field_name, value);
            cur = &cur[consumed..];
        }
        let name = spec
            .format
            .split_whitespace()
            .next()
            .unwrap_or("#output")
            .to_string();
        Ok((name, params))
    }

    pub fn decode(&self, packet: &[u8]) -> Result<DecodedFrame, ParseError> {
        if packet.len() < MESSAGE_MIN {
            return Err(ParseError::ShortFrame);
        }
        let body = &packet[MESSAGE_HEADER_SIZE..packet.len() - MESSAGE_TRAILER_SIZE];
        if body.is_empty() {
            return Err(ParseError::EmptyBody);
        }

        let (msgid_signed, n) = decode_vlq(body)?;
        let msgid = msgid_signed as i32;
        let dispatch = self
            .by_msgid
            .get(&msgid)
            .ok_or(ParseError::UnknownMsgid(msgid))?;

        match dispatch {
            DispatchSpec::Response(spec) => {
                let params = self.decode_response(&body[n..], &spec.fields)?;
                Ok(DecodedFrame::Response {
                    name: spec.name.clone(),
                    params,
                })
            }
            DispatchSpec::Output(spec) => {
                let (name, params) = self.decode_output(&body[n..], spec)?;
                Ok(DecodedFrame::Output { name, params })
            }
        }
    }

    pub(crate) fn decode_output_canonical(
        &self,
        packet: &[u8],
    ) -> Result<(String, MessageParams), ParseError> {
        if packet.len() < MESSAGE_MIN {
            return Err(ParseError::ShortFrame);
        }
        let body = &packet[MESSAGE_HEADER_SIZE..packet.len() - MESSAGE_TRAILER_SIZE];
        let (msgid_signed, n) = decode_vlq(body)?;
        let msgid = msgid_signed as i32;
        let dispatch = self
            .by_msgid
            .get(&msgid)
            .ok_or(ParseError::UnknownMsgid(msgid))?;

        let DispatchSpec::Output(spec) = dispatch else {
            return Err(ParseError::MalformedField);
        };

        let mut cur = &body[n..];
        let mut values: Vec<MessageValue> = Vec::new();
        for wrapped in spec.fields.iter() {
            let (v, consumed) = self.decode_wrapped_field(cur, wrapped)?;
            values.push(v);
            cur = &cur[consumed..];
        }

        let formatted = format_output_message(&spec.format, &values);
        let mut params = MessageParams::new();
        params.insert("#msg", MessageValue::String(formatted));
        Ok(("#output".to_string(), params))
    }
}

const MESSAGE_MIN: usize = 5;
const MESSAGE_HEADER_SIZE: usize = 2;
const MESSAGE_TRAILER_SIZE: usize = 3;

#[derive(Debug)]
pub enum DecodedFrame {
    Response { name: String, params: MessageParams },
    Output { name: String, params: MessageParams },
}

pub fn decode_field_plain(body: &[u8], ty: FieldType) -> Result<(MessageValue, usize), ParseError> {
    match ty {
        FieldType::U32 | FieldType::U16 | FieldType::Byte => {
            let (raw_i64, n) = decode_vlq(body)?;
            Ok((MessageValue::U32(raw_i64 as u32), n))
        }
        FieldType::I32 | FieldType::I16 => {
            let (raw_i64, n) = decode_vlq(body)?;
            Ok((MessageValue::I32(raw_i64 as i32), n))
        }
        FieldType::String | FieldType::Buffer | FieldType::ProgmemBuffer => {
            if body.is_empty() {
                return Err(ParseError::Truncated);
            }
            let len = body[0] as usize;
            if body.len() < 1 + len {
                return Err(ParseError::Truncated);
            }
            Ok((MessageValue::Bytes(body[1..=len].to_vec()), 1 + len))
        }
    }
}

fn python_repr_bytes(bytes: &[u8]) -> String {
    let mut out = String::from("b'");
    for &b in bytes {
        if b == b'\\' || b == b'\'' {
            out.push('\\');
            out.push(b as char);
        } else if (0x20..=0x7E).contains(&b) {
            out.push(b as char);
        } else {
            out.push_str(&format!("\\x{:02x}", b));
        }
    }
    out.push('\'');
    out
}

pub fn format_output_message(format: &str, values: &[MessageValue]) -> String {
    let mut out = String::new();
    let mut iter = format.chars().peekable();
    let mut value_idx = 0;
    while let Some(ch) = iter.next() {
        if ch != '%' {
            out.push(ch);
            continue;
        }
        let mut code = String::from("%");
        while let Some(&c) = iter.peek() {
            code.push(c);
            iter.next();
            if matches!(c, 'u' | 'i' | 'c' | 's') {
                break;
            }
        }
        if value_idx >= values.len() {
            out.push_str(&code);
            continue;
        }
        let v = &values[value_idx];
        value_idx += 1;
        match v {
            MessageValue::U32(n) => out.push_str(&n.to_string()),
            MessageValue::I32(n) => out.push_str(&n.to_string()),
            MessageValue::Bytes(b) => out.push_str(&python_repr_bytes(b)),
            MessageValue::String(s) => out.push_str(&format!("{:?}", s)),
            _ => out.push_str(&code),
        }
    }
    out
}

#[cfg(test)]
mod from_dictionary_tests;

#[cfg(test)]
mod data_dictionary_tests;

#[cfg(test)]
mod format_string_tests;

#[cfg(test)]
mod enum_table_tests;

#[cfg(test)]
mod enum_matching_tests;

#[cfg(test)]
mod vlq_tests;

#[cfg(test)]
mod encode_field_tests;

#[cfg(test)]
mod encode_method_tests;

#[cfg(test)]
mod decode_tests;
