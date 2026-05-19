//! Production MsgProtoParser. Spec §4.

use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;
use serde::Deserialize;

use crate::transport::MessageParams;
use crate::transport::MessageValue;

#[derive(Debug, Deserialize)]
pub struct DataDictionary {
    pub commands:     IndexMap<String, i32>,
    pub responses:    IndexMap<String, i32>,
    // `output` is emitted by Kalico-runtime firmware and most modern Klipper
    // builds, but may be absent on older / minimal firmware. Default to empty.
    #[serde(default)]
    pub output:       IndexMap<String, i32>,
    #[serde(default)]
    pub enumerations: IndexMap<String, IndexMap<String, EnumValue>>,
    pub config:       serde_json::Value,
    // `version` and `app` are present on Klipper / Kalico firmware but absent
    // on third-party MCUs that ship a Klipper-compatible identify dict (e.g.
    // Beacon probe). They are stored for diagnostic logging only — nothing
    // downstream reads them — so default to empty strings.
    #[serde(default)]
    pub version:      String,
    #[serde(default)]
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

/// Scan a format string for `%`-codes positionally — used for free-form
/// `output(...)` formats where individual fields are not `name=%type`-tagged
/// (e.g. `output("debug %u %s", x, y)`). Returns the list of field types in
/// declaration order. `%%` is treated as a literal percent sign and skipped.
///
/// Per spec §4.7, free-form output formats are decoded positionally and the
/// decoded values are interpolated through the format string for the canonical
/// `("#output", {"#msg": formatted})` shape. The first whitespace-separated
/// token is still treated as the message-name leader for routing purposes
/// (matches the runtime convention used by `decode_output`).
pub fn extract_free_form_field_types(s: &str) -> Result<Vec<FieldType>, ParseError> {
    let bytes = s.as_bytes();
    let mut codes = Vec::new();
    let mut i = 0;
    // Order matters: longer prefixes first so `%hu` doesn't match as `%h` + `u`,
    // and `%.*s` / `%*s` resolve before `%s`.
    const CANDIDATES: &[&str] = &["%hu", "%hi", "%.*s", "%*s", "%u", "%i", "%c", "%s"];
    while i < bytes.len() {
        if bytes[i] != b'%' {
            i += 1;
            continue;
        }
        if i + 1 < bytes.len() && bytes[i + 1] == b'%' {
            i += 2; // literal `%%`
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
                // Unknown `%X` — surface so we don't silently drop fields.
                let next = rest.chars().nth(1).map(|c| format!("%{c}")).unwrap_or_else(|| "%".into());
                return Err(ParseError::UnknownFormatCode(next));
            }
        }
    }
    Ok(codes)
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
    /// True when the format string lacks `name=%type` recovery for at least
    /// one field (free-form `output(...)` per spec §4.7). Decode falls back to
    /// `("#output", {"#msg": formatted_string})` instead of structured fields.
    pub is_free_form: bool,
}

#[derive(Debug)]
pub struct OutboundSpec {
    pub msgid:  i32,
    pub fields: Vec<(String, WrappedField)>,
}

impl MsgProtoParser {
    /// Construct a parser with an empty data dictionary — useful for tests that
    /// only exercise the wire-protocol layer and never encode/decode messages.
    #[cfg(any(test, feature = "test-harness"))]
    pub fn new_empty() -> Self {
        use indexmap::IndexMap;
        Self {
            by_msgid:       std::collections::HashMap::new(),
            by_command_name: IndexMap::new(),
            enumerations:   IndexMap::new(),
            static_strings: std::collections::HashMap::new(),
            config:         serde_json::json!({}),
            version:        "empty".into(),
        }
    }

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
            // Spec §4.7: prefer `name=%type` recovery so subscribers see
            // structured `MessageParams`. If recovery fails — a free-form
            // `output(...)` such as `output("debug %u trace")` — fall back to
            // positional `%`-code extraction; decode emits the canonical
            // `("#output", {"#msg": formatted})` shape downstream.
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
    // Klipper's signed VLQ encoding (matches klippy/msgproto.py PT_uint32.parse):
    // The sign is encoded in bits [6:5] of the first byte. If those bits are
    // both set (0x60 mask == 0x60), the 7-bit value is sign-extended by OR-ing
    // with -0x20, making the accumulator negative before any continuation bytes
    // are shifted in.
    let first = *buf.first().ok_or(ParseError::BadVlq)?;
    let mut value = i64::from(first & 0x7F);
    if (first & 0x60) == 0x60 {
        value |= -0x20_i64; // sign-extend the initial 7-bit chunk
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
    // Mirror klippy/msgproto.py PT_uint32.encode exactly. Each threshold
    // determines whether an extra 7-bit group is needed. Arithmetic right
    // shifts on i64 propagate the sign bit, matching Python's behaviour.
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
                // Clipper-protocol clock fields (clock=%u) carry 64-bit
                // host-side absolute clocks; the firmware reads 32 bits and
                // truncates. Mirror klippy/msgproto.py PT_uint32 / PT_int32:
                // for U32/I32 fields, mask down to 32 bits before VLQ
                // encoding so encode_vlq's strict [i32::MIN, u32::MAX] range
                // accepts the value.
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
    // Klipper's msgproto allows U32/I32 fields to carry 64-bit clock values
    // and lets the firmware truncate to its native 32-bit register on receipt
    // (encode_vlq already produces a self-delimited VLQ that the firmware
    // decodes and masks). Mirror that: U32/I32 accept any i64. U16/I16/Byte
    // remain strict — those are never used for clocks and a wrong value there
    // is always a real bug.
    let in_range = match ty {
        // Klipper's reference msgproto (klippy/msgproto.py PT_byte) inherits
        // PT_uint32's encoder and allows signed values: the host sends a
        // VLQ-encoded i64, the MCU parses it, and per-handler C code casts
        // args[i] to either `uint8_t` or `int_fast8_t` depending on the
        // intended interpretation. config_stepper's invert_step=-1 from the
        // bridge path (klippy/stepper.py::_build_config_bridge after commit
        // 8649861c9) is the live example — the F4/H7 firmware reads it as
        // int_fast8_t and treats <0 as SF_SINGLE_SCHED.
        // Accept the union of signed [-128..=127] and unsigned [0..=255]
        // byte ranges.
        FieldType::Byte => (-128..=255).contains(&v),
        FieldType::U16  => (0..=65535).contains(&v),
        // PT_int16 is also VLQ-extended in reference msgproto; the host can
        // legitimately send the full -0x8000..=0x7FFF range and the firmware
        // truncates to int16 on read.
        FieldType::I16  => (-32768..=32767).contains(&v),
        FieldType::U32 | FieldType::I32 => return Ok(()),
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

    pub fn decode_wrapped_field(&self, body: &[u8], wrapped: &WrappedField)
        -> Result<(MessageValue, usize), ParseError>
    {
        match wrapped {
            WrappedField::Plain(ty) => decode_field_plain(body, *ty),
            WrappedField::Enumerated { inner, enum_name } => {
                let (raw, consumed) = decode_field_plain(body, *inner)?;
                let int = match raw {
                    MessageValue::U32(v) => v as i32,
                    MessageValue::I32(v) => v,
                    _ => return Err(ParseError::MalformedField),
                };
                let table = self.enumerations.get(enum_name)
                    .ok_or_else(|| ParseError::UnknownEnumName(enum_name.clone()))?;
                let resolved = table.by_int.get(&int)
                    .cloned()
                    .unwrap_or_else(|| format!("?{}", int));
                Ok((MessageValue::String(resolved), consumed))
            }
        }
    }

    /// Decode a raw passthrough body (msgid VLQ + fields) into
    /// `(name, MessageParams)`. Used by `RouterTransport` in the bridge
    /// where only the body bytes from `dispatch_response` are available.
    pub fn decode_body(&self, body: &[u8]) -> Result<(String, MessageParams), ParseError> {
        let (msgid_signed, n) = decode_vlq(body)?;
        let msgid = msgid_signed as i32;
        let dispatch = self.by_msgid.get(&msgid)
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

    pub fn decode_response(&self, mut body: &[u8], fields: &[(String, WrappedField)])
        -> Result<MessageParams, ParseError>
    {
        let mut params = MessageParams::new();
        for (field_name, wrapped) in fields {
            let (value, consumed) = self.decode_wrapped_field(body, wrapped)?;
            params.insert(field_name, value);
            body = &body[consumed..];
        }
        Ok(params)
    }

    pub fn decode_output(&self, body: &[u8], spec: &OutputSpec)
        -> Result<(String, MessageParams), ParseError>
    {
        if spec.is_free_form {
            // Spec §4.7 fallback: positional decode + format-string interpolation,
            // surfaced as the canonical Python `("#output", {"#msg": formatted})`
            // shape. RuntimeEvent::lift routes this to `RuntimeEvent::UnknownOutput`.
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
            // Carry the original format string so RuntimeEvent::lift can
            // propagate it into UnknownOutput.format (spec §4.8). lift has no
            // access to MsgProtoParser, so this is the only path.
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
        let name = spec.format.split_whitespace().next().unwrap_or("#output").to_string();
        Ok((name, params))
    }

    pub fn decode(&self, packet: &[u8]) -> Result<DecodedFrame, ParseError> {
        if packet.len() < MESSAGE_MIN { return Err(ParseError::ShortFrame); }
        let body = &packet[MESSAGE_HEADER_SIZE..packet.len() - MESSAGE_TRAILER_SIZE];
        if body.is_empty() { return Err(ParseError::EmptyBody); }

        let (msgid_signed, n) = decode_vlq(body)?;
        let msgid = msgid_signed as i32;
        let dispatch = self.by_msgid.get(&msgid)
            .ok_or(ParseError::UnknownMsgid(msgid))?;

        match dispatch {
            DispatchSpec::Response(spec) => {
                let params = self.decode_response(&body[n..], &spec.fields)?;
                Ok(DecodedFrame::Response { name: spec.name.clone(), params })
            }
            DispatchSpec::Output(spec) => {
                let (name, params) = self.decode_output(&body[n..], spec)?;
                Ok(DecodedFrame::Output { name, params })
            }
        }
    }

    /// Decodes a packet into the canonical `('#output', {'#msg': formatted})` form, for diagnostic use.
    pub(crate) fn decode_output_canonical(&self, packet: &[u8]) -> Result<(String, MessageParams), ParseError> {
        if packet.len() < MESSAGE_MIN { return Err(ParseError::ShortFrame); }
        let body = &packet[MESSAGE_HEADER_SIZE..packet.len() - MESSAGE_TRAILER_SIZE];
        let (msgid_signed, n) = decode_vlq(body)?;
        let msgid = msgid_signed as i32;
        let dispatch = self.by_msgid.get(&msgid).ok_or(ParseError::UnknownMsgid(msgid))?;

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

/// Per spec §4.7. The §3.6 receive flow branches on this tag.
#[derive(Debug)]
pub enum DecodedFrame {
    Response { name: String, params: MessageParams },
    Output   { name: String, params: MessageParams },
}

/// Decode a single field from `body` according to `ty`.
///
/// Per spec §4.7:
///   - %u/%hu/%c → U32 via (raw_i64 as u32) (matches Python's & 0xFFFFFFFF mask exactly)
///   - %i/%hi → I32 (sign-preserved)
///   - %s/%*s/%.*s → Bytes, length-prefixed (NOT null-terminated)
pub fn decode_field_plain(body: &[u8], ty: FieldType) -> Result<(MessageValue, usize), ParseError> {
    match ty {
        FieldType::U32 | FieldType::U16 | FieldType::Byte => {
            let (raw_i64, n) = decode_vlq(body)?;
            // Mask to u32 to match Python PT_uint32.parse's & 0xFFFFFFFF.
            Ok((MessageValue::U32(raw_i64 as u32), n))
        }
        FieldType::I32 | FieldType::I16 => {
            let (raw_i64, n) = decode_vlq(body)?;
            Ok((MessageValue::I32(raw_i64 as i32), n))
        }
        FieldType::String | FieldType::Buffer | FieldType::ProgmemBuffer => {
            if body.is_empty() { return Err(ParseError::Truncated); }
            let len = body[0] as usize;
            if body.len() < 1 + len { return Err(ParseError::Truncated); }
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

/// Format an output message string by substituting decoded values for
/// printf-style format codes. Mirrors Python's `debugformat % tuple`.
/// %c renders as decimal int (NOT character); %s/%*s/%.*s as repr(bytes)-equivalent.
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
            if matches!(c, 'u' | 'i' | 'c' | 's') { break; }
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

    /// Spec §4.7: free-form `output(...)` formats whose fields aren't
    /// `name=%type`-tagged must accept-and-fall-back, not reject the dict.
    #[test]
    fn accepts_free_form_output_format() {
        let mut d = empty_dict();
        d.output.insert("debug_blob count=%u %s".into(), 7);
        let p = MsgProtoParser::from_dictionary(d).expect("free-form output must parse");
        match p.by_msgid.get(&7) {
            Some(DispatchSpec::Output(spec)) => {
                assert!(spec.is_free_form, "must mark as free-form");
                assert_eq!(spec.fields.len(), 2, "two %-codes recovered positionally");
                assert!(spec.field_names.is_empty());
            }
            other => panic!("expected free-form Output, got {other:?}"),
        }
    }

    #[test]
    fn free_form_output_decodes_to_canonical_msg() {
        let mut d = empty_dict();
        d.output.insert("debug_blob %u %s".into(), 8);
        let parser = MsgProtoParser::from_dictionary(d).unwrap();
        // Body: msgid VLQ + u32 VLQ value 5 + length-prefixed string "hi"
        let mut body = Vec::new();
        encode_vlq(&mut body, 8).unwrap();        // msgid
        encode_vlq(&mut body, 5).unwrap();        // %u value
        body.push(2);                              // %s length prefix
        body.extend_from_slice(b"hi");

        let mut packet = vec![0u8, 0u8];           // 2-byte header (len + dest|seq)
        packet.extend_from_slice(&body);
        packet.extend_from_slice(&[0, 0, 0]);      // 3-byte trailer (CRC + sync)

        let frame = parser.decode(&packet).expect("decode succeeds");
        match frame {
            DecodedFrame::Output { name, params } => {
                assert_eq!(name, "#output", "free-form must surface as #output");
                let msg = params.try_get_str("#msg").unwrap_or("");
                assert!(msg.contains("5"),  "formatted message contains %u value: {msg:?}");
                assert!(msg.contains("hi"), "formatted message contains %s value: {msg:?}");
            }
            other => panic!("expected Output, got {other:?}"),
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
        // Klipper's signed VLQ preserves the original i64 value exactly —
        // including u32::MAX (4294967295) which is distinct from -1 on the
        // wire (5 bytes vs 1 byte). The caller truncates to i32 via `as i32`
        // after decode when the field type requires it (see decode_response).
        for v in [0i64, 1, -1, 100, 100_000, i64::from(i32::MIN), i64::from(u32::MAX)] {
            let mut buf = Vec::new();
            encode_vlq(&mut buf, v).unwrap();
            let (decoded, consumed) = decode_vlq(&buf).unwrap();
            assert_eq!(consumed, buf.len(), "consumed != encoded length for {}", v);
            assert_eq!(decoded, v, "round-trip for {} produced {}", v, decoded);
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
    fn byte_field_accepts_signed_negative() {
        // Klipper's reference msgproto (PT_byte → PT_uint32 VLQ) accepts
        // signed values for %c. The bridge path's config_stepper emits
        // invert_step=-1 (commit 8649861c9); rejecting it here breaks every
        // config_stepper on every bridge-mode MCU. Regression guard.
        use indexmap::IndexMap;
        let enums: IndexMap<String, EnumTable> = IndexMap::new();
        for v in &["-1", "-128", "0", "127", "255"] {
            let mut buf = Vec::new();
            encode_field_str(
                &mut buf,
                &WrappedField::Plain(FieldType::Byte),
                v,
                &enums,
            ).unwrap_or_else(|e| panic!("Byte should accept {v:?}: {e:?}"));
            assert!(!buf.is_empty(), "encoded payload non-empty for {v:?}");
        }
    }

    #[test]
    fn byte_field_still_rejects_truly_out_of_range() {
        use indexmap::IndexMap;
        let enums: IndexMap<String, EnumTable> = IndexMap::new();
        // -129 and 256 are outside the signed+unsigned byte envelope.
        for v in &["-129", "256", "1000", "-1000"] {
            let mut buf = Vec::new();
            let r = encode_field_str(
                &mut buf,
                &WrappedField::Plain(FieldType::Byte),
                v,
                &enums,
            );
            assert!(matches!(r, Err(ParseError::OutOfRange { .. })),
                "Byte should reject {v:?}, got {r:?}");
        }
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

    #[test]
    fn enum_encode_rejects_unknown_name() {
        let mut d = DataDictionary {
            commands: IndexMap::new(), responses: IndexMap::new(), output: IndexMap::new(),
            enumerations: IndexMap::new(), config: serde_json::json!({}),
            version: "v".into(), app: "kalico".into(),
            build_versions: None, license: None,
        };
        d.commands.insert("config_pin pin=%c".into(), 1);
        let mut pin_table = IndexMap::new();
        pin_table.insert("PA0".to_string(), EnumValue::Single(0));
        d.enumerations.insert("pin".to_string(), pin_table);

        let p = MsgProtoParser::from_dictionary(d).unwrap();
        match p.encode("config_pin pin=PZZZ") {
            Err(ParseError::UnknownEnumValue { value, .. }) => assert_eq!(value, "PZZZ"),
            other => panic!("expected UnknownEnumValue, got {:?}", other),
        }
    }
}

#[cfg(test)]
mod decode_tests {
    use super::*;

    fn build_packet(msgid: i32, payload: &[u8]) -> Vec<u8> {
        let mut frame = Vec::new();
        let mut body = Vec::new();
        encode_vlq(&mut body, i64::from(msgid)).unwrap();
        body.extend_from_slice(payload);
        let msglen = MESSAGE_MIN + body.len();
        frame.push(msglen as u8);
        frame.push(0x10);    // dest|seq=0
        frame.extend_from_slice(&body);
        frame.extend_from_slice(&[0, 0]);    // dummy CRC
        frame.push(0x7E);
        frame
    }

    #[test]
    fn decode_response_round_trips() {
        let mut d = DataDictionary {
            commands: IndexMap::new(), responses: IndexMap::new(), output: IndexMap::new(),
            enumerations: IndexMap::new(), config: serde_json::json!({}),
            version: "v".into(), app: "kalico".into(),
            build_versions: None, license: None,
        };
        d.responses.insert("rsp val=%u".into(), 7);
        let parser = MsgProtoParser::from_dictionary(d).unwrap();

        let mut payload = Vec::new();
        encode_vlq(&mut payload, 12345).unwrap();
        let packet = build_packet(7, &payload);

        match parser.decode(&packet).unwrap() {
            DecodedFrame::Response { name, params } => {
                assert_eq!(name, "rsp");
                assert_eq!(params.get_u32("val"), 12345);
            }
            other => panic!("expected Response, got {:?}", other),
        }
    }

    #[test]
    fn decode_unknown_msgid_returns_error() {
        let d = DataDictionary {
            commands: IndexMap::new(), responses: IndexMap::new(), output: IndexMap::new(),
            enumerations: IndexMap::new(), config: serde_json::json!({}),
            version: "v".into(), app: "kalico".into(),
            build_versions: None, license: None,
        };
        let parser = MsgProtoParser::from_dictionary(d).unwrap();
        let packet = build_packet(99, &[]);
        match parser.decode(&packet) {
            Err(ParseError::UnknownMsgid(99)) => {}
            other => panic!("expected UnknownMsgid(99), got {:?}", other),
        }
    }

    #[test]
    fn decode_output_recovers_field_names() {
        let mut d = DataDictionary {
            commands: IndexMap::new(), responses: IndexMap::new(), output: IndexMap::new(),
            enumerations: IndexMap::new(), config: serde_json::json!({}),
            version: "v".into(), app: "kalico".into(),
            build_versions: None, license: None,
        };
        d.output.insert("kalico_credit_freed retired_through_segment_id=%u free_slots=%c".into(), 50);
        let p = MsgProtoParser::from_dictionary(d).unwrap();

        let mut payload = Vec::new();
        encode_vlq(&mut payload, 42).unwrap();
        encode_vlq(&mut payload, 11).unwrap();
        let packet = build_packet(50, &payload);

        match p.decode(&packet).unwrap() {
            DecodedFrame::Output { name, params } => {
                assert_eq!(name, "kalico_credit_freed");
                assert_eq!(params.get_u32("retired_through_segment_id"), 42);
                assert_eq!(params.get_u32("free_slots"), 11);
            }
            other => panic!("expected Output, got {:?}", other),
        }
    }

    #[test]
    fn decode_output_canonical_produces_msg_form() {
        let mut d = DataDictionary {
            commands: IndexMap::new(), responses: IndexMap::new(), output: IndexMap::new(),
            enumerations: IndexMap::new(), config: serde_json::json!({}),
            version: "v".into(), app: "kalico".into(),
            build_versions: None, license: None,
        };
        d.output.insert("kalico_credit_freed retired_through_segment_id=%u free_slots=%c".into(), 50);
        let p = MsgProtoParser::from_dictionary(d).unwrap();

        let mut payload = Vec::new();
        encode_vlq(&mut payload, 42).unwrap();
        encode_vlq(&mut payload, 11).unwrap();
        let packet = build_packet(50, &payload);

        let (name, params) = p.decode_output_canonical(&packet).unwrap();
        assert_eq!(name, "#output");
        let msg = params.try_get_str("#msg").unwrap();
        assert!(msg.contains("kalico_credit_freed"));
        assert!(msg.contains("42"));
        assert!(msg.contains("11"));
    }
}
