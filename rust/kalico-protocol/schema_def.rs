// Shared schema definition included by `build.rs` and the integration test
// `tests/schema_hash_change.rs`. Pure data + a pure canonicalization function.
// MUST NOT depend on any other module of this crate (it's `include!`'d, not
// imported as a Rust module).

#[derive(Clone, Copy)]
struct SchemaField {
    name: &'static str,
    ty: &'static str,
}

#[derive(Clone, Copy)]
struct SchemaMessage {
    type_tag: u16,
    name: &'static str,
    version: u8,
    channel: &'static str, // "control" | "events"
    fields: &'static [SchemaField],
}

// Bootstrap messages (Identify=0x0001, IdentifyResponse=0x0002) are
// intentionally excluded — see spec §5. Their byte layout is frozen forever
// and decoupled from `schema_hash`. Including them would make `schema_hash`
// itself depend on the bootstrap layout, which breaks the "fixed forever"
// property of the bootstrap.
//
// Message order: ascending type-tag.
const SCHEMA_MESSAGES: &[SchemaMessage] = &[
    SchemaMessage {
        type_tag: 0x0010,
        name: "LoadCurve",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u16" },
            SchemaField { name: "degree", ty: "u8" },
            SchemaField { name: "n_cps", ty: "u32" },
            SchemaField { name: "n_knots", ty: "u32" },
            SchemaField { name: "cps", ty: "array<f32>" },
            SchemaField { name: "knots", ty: "array<f32>" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0011,
        name: "LoadCurveResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "curve_handle_packed", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0020,
        name: "PushSegment",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "id", ty: "u32" },
            SchemaField { name: "handle_x", ty: "u32" },
            SchemaField { name: "handle_y", ty: "u32" },
            SchemaField { name: "handle_z", ty: "u32" },
            SchemaField { name: "handle_e", ty: "u32" },
            SchemaField { name: "t_start", ty: "u64" },
            SchemaField { name: "t_end", ty: "u64" },
            SchemaField { name: "kinematics", ty: "u8" },
            SchemaField { name: "e_mode", ty: "u8" },
            SchemaField { name: "extrusion_ratio", ty: "f32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0021,
        name: "PushSegmentResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "accepted_segment_id", ty: "u32" },
            SchemaField { name: "credit_epoch", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0080,
        name: "StatusEvent",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "engine_status", ty: "u8" },
            SchemaField { name: "queue_depth", ty: "u8" },
            SchemaField { name: "current_segment_id", ty: "u32" },
            SchemaField { name: "last_fault", ty: "i32" },
            SchemaField { name: "fault_detail", ty: "u32" },
            SchemaField { name: "reset_epoch", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0081,
        name: "CreditFreed",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "retired_through_segment_id", ty: "u32" },
            SchemaField { name: "free_slots", ty: "u8" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0082,
        name: "FaultEvent",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "fault_code", ty: "u16" },
            SchemaField { name: "fault_detail", ty: "u32" },
            SchemaField { name: "segment_id", ty: "u32" },
        ],
    },
];

/// Bootstrap type tags that the C header must define alongside the schema
/// messages. Bootstrap tags are NOT part of `schema_hash`.
#[allow(dead_code)] // used by build.rs; unused by the schema_hash integration test
const BOOTSTRAP_TAGS: &[(u16, &str)] =
    &[(0x0001, "Identify"), (0x0002, "IdentifyResponse")];

/// Canonical text form. One line per message:
///
///     0xTTTT:NAME:vNN:CHAN:[field1:type1,field2:type2,...]\n
///
/// Hex tag is lowercase, zero-padded to 4 hex digits. Version is `v` + decimal
/// (no padding). Bootstrap messages are excluded.
fn canonicalize_schema(messages: &[SchemaMessage]) -> String {
    let mut out = String::new();
    for m in messages {
        out.push_str(&format!("0x{:04x}:{}:v{}:{}:[", m.type_tag, m.name, m.version, m.channel));
        for (i, f) in m.fields.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(f.name);
            out.push(':');
            out.push_str(f.ty);
        }
        out.push_str("]\n");
    }
    out
}
