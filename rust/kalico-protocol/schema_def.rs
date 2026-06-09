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
    channel: &'static str, // "control" | "events" | "pieces"
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
        type_tag: 0x0030,
        name: "ConfigureAxes",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "kinematics", ty: "u8" },
            SchemaField { name: "present_mask", ty: "u8" },
            SchemaField { name: "awd_mask", ty: "u8" },
            SchemaField { name: "invert_mask", ty: "u8" },
            SchemaField { name: "steps_per_mm_0", ty: "f32" },
            SchemaField { name: "steps_per_mm_1", ty: "f32" },
            SchemaField { name: "steps_per_mm_2", ty: "f32" },
            SchemaField { name: "steps_per_mm_3", ty: "f32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0031,
        name: "ConfigureAxesResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0040,
        name: "QueryRuntimeCaps",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0041,
        name: "RuntimeCapsResponse",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "total_piece_memory", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0060,
        name: "PushPieces",
        version: 2,
        channel: "pieces",
        fields: &[
            SchemaField { name: "axis_idx", ty: "u8" },
            SchemaField { name: "piece_count", ty: "u8" },
            SchemaField { name: "start_slot", ty: "u16" },
            SchemaField { name: "new_head", ty: "u32" },
            SchemaField { name: "pieces_bytes", ty: "array<u8>" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0061,
        name: "PushPiecesResponse",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "arrival_clock", ty: "u64" },
            SchemaField { name: "front_start_time", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0070,
        name: "SetTorque",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "value", ty: "u8" },
            SchemaField { name: "execute_at_ns", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0071,
        name: "SetTorqueResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0072,
        name: "Stop",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0073,
        name: "StopResponse",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "discard_clock", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0074,
        name: "SetDriveLimits",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "following_error_counts", ty: "u32" },
            SchemaField { name: "max_torque_tenth_pct", ty: "u16" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0075,
        name: "SetDriveLimitsResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0076,
        name: "RestoreDriveLimits",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0077,
        name: "RestoreDriveLimitsResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
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
    SchemaMessage {
        type_tag: 0x0083,
        name: "StatusHeartbeat",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "engine_state", ty: "u8" },
            SchemaField { name: "fault_code", ty: "u16" },
            SchemaField { name: "num_axes", ty: "u8" },
            SchemaField { name: "retired_counts", ty: "array<u32>" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0084,
        name: "McuLog",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "mcu_tick", ty: "u64" },
            SchemaField { name: "level", ty: "u8" },
            SchemaField { name: "subsystem", ty: "u8" },
            SchemaField { name: "event", ty: "u16" },
            SchemaField { name: "code", ty: "u16" },
            SchemaField { name: "seq", ty: "u16" },
            SchemaField { name: "arg0", ty: "u32" },
            SchemaField { name: "arg1", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0085,
        name: "EndstopTrip",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "endstop_id", ty: "u8" },
            SchemaField { name: "trip_clock", ty: "u64" },
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
