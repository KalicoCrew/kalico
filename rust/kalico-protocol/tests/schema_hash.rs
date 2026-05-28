//! Schema-hash integration tests.
//!
//! - **Determinism**: hashing the published `SCHEMA_CANONICAL` text twice
//!   yields the same result, and matches `SCHEMA_HASH`.
//! - **Change detection**: re-canonicalize a slightly-modified schema (one
//!   field type changed) using the same canonicalization rules as
//!   `build.rs` and assert the resulting hash differs from the published
//!   one. This is what stops silent wire-format breakage from sneaking past
//!   the `schema_hash` gate.

use sha2::{Digest, Sha256};

include!("../schema_def.rs");

fn sha256(s: &str) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(s.as_bytes());
    h.finalize().into()
}

#[test]
fn schema_hash_is_deterministic_and_matches_published_constant() {
    let text = canonicalize_schema(SCHEMA_MESSAGES);
    let h1 = sha256(&text);
    let h2 = sha256(&text);
    assert_eq!(h1, h2, "SHA-256 must be deterministic");
    assert_eq!(
        h1,
        kalico_protocol::SCHEMA_HASH,
        "test-side hash must match build.rs-emitted SCHEMA_HASH"
    );
    // SCHEMA_CANONICAL exposes the exact text the build hashed; ensure the
    // canonicalize_schema fn we re-use here produces identical output.
    assert_eq!(text, kalico_protocol::SCHEMA_CANONICAL);
}

#[test]
fn schema_hash_changes_when_a_field_type_changes() {
    // Take the real schema and mutate one field's type. The result must
    // hash differently. This is the "what if I quietly changed a field"
    // canary: if this test ever passes when the field type is the same,
    // the canonicalization is broken.
    let mut mutated_fields: Vec<SchemaField> = SCHEMA_MESSAGES[0].fields.to_vec();
    // Change `slot:u16` to `slot:u32` on LoadCurve — wire-incompatible.
    mutated_fields[0] = SchemaField {
        name: "slot",
        ty: "u32",
    };
    let mutated_msg = SchemaMessage {
        type_tag: SCHEMA_MESSAGES[0].type_tag,
        name: SCHEMA_MESSAGES[0].name,
        version: SCHEMA_MESSAGES[0].version,
        channel: SCHEMA_MESSAGES[0].channel,
        fields: Box::leak(mutated_fields.into_boxed_slice()),
    };
    let mut messages: Vec<SchemaMessage> = SCHEMA_MESSAGES.to_vec();
    messages[0] = mutated_msg;

    let mutated_hash = sha256(&canonicalize_schema(&messages));
    assert_ne!(
        mutated_hash,
        kalico_protocol::SCHEMA_HASH,
        "a field-type change must produce a different schema_hash"
    );
}

#[test]
fn schema_hash_changes_when_a_field_is_added() {
    let mut extra_fields: Vec<SchemaField> = SCHEMA_MESSAGES[0].fields.to_vec();
    extra_fields.push(SchemaField {
        name: "new_field",
        ty: "u32",
    });
    let mutated_msg = SchemaMessage {
        type_tag: SCHEMA_MESSAGES[0].type_tag,
        name: SCHEMA_MESSAGES[0].name,
        version: SCHEMA_MESSAGES[0].version,
        channel: SCHEMA_MESSAGES[0].channel,
        fields: Box::leak(extra_fields.into_boxed_slice()),
    };
    let mut messages: Vec<SchemaMessage> = SCHEMA_MESSAGES.to_vec();
    messages[0] = mutated_msg;

    let mutated_hash = sha256(&canonicalize_schema(&messages));
    assert_ne!(mutated_hash, kalico_protocol::SCHEMA_HASH);
}

#[test]
fn schema_hash_changes_when_a_version_bumps() {
    let mut messages: Vec<SchemaMessage> = SCHEMA_MESSAGES.to_vec();
    messages[0].version = SCHEMA_MESSAGES[0].version + 1;
    let mutated_hash = sha256(&canonicalize_schema(&messages));
    assert_ne!(mutated_hash, kalico_protocol::SCHEMA_HASH);
}
