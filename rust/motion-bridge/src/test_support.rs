#![cfg(any(test, feature = "test-support"))]

use std::sync::Arc;

use kalico_host_rt::host_io::parser::{DataDictionary, MsgProtoParser};
use kalico_host_rt::host_io::wire;

pub fn build_extension_parser() -> Arc<MsgProtoParser> {
    let dict_json = serde_json::json!({
        "commands": {
            "trsync_set_timeout oid=%c clock=%u": 31
        },
        "responses": {
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u": 30
        },
        "output": {},
        "enumerations": {},
        "config": {},
        "version": "test",
        "app": "test"
    });
    let dict: DataDictionary = serde_json::from_value(dict_json).expect("bad extension dict");
    Arc::new(MsgProtoParser::from_dictionary(dict).expect("extension parser build failed"))
}

pub fn build_trigger_relay_parser() -> Arc<MsgProtoParser> {
    let dict_json = serde_json::json!({
        "commands": {
            "trsync_set_timeout oid=%c clock=%u": 31,
            "trsync_trigger oid=%c reason=%c": 32
        },
        "responses": {
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u": 30
        },
        "output": {},
        "enumerations": {},
        "config": {},
        "version": "test",
        "app": "test"
    });
    let dict: DataDictionary = serde_json::from_value(dict_json).expect("bad trigger relay dict");
    Arc::new(MsgProtoParser::from_dictionary(dict).expect("trigger relay parser build failed"))
}

pub fn build_trsync_state_frame(oid: u8, can_trigger: u8, clock: u32, seq: u8) -> Vec<u8> {
    use kalico_host_rt::host_io::parser::encode_vlq;
    let mut payload = Vec::new();
    encode_vlq(&mut payload, 30).unwrap();
    encode_vlq(&mut payload, i64::from(oid)).unwrap();
    encode_vlq(&mut payload, i64::from(can_trigger)).unwrap();
    encode_vlq(&mut payload, 0i64).unwrap();
    encode_vlq(&mut payload, i64::from(clock)).unwrap();
    wire::build_frame(&payload, seq)
}

pub fn extract_payloads(tx_bytes: Vec<u8>) -> Vec<Vec<u8>> {
    let mut buf = tx_bytes;
    let mut payloads = Vec::new();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
        let msglen = pkt[0] as usize;
        if msglen > wire::MESSAGE_MIN {
            payloads.push(pkt[2..msglen - 3].to_vec());
        }
    }
    payloads
}
