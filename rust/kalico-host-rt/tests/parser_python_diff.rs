//! Phase-0 Python differential test. Spec §4.13.
//! Gated behind the `python-diff-test` Cargo feature.

#![cfg(feature = "python-diff-test")]

use std::path::PathBuf;
use std::process::Command;

fn repo_root() -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.pop();    // rust/kalico-host-rt -> rust
    p.pop();    // rust -> repo root
    p
}

fn run_python(script: &str) -> Vec<u8> {
    let py = Command::new("python3")
        .arg("-c")
        .arg(script)
        .current_dir(repo_root())
        .output()
        .expect("python3 must be available for python-diff-test");
    if !py.status.success() {
        panic!("python3 failed: stderr={}", String::from_utf8_lossy(&py.stderr));
    }
    py.stdout
}

#[test]
fn diff_kalico_clock_sync_request_encode() {
    let dict_path = repo_root().join("out/klipper.dict");
    let blob = std::fs::read(&dict_path).expect("out/klipper.dict must exist; run `make`");
    let parser_input = std::str::from_utf8(&blob).expect("klipper.dict is JSON");
    let dict: kalico_host_rt::host_io::parser::DataDictionary =
        serde_json::from_str(parser_input).unwrap();
    let rust_parser = kalico_host_rt::host_io::parser::MsgProtoParser::from_dictionary(dict).unwrap();

    let cmd = "kalico_clock_sync_request request_id=42 host_send_time_lo=0 host_send_time_hi=0";
    let rust_bytes = rust_parser.encode(cmd).unwrap();

    let py_script = format!(r#"
import sys
sys.path.insert(0, ".")
from klippy import msgproto
with open("out/klipper.dict", "rb") as f:
    blob = f.read()
parser = msgproto.MessageParser()
parser.process_identify(blob, decompress=False)
encoded = parser.create_command({cmd:?})
sys.stdout.buffer.write(bytes(encoded))
"#, cmd = cmd);
    let py_bytes = run_python(&py_script);

    assert_eq!(rust_bytes, py_bytes,
        "encode mismatch: rust={:02x?} python={:02x?}", rust_bytes, py_bytes);
}
