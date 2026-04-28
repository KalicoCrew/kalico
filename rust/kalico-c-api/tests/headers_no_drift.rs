//! Verifies that `cargo run --bin gen-headers` produces a no-op diff against
//! the committed header. Run as `cargo test -p kalico-c-api --features host
//! --test headers_no_drift`.

#[test]
#[cfg(feature = "host")]
fn header_in_repo_matches_generated() {
    let crate_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let header_path = crate_dir.join("include").join("kalico_nurbs.h");
    let committed = std::fs::read_to_string(&header_path).expect("committed header must exist");

    let config =
        cbindgen::Config::from_file(crate_dir.join("cbindgen.toml")).expect("cbindgen config");
    let regenerated = cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("regeneration must succeed");

    let mut buf = Vec::new();
    regenerated.write(&mut buf);
    let regenerated_str = String::from_utf8(buf).expect("utf-8 output");

    assert!(
        committed == regenerated_str,
        "kalico_nurbs.h is out of date. Run:\n  \
         cargo run -p kalico-c-api --bin gen-headers --features host"
    );
}
