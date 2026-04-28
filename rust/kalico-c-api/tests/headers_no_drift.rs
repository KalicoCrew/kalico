//! Verifies that `cargo run --bin gen-headers` produces a no-op diff against
//! the committed headers.
//!
//! Each test is gated by the matching `header-*` cargo feature so cbindgen
//! sees only the FFI module for the header it's checking. CI runs both
//! invocations:
//!
//! ```bash
//! cargo test -p kalico-c-api --no-default-features \
//!     --features host,header-nurbs --test headers_no_drift
//! cargo test -p kalico-c-api --no-default-features \
//!     --features host,header-runtime --test headers_no_drift
//! ```

#[test]
#[cfg(all(feature = "host", feature = "header-nurbs"))]
fn nurbs_header_in_repo_matches_generated() {
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
         ./tools/regen_headers.sh"
    );
}

#[test]
#[cfg(all(feature = "host", feature = "header-runtime"))]
fn runtime_header_in_repo_matches_generated() {
    let crate_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let header_path = crate_dir.join("include").join("kalico_runtime.h");
    let committed = std::fs::read_to_string(&header_path).expect("committed header must exist");

    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen-runtime.toml"))
        .expect("cbindgen-runtime config");
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
        "kalico_runtime.h is out of date. Run:\n  \
         ./tools/regen_headers.sh"
    );
}
