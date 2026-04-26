//! Run with `cargo run -p nurbs-c-api --bin gen-headers --features host`.
//! Regenerates `nurbs-c-api/include/kalico_nurbs.h` from the cbindgen config.
//! Must produce no diff in CI.

use std::path::PathBuf;

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR set by cargo");
    let crate_dir = PathBuf::from(crate_dir);
    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml"))
        .expect("cbindgen.toml should be parseable");
    let output_path = crate_dir.join("include").join("kalico_nurbs.h");

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("cbindgen generation must succeed")
        .write_to_file(&output_path);

    println!("wrote header to {}", output_path.display());
}
