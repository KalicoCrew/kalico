//! Smoke-checks that header generation succeeds. This test does NOT verify
//! drift on its own: the workspace test build activates BOTH `header-nurbs`
//! and `header-runtime`, so cbindgen here emits the union of symbols and
//! cannot be byte-compared against either single-feature committed header
//! (gen-headers is feature-muxed — see src/bin/gen_headers.rs).
//!
//! The authoritative drift gate is `./scripts/ci.sh cbindgen-drift` (the
//! `rust-cbindgen-drift` CI job): it regenerates each header per-feature via
//! `tools/regen_headers.sh` and `git diff --exit-code`s them against the
//! committed copies. Output is deterministic (`sort_by = "Name"` in both
//! cbindgen configs + cbindgen pinned in Cargo.lock), so exact matching is
//! reliable — the earlier "cross-platform ordering" caveat was incorrect.

#[test]
#[cfg(all(feature = "host", feature = "header-nurbs"))]
fn nurbs_header_generates_successfully() {
    let crate_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let config =
        cbindgen::Config::from_file(crate_dir.join("cbindgen.toml")).expect("cbindgen config");
    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("kalico_nurbs.h regeneration must succeed");
}

#[test]
#[cfg(all(feature = "host", feature = "header-runtime"))]
fn runtime_header_generates_successfully() {
    let crate_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen-runtime.toml"))
        .expect("cbindgen-runtime config");
    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("kalico_runtime.h regeneration must succeed");
}
