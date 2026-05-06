//! Compiles the C smoke `main.c` against the regenerated header + the
//! `libkalico_c_api.a` staticlib. Spec §6.3.
//!
//! The C source carries `_Static_assert(offsetof(TraceSample, ...))` for
//! every ABI-relevant field — this test fails at compile-time if cbindgen
//! drifts away from the Rust struct layout. Link success additionally
//! verifies every `kalico_runtime_*` symbol the header declares is actually
//! exported by the staticlib.
//!
//! Prerequisite: a release-mode build of `libkalico_c_api.a` must exist.
//! `cargo test` does not invoke `cargo build` recursively (and trying to
//! shell out to `cargo build` from inside `cargo test` would deadlock on
//! the build lock), so we check for the artifact and skip with a clear
//! diagnostic if it's missing — CI is expected to sequence build then test.
//!
//! Run manually with:
//! ```bash
//! cargo build -p kalico-c-api --no-default-features \
//!     --features host,header-nurbs,header-runtime --release
//! cargo test -p kalico-c-api --no-default-features \
//!     --features host,header-runtime --test c_smoke_build
//! ```

#[test]
fn c_smoke_compiles_and_links() {
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".to_string());
    let crate_dir = env!("CARGO_MANIFEST_DIR");
    let c_src = format!("{crate_dir}/tests/c_smoke/main.c");
    let header_dir = format!("{crate_dir}/include");
    let target_dir =
        std::env::var("CARGO_TARGET_DIR").unwrap_or_else(|_| format!("{crate_dir}/../target"));
    let static_lib = format!("{target_dir}/release/libkalico_c_api.a");
    let out = format!("{target_dir}/c_smoke_test");

    if !std::path::Path::new(&static_lib).exists() {
        eprintln!(
            "c_smoke skipped: {static_lib} not found.\n\
             Run: cargo build -p kalico-c-api --no-default-features \
             --features host,header-nurbs,header-runtime --release\n\
             then re-run this test."
        );
        return;
    }

    // macOS link line: a bare `-lkalico_c_api` is sufficient. The staticlib's
    // only undefined externals are `runtime_clock_freq` and the `kalico_h7_*`
    // helpers, which `main.c` itself defines as host stubs. libSystem on
    // darwin already provides pthread/dl/m, so we don't add them. On Linux,
    // pthread/dl/m are typically required when a Rust staticlib pulls in
    // `std`; add them via the `target_os` branch below.
    let mut args: Vec<String> = vec![
        c_src.clone(),
        format!("-I{header_dir}"),
        format!("-L{target_dir}/release"),
        "-lkalico_c_api".into(),
    ];

    if cfg!(target_os = "linux") {
        args.push("-lpthread".into());
        args.push("-ldl".into());
        args.push("-lm".into());
    }

    args.push("-o".into());
    args.push(out.clone());

    let status = std::process::Command::new(&cc)
        .args(&args)
        .status()
        .expect("failed to spawn cc");

    assert!(
        status.success(),
        "C smoke build did not compile/link cleanly (cc args: {args:?})"
    );
}
