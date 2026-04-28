//! Generate `kalico_nurbs.h` and `kalico_runtime.h` via cbindgen.
//!
//! Per spec §3.2: cbindgen has no prefix-filter mode, so we run it *twice*
//! against the same staticlib crate with different `cfg` flags via
//! `cargo run --features` to gate which FFI module expands. Each invocation
//! produces exactly one header.
//!
//! ## Invocation
//!
//! The crate's `default = ["host", "header-nurbs", "header-runtime"]` would
//! activate *both* header gates simultaneously, which this binary rejects
//! (cbindgen would emit the union of symbols into whichever header runs
//! last). Always invoke with `--no-default-features` and the desired single
//! header gate:
//!
//! ```text
//! cargo run -p kalico-c-api --bin gen-headers \
//!     --no-default-features --features host,header-nurbs
//! cargo run -p kalico-c-api --bin gen-headers \
//!     --no-default-features --features host,header-runtime
//! ```
//!
//! The wrapper script `tools/regen_headers.sh` runs both invocations.

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR set by cargo");
    let want_nurbs = cfg!(feature = "header-nurbs");
    let want_runtime = cfg!(feature = "header-runtime");
    if want_nurbs && want_runtime {
        eprintln!(
            "error: gen-headers must be invoked with EXACTLY ONE of \
             --features header-nurbs / --features header-runtime so \
             cbindgen sees only the symbols for that header. Pass \
             --no-default-features to disable the crate-default that \
             activates both."
        );
        std::process::exit(1);
    }
    if want_nurbs {
        let cfg = cbindgen::Config::from_file(format!("{crate_dir}/cbindgen.toml"))
            .expect("cbindgen.toml should be parseable");
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(cfg)
            .generate()
            .expect("kalico_nurbs.h generation failed")
            .write_to_file(format!("{crate_dir}/include/kalico_nurbs.h"));
        println!("Generated kalico_nurbs.h");
        return;
    }
    if want_runtime {
        let cfg = cbindgen::Config::from_file(format!("{crate_dir}/cbindgen-runtime.toml"))
            .expect("cbindgen-runtime.toml should be parseable");
        cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(cfg)
            .generate()
            .expect("kalico_runtime.h generation failed")
            .write_to_file(format!("{crate_dir}/include/kalico_runtime.h"));
        println!("Generated kalico_runtime.h");
        return;
    }
    eprintln!("error: invoke with --features header-nurbs OR --features header-runtime");
    std::process::exit(1);
}
