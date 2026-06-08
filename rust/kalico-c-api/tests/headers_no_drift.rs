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
