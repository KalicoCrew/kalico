use std::env;

fn main() {
    println!("cargo:rerun-if-env-changed=ECRT_LIB_DIR");
    println!("cargo:rerun-if-env-changed=SOEM_LIB_DIR");

    // Native linking to libecrt/SOEM only happens under the `hw` feature. Without
    // it (the default), the crate is pure Rust — so `cargo test`/`cargo build`
    // run the scale/wire/curves unit tests on any machine without the C libs.
    if env::var_os("CARGO_FEATURE_HW").is_none() {
        return;
    }

    let lib_dir = env::var("ECRT_LIB_DIR")
        .unwrap_or_else(|_| format!("{}/../../bench", env!("CARGO_MANIFEST_DIR")));
    println!("cargo:rustc-link-search=native={lib_dir}");
    println!("cargo:rustc-link-lib=static=ecrt");

    if let Ok(soem_dir) = env::var("SOEM_LIB_DIR") {
        println!("cargo:rustc-link-search=native={soem_dir}");
    }
    println!("cargo:rustc-link-lib=static=soem");

    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=rt");
    println!("cargo:rustc-link-lib=m");

    println!("cargo:rerun-if-changed=../../bench/libecrt.c");
    println!("cargo:rerun-if-changed=../../bench/libecrt.h");
    println!("cargo:rerun-if-changed=../../bench/Makefile");
    println!("cargo:rerun-if-changed=../../bench/libecrt.a");
}
