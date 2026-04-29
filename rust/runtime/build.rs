// build.rs — register `cfg(loom)` so the `unexpected_cfgs` lint doesn't
// fire on `#![cfg(loom)]` markers in our loom-only test files.
//
// Step-6 plan Phase 1 Task 1.4 introduces `RUSTFLAGS="--cfg loom"` test
// runs against `tests/loom_*.rs`; the regular host clippy/check build
// doesn't set the cfg, so without this hint the rustc lint warns on the
// attribute. Workspace `[lints]` inheritance prevents per-crate
// `[lints.rust]` overrides, so the registration goes here.

fn main() {
    println!("cargo::rustc-check-cfg=cfg(loom)");
}
