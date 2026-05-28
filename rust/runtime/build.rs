//! Build script for the runtime crate.
//!
//! Three responsibilities:
//!
//! 1. Register `cfg(loom)` so the `unexpected_cfgs` lint doesn't fire on
//!    `#![cfg(loom)]` markers in loom-only test files. Step-6 plan Phase 1
//!    Task 1.4 runs `RUSTFLAGS="--cfg loom"` against `tests/loom_*.rs`; the
//!    regular host clippy/check build doesn't set the cfg, so without this
//!    hint rustc warns on the attribute. Workspace `[lints]` inheritance
//!    prevents per-crate `[lints.rust]` overrides, so the registration goes
//!    here.
//!
//! 2. Emit sizing constants that vary per target MCU build. Reads two env
//!    vars exported by Klipper's Makefile (which sources them from the
//!    matching `CONFIG_RUNTIME_*` Kconfig values). Defaults match the H7
//!    `large` profile so host-only / sim builds (which don't go through the
//!    Klipper Makefile) still compile. **For bare-metal MCU builds
//!    (`CARGO_CFG_TARGET_OS == "none"`) a missing or empty env var is a
//!    hard build error** — a silent default would desync Rust's
//!    `RT_STORAGE_SIZE` / `PIECE_RING_SIZE` from the C-declared
//!    `rt_storage[]` / piece-ring buffers and cause memory corruption at
//!    runtime (the exact failure mode observed with stale `.config` on F4:
//!    C buffer = 73728, Rust constant silently fell back to 122880).
//!
//!    Spec: docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md §4.3.
//!    `RT_STORAGE_SIZE` is the byte ceiling for the C-declared `rt_storage`
//!    buffer (replaces the prior Rust-side `RT_CELL` with `#[link_section]`).
//!    See `docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md`.
//!
//!    Curve-pool sizing constants (`CURVE_POOL_N`, `MAX_PIECES_PER_CURVE`) were
//!    removed in the 2026-05-28 dead-code purge; the runtime uses the piece-ring
//!    model exclusively. NURBS sizing constants were removed earlier (2026-05-20).
//!
//! 3. Pre-compute an identity-sinusoid LUT for phase stepping (Step 10).
//!    Writes `phase_lut_table.rs` into `OUT_DIR` with a `pub const LUT_ENTRIES`
//!    array of (i16, i16). Included by `rust/runtime/src/phase_lut.rs`.
//!    `f32::sin`/`cos` are not `const`, so table generation must happen here.

use std::env;
use std::fs;
use std::io::Write;
use std::path::PathBuf;

/// Resolve a build-env var.
///
/// - When the env var is set and non-empty, its value is returned regardless
///   of target.
/// - When the env var is missing or empty **and `is_mcu` is `false`** (host /
///   sim build), the supplied `default` is returned — these callers do not go
///   through Klipper's Makefile and the default is intentional.
/// - When the env var is missing or empty **and `is_mcu` is `true`** (bare-
///   metal `thumbv*-none-*` target), the build panics with an actionable
///   message.  A silent default would desync this Rust constant from the
///   C-declared buffer size and cause runtime memory corruption (stale
///   `.config` F4 incident: C buffer = 73728, Rust silently used 122880).
fn lookup(name: &str, default: &str, is_mcu: bool) -> String {
    println!("cargo:rerun-if-env-changed={name}");
    match env::var(name) {
        Ok(s) if !s.is_empty() => s,
        _ => {
            if is_mcu {
                panic!(
                    "\n\
                     build.rs: `{name}` is missing or empty for a bare-metal MCU build.\n\
                     This variable must be exported by Klipper's Makefile from the\n\
                     `CONFIG_RUNTIME_*` Kconfig value matching the active `.config`.\n\
                     A missing value silently desyncs Rust's compile-time constant from\n\
                     the C-declared `rt_storage[]` / piece-ring buffer size, causing\n\
                     memory corruption at runtime (observed on F4 with stale `.config`:\n\
                     C buffer = 73728 bytes, Rust constant fell back to 122880 bytes).\n\
                     Fix: ensure `make` is run from the Klipper tree with a current\n\
                     `.config` so the KALICO_RUNTIME_* vars are in the environment\n\
                     when Cargo is invoked.\n"
                );
            }
            default.to_string()
        }
    }
}

fn main() {
    println!("cargo::rustc-check-cfg=cfg(loom)");
    println!("cargo:rerun-if-changed=build.rs");

    // Cargo sets CARGO_CFG_TARGET_OS to "none" for bare-metal thumbv*-none-*
    // targets.  For host and MACH_LINUX sim builds it is "linux", "macos",
    // etc.  Missing means the toolchain is very old; treat it as host-like.
    let is_mcu = env::var("CARGO_CFG_TARGET_OS")
        .map(|v| v == "none")
        .unwrap_or(false);

    let rss = lookup("KALICO_RUNTIME_STORAGE_SIZE", "122880", is_mcu);
    // PIECE_RING_SIZE: total bytes for piece ring storage.
    // Default 63488 on H7; override via KALICO_RUNTIME_PIECE_RING_SIZE.
    let prs = lookup("KALICO_RUNTIME_PIECE_RING_SIZE", "63488", is_mcu);

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));

    let sizing_body = format!(
        "// Auto-generated by runtime/build.rs — do not edit.\n\
         pub const RT_STORAGE_SIZE: usize = {rss};\n\
         /// Total byte budget for all per-axis piece rings. Each entry is 32 bytes.\n\
         pub const PIECE_RING_SIZE: usize = {prs};\n\
         /// Maximum number of piece entries across all axes combined.\n\
         pub const TOTAL_RING_PIECES: usize = {prs} / 32;\n"
    );
    fs::write(out_dir.join("sizing.rs"), sizing_body).expect("write sizing.rs");

    gen_phase_lut(&out_dir);
}

fn gen_phase_lut(out_dir: &std::path::Path) {
    const MOTOR_PERIOD: usize = 1024;
    const CURRENT_AMPLITUDE: i16 = 248;

    let dest = out_dir.join("phase_lut_table.rs");
    let mut f = fs::File::create(&dest).expect("create phase_lut_table.rs");

    writeln!(f, "// Auto-generated by build.rs — do not edit.").unwrap();

    // Legacy `(i_a = sin, i_b = cos)` table consumed by `modulator.rs`.
    writeln!(f, "pub const LUT_ENTRIES: [(i16, i16); {MOTOR_PERIOD}] = [").unwrap();
    for i in 0..MOTOR_PERIOD {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (MOTOR_PERIOD as f64);
        let i_a = (f64::from(CURRENT_AMPLITUDE) * angle.sin()).round() as i16;
        let i_b = (f64::from(CURRENT_AMPLITUDE) * angle.cos()).round() as i16;
        // Clamp to the i16 representable range (the multiply can produce 248.0
        // exactly at the anchors; rounding cannot escape the bound, but be
        // explicit so future amplitude changes don't silently overflow).
        let i_a = i_a.clamp(-CURRENT_AMPLITUDE, CURRENT_AMPLITUDE);
        let i_b = i_b.clamp(-CURRENT_AMPLITUDE, CURRENT_AMPLITUDE);
        writeln!(f, "    ({i_a}, {i_b}),").unwrap();
    }
    writeln!(f, "];").unwrap();

    // Plan-canonical `(coil_A = cos, coil_B = sin)` table consumed by
    // `crate::tick::dispatch_axis`. Identical amplitude and period as
    // `LUT_ENTRIES`; the pair order is swapped so the anchors match the
    // spec convention `PHASE_LUT[0] == (248, 0)` and
    // `PHASE_LUT[256] == (0, 248)`.
    writeln!(f, "pub const PHASE_LUT: [(i16, i16); PHASE_LUT_SIZE] = [").unwrap();
    for i in 0..MOTOR_PERIOD {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (MOTOR_PERIOD as f64);
        let cos = (f64::from(CURRENT_AMPLITUDE) * angle.cos()).round() as i16;
        let sin = (f64::from(CURRENT_AMPLITUDE) * angle.sin()).round() as i16;
        let cos = cos.clamp(-CURRENT_AMPLITUDE, CURRENT_AMPLITUDE);
        let sin = sin.clamp(-CURRENT_AMPLITUDE, CURRENT_AMPLITUDE);
        writeln!(f, "    ({cos}, {sin}),").unwrap();
    }
    writeln!(f, "];").unwrap();
}
