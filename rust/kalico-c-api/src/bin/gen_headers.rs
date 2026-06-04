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

/// Re-sort cbindgen's leading block of fieldless (untagged) C enums into a
/// canonical, name-sorted order.
///
/// cbindgen 0.29.2 emits fieldless enums in filesystem-discovery order (its
/// `dependencies.rs::sort` returns `Ordering::Equal` for two untagged enums
/// under a *stable* sort), so the same source produces byte-different headers
/// on macOS vs. Linux CI and the `cbindgen-drift` gate false-fails. cbindgen
/// groups untagged enums into their own leading layer ("they don't depend on
/// each other or anything else"), so re-sorting that block by name is
/// dependency-safe and makes header generation deterministic on every platform.
///
/// # Algorithm
///
/// 1. Split on `"\n\n"` to recover cbindgen's paragraph-per-item layout.
/// 2. Identify paragraphs that are fieldless C enums via `fieldless_enum_name`.
/// 3. Collect their indices, sort the paragraphs themselves by name, and write
///    the sorted paragraphs back into those same index positions (all other
///    paragraphs are untouched).
/// 4. Re-join with `"\n\n"` — identity transform for everything non-enum.
///
/// # Examples
///
/// ```
/// let src = "preamble\n\nenum Z {\n  V = 0,\n};\ntypedef uint8_t Z;\n\nenum A {\n  W = 0,\n};\ntypedef uint8_t A;\n\ntypedef struct Foo Foo;";
/// let out = canonicalize_untagged_enums(src);
/// assert!(out.find("enum A").unwrap() < out.find("enum Z").unwrap());
/// ```
fn canonicalize_untagged_enums(src: &str) -> String {
    let mut paragraphs: Vec<&str> = src.split("\n\n").collect();

    // Collect (index, name) for every fieldless-enum paragraph.
    let enum_positions: Vec<usize> = (0..paragraphs.len())
        .filter(|&i| fieldless_enum_name(paragraphs[i]).is_some())
        .collect();

    if enum_positions.len() < 2 {
        // Nothing to reorder.
        return paragraphs.join("\n\n");
    }

    // Gather the paragraphs that correspond to fieldless enums, then sort by
    // name.  We need owned strings here because we are about to mutate
    // `paragraphs` in place.
    let mut enum_paragraphs: Vec<&str> = enum_positions.iter().map(|&i| paragraphs[i]).collect();

    enum_paragraphs.sort_by_key(|p| fieldless_enum_name(p).unwrap());

    // Write sorted paragraphs back into the slots they originally occupied.
    for (slot, paragraph) in enum_positions.iter().zip(enum_paragraphs) {
        paragraphs[*slot] = paragraph;
    }

    paragraphs.join("\n\n")
}

/// Return `Some(name)` when `paragraph` is a cbindgen fieldless (untagged) C
/// enum paragraph, or `None` otherwise.
///
/// A fieldless enum paragraph looks like:
///
/// ```text
/// enum SomeName {
///   Variant = 0,
/// };
/// typedef uint8_t SomeName;
/// ```
///
/// The paragraph may optionally be preceded by a doc-comment block
/// (`/** ... */`) with no intervening blank line (cbindgen attaches comments
/// directly to their item in the same paragraph).
///
/// Detection criteria (all must hold):
/// - After skipping any leading `/** ... */` block, the next non-empty line
///   (trimmed) starts with `"enum "` and ends with `" {"`.
/// - The name between `"enum "` and `" {"` consists solely of ASCII
///   alphanumeric or `_` characters.
/// - The paragraph contains a line whose trimmed form starts with `"typedef "`
///   and ends with `" <name>;"` (cbindgen's companion typedef for fieldless
///   enums).
fn fieldless_enum_name(paragraph: &str) -> Option<String> {
    let mut lines = paragraph.lines();

    // Skip a leading doc-comment block if present.
    let first_non_empty = loop {
        match lines.next() {
            None => return None,
            Some(line) => {
                let trimmed = line.trim();
                if trimmed.is_empty() {
                    continue;
                }
                if trimmed.starts_with("/**") || trimmed.starts_with("/*") {
                    // Consume lines through the closing `*/`.
                    loop {
                        match lines.next() {
                            None => return None,
                            Some(comment_line) => {
                                if comment_line.contains("*/") {
                                    break;
                                }
                            }
                        }
                    }
                    continue;
                }
                break trimmed;
            }
        }
    };

    // The next non-empty non-comment line must be `enum <Name> {`.
    if !first_non_empty.starts_with("enum ") || !first_non_empty.ends_with(" {") {
        return None;
    }

    let after_enum = &first_non_empty["enum ".len()..];
    let name = after_enum.strip_suffix(" {")?;

    // Validate: name must be a non-empty C identifier (ASCII alnum or `_`).
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return None;
    }

    // The paragraph must also contain the companion typedef line.
    let expected_typedef = format!("typedef uint8_t {name};");
    let has_typedef = paragraph
        .lines()
        .any(|l| l.trim() == expected_typedef.as_str());

    if has_typedef {
        Some(name.to_owned())
    } else {
        None
    }
}

/// Capture cbindgen's output into a `String`, canonicalize fieldless enum
/// order, and write to `out_path`.
fn write_canonical(bindings: cbindgen::Bindings, out_path: &str) {
    let mut buf: Vec<u8> = Vec::new();
    bindings.write(&mut buf);
    let raw = String::from_utf8(buf).expect("cbindgen output is UTF-8");
    let canonical = canonicalize_untagged_enums(&raw);
    std::fs::write(out_path, canonical).expect("write header");
}

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
        let bindings = cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(cfg)
            .generate()
            .expect("kalico_nurbs.h generation failed");
        write_canonical(bindings, &format!("{crate_dir}/include/kalico_nurbs.h"));
        println!("Generated kalico_nurbs.h");
        return;
    }
    if want_runtime {
        let cfg = cbindgen::Config::from_file(format!("{crate_dir}/cbindgen-runtime.toml"))
            .expect("cbindgen-runtime.toml should be parseable");
        let bindings = cbindgen::Builder::new()
            .with_crate(&crate_dir)
            .with_config(cfg)
            .generate()
            .expect("kalico_runtime.h generation failed");
        write_canonical(bindings, &format!("{crate_dir}/include/kalico_runtime.h"));
        println!("Generated kalico_runtime.h");
        return;
    }
    eprintln!("error: invoke with --features header-nurbs OR --features header-runtime");
    std::process::exit(1);
}

#[cfg(test)]
mod tests {
    use super::{canonicalize_untagged_enums, fieldless_enum_name};

    // Realistic preamble paragraph (copyright + include guard + includes).
    const PREAMBLE: &str = "/*\n* kalico_runtime.h — generated by cbindgen.\n*/\n\n\
        #ifndef KALICO_RUNTIME_H\n#define KALICO_RUNTIME_H\n\n\
        #pragma once\n\n\
        #include <stdint.h>";

    fn source_kind_para() -> &'static str {
        "enum SourceKind {\n  Physical = 0,\n  TmcDiag = 1,\n};\ntypedef uint8_t SourceKind;"
    }

    fn arm_policy_para() -> &'static str {
        "enum ArmPolicy {\n  TripImmediately = 0,\n  WaitForClear = 1,\n};\ntypedef uint8_t ArmPolicy;"
    }

    fn struct_para() -> &'static str {
        "typedef struct Foo Foo;"
    }

    /// Build a full header string from the given ordered paragraphs.
    fn join(paragraphs: &[&str]) -> String {
        paragraphs.join("\n\n")
    }

    // ------------------------------------------------------------------
    // canonicalize_untagged_enums — behavioural tests
    // ------------------------------------------------------------------

    #[test]
    fn canonicalize_reorders_two_fieldless_enums_into_name_order() {
        // SourceKind comes BEFORE ArmPolicy — that is the "wrong" Linux order.
        let src = join(&[
            PREAMBLE,
            source_kind_para(),
            arm_policy_para(),
            struct_para(),
        ]);
        let out = canonicalize_untagged_enums(&src);

        let pos_arm = out.find("enum ArmPolicy").expect("ArmPolicy present");
        let pos_source = out.find("enum SourceKind").expect("SourceKind present");
        assert!(
            pos_arm < pos_source,
            "ArmPolicy must appear before SourceKind after canonicalization"
        );

        // Preamble is unchanged and leads.
        assert!(
            out.starts_with(PREAMBLE),
            "preamble must be unchanged and first"
        );

        // Struct paragraph is unchanged and trails.
        assert!(
            out.ends_with(struct_para()),
            "struct paragraph must be unchanged and last"
        );
    }

    #[test]
    fn canonicalize_is_idempotent() {
        let src = join(&[
            PREAMBLE,
            source_kind_para(),
            arm_policy_para(),
            struct_para(),
        ]);
        let once = canonicalize_untagged_enums(&src);
        let twice = canonicalize_untagged_enums(&once);
        assert_eq!(once, twice, "canonicalize must be idempotent");
    }

    #[test]
    fn canonicalize_is_noop_when_no_fieldless_enums() {
        // A nurbs-style header: only typedef struct / function declarations.
        let src = join(&[
            PREAMBLE,
            "typedef struct KalicoNurbs KalicoNurbs;",
            "void kalico_nurbs_free(struct KalicoNurbs *ptr);",
        ]);
        let out = canonicalize_untagged_enums(&src);
        assert_eq!(
            out, src,
            "output must be byte-identical to input when no fieldless enums"
        );
    }

    #[test]
    fn canonicalize_handles_doc_commented_fieldless_enum() {
        // A paragraph with a leading /** ... */ doc-comment block before `enum X {`.
        let commented_enum = "/**\n * A doc comment.\n */\n\
            enum Zephyr {\n  Alpha = 0,\n};\ntypedef uint8_t Zephyr;";
        let plain_enum = "enum Alpha {\n  Beta = 0,\n};\ntypedef uint8_t Alpha;";

        // Zephyr (Z) before Alpha (A) — wrong order.
        let src = join(&[PREAMBLE, commented_enum, plain_enum, struct_para()]);
        let out = canonicalize_untagged_enums(&src);

        let pos_alpha = out.find("enum Alpha").expect("Alpha present");
        let pos_zephyr = out.find("enum Zephyr").expect("Zephyr present");
        assert!(
            pos_alpha < pos_zephyr,
            "Alpha must appear before Zephyr after canonicalization"
        );

        // The doc comment must still be attached to the Zephyr paragraph
        // (it moved as a unit with the paragraph).
        let zephyr_start = out.find("/**").expect("doc comment present");
        let zephyr_enum = out.find("enum Zephyr").expect("enum Zephyr present");
        assert!(
            zephyr_start < zephyr_enum,
            "doc comment must precede enum Zephyr in output"
        );
    }

    // ------------------------------------------------------------------
    // fieldless_enum_name — unit tests
    // ------------------------------------------------------------------

    #[test]
    fn fieldless_enum_name_returns_some_for_valid_fieldless_enum() {
        let para = "enum ArmPolicy {\n  TripImmediately = 0,\n};\ntypedef uint8_t ArmPolicy;";
        assert_eq!(
            fieldless_enum_name(para),
            Some("ArmPolicy".to_owned()),
            "should detect ArmPolicy as a fieldless enum"
        );
    }

    #[test]
    fn fieldless_enum_name_returns_none_for_typedef_struct() {
        let para = "typedef struct Foo Foo;";
        assert_eq!(
            fieldless_enum_name(para),
            None,
            "typedef struct paragraph is not a fieldless enum"
        );
    }

    #[test]
    fn fieldless_enum_name_returns_none_for_function_pointer_paragraph() {
        let para = "typedef void (*KalicoCallback)(uint32_t event_id, void *user_data);";
        assert_eq!(
            fieldless_enum_name(para),
            None,
            "function-pointer typedef paragraph is not a fieldless enum"
        );
    }

    #[test]
    fn fieldless_enum_name_returns_none_when_typedef_missing() {
        // Has the enum opener but no companion typedef uint8_t line.
        let para = "enum NoTypedef {\n  X = 0,\n};";
        assert_eq!(
            fieldless_enum_name(para),
            None,
            "enum without companion typedef is not detected as fieldless"
        );
    }

    #[test]
    fn fieldless_enum_name_returns_some_for_doc_commented_enum() {
        let para = "/**\n * Describes arm policy.\n */\n\
            enum ArmPolicy {\n  TripImmediately = 0,\n};\ntypedef uint8_t ArmPolicy;";
        assert_eq!(
            fieldless_enum_name(para),
            Some("ArmPolicy".to_owned()),
            "doc-commented fieldless enum should be detected"
        );
    }
}
