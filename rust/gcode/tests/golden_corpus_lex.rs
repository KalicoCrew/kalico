//! Tokenize the `OrcaSlicer` corpus end-to-end. Asserts:
//!  - No panics.
//!  - Token counts match expected order-of-magnitude.
//!  - At least one `LayerChange` marker is recognized.
//!  - At least 100k Command tokens for G/M/T heads.

use gcode::{lex, Token};
use std::path::Path;

const CORPUS_DIR: &str = "../../scripts/fitter_prototype/corpus";

fn read_corpus_file(name: &str) -> Option<String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(CORPUS_DIR).join(name);
    std::fs::read_to_string(&path).ok()
}

#[test]
fn arc_fitted_corpus_lexes_without_panic() {
    let Some(text) = read_corpus_file("voron_cube_arc_fitted.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };

    let mut commands = 0u64;
    let mut comments = 0u64;
    let mut markers = 0u64;
    let mut errors = 0u64;
    let mut layer_changes = 0u64;

    for item in lex(&text) {
        match item {
            Ok(Token::Command { .. }) => commands += 1,
            Ok(Token::Comment { .. }) => comments += 1,
            Ok(Token::Marker { kind, .. }) => {
                markers += 1;
                if matches!(kind, gcode::MarkerKind::LayerChange { .. }) {
                    layer_changes += 1;
                }
            }
            Err(_) => errors += 1,
            Ok(_) => {}
        }
    }

    eprintln!(
        "arc_fitted: commands={commands} comments={comments} markers={markers} \
         errors={errors} layer_changes={layer_changes}"
    );

    assert!(commands > 100_000, "expected > 100k Command tokens, got {commands}");
    assert!(layer_changes >= 1, "expected at least one LayerChange marker");
    assert!(
        errors < commands / 100,
        "more than 1% of commands errored: {errors} errors vs {commands} commands"
    );
}

#[test]
fn straight_line_corpus_lexes_without_panic() {
    let Some(text) = read_corpus_file("voron_cube_straight_line.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };
    let mut commands = 0u64;
    for item in lex(&text) {
        if let Ok(Token::Command { .. }) = item {
            commands += 1;
        }
    }
    assert!(commands > 150_000, "expected > 150k Command tokens, got {commands}");
}
