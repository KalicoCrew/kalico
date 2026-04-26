//! The lexer must never panic on arbitrary input. It must always terminate
//! (yielding either a `Token` or a `ParseError` per non-empty line, and
//! eventually returning `None`).

use gcode::lex;
use proptest::prelude::*;

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 1024,
        ..Default::default()
    })]

    #[test]
    fn lexer_never_panics_on_arbitrary_text(s in ".{0,4096}") {
        let _: Vec<_> = lex(&s).collect();
    }

    #[test]
    fn lexer_never_panics_on_arbitrary_lines(
        lines in proptest::collection::vec(".{0,128}", 0..64)
    ) {
        let s = lines.join("\n");
        let _: Vec<_> = lex(&s).collect();
    }

    #[test]
    fn lexer_terminates_on_long_input(s in ".{0,16384}") {
        let count = lex(&s).count();
        // Per-line tokens at most; must terminate.
        prop_assert!(count <= s.lines().count() + 1);
    }
}
