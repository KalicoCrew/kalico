use gcode::lex;
use proptest::prelude::*;

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 1024,
        ..Default::default()
    })]

    #[test]
    fn lexer_never_panics_on_arbitrary_text(s in ".{0,4096}") {
        let line_count = s.lines().count() as u32;
        for item in lex(&s) {
            match item {
                Ok(gcode::Token::Command { letter, line_no, .. }) => {
                    prop_assert!(letter.is_ascii_uppercase(),
                        "Command letter must be uppercase ASCII; got {letter}");
                    prop_assert!(line_no >= 1 && line_no <= line_count,
                        "line_no {line_no} out of range 1..={line_count}");
                }
                Ok(gcode::Token::Comment { line_no, .. }
                    | gcode::Token::Marker { line_no, .. }) => {
                    prop_assert!(line_no >= 1 && line_no <= line_count,
                        "line_no {line_no} out of range 1..={line_count}");
                }
                // Token is non_exhaustive; ParseError doesn't expose line_no uniformly;
                // both arms exist purely for the no-panic guarantee.
                Ok(_) | Err(_) => {}
            }
        }
    }

    #[test]
    fn lexer_never_panics_on_arbitrary_lines(
        lines in proptest::collection::vec(".{0,128}", 0..64)
    ) {
        let s = lines.join("\n");
        let line_count = s.lines().count() as u32;
        let commands = lex(&s).flatten().filter_map(|t| {
            if let gcode::Token::Command { letter, line_no, .. } = t {
                Some((letter, line_no))
            } else {
                None
            }
        });
        for (letter, line_no) in commands {
            prop_assert!(letter.is_ascii_uppercase());
            prop_assert!(line_no >= 1 && line_no <= line_count);
        }
    }

    #[test]
    fn lexer_terminates_on_long_input(s in ".{0,16384}") {
        let count = lex(&s).count();
        prop_assert!(count <= s.lines().count());
    }
}
