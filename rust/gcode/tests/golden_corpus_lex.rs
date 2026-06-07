use gcode::{Token, lex};

fn gen_straight_line_corpus(passes: u32) -> String {
    let mut out = String::with_capacity((passes as usize * 2 + 8) * 72);

    out.push_str("G21\nG90\nM82\nG92 E0\n");

    let mut e: f64 = 0.0;
    for pass in 0..passes {
        let y = f64::from(pass) * 0.300;
        e += 0.0500;
        out.push_str(&format!("G1 X220.000 Y{y:.3} E{e:.4} F3000\n"));
        e += 0.0500;
        out.push_str(&format!("G1 X0.000 Y{y:.3} E{e:.4} F3000\n"));
    }

    out
}

fn gen_arc_corpus(cycles: u32, layer_every: u32) -> String {
    let mut out = String::with_capacity((cycles as usize * 4 + 32) * 64);

    out.push_str("G21\nG90\nM82\nG92 E0\n");

    let mut e: f64 = 0.0;
    let mut arc_count: u32 = 0;

    let arcs: [(f64, f64, f64, f64); 4] = [
        (110.000, 160.000, -50.000, 0.000),
        (60.000, 110.000, 0.000, -50.000),
        (110.000, 60.000, 50.000, 0.000),
        (160.000, 110.000, 0.000, 50.000),
    ];

    for _ in 0..cycles {
        for (x, y, i, j) in arcs {
            if arc_count > 0 && arc_count % layer_every == 0 {
                out.push_str(";LAYER_CHANGE\n");
            }
            e += 0.0500;
            out.push_str(&format!(
                "G2 X{x:.3} Y{y:.3} I{i:.3} J{j:.3} E{e:.4} F3000\n"
            ));
            arc_count += 1;
        }
    }

    out
}

#[test]
fn arc_fitted_corpus_lexes_without_panic() {
    let text = gen_arc_corpus(26_000, 2_000);

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

    assert!(
        commands > 100_000,
        "expected > 100k Command tokens, got {commands}"
    );
    assert!(
        layer_changes >= 1,
        "expected at least one LayerChange marker, got {layer_changes}"
    );
    assert!(
        errors < commands / 100,
        "more than 1% of commands errored: {errors} errors vs {commands} commands"
    );
}

#[test]
fn straight_line_corpus_lexes_without_panic() {
    let text = gen_straight_line_corpus(76_000);

    let mut commands = 0u64;
    for token in lex(&text).flatten() {
        if matches!(token, Token::Command { .. }) {
            commands += 1;
        }
    }

    eprintln!("straight_line: commands={commands}");

    assert!(
        commands > 150_000,
        "expected > 150k Command tokens, got {commands}"
    );
}
