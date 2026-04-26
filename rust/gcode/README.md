# `gcode`

G-code lexer for the kalico motion planner. Pure text → typed tokens. No
NURBS dependency; no motion semantics. Reusable outside the planner for
offline analysis tools, replay tooling, or fuzz-target use.

See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`
for the architecture.

## Public surface

- `gcode::lex(&str)` returns `impl Iterator<Item = Result<Token, ParseError>>`.
- `Token` variants: `Command { letter, major, minor, params, line_no }`,
  `Comment { text, line_no }`, `Marker { kind, line_no }`.
- `MarkerKind` variants (slicer dialect): `LayerChange { layer: Option<u32> }`
  (`Some(N)` for Cura's `;LAYER:N`, `None` for OrcaSlicer/PrusaSlicer/Bambu's
  `;LAYER_CHANGE`), `LayerType`, `EndOfPrint`.
- `ParseError` variants: `MalformedNumber`, `UnrecognizedHead`,
  `EmptyCommand`, `DuplicateParam`.

## Fuzzing

`cargo +nightly fuzz run lex` from `rust/gcode/fuzz/`. Treat any panic or
hang as P0.

## What this crate does NOT do

- Interpret motion semantics (G2 = arc, G92 = set position, etc.).
- Track modal state.
- Construct NURBS or geometric segments.

Those are `geometry/`'s job.
