# Layer 1 Fitter Prototype

Python prototype for Layer 1 of the Kalico motion-planner rewrite. See
`docs/superpowers/specs/2026-04-26-layer-1-fitter-prototype-design.md` for
the design context.

## Run

    uv sync --group prototype
    uv run python -m scripts.fitter_prototype.run \
        scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode \
        --out results/

## Test

    uv run pytest scripts/fitter_prototype/tests/
