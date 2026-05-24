#!/usr/bin/env python3
"""Preprocess slicer G-code for the Kalico Simulator batch mode.

Strips custom macros (PRINT_START, PRINT_END, EXCLUDE_OBJECT) and
temperature/fan commands that don't affect motion timing, replacing
the startup sequence with SET_KINEMATIC_POSITION so homing isn't needed.

Usage:
    python3 preprocess_gcode.py input.gcode output.gcode
    python3 preprocess_gcode.py input.gcode -  # stdout
"""
import re
import sys

SKIP_PREFIXES = (
    "PRINT_END",
    "EXCLUDE_OBJECT",
    "M104 ",  # extruder temp
    "M109 ",  # wait for extruder temp
    "M140 ",  # bed temp
    "M190 ",  # wait for bed temp
    "M106 ",  # fan
    "M107",   # fan off
    "M73 ",   # progress display
    "M141 ",  # chamber temp
    "M191 ",  # wait for chamber
    "M116 ",  # wait all temps
    "RESPOND ",
    "STATUS_",
    "BED_MESH",
    "QUAD_GANTRY_LEVEL",
    "Z_TILT_ADJUST",
    "CLEAN_NOZZLE",
    "CALIBRATE_Z",
    "ADAPTIVE_BED_MESH",
    "_",  # macro helpers
)


def preprocess(input_path: str, output_path: str) -> dict:
    with open(input_path) as f:
        lines = f.readlines()

    out = []
    stats = {"total": len(lines), "kept": 0, "skipped": 0, "replaced": 0}

    for line in lines:
        stripped = line.strip()

        # Replace PRINT_START with position initialization
        if stripped.startswith("PRINT_START"):
            out.append("SET_KINEMATIC_POSITION X=150 Y=150 Z=150\n")
            stats["replaced"] += 1
            continue

        # Skip non-motion commands
        if any(stripped.startswith(p) for p in SKIP_PREFIXES):
            stats["skipped"] += 1
            continue

        out.append(line)
        stats["kept"] += 1

    if output_path == "-":
        sys.stdout.writelines(out)
    else:
        with open(output_path, "w") as f:
            f.writelines(out)

    return stats


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} input.gcode output.gcode")
        sys.exit(1)
    stats = preprocess(sys.argv[1], sys.argv[2])
    g1_count = sum(1 for l in open(sys.argv[2] if sys.argv[2] != "-" else "/dev/stdin")
                   if l.strip().startswith("G1 "))
    print(f"Preprocessed: {stats['total']} → {stats['kept']} lines "
          f"({stats['skipped']} skipped, {stats['replaced']} replaced), "
          f"{g1_count} G1 moves", file=sys.stderr)
