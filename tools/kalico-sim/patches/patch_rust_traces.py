#!/usr/bin/env python3
"""Inject sim diagnostic traces into Rust bridge source before compilation."""

import pathlib
import sys


def patch_bridge(path):
    text = path.read_text()

    # Add trace right before planner.submit_move(classified) in submit_homing_move_inner
    target = "if let Err(e) = planner.submit_move(classified) {"
    if target in text and "[sim-diag]" not in text:
        # Replace the entire submit_move + error handling block
        old_block = (
            "if let Err(e) = planner.submit_move(classified) {\n"
            "            self.homing.reset_to_idle();\n"
            "            return Err(planner_err(e));\n"
            "        }"
        )
        new_block = (
            'log::info!("[sim-diag] homing: calling submit_move dur={:.6} dist={:.3}", classified.nominal_duration(), classified.distance_mm);\n'
            "        match planner.submit_move(classified) {\n"
            '            Ok(()) => log::info!("[sim-diag] homing: submit_move OK"),\n'
            "            Err(e) => {\n"
            '                log::error!("[sim-diag] homing: submit_move FAILED: {:?}", e);\n'
            "                self.homing.reset_to_idle();\n"
            "                return Err(planner_err(e));\n"
            "            }\n"
            "        }"
        )
        text = text.replace(old_block, new_block, 1)
        print(f"patched submit_homing_move trace in {path}")

    path.write_text(text)


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        p = pathlib.Path(arg)
        if p.name == "bridge.rs":
            patch_bridge(p)
