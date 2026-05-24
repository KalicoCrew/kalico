#!/usr/bin/env python3
"""Inject sim diagnostic traces into klippy Python files.

Called from the Dockerfile runtime stage after klippy/ is copied fresh.
"""
import sys
import pathlib

def patch_motion_toolhead(path):
    text = path.read_text()
    old = "            self.bridge.submit_homing_move(pos3, speed, arm_ids)"
    if old not in text:
        print(f"patch_klippy_traces: submit_homing_move not found in {path}")
        return
    new = (
        '            logging.info("[sim-trace] submit_homing pos3=%s speed=%s arms=%s cmd=%s",'
        " pos3, speed, arm_ids, self.commanded_pos[:3])\n"
        + old
    )
    text = text.replace(old, new, 1)
    path.write_text(text)
    print(f"patch_klippy_traces: patched {path}")

if __name__ == "__main__":
    for arg in sys.argv[1:]:
        p = pathlib.Path(arg)
        if p.name == "motion_toolhead.py":
            patch_motion_toolhead(p)
