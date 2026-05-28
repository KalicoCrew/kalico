#!/usr/bin/env python3
"""Inject sim diagnostic traces into klippy Python files.

Called from the Dockerfile runtime stage after klippy/ is copied fresh.
"""

import pathlib
import sys


def patch_motion_toolhead(path):
    text = path.read_text()
    old = "            self.bridge.submit_homing_move(pos3, speed, arm_ids)"
    if old not in text:
        print(f"patch_klippy_traces: submit_homing_move not found in {path}")
        return
    new = (
        '            logging.info("[sim-trace] submit_homing pos3=%s speed=%s arms=%s cmd=%s",'
        " pos3, speed, arm_ids, self.commanded_pos[:3])\n" + old
    )
    text = text.replace(old, new, 1)
    path.write_text(text)
    print(f"patch_klippy_traces: patched {path}")


def patch_motion_bridge(path):
    """Increase attach_serial timeout for sim (vtime makes clock advance faster)."""
    text = path.read_text()
    old = "return self._bridge.attach_serial(mcu_handle, serial_path, baud, timeout_s)"
    if old in text and "# sim-patched" not in text:
        new = (
            "# sim-patched: increase timeout for vtime\n"
            "        return self._bridge.attach_serial(mcu_handle, serial_path, baud, max(timeout_s, 120.0))"
        )
        text = text.replace(old, new, 1)
        path.write_text(text)
        print(f"patch_klippy_traces: patched attach_serial timeout in {path}")


def patch_mcu_homing(path):
    """Increase homing backstop timeout for sim.

    The planner's dispatch closure can block for ~5s on compute_ack_clock
    when the MCU runs on vtime (clock sync takes time to converge).
    The default 1.0s slack isn't enough. Add 15s for the sim.
    """
    text = path.read_text()
    old = "slack = max(0.0, home_end_time - est_now) + 1.0"
    if old in text and "# sim-patched" not in text:
        new = "slack = max(0.0, home_end_time - est_now) + 16.0  # sim-patched: +15s for vtime dispatch latency"
        text = text.replace(old, new, 1)
        path.write_text(text)
        print(f"patch_klippy_traces: patched homing backstop slack in {path}")


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        p = pathlib.Path(arg)
        if p.name == "motion_toolhead.py":
            patch_motion_toolhead(p)
        elif p.name == "motion_bridge.py":
            patch_motion_bridge(p)
        elif p.name == "mcu.py":
            patch_mcu_homing(p)
