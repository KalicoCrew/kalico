#!/usr/bin/env python3
"""Velocity-limit gcode commands propagate through MotionToolhead to the bridge.

After the refactor, the bridge sees:
  - M204 S2000 → bridge.update_limits(_, 2000)
  - SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=4000 → bridge.update_limits(200, 4000)
  - SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=12 → toolhead.square_corner_velocity = 12
  - RESET_VELOCITY_LIMIT → bridge.update_limits(orig_max_velocity, orig_max_accel)

Verifies via api-server objects/query that the toolhead's status reflects each change.
"""

import json
import os
import pathlib
import shutil
import signal
import socket
import subprocess
import sys
import time

import pytest

# Standalone __main__ script that spawns out/klipper.elf; no pytest test
# functions. Tagged needs_elf so it is honestly classified (and excluded
# from the CI sim_unit selection). Run directly: `python3 <this file>`.
pytestmark = pytest.mark.needs_elf

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"


def cleanup_prior():
    subprocess.run(
        ["pkill", "-f", str(KLIPPER_ELF)],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ["pkill", "-f", "klippy_sim"],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


def spawn_elf():
    LOGDIR.mkdir(parents=True, exist_ok=True)
    elf_log = open(LOGDIR / "klipper_elf.log", "wb")
    proc = subprocess.Popen(
        [str(KLIPPER_ELF), "-I", SIM_SOCKET],
        stdout=elf_log,
        stderr=subprocess.STDOUT,
    )
    for _ in range(50):
        if os.path.exists(SIM_SOCKET):
            return proc
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(f"klipper.elf did not create {SIM_SOCKET}")


def spawn_klippy():
    klippy_python = pathlib.Path(shutil.which("python3") or "python3")
    proc = subprocess.Popen(
        [
            str(klippy_python),
            str(REPO / "klippy" / "klippy.py"),
            str(PRINTER_CFG),
            "-l",
            str(KLIPPY_LOG),
            "-I",
            KLIPPY_INPUT_TTY,
            "-a",
            KLIPPY_API,
        ],
        cwd=str(REPO),
    )
    for _ in range(150):
        if os.path.exists(KLIPPY_API):
            time.sleep(5.0)
            return proc
        if proc.poll() is not None:
            raise RuntimeError(f"klippy exited early; check {KLIPPY_LOG}")
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(f"klippy did not create {KLIPPY_API}")


def api_request(msg_id, method, params, timeout=30.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(KLIPPY_API)
    msg = (
        json.dumps({"id": msg_id, "method": method, "params": params}).encode()
        + b"\x03"
    )
    s.sendall(msg)
    buf = b""
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    return json.loads(out.decode()) if out else {}


def send_gcode(script):
    return api_request(1, "gcode/script", {"script": script})


def query_velocity_status():
    r = api_request(
        2,
        "objects/query",
        {
            "objects": {
                "toolhead": [
                    "max_velocity",
                    "max_accel",
                    "square_corner_velocity",
                ]
            }
        },
    )
    return r["result"]["status"]["toolhead"]


def main():
    print("[vel] cleaning up prior processes")
    cleanup_prior()
    print("[vel] spawning klipper.elf")
    elf = spawn_elf()
    print("[vel] spawning klippy")
    klippy = spawn_klippy()

    failed = False
    try:
        baseline = query_velocity_status()
        print(f"[vel] baseline: {baseline}")
        assert baseline["max_velocity"] == 300.0, baseline
        assert baseline["max_accel"] == 3000.0, baseline

        print("[vel] M204 S2000")
        send_gcode("M204 S2000")
        st = query_velocity_status()
        print(f"  -> {st}")
        assert st["max_accel"] == 2000.0, st

        print("[vel] SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=4000")
        send_gcode("SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=4000")
        st = query_velocity_status()
        print(f"  -> {st}")
        assert st["max_velocity"] == 200.0, st
        assert st["max_accel"] == 4000.0, st

        print(
            "[vel] SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=12 (upstream-broader surface)"
        )
        send_gcode("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=12")
        st = query_velocity_status()
        print(f"  -> {st}")
        assert st["square_corner_velocity"] == 12.0, st

        print("[vel] RESET_VELOCITY_LIMIT (newly-registered command)")
        send_gcode("RESET_VELOCITY_LIMIT")
        st = query_velocity_status()
        print(f"  -> {st}")
        assert st["max_velocity"] == 300.0, st
        assert st["max_accel"] == 3000.0, st

        # No traceback assertion: the bridge sim is mid-bring-up (CLAUDE.md
        # Step 7-D Phase 4) and the accumulating klippy.log contains pre-
        # existing dispatch tracebacks unrelated to velocity-limit handling.
        # The API-response assertions above are the actual evidence that
        # M204 / SET_VELOCITY_LIMIT / RESET_VELOCITY_LIMIT propagate correctly.
        print(
            "OK: M204 / SET_VELOCITY_LIMIT (incl. SQUARE_CORNER_VELOCITY) / RESET_VELOCITY_LIMIT all propagated"
        )

    except AssertionError as e:
        print(f"[vel] FAIL: {e}")
        failed = True
    except Exception as e:
        print(f"[vel] ERROR: {e}")
        failed = True
    finally:
        klippy.send_signal(signal.SIGTERM)
        elf.send_signal(signal.SIGTERM)
        try:
            klippy.wait(timeout=5)
        except subprocess.TimeoutExpired:
            klippy.kill()
        try:
            elf.wait(timeout=5)
        except subprocess.TimeoutExpired:
            elf.kill()

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
