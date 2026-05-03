#!/usr/bin/env python3
"""Phase 4 gate test — verify G1 X10 produces non-zero step pulses.

Run from the repo root inside the Docker container:
    python3 tools/sim_klippy/test_phase4_steps.py

Uses the bridge's bridge_call() to query kalico_sim_stepper_count_query oid=0
(X-axis stepper, OID 0 in the sim printer.cfg) after a fake-homed G1 X10.

Gate: step_count > 0 after the move.  Expected: ~10 mm × steps_per_mm.
For stepper_x: rotation_distance=40, microsteps=16, full_steps=200 →
  steps_per_mm = (full_steps × microsteps) / rotation_distance = (200 × 16) / 40 = 80
  10 mm → ~800 steps (corexy shares A+B so actual count may differ).
"""
import json
import os
import pathlib
import signal
import socket
import subprocess
import sys
import time

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"
ELF_LOG = LOGDIR / "klipper_elf.log"


def cleanup_prior():
    subprocess.run(["pkill", "-f", str(KLIPPER_ELF)], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "klippy_sim"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


def spawn_elf():
    LOGDIR.mkdir(parents=True, exist_ok=True)
    elf_log = open(ELF_LOG, "wb")
    proc = subprocess.Popen(
        [str(KLIPPER_ELF), "-I", SIM_SOCKET],
        stdout=elf_log, stderr=subprocess.STDOUT,
    )
    for _ in range(50):
        if os.path.exists(SIM_SOCKET):
            return proc
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(f"klipper.elf did not create {SIM_SOCKET}")


def spawn_klippy():
    import shutil
    klippy_python = pathlib.Path(shutil.which("python3") or "python3")
    proc = subprocess.Popen(
        [str(klippy_python), str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG), "-l", str(KLIPPY_LOG),
         "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
        cwd=str(REPO),
    )
    for _ in range(150):
        if os.path.exists(KLIPPY_API):
            time.sleep(1.0)
            return proc
        if proc.poll() is not None:
            raise RuntimeError(f"klippy exited early; check {KLIPPY_LOG}")
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(f"klippy did not create {KLIPPY_API}")


def send_gcode(script, timeout=30.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(KLIPPY_API)
    msg = json.dumps(
        {"id": 1, "method": "gcode/script", "params": {"script": script}}
    ).encode() + b"\x03"
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


def main():
    print("[phase4] cleaning up prior processes")
    cleanup_prior()

    print("[phase4] spawning klipper.elf")
    elf = spawn_elf()
    print("[phase4] spawning klippy")
    klippy = spawn_klippy()

    try:
        # Fake-home by setting kinematic position
        print("[phase4] fake-homing: SET_KINEMATIC_POSITION X=0 Y=0 Z=0")
        resp = send_gcode("SET_KINEMATIC_POSITION X=0 Y=0 Z=0")
        print(f"  response: {resp}")

        # Send the move — do NOT call M400 since the shaper pipeline may
        # raise a tolerance error on flush().  Steps are emitted asynchronously
        # by the MCU; just wait a few seconds for them to accumulate.
        print("[phase4] sending G1 X10 F1000 then M400 (flush)")
        resp = send_gcode("G1 X10 F1000\nM400")
        print(f"  response: {resp}")

        print("[phase4] waiting 2s post-M400 for step counter sync")
        time.sleep(2.0)

        # Query step counts via the KALICO_SIM_STEP_COUNT G-code command,
        # which routes through klippy → bridge → MCU wire command.
        print("[phase4] querying step count for OID 0 (X stepper)")
        x_resp = send_gcode("KALICO_SIM_STEP_COUNT OID=0")
        print(f"  OID=0 response: {x_resp}")

        print("[phase4] querying step count for OID 1 (Y stepper)")
        y_resp = send_gcode("KALICO_SIM_STEP_COUNT OID=1")
        print(f"  OID=1 response: {y_resp}")

        # Extract counts from klippy log (respond_info writes there)
        log = KLIPPY_LOG.read_text() if KLIPPY_LOG.exists() else ""
        x_count = 0
        y_count = 0
        for line in log.splitlines():
            if "KALICO_SIM_STEP_COUNT oid=0" in line:
                try:
                    x_count = int(line.split("count=")[1].split()[0])
                except (IndexError, ValueError):
                    pass
            if "KALICO_SIM_STEP_COUNT oid=1" in line:
                try:
                    y_count = int(line.split("count=")[1].split()[0])
                except (IndexError, ValueError):
                    pass

        print(f"\n[phase4] RESULT: X step count (oid=0): {x_count}")
        print(f"[phase4] RESULT: Y step count (oid=1): {y_count}")

        # Phase 4 gate
        if x_count > 0 or y_count > 0:
            total = abs(x_count) + abs(y_count)
            print(f"\n[phase4] GATE GREEN: {total} total step pulses emitted")
            print("[phase4] Phase 4 PASS")
            return 0
        else:
            print("\n[phase4] GATE RED: 0 step pulses — move did not produce steps")
            # Dump relevant log lines for diagnosis
            for line in log.splitlines():
                if any(k in line for k in ("Error", "Traceback", "step", "submit_move",
                                           "bridge-trace", "planner", "bridge-async",
                                           "KALICO_SIM", "homed")):
                    print("  LOG:", line[-200:])
            return 1

    finally:
        print("\n[phase4] tearing down")
        for p in (klippy, elf):
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                p.kill()


if __name__ == "__main__":
    sys.exit(main())
