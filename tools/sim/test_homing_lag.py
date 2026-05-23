#!/usr/bin/env python3
"""Reproduce the homing-lag timing bug against Renode.

Launches Renode + klippy, sends G28 X, injects GPIO endstop triggers
via Renode's telnet monitor, and measures wall-clock timing.

Usage:
    bash tools/sim/build_sim_firmware.sh
    python3 tools/sim/test_homing_lag.py
"""
import json
import os
import pathlib
import signal
import socket
import subprocess
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parents[2]
LOGDIR = REPO / "tools" / "sim" / ".homing-test-logs"
PRINTER_CFG = REPO / "tools" / "sim" / "homing_test.cfg"
KLIPPY_LOG = LOGDIR / "klippy.log"
KLIPPY_API = "/tmp/klippy_homing_test_api"

RENODE_UART_PORT = 3334
RENODE_MONITOR_PORT = 3335
ENDSTOP_PIN = "PA4"  # stepper_x endstop
ENDSTOP_PORT = "gpioPortA"
ENDSTOP_LINE = 4


def cleanup():
    subprocess.run(["pkill", "-f", "renode.*h723_sim"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    try:
        os.unlink(KLIPPY_API)
    except FileNotFoundError:
        pass


def spawn_renode():
    """Launch Renode in background, wait for UART port."""
    proc = subprocess.Popen(
        ["renode", "--port", str(RENODE_MONITOR_PORT), "--disable-gui",
         "-e", f"include @{REPO}/tools/sim/h723_sim.resc",
         "-e", "logLevel 3 sysbus",
         "-e", "logLevel 3 rcc",
         "-e", "logLevel 3 nvic",
         "-e", "logLevel 0 usart2",
         "-e", "start"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(100):
        try:
            s = socket.create_connection(("localhost", RENODE_UART_PORT), timeout=1)
            s.close()
            return proc
        except (ConnectionRefusedError, OSError):
            time.sleep(0.2)
    proc.terminate()
    raise RuntimeError("Renode UART port never opened")


def spawn_klippy():
    """Launch klippy connected to Renode's UART."""
    LOGDIR.mkdir(parents=True, exist_ok=True)
    stderr_log = open(LOGDIR / "klippy_stderr.log", "wb")
    proc = subprocess.Popen(
        ["python3", str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG),
         "-l", str(KLIPPY_LOG),
         "-a", KLIPPY_API],
        cwd=str(REPO),
        stdout=stderr_log, stderr=subprocess.STDOUT,
    )
    # Wait for API socket
    for _ in range(300):
        if os.path.exists(KLIPPY_API):
            time.sleep(3.0)
            return proc
        if proc.poll() is not None:
            raise RuntimeError(f"klippy exited early: {proc.returncode}")
        time.sleep(0.2)
    proc.terminate()
    raise RuntimeError("klippy API socket never appeared")


def send_gcode(script, timeout=30.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(KLIPPY_API)
    msg = json.dumps({
        "id": 1, "method": "gcode/script",
        "params": {"script": script},
    }).encode() + b"\x03"
    s.sendall(msg)
    buf = b""
    while True:
        c = s.recv(4096)
        if not c:
            break
        buf += c
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    return json.loads(out.decode()) if out else {}


def renode_set_gpio(port, line, value):
    """Set a GPIO input level via Renode's telnet monitor."""
    s = socket.create_connection(("localhost", RENODE_MONITOR_PORT), timeout=5)
    s.settimeout(2)
    try:
        s.recv(4096)
    except socket.timeout:
        pass
    cmd = f"sysbus.{port} Set {line} {'true' if value else 'false'}\n"
    s.sendall(cmd.encode())
    time.sleep(0.2)
    try:
        s.recv(4096)
    except socket.timeout:
        pass
    s.close()


def main():
    cleanup()
    print("Spawning Renode...")
    renode = spawn_renode()
    print("Renode started. Spawning klippy...")
    klippy = None
    try:
        klippy = spawn_klippy()
        print("Klippy started. Running homing test...")

        # Pre-trip: set endstop HIGH before G28
        renode_set_gpio(ENDSTOP_PORT, ENDSTOP_LINE, 1)
        time.sleep(0.5)

        t0 = time.time()
        r = send_gcode("G28 X", timeout=30.0)
        elapsed = time.time() - t0

        print(f"\nG28 X result: {r}")
        print(f"Elapsed: {elapsed:.2f}s")

        if elapsed > 5.0:
            print(f"\nBUG CONFIRMED: G28 X took {elapsed:.1f}s "
                  f"(expected < 5s with pre-tripped endstop)")
        else:
            print(f"\nOK: G28 X completed in {elapsed:.1f}s")

        # Check klippy.log for details
        if KLIPPY_LOG.exists():
            log = KLIPPY_LOG.read_text()
            if "needs rehome" in log:
                print("Log contains 'needs rehome'")
            if "No trigger" in log:
                print("Log contains 'No trigger' — second home failed!")
            # Print homing-related lines
            for line in log.splitlines():
                if any(k in line for k in [
                    "homing:", "needs rehome", "bridge-trace",
                    "No trigger", "steps_moved",
                ]):
                    print(f"  {line.strip()}")

    finally:
        if klippy is not None:
            klippy.terminate()
            klippy.wait(timeout=3)
        renode.terminate()
        renode.wait(timeout=3)


if __name__ == "__main__":
    sys.exit(main() or 0)
