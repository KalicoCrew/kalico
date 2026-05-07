#!/usr/bin/env python3
"""Inherited set_position does not corrupt the (unused) bridge trapq.

Upstream ToolHead.set_position calls
  ffi_lib.trapq_set_position(self.trapq, self.print_time, x, y, z)
on every set_position. Under bridge mode, self.trapq is allocated (so
hardware-init's set_trapq doesn't crash) but never has segments
appended. We verify these writes are harmless: get_status keeps
returning sensible values, no segfault, no exception in 20+ rapid
SET_KINEMATIC_POSITION calls.

This is the regression test for spec test §4.9, addressing the Codex
review concern that "inherited unchanged equals beneficial" needs
empirical verification rather than reasoning alone.
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

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"


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
    elf_log = open(LOGDIR / "klipper_elf.log", "wb")
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
    klippy_python = pathlib.Path(shutil.which("python3") or "python3")
    proc = subprocess.Popen(
        [str(klippy_python), str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG), "-l", str(KLIPPY_LOG),
         "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
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
    msg = json.dumps({"id": msg_id, "method": method, "params": params}).encode() + b"\x03"
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
    print("[trapq] cleaning up prior processes")
    cleanup_prior()
    print("[trapq] spawning klipper.elf")
    elf = spawn_elf()
    print("[trapq] spawning klippy")
    klippy = spawn_klippy()

    failed = False
    try:
        for i in range(20):
            x = float(10 + i)
            y = float(20 + i)
            z = float(30 + i)
            req_id = 100 + i
            api_request(req_id, "gcode/script", {
                "script": f"SET_KINEMATIC_POSITION X={x} Y={y} Z={z}"
            })
            r = api_request(200 + i, "objects/query", {"objects": {
                "toolhead": ["position", "max_velocity", "estimated_print_time"]
            }})
            th = r["result"]["status"]["toolhead"]
            assert abs(th["position"][0] - x) < 1e-6, f"iter {i}: pos[0]={th['position'][0]} expected {x}"
            assert th["max_velocity"] == 300.0, f"iter {i}: max_velocity drifted to {th['max_velocity']}"
            assert th["estimated_print_time"] >= 0.0, f"iter {i}: bad est_print_time {th['estimated_print_time']}"
            if i % 5 == 0:
                print(f"  iter {i}: pos={th['position'][:3]} ok")

        print("OK: set_position with empty trapq is benign across 20 iterations")

    except AssertionError as e:
        print(f"[trapq] FAIL: {e}")
        failed = True
    except Exception as e:
        print(f"[trapq] ERROR: {e}")
        failed = True
    finally:
        klippy.send_signal(signal.SIGTERM)
        elf.send_signal(signal.SIGTERM)
        try: klippy.wait(timeout=5)
        except subprocess.TimeoutExpired: klippy.kill()
        try: elf.wait(timeout=5)
        except subprocess.TimeoutExpired: elf.kill()

    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
