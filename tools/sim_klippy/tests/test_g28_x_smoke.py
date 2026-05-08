"""Probe what G28 X does against the faithful sim.

Smoke test: we want to see how far the real-config G28 X path gets
in the sim — does klippy enter homing, what does the trsync layer do,
and where does it stall. Real-config X uses sensorless homing via the
TMC5160 DIAG line on stepper_x1.
"""
import json
import socket
import time

import pytest


def _info(api_socket: str) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect(api_socket)
    s.sendall(
        json.dumps({"id": 1, "method": "info", "params": {}}).encode() + b"\x03"
    )
    buf = b""
    while True:
        try:
            c = s.recv(4096)
        except Exception:
            break
        if not c:
            break
        buf += c
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    try:
        return json.loads(out.decode()) if out else {}
    except Exception:
        return {}


def test_g28_x_smoke(sim):
    # Wait for printer state == "ready" (the api state, not just
    # "Welcome to Kalico" log line — config init can still fail
    # between those two markers).
    deadline = time.time() + 30.0
    state = None
    while time.time() < deadline:
        r = _info(sim.api_socket)
        state = (r.get("result") or {}).get("state")
        if state == "ready":
            break
        time.sleep(0.5)
    print(f"\n[smoke] printer state at G28 send: {state}")
    assert state == "ready", f"klippy not ready before G28 X (state={state})"

    t0 = time.time()
    r = sim.gcode("G28 X", timeout=20.0)
    elapsed = time.time() - t0
    print(f"[smoke] G28 X result: {r}")
    print(f"[smoke] elapsed: {elapsed:.2f}s")

    log = sim.klippy_log.read_text() if sim.klippy_log.exists() else ""
    print("[smoke] klippy.log tail (last 80 lines):")
    for line in log.splitlines()[-80:]:
        if "kalico_status_v6" in line:
            continue
        print("  " + line)
