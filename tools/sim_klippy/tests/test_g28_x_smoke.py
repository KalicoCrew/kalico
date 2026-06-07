import json
import pathlib
import shutil
import socket
import time

import pytest

pytestmark = pytest.mark.needs_elf

_ARTIFACT_DIR = pathlib.Path("/work/.local-logs/test_g28_x_smoke")


def _save_artifacts(sim) -> None:
    _ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    if sim.klippy_log.exists():
        shutil.copy(sim.klippy_log, _ARTIFACT_DIR / "klippy.log")
    for name in ("klippy.stdout", "h7.log", "f4.log", "beacon_traffic.log"):
        src = sim.log_dir / name
        if src.exists():
            shutil.copy(src, _ARTIFACT_DIR / name)
    rendered = sim.klippy_log.parent.parent / "printer.cfg"
    if rendered.exists():
        shutil.copy(rendered, _ARTIFACT_DIR / "printer.cfg")


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
    try:
        # Poll API state, not log line: "Welcome to Kalico" precedes config init,
        # which can still fail before the printer is actually ready.
        deadline = time.time() + 30.0
        state = None
        while time.time() < deadline:
            r = _info(sim.api_socket)
            state = (r.get("result") or {}).get("state")
            if state == "ready":
                break
            time.sleep(0.5)
        print(f"\n[smoke] printer state at G28 send: {state}")
        assert state == "ready", (
            f"klippy not ready before G28 X (state={state})"
        )

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
    finally:
        _save_artifacts(sim)
