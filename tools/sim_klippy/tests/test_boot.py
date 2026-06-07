# "Printer is ready" is not written to klippy.log — it only appears in the
# printer.state attribute exposed over the API socket; poll info{} there.

import json
import pathlib
import shutil
import socket
import time

import pytest

pytestmark = pytest.mark.needs_elf


def _save_logs_for_inspection(sim, name: str) -> None:
    dest = pathlib.Path("/work/.local-logs") / name
    dest.mkdir(parents=True, exist_ok=True)
    if sim.klippy_log.exists():
        shutil.copy(sim.klippy_log, dest / "klippy.log")
    stdout = sim.log_dir / "klippy.stdout"
    if stdout.exists():
        shutil.copy(stdout, dest / "klippy.stdout")
    for mcu_name in ("h7.log", "f4.log"):
        src = sim.log_dir / mcu_name
        if src.exists():
            shutil.copy(src, dest / mcu_name)
    bt = sim.log_dir / "beacon_traffic.log"
    if bt.exists():
        shutil.copy(bt, dest / "beacon_traffic.log")
    rendered = sim.klippy_log.parent.parent / "printer.cfg"
    if rendered.exists():
        shutil.copy(rendered, dest / "printer.cfg")


def _query_state(api_socket: str, timeout: float = 5.0) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    sock.connect(api_socket)
    req = {"id": 1, "method": "info", "params": {}}
    sock.sendall(json.dumps(req).encode() + b"\x03")
    buf = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if b"\x03" in buf:
            break
    sock.close()
    if b"\x03" in buf:
        body = buf.split(b"\x03", 1)[0]
        try:
            return json.loads(body.decode())
        except json.JSONDecodeError:
            return {}
    return {}


def test_boot_clean(sim):
    try:
        deadline = time.monotonic() + 30.0
        info = {}
        while time.monotonic() < deadline:
            try:
                info = _query_state(sim.api_socket)
            except (FileNotFoundError, ConnectionRefusedError):
                info = {}
            result = info.get("result", {}) if info else {}
            state = result.get("state")
            if state == "ready":
                break
            time.sleep(0.5)

        log = sim.klippy_log.read_text() if sim.klippy_log.exists() else ""
        last_lines = "\n".join(log.splitlines()[-80:])

        assert "Traceback" not in log, (
            f"klippy crashed during boot:\n{log[-3000:]}"
        )
        if "MCU '" in log and " shutdown:" in log:
            if "Command request" in log or "Emergency stop" in log:
                pytest.fail(f"MCU shutdown during boot:\n{log[-3000:]}")
        assert "transport closed" not in log
        assert "transport timed out" not in log

        result = info.get("result", {}) if info else {}
        assert result.get("state") == "ready", (
            f"klippy did not reach ready (state={result.get('state')!r}, "
            f"state_message={result.get('state_message')!r}). "
            f"Last 80 lines:\n{last_lines}"
        )
    finally:
        _save_logs_for_inspection(sim, "test_boot_clean")
