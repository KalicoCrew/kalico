#!/usr/bin/env python3
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


def query_positions():
    r = api_request(
        2,
        "objects/query",
        {
            "objects": {
                "toolhead": ["position"],
                "gcode_move": ["gcode_position"],
            }
        },
    )
    return r["result"]["status"]


def main():
    print("[gms] cleaning up prior processes")
    cleanup_prior()
    print("[gms] spawning klipper.elf")
    elf = spawn_elf()
    print("[gms] spawning klippy")
    klippy = spawn_klippy()

    failed = False
    try:
        print("[gms] SET_KINEMATIC_POSITION X=50 Y=60 Z=10")
        send_gcode("SET_KINEMATIC_POSITION X=50 Y=60 Z=10")
        st = query_positions()
        th_pos = st["toolhead"]["position"]
        gm_pos = st["gcode_move"]["gcode_position"]
        print(f"  toolhead.position={th_pos}")
        print(f"  gcode_move.gcode_position={gm_pos}")
        assert abs(th_pos[0] - 50.0) < 1e-6, f"toolhead X: {th_pos}"
        assert abs(th_pos[1] - 60.0) < 1e-6, f"toolhead Y: {th_pos}"
        assert abs(th_pos[2] - 10.0) < 1e-6, f"toolhead Z: {th_pos}"
        assert abs(gm_pos[0] - 50.0) < 1e-6, (
            f"gcode_move drifted from toolhead after SET_KINEMATIC_POSITION: "
            f"{gm_pos} vs {th_pos}"
        )
        assert abs(gm_pos[1] - 60.0) < 1e-6, f"gcode_move Y: {gm_pos}"

        print("[gms] G92 X0 Y0")
        send_gcode("G92 X0 Y0")
        st = query_positions()
        gm_pos = st["gcode_move"]["gcode_position"]
        th_pos = st["toolhead"]["position"]
        print(f"  toolhead.position={th_pos}")
        print(f"  gcode_move.gcode_position={gm_pos}")
        assert abs(gm_pos[0] - 0.0) < 1e-6, gm_pos
        assert abs(gm_pos[1] - 0.0) < 1e-6, gm_pos
        assert abs(th_pos[0] - 50.0) < 1e-6, th_pos
        assert abs(th_pos[1] - 60.0) < 1e-6, th_pos

        print(
            "OK: gcode_move stays synced through set_position event + G92 frame reset"
        )

    except AssertionError as e:
        print(f"[gms] FAIL: {e}")
        failed = True
    except Exception as e:
        print(f"[gms] ERROR: {e}")
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
