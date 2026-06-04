#!/usr/bin/env python3
"""Klippy-in-loop sim launcher.

Brings up a host-process klipper.elf (kalico runtime as Linux MCU) and a
fresh klippy instance pointed at it via /tmp/klipper_sim_socket. Sends
gcode through the api-server unix socket and captures klippy.log.

Run on the Pi after `make` builds out/klipper.elf in the sim repo:
    python3 run.py G28 X

Defaults are tuned for ~/klipper-sim/repo on trident.local.
"""

import argparse
import json
import os
import pathlib
import socket
import subprocess
import time

# Repo root: env override → script dir's grandparent → ~/klipper-sim/repo
# Lets the same script work both inside the Docker container (where the
# repo is mounted at /work) and on a Pi with the original layout.
_DEFAULT_REPO = (
    pathlib.Path(os.environ.get("KALICO_REPO"))
    if os.environ.get("KALICO_REPO")
    else pathlib.Path(__file__).resolve().parents[2]
)
REPO = _DEFAULT_REPO
LOGDIR = pathlib.Path(
    os.environ.get(
        "KALICO_SIM_LOGDIR", str(REPO / "tools" / "sim_klippy" / ".local-logs")
    )
)
RUNDIR = LOGDIR / "run"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"
ELF_LOG = LOGDIR / "klipper_elf.log"


def cleanup_prior():
    """Best-effort kill of any prior sim processes + stale sockets."""
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
    RUNDIR.mkdir(parents=True, exist_ok=True)
    elf_log = open(ELF_LOG, "wb")
    proc = subprocess.Popen(
        [str(KLIPPER_ELF), "-I", SIM_SOCKET],
        stdout=elf_log,
        stderr=subprocess.STDOUT,
        cwd=str(REPO),
    )
    # Wait for the PTY symlink to appear.
    for _ in range(50):
        if os.path.exists(SIM_SOCKET):
            return proc
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(
        f"klipper.elf did not create {SIM_SOCKET}; check {ELF_LOG}"
    )


def spawn_klippy():
    env = os.environ.copy()
    # Run klippy from the sim repo so it picks up our motion_toolhead etc.
    # Prefer ~/klippy-env (Pi); fall back to system python3 (Docker etc.).
    klippy_python = pathlib.Path.home() / "klippy-env" / "bin" / "python"
    if not klippy_python.exists():
        import shutil

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
        env=env,
        cwd=str(REPO),
    )
    # Wait for the api socket.
    for _ in range(150):
        if os.path.exists(KLIPPY_API):
            time.sleep(1.0)  # let the connect finish
            return proc
        if proc.poll() is not None:
            raise RuntimeError(
                f"klippy exited early; tail {KLIPPY_LOG}: \n"
                + (
                    KLIPPY_LOG.read_text()[-2000:]
                    if KLIPPY_LOG.exists()
                    else "(no log)"
                )
            )
        time.sleep(0.1)
    proc.terminate()
    raise RuntimeError(
        f"klippy did not create {KLIPPY_API}; check {KLIPPY_LOG}"
    )


def send_gcode(script: str, timeout: float = 30.0) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(KLIPPY_API)
    msg = (
        json.dumps(
            {"id": 1, "method": "gcode/script", "params": {"script": script}}
        ).encode("utf-8")
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
    return json.loads(out.decode("utf-8")) if out else {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("script", nargs="*", default=["G28 X"])
    ap.add_argument(
        "--keep", action="store_true", help="leave processes running after"
    )
    args = ap.parse_args()

    cleanup_prior()
    print(f"[sim] spawning klipper.elf -> {SIM_SOCKET}")
    elf = spawn_elf()
    print(f"[sim] spawning klippy (api={KLIPPY_API}, log={KLIPPY_LOG})")
    klippy = spawn_klippy()

    try:
        script = " ".join(args.script)
        print(f"[sim] sending: {script}")
        try:
            resp = send_gcode(script)
            print(f"[sim] response: {json.dumps(resp, indent=2)[:500]}")
        except Exception as e:
            print(f"[sim] send_gcode raised: {e}")

        time.sleep(2.0)
        print(f"[sim] tail {KLIPPY_LOG}:")
        log = KLIPPY_LOG.read_text() if KLIPPY_LOG.exists() else ""
        # Filter heartbeat noise
        for line in log.splitlines()[-200:]:
            if "kalico_status_v6" in line:
                continue
            print("  " + line)
    finally:
        if not args.keep:
            print("[sim] tearing down")
            for p in (klippy, elf):
                try:
                    p.terminate()
                    p.wait(timeout=3)
                except Exception:
                    p.kill()


if __name__ == "__main__":
    main()
