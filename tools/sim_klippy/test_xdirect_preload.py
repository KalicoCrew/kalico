#!/usr/bin/env python3
"""TDD test: XDIRECT preload after phase stepping enable.

When a TMC5160 is configured with phase_stepping=True and the stepper
is enabled, the driver must receive a non-zero XDIRECT write BEFORE
any motion is commanded.  Without this preload, XDIRECT stays at its
reset value (0x00000000) and the motor outputs zero current — the
stepper doesn't energize even though direct_mode=1 and toff>0.

This test verifies the preload by checking the TMC5160 emulator's
XDIRECT log after klippy startup + SET_KINEMATIC_POSITION (which
triggers stepper enable) but BEFORE any G1 move.

Expected: FAIL until the XDIRECT preload is implemented.
"""

import json
import os
import pathlib
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
ELF_LOG = LOGDIR / "klipper_elf.log"
SIM_SOCK_DIR = pathlib.Path("/tmp/kalico_sim_socks")


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


def spawn_tmc_emulators():
    SIM_SOCK_DIR.mkdir(exist_ok=True)
    emu_script = REPO / "tools" / "kalico-sim" / "emulators" / "tmc5160_emu.py"
    procs = []
    for line in (27, 26):
        sock_path = SIM_SOCK_DIR / f"spi_cs_0_{line}"
        emu_log = open(LOGDIR / f"tmc_emu_{line}.log", "w")
        p = subprocess.Popen(
            [sys.executable, str(emu_script), str(sock_path)],
            stdout=emu_log,
            stderr=emu_log,
        )
        for _ in range(20):
            if sock_path.exists():
                break
            time.sleep(0.05)
        procs.append(p)
    return procs


def spawn_elf():
    LOGDIR.mkdir(parents=True, exist_ok=True)
    elf_log = open(ELF_LOG, "wb")
    shim_so = REPO / "tools" / "kalico-sim" / "libvtime" / "libsim_intercept.so"
    if not shim_so.exists():
        subprocess.check_call(
            ["make", "-C", str(shim_so.parent)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    env = os.environ.copy()
    env["LD_PRELOAD"] = str(shim_so)
    env["KALICO_SIM_SOCK_DIR"] = str(SIM_SOCK_DIR)
    env["KALICO_SIM_SHIM_VERBOSE"] = "1"
    proc = subprocess.Popen(
        [str(KLIPPER_ELF), "-I", SIM_SOCKET],
        stdout=elf_log,
        stderr=subprocess.STDOUT,
        env=env,
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


def send_gcode(script, timeout=30.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(KLIPPY_API)
    msg = (
        json.dumps(
            {"id": 1, "method": "gcode/script", "params": {"script": script}}
        ).encode()
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


def read_emulator_xdirect_log(line_id):
    """Read the XDIRECT write log from the emulator's stdout log."""
    log_path = LOGDIR / f"tmc_emu_{line_id}.log"
    if not log_path.exists():
        return []
    entries = []
    for line in log_path.read_text(errors="replace").splitlines():
        if line.startswith("XDIRECT "):
            parts = {}
            for token in line.split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    parts[k] = v
            try:
                entries.append(
                    {
                        "coil_a": int(parts.get("coil_a", "0")),
                        "coil_b": int(parts.get("coil_b", "0")),
                        "raw": parts.get("raw", "0x0"),
                    }
                )
            except (ValueError, KeyError):
                pass
    return entries


def read_emulator_gconf_log(line_id):
    """Read GCONF writes from the emulator's stdout log."""
    log_path = LOGDIR / f"tmc_emu_{line_id}.log"
    if not log_path.exists():
        return []
    entries = []
    for line in log_path.read_text(errors="replace").splitlines():
        if line.startswith("GCONF "):
            parts = {}
            for token in line.split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    parts[k] = v
            entries.append(
                {
                    "raw": parts.get("raw", "0x0"),
                    "direct_mode": parts.get("direct_mode", "False") == "True",
                }
            )
    return entries


def main():
    print("[preload] cleaning up prior processes")
    cleanup_prior()

    print("[preload] spawning TMC5160 emulators")
    tmc_procs = spawn_tmc_emulators()

    print("[preload] spawning klipper.elf")
    elf = spawn_elf()
    print("[preload] spawning klippy")
    klippy = spawn_klippy()

    try:
        # ── Gate 1: GCONF.direct_mode was set ──
        print("[preload] checking GCONF.direct_mode was set on emulators")
        gconf_27 = read_emulator_gconf_log(27)
        gconf_26 = read_emulator_gconf_log(26)
        dm_27 = any(e["direct_mode"] for e in gconf_27)
        dm_26 = any(e["direct_mode"] for e in gconf_26)
        print(
            f"  CS27 GCONF writes: {len(gconf_27)}, direct_mode seen: {dm_27}"
        )
        print(
            f"  CS26 GCONF writes: {len(gconf_26)}, direct_mode seen: {dm_26}"
        )
        if not dm_27 and not dm_26:
            print(
                "\n[preload] FAIL: GCONF.direct_mode never set on any TMC5160"
            )
            return 1

        # ── Gate 2: fake-home triggers stepper enable ──
        print("[preload] fake-homing: SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
        resp = send_gcode("SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
        print(f"  response: {resp}")

        # Allow time for stepper enable callbacks to fire and SPI to settle
        time.sleep(3.0)

        # ── Gate 3: XDIRECT preloaded with non-zero current ──
        # After stepper enable, XDIRECT must have been written with non-zero
        # coil currents.  This is the preload that prevents zero-current
        # silence between direct_mode enable and the first ISR write.
        print("[preload] checking XDIRECT preload on emulators")
        xd_27 = read_emulator_xdirect_log(27)
        xd_26 = read_emulator_xdirect_log(26)
        print(f"  CS27 (stepper_x) XDIRECT writes: {len(xd_27)}")
        for e in xd_27[:5]:
            print(
                f"    coil_a={e['coil_a']} coil_b={e['coil_b']} raw={e['raw']}"
            )
        print(f"  CS26 (stepper_y) XDIRECT writes: {len(xd_26)}")
        for e in xd_26[:5]:
            print(
                f"    coil_a={e['coil_a']} coil_b={e['coil_b']} raw={e['raw']}"
            )

        nonzero_27 = any(e["coil_a"] != 0 or e["coil_b"] != 0 for e in xd_27)
        nonzero_26 = any(e["coil_a"] != 0 or e["coil_b"] != 0 for e in xd_26)

        if not nonzero_27 and not nonzero_26:
            if not xd_27 and not xd_26:
                print(
                    "\n[preload] FAIL: no XDIRECT writes at all — "
                    "phase stepping never wrote coil currents"
                )
            else:
                print(
                    "\n[preload] FAIL: XDIRECT written but all values are zero — "
                    "motor would not energize (no holding current)"
                )
            return 1

        print(
            f"\n[preload] PASS: XDIRECT preloaded with non-zero current "
            f"(CS27={'yes' if nonzero_27 else 'no'} "
            f"CS26={'yes' if nonzero_26 else 'no'})"
        )
        return 0

    finally:
        print("\n[preload] tearing down")
        for p in (klippy, elf):
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                p.kill()
        for p in tmc_procs:
            try:
                p.terminate()
                p.wait(timeout=1)
            except Exception:
                p.kill()


if __name__ == "__main__":
    sys.exit(main())
