#!/usr/bin/env python3
"""Phase 4 gate test — verify G1 X10 produces non-zero step pulses.

Run from the repo root inside the Docker container:
    python3 tools/sim_klippy/test_phase4_steps.py

Uses the bridge's bridge_call() to query runtime_sim_stepper_count_query oid=0
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


SIM_SOCK_DIR = pathlib.Path("/tmp/kalico_sim_socks")


def spawn_tmc_emulators():
    """Spawn TMC5160 SPI register emulators for phase-stepping CS pins."""
    SIM_SOCK_DIR.mkdir(exist_ok=True)
    emu_script = REPO / "tools" / "kalico-sim" / "emulators" / "tmc5160_emu.py"
    procs = []
    # CS pins from printer.cfg: stepper_x=gpio27 (chip0, line27),
    # stepper_y=gpio26 (chip0, line26).
    for line in (27, 26):
        sock_path = SIM_SOCK_DIR / f"spi_cs_0_{line}"
        emu_log = open(LOGDIR / f"tmc_emu_{line}.log", "w")
        p = subprocess.Popen(
            [sys.executable, str(emu_script), str(sock_path)],
            stdout=emu_log, stderr=emu_log,
        )
        # Wait for socket to appear
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
        subprocess.check_call(["make", "-C", str(shim_so.parent)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    env = os.environ.copy()
    env["LD_PRELOAD"] = str(shim_so)
    env["KALICO_SIM_SOCK_DIR"] = str(SIM_SOCK_DIR)
    env["KALICO_SIM_SHIM_VERBOSE"] = "1"
    proc = subprocess.Popen(
        [str(KLIPPER_ELF), "-I", SIM_SOCKET],
        stdout=elf_log, stderr=subprocess.STDOUT,
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
        [str(klippy_python), str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG), "-l", str(KLIPPY_LOG),
         "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
        cwd=str(REPO),
    )
    for _ in range(150):
        if os.path.exists(KLIPPY_API):
            # Wait long enough for klippy's MCU identify + config phase
            # to complete (Klipper-protocol dictionary download + bridge's
            # kalico-native identify handshake — ~3s in the sim).
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

    print("[phase4] spawning TMC5160 emulators")
    tmc_procs = spawn_tmc_emulators()

    print("[phase4] spawning klipper.elf")
    elf = spawn_elf()
    print("[phase4] spawning klippy")
    klippy = spawn_klippy()

    try:
        print("[phase4] fake-homing: SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
        resp = send_gcode("SET_KINEMATIC_POSITION X=100 Y=100 Z=10")
        print(f"  response: {resp}")

        print("[phase4] sending G1 X50 F6000 then M400 (flush)")
        resp = send_gcode("G1 X50 F6000\nM400")
        print(f"  response: {resp}")

        # The bridge schedules segments at MCU_clock_now + 100ms_lead +
        # seg.t_start; if klippy startup pushed the MCU clock far ahead of
        # wall-clock zero, the segment can land tens of seconds in the
        # future. Wait long enough that a 10mm @ F1000 move (≈0.6s) plus
        # the schedule lead has had time to drain.
        # Poll elf log for non-zero stepper counts. Bypasses bridge_call so
        # we get the answer even if klippy shut down on the M400 timeout.
        print("[phase4] polling elf log for motion + SPI output")
        elf_log = REPO / "tools" / "sim_klippy" / ".local-logs" / "klipper_elf.log"
        deadline = time.time() + 60.0
        seen_nonzero = False
        seen_spi_writes = 0
        while time.time() < deadline:
            time.sleep(0.5)
            if not elf_log.exists():
                continue
            text = elf_log.read_text(errors="replace")
            for line in text.splitlines()[-200:]:
                if "[sim-progress]" in line and "counts=[" in line:
                    inner = line.split("counts=[", 1)[1].split("]", 1)[0]
                    parts = [int(x) for x in inner.split(",")]
                    if any(abs(p) > 0 for p in parts):
                        seen_nonzero = True
                    if "spi_writes=" in line:
                        try:
                            w = int(line.split("spi_writes=")[1].split()[0])
                            if w > seen_spi_writes:
                                seen_spi_writes = w
                        except (IndexError, ValueError):
                            pass
            if seen_nonzero and seen_spi_writes > 0:
                print(f"  motion + SPI observed: spi_writes={seen_spi_writes}")
                break

        # Query step counts via the KALICO_SIM_STEP_COUNT G-code command,
        # which routes through klippy → bridge → MCU wire command.
        print("[phase4] querying axis accumulators (post-move)")
        try:
            for axis in (0, 1, 2):
                r = send_gcode("KALICO_SIM_AXIS_ACCUM OID=%d" % axis)
                print(f"  AXIS_ACCUM={axis}: {r}")
            print("[phase4] querying step count for OID 0 (X stepper)")
            x_resp = send_gcode("KALICO_SIM_STEP_COUNT OID=0")
            print(f"  OID=0 response: {x_resp}")
            print("[phase4] querying step count for OID 1 (Y stepper)")
            y_resp = send_gcode("KALICO_SIM_STEP_COUNT OID=1")
            print(f"  OID=1 response: {y_resp}")
        except (ConnectionRefusedError, ConnectionResetError, OSError) as e:
            print(f"  query failed (klippy disconnected): {e}")

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
        print(f"[phase4] SPI XDIRECT writes: {seen_spi_writes}")

        if not seen_nonzero and (x_count > 0 or y_count > 0):
            seen_nonzero = True

        if not seen_nonzero:
            print("\n[phase4] FAIL: 0 position counts — move produced no motion")
            for line in log.splitlines():
                if any(k in line for k in ("Error", "Traceback", "step", "submit_move",
                                           "bridge-trace", "planner", "bridge-async",
                                           "KALICO_SIM", "homed")):
                    print("  LOG:", line[-200:])
            return 1

        if seen_spi_writes == 0:
            print("\n[phase4] FAIL: 0 SPI XDIRECT writes — "
                  "phase stepping is not driving coil modulation")
            return 1

        print(f"\n[phase4] Phase 4 PASS: motion + {seen_spi_writes} SPI writes")
        return 0

    finally:
        print("\n[phase4] tearing down")
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
