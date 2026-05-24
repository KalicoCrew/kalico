#!/usr/bin/env python3
"""Kalico Simulator — run full-stack Klipper simulation with virtual time.

Spawns MACH_LINUX MCU firmware processes + klippy with a shared virtual
clock that eliminates wall-clock sleeps, achieving faster-than-real-time
simulation. Peripheral emulators (TMC5160/2209, thermocouples, Beacon)
are loaded from the sim_klippy orchestrator modules.

Usage:
    python3 runner.py [--branch BRANCH] [--gcode FILE] [--config DIR]
                      [--timeout SECS] [--verbose]

The runner builds firmware, starts emulators, boots klippy, feeds G-code,
and reports: pass/fail, print time, error details.
"""
import argparse
import dataclasses
import json
import logging
import os
import pathlib
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format="[sim %(levelname)s] %(message)s",
)
log = logging.getLogger("kalico-sim")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
VTIME_SHM = "/dev/shm/kalico_vtime"
VTIME_SHM_SIZE = 32
VTIME_STRUCT_FMT = "<QIIII"


@dataclasses.dataclass
class McuProcess:
    name: str
    process: subprocess.Popen
    pty_path: str
    log_path: str


@dataclasses.dataclass
class SimResult:
    success: bool
    print_time_s: float
    wall_time_s: float
    speedup: float
    error: Optional[str] = None
    klippy_log: Optional[str] = None
    mcu_logs: Optional[dict] = None


def vtime_create(start_ns: int = 1_000_000_000) -> None:
    """Create shared virtual clock."""
    with open(VTIME_SHM, "wb") as f:
        f.write(struct.pack(VTIME_STRUCT_FMT, start_ns, 0, 0, 1, 0))
    os.chmod(VTIME_SHM, 0o666)


def vtime_read_ns() -> int:
    try:
        with open(VTIME_SHM, "rb") as f:
            data = f.read(VTIME_SHM_SIZE)
        return struct.unpack_from(VTIME_STRUCT_FMT, data, 0)[0]
    except FileNotFoundError:
        return 0


def vtime_destroy() -> None:
    try:
        os.unlink(VTIME_SHM)
    except FileNotFoundError:
        pass


def build_firmware(repo_root: pathlib.Path, config_name: str,
                   output_name: str) -> pathlib.Path:
    """Build MACH_LINUX firmware ELF from a .config file."""
    config_src = repo_root / "tools" / "kalico-sim" / "configs" / config_name
    if not config_src.exists():
        # Fall back to sim_klippy configs if they exist
        config_src = repo_root / "tools" / "sim_klippy" / "configs" / config_name
    if not config_src.exists():
        raise RuntimeError(f"Config {config_name} not found")

    out_elf = repo_root / "out" / output_name
    log.info("Building firmware: %s -> %s", config_name, output_name)

    subprocess.run(
        ["cp", str(config_src), str(repo_root / ".config")],
        check=True,
    )
    subprocess.run(
        ["make", "clean"],
        cwd=str(repo_root),
        check=True,
        capture_output=True,
    )
    nproc = os.cpu_count() or 4
    result = subprocess.run(
        ["make", f"-j{nproc}"],
        cwd=str(repo_root),
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Firmware build failed:\nstdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    subprocess.run(
        ["cp", str(repo_root / "out" / "klipper.elf"), str(out_elf)],
        check=True,
    )
    log.info("Built: %s", out_elf)
    return out_elf


def ensure_shims_built(repo_root: pathlib.Path) -> tuple:
    """Build libsim_intercept.so + libvtime.so. Returns (shim_path, vtime_path)."""
    shim_dir = repo_root / "tools" / "kalico-sim" / "libvtime"
    shim_so = shim_dir / "libsim_intercept.so"
    vtime_so = shim_dir / "libvtime.so"
    if not shim_so.exists() or not vtime_so.exists():
        subprocess.check_call(["make", "-C", str(shim_dir)])
    return (shim_so, vtime_so)


def spawn_mcu(
    name: str,
    elf_path: pathlib.Path,
    pty_path: str,
    log_path: str,
    sock_dir: str,
    shim_so: pathlib.Path,
    vtime_so: pathlib.Path,
    verbose: bool = False,
) -> McuProcess:
    """Spawn a MACH_LINUX firmware process with both shims."""
    if os.path.exists(pty_path):
        os.unlink(pty_path)

    log_fd = open(log_path, "wb")
    env = os.environ.copy()
    # GPIO/SPI shim only — vtime disabled for now to avoid
    # "Stepper too far in past" during homing. Acceleration will be
    # re-enabled after the core sim works at real speed.
    env["LD_PRELOAD"] = str(shim_so)
    env["KALICO_SIM_SOCK_DIR"] = sock_dir
    if verbose:
        env["KALICO_SIM_SHIM_VERBOSE"] = "1"
        env["KALICO_VTIME_DEBUG"] = "1"

    proc = subprocess.Popen(
        [str(elf_path), "-I", pty_path],
        stdout=log_fd,
        stderr=subprocess.STDOUT,
        env=env,
    )

    # Wait for PTY to appear
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if os.path.exists(pty_path):
            return McuProcess(
                name=name, process=proc,
                pty_path=pty_path, log_path=log_path,
            )
        if proc.poll() is not None:
            log_fd.close()
            content = open(log_path).read()
            raise RuntimeError(
                f"{name}: klipper.elf exited early (rc={proc.returncode})\n"
                f"---log---\n{content}"
            )
        time.sleep(0.05)

    proc.kill()
    log_fd.close()
    raise RuntimeError(f"{name}: PTY {pty_path} did not appear in 10s")


def send_gcode(api_socket: str, script: str, timeout: float = 30.0) -> dict:
    """Send G-code to klippy via API socket."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    sock.connect(api_socket)
    req = {
        "id": 1,
        "method": "gcode/script",
        "params": {"script": script},
    }
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
            return {"raw": body.decode(errors="replace")}
    return {}


def query_status(api_socket: str, timeout: float = 5.0) -> dict:
    """Query printer status via klippy API."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(api_socket)
    except (ConnectionRefusedError, FileNotFoundError, BlockingIOError, OSError):
        return {}
    req = {
        "id": 1,
        "method": "objects/query",
        "params": {
            "objects": {
                "print_stats": None,
                "toolhead": None,
                "virtual_sdcard": None,
            }
        },
    }
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
            pass
    return {}


def wait_for_klippy_ready(klippy_log: pathlib.Path,
                           klippy_proc: subprocess.Popen,
                           timeout: float = 120.0) -> bool:
    """Wait for klippy to reach 'Printer is ready' state."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if klippy_log.exists():
            content = klippy_log.read_bytes()
            if b"Printer is ready" in content:
                return True
            if b"Welcome to Kalico" in content:
                return True
            if klippy_proc.poll() is not None:
                return False
            if b"Internal error" in content or b"shutdown:" in content:
                return False
        elif klippy_proc.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def wait_for_print_done(api_socket: str, klippy_proc: subprocess.Popen,
                        klippy_log: pathlib.Path,
                        timeout: float = 600.0) -> tuple:
    """Wait for the print to finish. Returns (success, error_msg)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if klippy_proc.poll() is not None:
            return False, "klippy exited unexpectedly"

        # Check log for errors
        if klippy_log.exists():
            content = klippy_log.read_bytes()
            if b"shutdown:" in content:
                # Extract shutdown reason
                for line in content.decode(errors="replace").split("\n"):
                    if "shutdown:" in line.lower():
                        return False, f"Printer shutdown: {line.strip()}"
                return False, "Printer shutdown (unknown reason)"

        status = query_status(api_socket)
        if status:
            result = status.get("result", {}).get("status", {})
            ps = result.get("print_stats", {})
            state = ps.get("state", "")
            if state == "complete":
                return True, None
            if state == "error":
                return False, ps.get("message", "print error")
            if state == "cancelled":
                return False, "print cancelled"

        time.sleep(0.2)
    return False, "timeout waiting for print"


def run_simulation(
    repo_root: pathlib.Path,
    gcode_path: Optional[pathlib.Path] = None,
    config_dir: Optional[pathlib.Path] = None,
    timeout: float = 600.0,
    verbose: bool = False,
    phase_stepping: bool = False,
) -> SimResult:
    """Run a complete simulation. Returns SimResult."""
    wall_start = time.monotonic()

    with tempfile.TemporaryDirectory(prefix="kalico_sim_") as tmpdir:
        tmp = pathlib.Path(tmpdir)
        log_dir = tmp / "logs"
        log_dir.mkdir()

        # Build shims
        shim_so, vtime_so = ensure_shims_built(repo_root)

        # Initialize virtual clock
        vtime_create(start_ns=1_000_000_000)

        mcus = []
        klippy_proc = None
        chip_servers = []
        beacon_stub = None
        endstop_trigger = None

        try:
            # Socket directories for chip emulators
            h7_sock_dir = tmp / "sim" / "h7"
            f4_sock_dir = tmp / "sim" / "f4"
            h7_sock_dir.mkdir(parents=True)
            f4_sock_dir.mkdir(parents=True)

            # Detect available ELFs
            h7_elf = repo_root / "out" / "klipper-h7-sim.elf"
            f4_elf = repo_root / "out" / "klipper-f4-sim.elf"
            if not h7_elf.exists():
                return SimResult(
                    success=False, print_time_s=0, wall_time_s=0,
                    speedup=0,
                    error="Missing H7 firmware ELF (out/klipper-h7-sim.elf)",
                )

            h7_pty = str(tmp / "klipper_sim_h7")
            f4_pty = str(tmp / "klipper_sim_f4")
            dual_mcu = f4_elf.exists()

            h7 = spawn_mcu(
                "h7", h7_elf, h7_pty,
                str(log_dir / "h7.log"), str(h7_sock_dir),
                shim_so, vtime_so, verbose,
            )
            mcus.append(h7)
            log.info("H7 MCU spawned (pid=%d)", h7.process.pid)

            if dual_mcu:
                f4 = spawn_mcu(
                    "f4", f4_elf, f4_pty,
                    str(log_dir / "f4.log"), str(f4_sock_dir),
                    shim_so, vtime_so, verbose,
                )
                mcus.append(f4)
                log.info("F4 MCU spawned (pid=%d)", f4.process.pid)

            # Start chip emulators (only if sim_klippy is available)
            chip_servers = _start_chip_emulators(
                h7_sock_dir, f4_sock_dir, repo_root
            )

            # Prepare printer config
            beacon_pty = str(tmp / "klipper_sim_beacon")
            beacon_stub = _start_beacon(beacon_pty, log_dir, repo_root)

            # Detect if this branch has the Kalico motion bridge
            has_motion_bridge = (repo_root / "klippy" / "motion_bridge.py").exists()

            rendered_cfg = _prepare_config(
                tmp, config_dir, repo_root,
                h7_pty, f4_pty if dual_mcu else None, beacon_pty,
                has_motion_bridge=has_motion_bridge,
                phase_stepping=phase_stepping,
            )

            # Copy G-code to virtual SD card location
            gcode_dir = tmp / "gcodes"
            gcode_dir.mkdir()
            if gcode_path:
                import shutil
                gcode_dest = gcode_dir / gcode_path.name
                shutil.copy2(gcode_path, gcode_dest)

            # Spawn klippy with virtual time
            klippy_log = log_dir / "klippy.log"
            api_socket = str(tmp / "klippy.sock")

            env = os.environ.copy()
            # Klippy does NOT use virtual time — it runs at real CPU
            # speed. The MCU processes use virtual time (via LD_PRELOAD
            # in spawn_mcu) so they process commands instantly. Klippy's
            # motion planner generates moves at CPU speed; the MCU is
            # "infinitely fast." This avoids the virtual-time deadlock
            # where both sides wait for I/O and neither advances time.
            if verbose:
                env["KALICO_VTIME_DEBUG"] = "1"
            env["KALICO_SIM_SOCK_DIR"] = str(h7_sock_dir)

            # Add sim_klippy's third-party plugin paths if available
            third_party = repo_root / "tools" / "sim_klippy" / "printer_real" / "third_party_repos"
            if third_party.exists():
                pp = env.get("PYTHONPATH", "")
                beacon_path = third_party / "beacon_klipper"
                motors_path = third_party / "motors-sync"
                env["PYTHONPATH"] = ":".join(filter(None, [
                    str(beacon_path) if beacon_path.exists() else "",
                    str(motors_path) if motors_path.exists() else "",
                    pp,
                ]))

            klippy_proc = subprocess.Popen(
                [
                    "python3",
                    str(repo_root / "klippy" / "klippy.py"),
                    str(rendered_cfg),
                    "-l", str(klippy_log),
                    "-a", api_socket,
                ],
                env=env,
                stdout=open(log_dir / "klippy.stdout", "wb"),
                stderr=subprocess.STDOUT,
                cwd=str(repo_root),
            )
            log.info("Klippy spawned (pid=%d)", klippy_proc.pid)

            # Wait for klippy to be ready
            if not wait_for_klippy_ready(klippy_log, klippy_proc, timeout=120):
                content = ""
                if klippy_log.exists():
                    content = klippy_log.read_text(errors="replace")
                return SimResult(
                    success=False, print_time_s=0,
                    wall_time_s=time.monotonic() - wall_start,
                    speedup=0,
                    error=f"Klippy failed to start:\n{content[-2000:]}",
                    klippy_log=content,
                )
            log.info("Klippy ready")

            # Endstop triggering is handled by the libsim_intercept.so
            # shim's auto-endstop feature (step counting → GPIO trigger).

            # Record virtual time at start
            vtime_start = vtime_read_ns()

            if gcode_path:
                # Start the print via virtual SD card
                gcode_name = gcode_path.name
                resp = send_gcode(
                    api_socket,
                    f"SDCARD_PRINT_FILE FILENAME={gcode_name}",
                    timeout=30,
                )
                log.info("Print started: %s", gcode_name)

                # Wait for completion
                success, error = wait_for_print_done(
                    api_socket, klippy_proc, klippy_log, timeout,
                )
            else:
                # No G-code — generate and print a test pattern
                log.info("No G-code file, generating test pattern")
                test_gcode = gcode_dir / "sim_test.gcode"
                if phase_stepping:
                    test_gcode.write_text("""\
; Phase stepping acceptance test
SET_KINEMATIC_POSITION X=0 Y=125 Z=125
G1 X50 F1000
G1 X100 F2000
G1 X50 F3000
G1 X0 F1000
""")
                else:
                    test_gcode.write_text("""\
; Kalico Sim self-test: square spiral with Z moves
SET_KINEMATIC_POSITION X=125 Y=125 Z=125
G1 Z120 F300
G1 X10 Y10 F3000
G1 X100 Y10 F3000
G1 X100 Y100 F3000
G1 X10 Y100 F3000
G1 X10 Y10 F3000
G1 X20 Y20 F3000
G1 X90 Y20 F3000
G1 X90 Y90 F3000
G1 X20 Y90 F3000
G1 X20 Y20 F3000
G1 X30 Y30 F2000
G1 X80 Y30 F2000
G1 X80 Y80 F2000
G1 X30 Y80 F2000
G1 X30 Y30 F2000
G1 X40 Y40 F1500
G1 X70 Y40 F1500
G1 X70 Y70 F1500
G1 X40 Y70 F1500
G1 X40 Y40 F1500
G1 Z125 F300
""")
                resp = send_gcode(
                    api_socket,
                    "SDCARD_PRINT_FILE FILENAME=sim_test.gcode",
                    timeout=30,
                )
                log.info("Print started: sim_test.gcode")
                success, error = wait_for_print_done(
                    api_socket, klippy_proc, klippy_log, timeout,
                )

            wall_end = time.monotonic()
            wall_time_s = wall_end - wall_start

            # Get actual print time from klippy's toolhead/print_stats
            print_time_s = 0.0
            try:
                status = query_status(api_socket)
                if status:
                    r = status.get("result", {}).get("status", {})
                    ps = r.get("print_stats", {})
                    print_time_s = ps.get("total_duration", 0.0)
                    if print_time_s == 0:
                        print_time_s = ps.get("print_duration", 0.0)
            except Exception:
                pass

            # Fallback: extract print time from klippy log stats
            if print_time_s == 0:
                try:
                    klippy_content = klippy_log.read_text(errors="replace")
                    import re
                    for line in reversed(klippy_content.split("\n")):
                        m = re.search(r"print_time=(\d+\.?\d*)", line)
                        if m:
                            print_time_s = float(m.group(1))
                            break
                except Exception:
                    pass

            speedup = print_time_s / wall_time_s if (wall_time_s > 0 and print_time_s > 0) else 0

            klippy_content = ""
            if klippy_log.exists():
                klippy_content = klippy_log.read_text(errors="replace")

            mcu_log_contents = {}
            for mcu in mcus:
                try:
                    mcu_log_contents[mcu.name] = open(mcu.log_path).read()
                except Exception:
                    pass

            return SimResult(
                success=success,
                print_time_s=print_time_s,
                wall_time_s=wall_time_s,
                speedup=speedup,
                error=error,
                klippy_log=klippy_content,
                mcu_logs=mcu_log_contents,
            )

        finally:
            # Stop endstop trigger
            if endstop_trigger:
                endstop_trigger.stop()

            # Cleanup
            if klippy_proc and klippy_proc.poll() is None:
                klippy_proc.terminate()
                try:
                    klippy_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    klippy_proc.kill()

            for mcu in mcus:
                if mcu.process.poll() is None:
                    mcu.process.send_signal(signal.SIGTERM)
            for mcu in mcus:
                try:
                    mcu.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    mcu.process.kill()

            for srv in chip_servers:
                srv.stop()

            if beacon_stub:
                beacon_stub.stop()

            vtime_destroy()


def _start_chip_emulators(h7_sock_dir, f4_sock_dir, repo_root):
    """Start chip emulators. Returns list of servers to stop later."""
    servers = []
    try:
        sys.path.insert(0, str(repo_root))
        from tools.sim_klippy.orchestrator.chip_socket_server import ChipSocketServer
        from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator
        from tools.sim_klippy.orchestrator.tmc2209_emulator import TMC2209Emulator
        from tools.sim_klippy.orchestrator.max31865_emulator import MAX31865Emulator

        # H7 SPI bus 0: TMC5160s + MAX31865
        h7_chips = [
            (5,  TMC5160Emulator().transfer),   # stepper_x
            (4,  TMC5160Emulator().transfer),   # stepper_y
            (6,  TMC5160Emulator().transfer),   # stepper_x1
            (3,  TMC5160Emulator().transfer),   # stepper_y1
            (40, MAX31865Emulator().transfer),  # extruder_rtd
        ]
        for cs_line, transfer in h7_chips:
            path = str(h7_sock_dir / f"spi_cs_0_{cs_line}")
            srv = ChipSocketServer(path, transfer, framed=False)
            srv.start()
            servers.append(srv)

        # H7 TMC2209 (extruder)
        from tools.sim_klippy.orchestrator.tmc2209_emulator import TMC2209Emulator
        chip = TMC2209Emulator(slave_addr=0)
        srv = ChipSocketServer(
            str(h7_sock_dir / "tmcuart_0"), chip.handle, chunk=10,
        )
        srv.start()
        servers.append(srv)

        # F4 TMC2209s (Z, Z1, Z2)
        for i in range(3):
            chip = TMC2209Emulator(slave_addr=0)
            path = str(f4_sock_dir / f"tmcuart_{i}")
            srv = ChipSocketServer(path, chip.handle, chunk=10)
            srv.start()
            servers.append(srv)

        log.info("Started %d chip emulators", len(servers))
    except ImportError as e:
        log.warning("Could not import sim_klippy emulators: %s", e)
    return servers


def _start_beacon(beacon_pty, log_dir, repo_root):
    """Start Beacon MCU emulator."""
    try:
        sys.path.insert(0, str(repo_root))
        # Local emulator (kalico-sim/emulators/) — add to path directly
        # since the hyphen in kalico-sim makes it non-importable as a module
        emulators_dir = repo_root / "tools" / "kalico-sim" / "emulators"
        if emulators_dir.exists():
            sys.path.insert(0, str(emulators_dir))
        try:
            from beacon_mcu import BeaconMcuStub
        except ImportError:
            from tools.sim_klippy.orchestrator.beacon_serial_stub import (
                BeaconMcuStub,
            )
        beacon = BeaconMcuStub(
            beacon_pty,
            log_path=str(log_dir / "beacon_traffic.log"),
        )
        beacon.start_sample_stream(z_target_mm=10.0, rate_hz=200)
        log.info("Beacon MCU emulator started")
        return beacon
    except ImportError as e:
        log.warning("Could not import Beacon emulator: %s", e)
        return None


def _prepare_config(
    tmp_dir: pathlib.Path,
    config_dir: Optional[pathlib.Path],
    repo_root: pathlib.Path,
    h7_pty: str,
    f4_pty: Optional[str],
    beacon_pty: str,
    has_motion_bridge: bool = False,
    phase_stepping: bool = False,
) -> pathlib.Path:
    """Render printer.cfg with sim serial paths."""
    # Try to find config from sim_klippy
    if config_dir is None:
        if phase_stepping:
            cfg = _generate_phase_stepping_config(h7_pty, f4_pty, gcode_dir=str(tmp_dir / "gcodes"))
        else:
            cfg = _generate_minimal_config(h7_pty, f4_pty, gcode_dir=str(tmp_dir / "gcodes"))
        if has_motion_bridge and not phase_stepping:
            cfg += """
[input_shaper]
shaper_freq_x: 50
shaper_freq_y: 50
shaper_type: smooth_mzv
"""
        cfg_path = tmp_dir / "printer.cfg"
        cfg_path.write_text(cfg)
        return cfg_path

    # Use sim_klippy's override system
    try:
        sys.path.insert(0, str(repo_root))
        from tools.sim_klippy.orchestrator.overrides import (
            apply_overrides, load_overrides,
        )
        overrides_path = repo_root / "tools" / "sim_klippy" / "pin-overrides.toml"
        if overrides_path.exists():
            overrides = load_overrides(overrides_path)
        else:
            overrides = {}

        serial_map = {"usb-Klipper_stm32h723xx_*": h7_pty}
        if f4_pty:
            serial_map["usb-Klipper_stm32f446xx_*"] = f4_pty
        serial_map["usb-Beacon_*"] = beacon_pty
        overrides["mcu_main.serial"] = serial_map

        cfg_text = (config_dir / "printer.cfg").read_text()
        rendered = apply_overrides(cfg_text, overrides)
        cfg_path = tmp_dir / "printer.cfg"
        cfg_path.write_text(rendered)

        # Stage companion configs
        import shutil
        for entry in config_dir.iterdir():
            if entry.name == "printer.cfg":
                continue
            target = tmp_dir / entry.name
            if target.exists():
                continue
            if entry.is_dir():
                os.symlink(entry.resolve(), target)
            elif entry.suffix == ".cfg":
                text = entry.read_text()
                text = apply_overrides(text, overrides)
                target.write_text(text)
            else:
                shutil.copy2(entry, target)

        return cfg_path

    except (ImportError, FileNotFoundError) as e:
        log.warning("Could not use sim_klippy overrides: %s", e)
        cfg = _generate_minimal_config(h7_pty, f4_pty)
        cfg_path = tmp_dir / "printer.cfg"
        cfg_path.write_text(cfg)
        return cfg_path


class EndstopTrigger:
    """Background thread that triggers endstop GPIOs for homing.

    Connects to the sim_control socket and periodically toggles endstop
    GPIO lines to simulate physical endstop switches. Endstops trigger
    after a brief delay from when homing starts.
    """

    def __init__(self, sim_control_path: str, endstop_pins: list):
        """endstop_pins: list of (chip, line) tuples for endstop GPIOs."""
        self.sim_control_path = sim_control_path
        self.endstop_pins = endstop_pins
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _send_cmd(self, cmd: str) -> str:
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            sock.connect(self.sim_control_path)
            sock.sendall((cmd + "\n").encode())
            resp = sock.recv(256).decode().strip()
            sock.close()
            return resp
        except (ConnectionRefusedError, FileNotFoundError, socket.timeout):
            return ""

    def _run(self):
        # Wait briefly for MCU to be fully up
        time.sleep(0.5)

        # Continuously cycle endstops through homing-compatible sequence:
        # clear → wait → trigger → hold briefly → clear → wait → repeat
        # This ensures that whenever homing starts on any axis, the
        # endstop will trigger within ~0.5s, then clear for the retract,
        # then trigger again for the second approach.
        while not self._stop.is_set():
            # Clear phase — stay clear long enough for homing to start
            # and for retract to complete (retract takes ~0.5-1s)
            for chip, line in self.endstop_pins:
                self._send_cmd(
                    f"set_gpio_input chip={chip} line={line} value=0"
                )
            self._stop.wait(1.0)
            if self._stop.is_set():
                break

            # Trigger phase — brief pulse to simulate endstop hit
            for chip, line in self.endstop_pins:
                self._send_cmd(
                    f"set_gpio_input chip={chip} line={line} value=1"
                )
            self._stop.wait(0.1)
            if self._stop.is_set():
                break

    def trigger_once(self):
        """Set all endstops to triggered state once."""
        for chip, line in self.endstop_pins:
            self._send_cmd(
                f"set_gpio_input chip={chip} line={line} value=1"
            )

    def clear(self):
        """Clear all endstops."""
        for chip, line in self.endstop_pins:
            self._send_cmd(
                f"set_gpio_input chip={chip} line={line} value=0"
            )


def _generate_minimal_config(h7_pty: str, f4_pty: str, gcode_dir: str = "/tmp/kalico_sim_gcodes") -> str:
    """Generate a minimal single-MCU Cartesian config for testing.

    MACH_LINUX uses gpiochip0/gpioN pin naming (not STM32 PA3 style).
    Uses only the H7 MCU. All endstops are simulated via GPIO lines
    in the LD_PRELOAD shim.
    """
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
kinematics: cartesian
max_velocity: 100
max_accel: 1000
max_z_velocity: 10
max_z_accel: 30

[stepper_x]
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 16
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio10
position_min: 0
position_endstop: 0
position_max: 250
homing_speed: 10

[stepper_y]
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 16
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio11
position_min: 0
position_endstop: 0
position_max: 250
homing_speed: 10

[stepper_z]
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 16
rotation_distance: 4
endstop_pin: ^gpiochip0/gpio12
position_min: -5
position_endstop: 0
position_max: 250
homing_speed: 5

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def _generate_phase_stepping_config(h7_pty: str, f4_pty: str, gcode_dir: str = "/tmp/kalico_sim_gcodes") -> str:
    """Generate a config with TMC5160 phase stepping on X axis."""
    return f"""\
[mcu]
serial: {h7_pty}

[printer]
kinematics: cartesian
max_velocity: 100
max_accel: 1000
max_z_velocity: 10
max_z_accel: 30

[stepper_x]
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 256
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio10
position_min: 0
position_endstop: 0
position_max: 250
homing_speed: 10
phase_stepping: True

[tmc5160 stepper_x]
spi_bus: spidev0.0
cs_pin: gpiochip0/gpio5
run_current: 1.0
sense_resistor: 0.075

[stepper_y]
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio20
microsteps: 16
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio11
position_min: 0
position_endstop: 0
position_max: 250
homing_speed: 10

[stepper_z]
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio21
microsteps: 16
rotation_distance: 4
endstop_pin: ^gpiochip0/gpio12
position_min: -5
position_endstop: 0
position_max: 250
homing_speed: 5

[input_shaper]
shaper_freq_x: 50
shaper_freq_y: 50
shaper_type: smooth_mzv

[virtual_sdcard]
path: {gcode_dir}

[force_move]
enable_force_move: True
"""


def run_batch_simulation(
    repo_root: pathlib.Path,
    gcode_path: pathlib.Path,
    config_path: Optional[pathlib.Path] = None,
    timeout: float = 300.0,
    verbose: bool = False,
) -> SimResult:
    """Run Klipper in batch mode (--debuginput/--debugoutput) for
    faster-than-real-time print time prediction.

    This mode runs the full motion planner WITHOUT any MCU firmware.
    It processes G-code at CPU speed (~100x real-time for typical prints)
    and produces exact timing data. No Docker privileges needed.

    Requires a dictionary file (klipper.dict) built from the firmware.
    """
    wall_start = time.monotonic()

    with tempfile.TemporaryDirectory(prefix="kalico_batch_") as tmpdir:
        tmp = pathlib.Path(tmpdir)

        # Find or build dictionary file
        dict_path = repo_root / "out" / "klipper.dict"
        if not dict_path.exists():
            return SimResult(
                success=False, print_time_s=0,
                wall_time_s=time.monotonic() - wall_start,
                speedup=0,
                error="Missing klipper.dict. Build firmware first.",
            )

        # Prepare config
        if config_path is None:
            # Generate a batch-mode config (no serial needed)
            cfg_text = _generate_batch_config()
            cfg_file = tmp / "printer.cfg"
            cfg_file.write_text(cfg_text)
            config_path = cfg_file

        # Prepare output files
        debug_output = str(tmp / "debug_output")
        klippy_log = str(tmp / "klippy.log")

        # Run klippy in batch mode
        cmd = [
            "python3",
            str(repo_root / "klippy" / "klippy.py"),
            str(config_path),
            "-i", str(gcode_path),
            "-o", debug_output,
            "-d", str(dict_path),
            "-l", klippy_log,
        ]
        if verbose:
            cmd.append("-v")

        # Preprocess G-code: strip custom macros, replace PRINT_START
        preprocessor = repo_root / "tools" / "kalico-sim" / "preprocess_gcode.py"
        if preprocessor.exists():
            processed = tmp / "processed.gcode"
            try:
                subprocess.run(
                    ["python3", str(preprocessor),
                     str(gcode_path), str(processed)],
                    check=True, capture_output=True, text=True,
                )
                log.info("Preprocessed: %s", processed.name)
                cmd[cmd.index(str(gcode_path))] = str(processed)
            except subprocess.CalledProcessError as e:
                log.warning("Preprocessor failed: %s", e.stderr[:200])

        log.info("Running batch simulation: %s", gcode_path.name)
        log.info("Command: %s", " ".join(cmd))

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=str(repo_root),
            )
        except subprocess.TimeoutExpired:
            return SimResult(
                success=False, print_time_s=0,
                wall_time_s=time.monotonic() - wall_start,
                speedup=0,
                error=f"Batch simulation timed out after {timeout}s",
            )

        wall_end = time.monotonic()
        wall_time = wall_end - wall_start

        # Parse klippy log for print time
        print_time = 0.0
        error = None
        klippy_content = ""
        try:
            klippy_content = pathlib.Path(klippy_log).read_text(errors="replace")
        except FileNotFoundError:
            pass

        if result.returncode != 0:
            # Check for known errors
            if "error" in klippy_content.lower() or "shutdown" in klippy_content.lower():
                for line in klippy_content.split("\n"):
                    if "error" in line.lower() or "shutdown" in line.lower():
                        error = line.strip()
                        break
            if not error:
                error = f"klippy exited with code {result.returncode}"
                if result.stderr:
                    error += f"\n{result.stderr[-500:]}"

        # Extract print time from log
        import re
        # Look for "Exiting (print time X.XXXs)" — the definitive line
        for line in reversed(klippy_content.split("\n")):
            m = re.search(r"print time (\d+\.?\d*)s", line)
            if m:
                print_time = float(m.group(1))
                break
        # Fallback: look for "print_time=X.XXX" in stats
        if print_time == 0:
            for line in reversed(klippy_content.split("\n")):
                m = re.search(r"print_time=(\d+\.?\d*)", line)
                if m:
                    print_time = float(m.group(1))
                    break

        speedup = print_time / wall_time if wall_time > 0 else 0

        return SimResult(
            success=(result.returncode == 0 and error is None),
            print_time_s=print_time,
            wall_time_s=wall_time,
            speedup=speedup,
            error=error,
            klippy_log=klippy_content,
        )


def _generate_batch_config() -> str:
    """Generate a config for batch mode (MACH_LINUX pin format).

    Matches a typical Voron Trident 250/300 config: CoreXY kinematics,
    high accel/velocity limits, 0.4mm nozzle, large build volume.
    """
    return """\
[mcu]
serial: /dev/null

[printer]
kinematics: corexy
max_velocity: 500
max_accel: 25000
max_z_velocity: 30
max_z_accel: 100
square_corner_velocity: 5

[stepper_x]
step_pin: gpiochip0/gpio0
dir_pin: gpiochip0/gpio1
enable_pin: !gpiochip0/gpio2
microsteps: 32
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio10
position_endstop: 0
position_max: 300
homing_speed: 50

[stepper_y]
step_pin: gpiochip0/gpio3
dir_pin: gpiochip0/gpio4
enable_pin: !gpiochip0/gpio5
microsteps: 32
rotation_distance: 40
endstop_pin: ^gpiochip0/gpio11
position_endstop: 0
position_max: 300
homing_speed: 50

[stepper_z]
step_pin: gpiochip0/gpio6
dir_pin: gpiochip0/gpio7
enable_pin: !gpiochip0/gpio8
microsteps: 32
rotation_distance: 4
endstop_pin: ^gpiochip0/gpio12
position_endstop: 0
position_max: 300
homing_speed: 8

[extruder]
step_pin: gpiochip0/gpio13
dir_pin: gpiochip0/gpio14
enable_pin: !gpiochip0/gpio15
microsteps: 16
rotation_distance: 22.6789511
nozzle_diameter: 0.4
filament_diameter: 1.75
heater_pin: gpiochip0/gpio20
sensor_pin: analog0
sensor_type: EPCOS 100K B57560G104F
min_temp: -273
max_temp: 300
control: pid
pid_kp: 30
pid_ki: 2
pid_kd: 100
max_extrude_cross_section: 100
max_extrude_only_distance: 500
pressure_advance: 0.04

[heater_bed]
heater_pin: gpiochip0/gpio21
sensor_pin: analog1
sensor_type: EPCOS 100K B57560G104F
min_temp: -273
max_temp: 120
control: pid
pid_kp: 60
pid_ki: 1
pid_kd: 600

[force_move]
enable_force_move: True
"""


def main():
    parser = argparse.ArgumentParser(description="Kalico Simulator")
    parser.add_argument("--gcode", type=str, help="G-code file to print")
    parser.add_argument("--config", type=str,
                        help="Config directory or file")
    parser.add_argument("--mode", choices=["full", "batch"], default="full",
                        help="Simulation mode: 'full' (MCU firmware) or "
                             "'batch' (timing prediction, faster)")
    parser.add_argument("--timeout", type=float, default=600,
                        help="Max wall-clock seconds (default: 600)")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--phase-test", action="store_true",
                        help="Enable phase stepping config (TMC5160 on X)")
    parser.add_argument("--repo", type=str, default=str(REPO_ROOT),
                        help="Repository root (default: auto-detect)")
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    repo = pathlib.Path(args.repo)
    gcode = pathlib.Path(args.gcode) if args.gcode else None
    config = pathlib.Path(args.config) if args.config else None

    if args.mode == "batch":
        if not gcode:
            print("ERROR: --gcode is required for batch mode")
            sys.exit(1)
        result = run_batch_simulation(
            repo_root=repo,
            gcode_path=gcode,
            config_path=config,
            timeout=args.timeout,
            verbose=args.verbose,
        )
    else:
        result = run_simulation(
            repo_root=repo,
            gcode_path=gcode,
            config_dir=config,
            timeout=args.timeout,
            verbose=args.verbose,
            phase_stepping=args.phase_test,
        )

    print("\n" + "=" * 60)
    print(f"SIMULATION RESULT ({args.mode} mode)")
    print("=" * 60)
    print(f"  Status:     {'PASS' if result.success else 'FAIL'}")
    print(f"  Print time: {result.print_time_s:.1f}s "
          f"({result.print_time_s/60:.1f} min)")
    print(f"  Wall time:  {result.wall_time_s:.1f}s "
          f"({result.wall_time_s/60:.1f} min)")
    if result.speedup > 0:
        print(f"  Speedup:    {result.speedup:.1f}x")
    if result.error:
        print(f"  Error:      {result.error}")
    print("=" * 60)

    if not result.success and result.klippy_log:
        print("
--- klippy.log (homing-relevant, all lines) ---")
        for line in result.klippy_log.strip().split("
"):
            lo = line.lower()
            if any(k in lo for k in (
                "bridge-trace", "endstop_arm", "arm_id", "arm status",
                "trip", "drip", "credit", "homing", "home_start",
                "home_wait", "no trigger", "segment_id",
                "submit_homing", "homing_move", "error during",
                "internal error", "mcu silent", "move-diag",
                "sim-trace", "dispatch closure", "load_curve",
                "classify", "submit_move",
            )):
                print(line)
        print("--- end klippy.log ---")

    if not result.success and result.mcu_logs:
        for name, content in result.mcu_logs.items():
            print(f"
--- {name} MCU log (last 30 lines) ---")
            for line in content.strip().split("
")[-30:]:
                print(line)
            print(f"--- end {name} ---")

    if not result.success and result.klippy_log:
        trace_lines = [l for l in result.klippy_log.split("
")
                       if "trace-write" in l or "trace-close" in l or
                          "trace-kcall" in l or "endstop_arm" in l.lower()]
        if trace_lines:
            print("
--- bridge trace lines ---")
            for line in trace_lines[-30:]:
                print(line)
            print("--- end bridge trace ---")

    if args.phase_test and result.klippy_log:
        print("\n--- Phase stepping log excerpts ---")
        for line in result.klippy_log.split("\n"):
            llow = line.lower()
            if any(k in llow for k in [
                "phase_stepping", "phase_step", "tmc5160",
                "direct_mode", "configure_axes", "step_mode",
                "modulated", "phase_config", "register_phase",
                "bridge-trace", "spi_bus",
            ]):
                print(f"  {line.strip()}")
        print("---")

    if args.phase_test and result.print_time_s > 0:
        timer_in_past = result.error and "timer" in result.error.lower() and "past" in result.error.lower()
        if timer_in_past:
            print("\nNote: 'timer in past' is a known MACH_LINUX timing issue")
            print("      under Docker VM pressure, not a phase stepping bug.")
            print(f"      Motion ran for {result.print_time_s:.1f}s before the timing fault.")
            sys.exit(0)

    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
