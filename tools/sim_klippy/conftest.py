"""Shared pytest fixtures for the faithful-sim test suite."""
import dataclasses
import json
import os
import pathlib
import shutil
import socket
import subprocess
import time
from typing import List, Optional

import pytest

from tools.sim_klippy.orchestrator.launcher import spawn_mcus, McuHandles
from tools.sim_klippy.orchestrator.chip_socket_server import ChipSocketServer
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator
from tools.sim_klippy.orchestrator.tmc2209_emulator import TMC2209Emulator
from tools.sim_klippy.orchestrator.max31865_emulator import MAX31865Emulator
from tools.sim_klippy.orchestrator.spi_router import SpiRouter
from tools.sim_klippy.orchestrator.beacon_serial_stub import BeaconSerialStub
from tools.sim_klippy.orchestrator.adc_stub import HeaterModel, temp_to_adc
from tools.sim_klippy.orchestrator.overrides import (
    apply_overrides,
    load_overrides,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

_CFG_DIR = (
    REPO_ROOT / "tools" / "sim_klippy" / "printer_real" / "config"
)
_THIRD_PARTY = (
    REPO_ROOT / "tools" / "sim_klippy" / "printer_real" / "third_party_repos"
)

# Map of third-party plugins → (source path, klippy/extras link target).
# Klippy discovers extras by walking klippy/extras/, so PYTHONPATH alone is
# not enough; the plugin must appear as klippy/extras/<name>.py. We
# install symlinks idempotently at fixture entry and leave them in place
# (they point at the vendored sources under tools/sim_klippy/printer_real/
# third_party_repos/, which are checked-in).
_THIRD_PARTY_PLUGINS = {
    "beacon": _THIRD_PARTY / "beacon_klipper" / "beacon.py",
    "motors_sync": _THIRD_PARTY / "motors-sync" / "motors_sync.py",
    "autotune_tmc": _THIRD_PARTY / "klipper_tmc_autotune" / "autotune_tmc.py",
    "motor_constants": _THIRD_PARTY / "klipper_tmc_autotune" / "motor_constants.py",
}


def _install_third_party_plugin_links() -> None:
    extras_dir = REPO_ROOT / "klippy" / "extras"
    for name, src in _THIRD_PARTY_PLUGINS.items():
        if not src.exists():
            continue
        link = extras_dir / f"{name}.py"
        if link.is_symlink():
            try:
                if pathlib.Path(os.readlink(link)) == src:
                    continue
            except OSError:
                pass
            link.unlink()
        elif link.exists():
            # A real file shadows our symlink; leave it alone.
            continue
        os.symlink(src, link)


@dataclasses.dataclass
class SimContext:
    mcus: McuHandles
    chip_servers: list
    beacon: BeaconSerialStub
    klippy_proc: subprocess.Popen
    klippy_log: pathlib.Path
    api_socket: str
    log_dir: pathlib.Path

    def gcode(self, script: str, timeout: float = 5.0) -> dict:
        """Send a gcode script via the klippy api socket."""
        return _send_gcode(self.api_socket, script, timeout=timeout)


def _send_gcode(api_socket: str, script: str, timeout: float = 5.0) -> dict:
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


def _ensure_elfs() -> None:
    """Raise with build instructions if either sim ELF is missing."""
    h7_elf = REPO_ROOT / "out" / "klipper-h7-sim.elf"
    f4_elf = REPO_ROOT / "out" / "klipper-f4-sim.elf"
    missing = [p for p in (h7_elf, f4_elf) if not p.exists()]
    if missing:
        raise RuntimeError(
            "Missing sim ELF(s): " + ", ".join(str(p) for p in missing) + "\n"
            "Build with:\n"
            "  cp tools/sim_klippy/configs/h7-sim.config .config && "
            "make clean && make -j4 && "
            "cp out/klipper.elf out/klipper-h7-sim.elf\n"
            "  cp tools/sim_klippy/configs/f4-sim.config .config && "
            "make clean && make -j4 && "
            "cp out/klipper.elf out/klipper-f4-sim.elf"
        )


def _stage_config_dir(
    cfg_dir: pathlib.Path,
    dest: pathlib.Path,
    overrides: Optional[dict] = None,
) -> None:
    """Symlink or copy every entry from cfg_dir into dest.

    printer.cfg is skipped — the caller writes a rendered version.
    Symlinks in cfg_dir are resolved and re-created as absolute symlinks
    in dest so klippy can follow them regardless of cwd.
    Directories are symlinked, not copied, to avoid duplicating large
    third-party trees.

    .cfg files have ``overrides`` applied (if given) and are written as
    regular files into dest so [include]d sections that reference
    real-hardware pin / bus names work after sim substitution.
    """
    needs_rewrite = overrides is not None
    for entry in cfg_dir.iterdir():
        if entry.name == "printer.cfg":
            continue
        target = dest / entry.name
        if target.exists() or target.is_symlink():
            continue
        if entry.is_symlink():
            # Resolve the symlink to an absolute path and re-create it.
            resolved = entry.resolve()
            if resolved.exists():
                os.symlink(resolved, target)
        elif entry.is_dir():
            os.symlink(entry.resolve(), target)
        elif entry.suffix == ".cfg" and needs_rewrite:
            text = entry.read_text()
            if overrides is not None:
                text = apply_overrides(text, overrides)
            target.write_text(text)
        else:
            shutil.copy2(entry, target)


@pytest.fixture
def sim(tmp_path):
    _ensure_elfs()
    _install_third_party_plugin_links()

    log_dir = tmp_path / "logs"
    log_dir.mkdir()

    h7_socket = str(tmp_path / "klipper_sim_h7")
    f4_socket = str(tmp_path / "klipper_sim_f4")
    beacon_pty = str(tmp_path / "klipper_sim_beacon")

    mcus: Optional[McuHandles] = None
    chip_servers: List[ChipSocketServer] = []
    beacon: Optional[BeaconSerialStub] = None
    klippy: Optional[subprocess.Popen] = None

    try:
        # 1) Spawn both MCUs.
        mcus = spawn_mcus(
            h7_elf=str(REPO_ROOT / "out" / "klipper-h7-sim.elf"),
            f4_elf=str(REPO_ROOT / "out" / "klipper-f4-sim.elf"),
            h7_socket=h7_socket,
            f4_socket=f4_socket,
            log_dir=str(log_dir),
        )

        # 2) Start chip emulators.
        # The MCU-side spidev / tmcuart stubs auto-route a sim_spi<N> bus
        # or a tmcuart oid<N> with no explicit route to a per-MCU socket
        # path of the form /tmp/klipper_sim_<flavor>_chip_<spi|uart><N>.
        # Flavor is `h7` for the KALICO_RUNTIME build (mcu_main) and
        # `f4` otherwise (mcu bottom). We bind those exact paths here.
        # ChipSocketServer unlinks any stale path before bind (single-
        # file unlink) so leftovers from a crashed previous run don't
        # block startup.
        # H7 SPI bus 0 (real-hardware spi1): 4 × TMC5160 stepper drivers
        # (stepper_x/y/x1/y1) AND 1 × MAX31865 RTD amplifier for the
        # extruder thermistor — five distinct chips behind one Unix
        # socket. The firmware-side spidev sim path publishes the active
        # CS pin's chardev gpio offset on every transfer (framed wire
        # protocol), and SpiRouter dispatches each frame to the right
        # per-chip emulator. CS values come from pin-overrides.toml's
        # [mcu_main.gpio] mapping: PC7=5, PC6=4, PD11=6, PC4=3, PF8=40.
        h7_spi_router = SpiRouter()
        h7_spi_router.attach(5, TMC5160Emulator().transfer)   # PC7 stepper_x
        h7_spi_router.attach(4, TMC5160Emulator().transfer)   # PC6 stepper_y
        h7_spi_router.attach(6, TMC5160Emulator().transfer)   # PD11 stepper_x1
        h7_spi_router.attach(3, TMC5160Emulator().transfer)   # PC4 stepper_y1
        h7_spi_router.attach(40, MAX31865Emulator().transfer) # PF8 extruder_rtd
        srv = ChipSocketServer(
            "/tmp/klipper_sim_h7_chip_spi0", h7_spi_router, framed=True,
        )
        srv.start()
        chip_servers.append(srv)
        # H7 UART: 1 × TMC2209 for the extruder (oid=0). chunk=10 covers
        # the longest UART-framed wire request klippy emits (8 logical
        # bytes × 10 wire bits = 10 bytes); the emulator strips the
        # start/stop framing internally.
        chip = TMC2209Emulator(slave_addr=0)
        srv = ChipSocketServer(
            "/tmp/klipper_sim_h7_chip_uart0", chip.handle, chunk=10,
        )
        srv.start()
        chip_servers.append(srv)
        # F4 UART: 3 × TMC2209 for Z, Z1, Z2 on oids 0..2. Each is
        # physically a separate chip on its own UART pin (uart_pin:
        # bottom:gpiochip0/gpio6, gpio3, gpio4 in printer.cfg) — they
        # all answer to slave_addr=0 because each is the only chip on
        # its bus. The auto-route routes oid=N to a per-oid socket so
        # the firmware-side multiplexing works without UART address
        # discrimination.
        for i in range(3):
            chip = TMC2209Emulator(slave_addr=0)
            path = f"/tmp/klipper_sim_f4_chip_uart{i}"
            srv = ChipSocketServer(path, chip.handle, chunk=10)
            srv.start()
            chip_servers.append(srv)

        # 3) Beacon stub.
        beacon = BeaconSerialStub(
            beacon_pty,
            log_path=str(log_dir / "beacon_traffic.log"),
        )
        beacon.start_sample_stream(z_target_mm=10.0, rate_hz=200)

        # 4) Apply pin / serial overrides and render printer.cfg into tmp_path.
        overrides_path = (
            REPO_ROOT / "tools" / "sim_klippy" / "pin-overrides.toml"
        )
        overrides = load_overrides(overrides_path)

        # Patch the serial mappings to point at our test-scoped socket paths
        # (the TOML has /tmp/klipper_sim_* as defaults; tmp_path is unique).
        overrides["mcu_main.serial"] = {
            "usb-Klipper_stm32h723xx_*": h7_socket,
            "usb-Klipper_stm32f446xx_*": f4_socket,
            "usb-Beacon_*": beacon_pty,
        }

        cfg_text = (_CFG_DIR / "printer.cfg").read_text()
        rendered_cfg_text = apply_overrides(cfg_text, overrides)
        rendered_cfg = tmp_path / "printer.cfg"
        rendered_cfg.write_text(rendered_cfg_text)

        # Stage companion .cfg files so klippy can resolve [include] lines.
        # Apply pin/bus overrides to included .cfg files — otherwise
        # extruder.cfg etc. still reference real-hardware names like
        # ``spi_bus: spi1`` that klippy cannot resolve.
        _stage_config_dir(
            _CFG_DIR,
            tmp_path,
            overrides=overrides,
        )

        # 5) Build PYTHONPATH so klippy finds the vendored third-party plugins.
        beacon_klipper_path = _THIRD_PARTY / "beacon_klipper"
        motors_sync_path = _THIRD_PARTY / "motors-sync"
        env = os.environ.copy()
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = ":".join(
            filter(None, [
                str(beacon_klipper_path),
                str(motors_sync_path),
                existing,
            ])
        )

        # 6) Spawn klippy.
        klippy_log = log_dir / "klippy.log"
        api_socket = str(tmp_path / "klippy.sock")
        stdout_log = open(log_dir / "klippy.stdout", "wb")
        klippy = subprocess.Popen(
            [
                "python3",
                str(REPO_ROOT / "klippy" / "klippy.py"),
                str(rendered_cfg),
                "-l", str(klippy_log),
                "-a", api_socket,
            ],
            env=env,
            stdout=stdout_log,
            stderr=subprocess.STDOUT,
            cwd=str(REPO_ROOT),
        )

        # 7) Wait until klippy finishes its connect callbacks (or
        # exits / hits a fatal). The klippy state machine sets
        # state_message="Printer is ready" but only exposes that via
        # the API socket — the log contains "Welcome to Kalico" from
        # the telemetry klippy:ready handler, which is a deterministic
        # post-ready marker we can grep for.
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if klippy_log.exists():
                content = klippy_log.read_bytes()
                if b"Welcome to Kalico" in content:
                    break
                if klippy.poll() is not None:
                    break
                if (b"Internal error" in content or
                        b"shutdown:" in content):
                    if klippy.poll() is not None:
                        break
            elif klippy.poll() is not None:
                break
            time.sleep(0.2)

        ctx = SimContext(
            mcus=mcus,
            chip_servers=chip_servers,
            beacon=beacon,
            klippy_proc=klippy,
            klippy_log=klippy_log,
            api_socket=api_socket,
            log_dir=log_dir,
        )

        yield ctx

    finally:
        # Teardown in reverse order: klippy → chip servers → beacon → MCUs.
        if klippy is not None and klippy.poll() is None:
            klippy.terminate()
            try:
                klippy.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                klippy.kill()
                try:
                    klippy.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
        for srv in chip_servers:
            srv.stop()
        if beacon is not None:
            beacon.stop()
        if mcus is not None:
            mcus.shutdown()
