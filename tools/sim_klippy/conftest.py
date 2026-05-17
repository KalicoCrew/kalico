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
# install symlinks idempotently at fixture entry. Sources live under
# tools/sim_klippy/printer_real/third_party_repos/, which is gitignored and
# populated by tools/sim_klippy/fetch_plugins.sh at pinned upstream revs.
_THIRD_PARTY_PLUGINS = {
    "beacon": _THIRD_PARTY / "beacon_klipper" / "beacon.py",
    "motors_sync": _THIRD_PARTY / "motors-sync" / "motors_sync.py",
    "autotune_tmc": _THIRD_PARTY / "klipper_tmc_autotune" / "autotune_tmc.py",
    "motor_constants": _THIRD_PARTY / "klipper_tmc_autotune" / "motor_constants.py",
}

_FETCH_SCRIPT = REPO_ROOT / "tools" / "sim_klippy" / "fetch_plugins.sh"


def _ensure_third_party_repos() -> None:
    """Run fetch_plugins.sh if any required plugin source is missing."""
    if all(src.exists() for src in _THIRD_PARTY_PLUGINS.values()):
        return
    if not _FETCH_SCRIPT.exists():
        raise RuntimeError(
            f"Third-party plugin sources missing and fetch script absent: "
            f"{_FETCH_SCRIPT}"
        )
    result = subprocess.run(
        ["bash", str(_FETCH_SCRIPT)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"fetch_plugins.sh failed (exit {result.returncode}):\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    missing = [str(src) for src in _THIRD_PARTY_PLUGINS.values() if not src.exists()]
    if missing:
        raise RuntimeError(
            "fetch_plugins.sh ran but these plugin sources are still missing: "
            + ", ".join(missing)
        )


def _install_third_party_plugin_links() -> None:
    _ensure_third_party_repos()
    extras_dir = REPO_ROOT / "klippy" / "extras"
    for name, src in _THIRD_PARTY_PLUGINS.items():
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
    h7_sim_control: str
    f4_sim_control: str

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
def sim_extra_overrides(request):
    """Per-test override hook for the ``sim`` fixture.

    A test can request additional ``apply_overrides``-style ``config_set``
    injections by parametrizing this fixture indirectly:

        @pytest.mark.parametrize(
            "sim_extra_overrides",
            [{"stepper_z.config_set": {"phase_stepping": "1"}}],
            indirect=True,
        )
        def test_something(sim): ...

    The default is an empty dict, which preserves the existing fixture
    behaviour for every test that doesn't opt in.
    """
    return getattr(request, "param", {}) or {}


@pytest.fixture
def sim(tmp_path, sim_extra_overrides):
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
        # The shim (linux/runtime_tick_host.c) reads KALICO_SIM_SOCK_DIR
        # from env (set by spawn_mcus) and binds its sim_control listener
        # there. For SPI transfers it CONNECTS to a per-CS-pin socket at
        # ${sock}/spi_cs_<bus>_<line>; for tmcuart it connects to
        # ${sock}/tmcuart_<oid>. We pre-create the same directories here
        # and bind one chip emulator per socket. ChipSocketServer unlinks
        # any stale path before bind so leftovers from a crashed run
        # don't block startup.
        h7_sock = tmp_path / "sim" / "h7"
        f4_sock = tmp_path / "sim" / "f4"
        h7_sock.mkdir(parents=True, exist_ok=True)
        f4_sock.mkdir(parents=True, exist_ok=True)

        # H7 SPI bus 0: 5 chips, one socket per CS pin. Shim demultiplexes
        # by currently-asserted CS pin and connects to the matching
        # spi_cs_0_<line> socket. CS line numbers come from
        # pin-overrides.toml [mcu_main.gpio]: PC7=5, PC6=4, PD11=6,
        # PC4=3, PF8=40.
        h7_chips_by_cs = [
            (5,  TMC5160Emulator().transfer),   # PC7 stepper_x
            (4,  TMC5160Emulator().transfer),   # PC6 stepper_y
            (6,  TMC5160Emulator().transfer),   # PD11 stepper_x1
            (3,  TMC5160Emulator().transfer),   # PC4 stepper_y1
            (40, MAX31865Emulator().transfer),  # PF8 extruder_rtd
        ]
        for cs_line, transfer in h7_chips_by_cs:
            path = str(h7_sock / f"spi_cs_0_{cs_line}")
            srv = ChipSocketServer(path, transfer, framed=False)
            srv.start()
            chip_servers.append(srv)

        # H7 tmcuart oid=0 → ${h7_sock}/tmcuart_0 (extruder TMC2209).
        # chunk=10 covers the longest UART-framed wire request klippy
        # emits (8 logical bytes × 10 wire bits = 10 bytes); the emulator
        # strips start/stop framing internally.
        chip = TMC2209Emulator(slave_addr=0)
        srv = ChipSocketServer(
            str(h7_sock / "tmcuart_0"), chip.handle, chunk=10,
        )
        srv.start()
        chip_servers.append(srv)

        # F4 tmcuart oids 0..2 → ${f4_sock}/tmcuart_{0,1,2} for Z, Z1, Z2.
        # Each is physically a separate chip on its own UART pin (uart_pin
        # bottom:gpiochip0/gpio{6,3,4} in printer.cfg); all answer to
        # slave_addr=0 because each is the only chip on its bus.
        for i in range(3):
            chip = TMC2209Emulator(slave_addr=0)
            path = str(f4_sock / f"tmcuart_{i}")
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

        # Per-test config_set injections from the ``sim_extra_overrides``
        # fixture. Merged after pin-overrides.toml so a test can override
        # individual settings (e.g. ``phase_stepping`` on stepper_z) for
        # parity with bench configs that the vendored printer.cfg lags.
        for section, kv in sim_extra_overrides.items():
            existing = overrides.setdefault(section, {})
            existing.update(kv)

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
        # Expose H7 sock_dir to klippy so cmd_KALICO_SIM_ENDSTOP_SET_PIN
        # can open the sim_control socket (endstops are wired to H7).
        env["KALICO_SIM_SOCK_DIR"] = str(h7_sock)

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
            h7_sim_control=str(h7_sock / "sim_control"),
            f4_sim_control=str(f4_sock / "sim_control"),
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
