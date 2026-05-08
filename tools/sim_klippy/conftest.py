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
}


def _strip_sections(cfg_text: str, prefixes: tuple) -> str:
    """Remove [<prefix> ...] sections (and their bodies) from cfg_text.

    A section starts at a line matching ``[<prefix>`` (with an optional
    instance suffix) and ends at the next top-level section header or
    EOF. We comment-out the entire range with a leading ``#`` per line so
    the bytes survive in the rendered cfg for forensics, but klippy
    treats them as comments.

    Also strips the entire klippy autosave block (lines starting with
    ``#*#``), which contains stored sections like ``[beacon model
    default]`` that would re-trigger module loads even with [beacon]
    stripped.
    """
    lines = cfg_text.splitlines(keepends=True)
    out = []
    in_strip = False
    in_autosave = False
    import re
    section_re = re.compile(r"^\s*\[([^\]]+)\]\s*$")
    for line in lines:
        if line.lstrip().startswith("#*#"):
            in_autosave = True
        if in_autosave:
            # Replace the autosave-marker prefix so klippy's configfile
            # parser no longer treats this block as autosave content.
            out.append("# [sim-autosave-strip] " + line)
            continue
        m = section_re.match(line)
        if m:
            head = m.group(1).split(None, 1)[0]
            in_strip = head in prefixes
        if in_strip:
            out.append("# [sim-strip] " + line if line.strip() else line)
        else:
            out.append(line)
    return "".join(out)


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
    strip_prefixes: tuple = (),
) -> None:
    """Symlink or copy every entry from cfg_dir into dest.

    printer.cfg is skipped — the caller writes a rendered version.
    Symlinks in cfg_dir are resolved and re-created as absolute symlinks
    in dest so klippy can follow them regardless of cwd.
    Directories are symlinked, not copied, to avoid duplicating large
    third-party trees.

    .cfg files are read, sim-stripped (if strip_prefixes non-empty),
    and written as regular files into dest so [include]d sections that
    reference unvendored plugins don't crash boot.
    """
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
        elif entry.suffix == ".cfg" and strip_prefixes:
            text = entry.read_text()
            target.write_text(_strip_sections(text, strip_prefixes))
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
        # 4 × TMC5160 for X, Y, X1, Y1 on the H7 SPI bus.
        for i in range(4):
            chip = TMC5160Emulator()
            path = str(tmp_path / f"chip_spi{i}")
            srv = ChipSocketServer(path, chip.transfer, chunk=5)
            srv.start()
            chip_servers.append(srv)
        # 3 × TMC2209 for Z, Z1, Z2 on the F4 UART bus.
        for i in range(3):
            chip = TMC2209Emulator(slave_addr=i)
            path = str(tmp_path / f"chip_uart{i}")
            srv = ChipSocketServer(path, chip.handle, chunk=8)
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
        # SCOPE-REDUCTION: with [beacon] stripped, the 'probe' pin chip
        # never registers. The real config uses
        # 'endstop_pin: probe:z_virtual_endstop' on stepper_z. Substitute
        # a benign GPIO pin so klippy can complete config parsing.
        cfg_text = cfg_text.replace(
            "probe:z_virtual_endstop", "gpiochip0/gpio15"
        )
        rendered_cfg_text = apply_overrides(cfg_text, overrides)
        # Strip sections whose plugin module isn't installed in this tree
        # OR whose stub isn't faithful enough to satisfy klippy's MCU
        # identify handshake. SCOPE-REDUCTION: [beacon] is stripped
        # because beacon_serial_stub.py is a logging scaffold — it doesn't
        # speak msgproto, so the beacon MCU's identify hangs forever and
        # klippy never reaches "ready". Restoring faithful beacon support
        # is a follow-up task. [bed_mesh], [resonance_tester], and
        # [motors_sync] all reference beacon as their probe / accel_chip
        # so they're stripped together.
        # [beacon model default] is a stored model section that depends
        # on [beacon] — strip together. [bed_mesh] (when configured for
        # beacon) likewise needs the probe.
        rendered_cfg_text = _strip_sections(
            rendered_cfg_text,
            prefixes=(
                "autotune_tmc", "motor_constants",
                "beacon", "bed_mesh", "resonance_tester", "motors_sync",
                "z_tilt_ng",
            ),
        )
        rendered_cfg = tmp_path / "printer.cfg"
        rendered_cfg.write_text(rendered_cfg_text)

        # Stage companion .cfg files so klippy can resolve [include] lines.
        # Apply the same sim-strip to included .cfg files that we applied
        # to the main rendered printer.cfg.
        _stage_config_dir(
            _CFG_DIR,
            tmp_path,
            strip_prefixes=(
                "autotune_tmc", "motor_constants",
                "beacon", "bed_mesh", "resonance_tester", "motors_sync",
                "z_tilt_ng",
            ),
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

        # 7) Wait for "Printer is ready" (or early exit / fatal error).
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if klippy_log.exists():
                content = klippy_log.read_bytes()
                if b"Printer is ready" in content:
                    break
                # Stop waiting early if klippy already died.
                if klippy.poll() is not None:
                    break
                # Also stop early on fatal conditions so we don't burn 30s.
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
