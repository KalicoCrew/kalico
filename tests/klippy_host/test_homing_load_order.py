import pathlib
import subprocess
import sys
import tempfile
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

CONFIG_PHASE_OK = "MotionToolhead: Phase 1 skeleton initialized"
CONFIG_PHASE_FAILED = "Config error"

NEPTUNE_SHAPED_CONFIG = """
[mcu]
serial: /tmp/kalico-test-no-such-serial

[printer]
kinematics: cartesian
max_velocity: 100
max_accel: 1000
max_z_velocity: 10
max_z_accel: 30

[stepper_x]
step_pin: PC12
dir_pin: PB3
enable_pin: !PD2
microsteps: 16
rotation_distance: 40
endstop_pin: PA13
position_endstop: 0
position_max: 235
homing_speed: 50

[stepper_y]
step_pin: PC11
dir_pin: PA15
enable_pin: !PC10
microsteps: 16
rotation_distance: 40
endstop_pin: PB8
position_endstop: 0
position_max: 234
homing_speed: 50

[stepper_z]
step_pin: PC7
dir_pin: PC9
enable_pin: !PC8
microsteps: 16
rotation_distance: 8
endstop_pin: probe:z_virtual_endstop
position_min: -5
position_max: 283
homing_speed: 10

[safe_z_home]
home_xy_position: 117.5, 117.5
z_hop: 10

[probe]
pin: ^PA8
speed: 5
x_offset: -28
y_offset: 20
z_offset: 3
"""


def _boot_through_config_phase(config_text):
    with tempfile.TemporaryDirectory(prefix="kalico_cfg_") as tmpdir:
        tmp = pathlib.Path(tmpdir)
        cfg = tmp / "printer.cfg"
        cfg.write_text(config_text)
        log = tmp / "klippy.log"
        proc = subprocess.Popen(
            [
                sys.executable,
                str(REPO_ROOT / "klippy" / "klippy.py"),
                str(cfg),
                "-l",
                str(log),
            ],
            cwd=str(REPO_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                text = log.read_text(errors="replace") if log.exists() else ""
                if CONFIG_PHASE_OK in text or CONFIG_PHASE_FAILED in text:
                    return text
                if proc.poll() is not None:
                    time.sleep(0.5)
                    return (
                        log.read_text(errors="replace")
                        if log.exists()
                        else ""
                    )
                time.sleep(0.2)
            raise AssertionError(
                "klippy reached neither config-phase marker; log:\n"
                + (log.read_text(errors="replace") if log.exists() else "")
            )
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()


def test_safe_z_home_section_before_probe_section_parses():
    log_text = _boot_through_config_phase(NEPTUNE_SHAPED_CONFIG)
    assert "Unknown pin chip name" not in log_text, log_text[-3000:]
    assert CONFIG_PHASE_FAILED not in log_text, log_text[-3000:]
    assert CONFIG_PHASE_OK in log_text, log_text[-3000:]
