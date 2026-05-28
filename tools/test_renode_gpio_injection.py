#!/usr/bin/env python3
"""
Renode GPIO injection + virtual-time fixture for kalico-rewrite.

This is the Step 7-D deterministic simulator fixture. It intentionally uses
Option A: a Renode monitor wrapper drives virtual time and GPIO state, while
the firmware is touched only by a tiny CONFIG_KALICO_SIM-only validation
command (`runtime_sim_gpio_sample`). That keeps the simulator controls outside
production firmware and still proves the host can poll async firmware output
through the normal Klipper msgproto path.

Invocation:
    tools/sim/build_sim_firmware.sh
    python3 tools/test_renode_gpio_injection.py

Expected runtime: roughly 20-60 seconds on Apple Silicon with Renode 1.16.

What this covers:
    - boots the existing H723 Renode sim from tools/sim/h723_sim.resc
    - advances simulated time with exact `emulation RunFor` durations
    - drives a GPIO input line at a precise virtual-time boundary
    - observes both the synchronous response and async output frame emitted
      by fixture-validation-only firmware scaffolding

What this does not cover:
    - real Step 7-D endstop arm/trip protocol (not implemented yet)
    - hardware endstops, TMC DIAG pins, motors, or stepper position snapshots
    - production firmware builds; the sample command exists only when
      CONFIG_KALICO_SIM=y
"""

import argparse
import os
import pathlib
import re
import select
import socket
import subprocess
import sys
import time

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

from kalico_host_io import KalicoHostIO  # noqa: E402

# Renode GPIO-injection fixture/harness. Boots the H723 Renode sim.
# Tagged needs_renode so it is honestly excluded from CI (no Renode there).
pytestmark = pytest.mark.needs_renode

# Renode emits the monitor prompt `(h723)` after a command completes, but
# Machine state-change log lines (e.g. `[INFO] h723: Machine paused.`) can
# arrive on the same line *after* the prompt — so a strict end-of-buffer
# anchor misses real prompts. Accept the prompt followed by an optional
# trailing log line containing only ASCII / ANSI / colon-separated text.
PROMPT_RE = re.compile(
    rb"\((?:monitor|h723)\)\s*(?:\x1b\[[0-9;]*m)?"
    rb"(?:[^\n]*\n)?(?:[\s\x1b\[0-9;m]*)$"
)
DEFAULT_UART_PORT = 3334
DEFAULT_GDB_PORT = 3333
DEFAULT_MONITOR_PORT = 0


class RenodeFixtureError(RuntimeError):
    pass


class RenodeMonitor:
    """Small monitor-stdin wrapper for deterministic Renode control."""

    def __init__(
        self,
        uart_port=DEFAULT_UART_PORT,
        gdb_port=DEFAULT_GDB_PORT,
        monitor_port=DEFAULT_MONITOR_PORT,
        log_path=None,
    ):
        self.uart_port = int(uart_port)
        self.gdb_port = int(gdb_port)
        self.monitor_port = int(monitor_port)
        self.log_path = pathlib.Path(log_path or "/tmp/kalico-renode-gpio.log")
        self.proc = None
        self._fd = None
        self._buf = bytearray()
        self._mon = None
        self._mon_buf = bytearray()

    def start(self):
        if not (REPO_ROOT / "out/klipper.elf").exists():
            raise RenodeFixtureError(
                "out/klipper.elf not found; run tools/sim/build_sim_firmware.sh"
            )
        resc = REPO_ROOT / "tools/sim/h723_sim.resc"
        config_path = pathlib.Path("/tmp/kalico-renode-fixture.cfg")
        config_path.write_text(
            "[general]\n"
            "terminal = Termsharp\n"
            "compiler-cache-enabled = False\n"
            "serialization-mode = Generated\n"
            "use-synchronous-logging = False\n"
            "always-log-machine-name = False\n"
            "collapse-repeated-log-entries = True\n"
            "log-history-limit = 1000\n"
            "history-path = /tmp/kalico-renode-history\n"
            "[monitor]\n"
            "consume-exceptions-from-command = True\n"
            "break-script-on-exception = True\n"
            "number-format = Hexadecimal\n"
            "[plugins]\n"
            "enabled-plugins = \n",
            encoding="utf-8",
        )
        args = [
            "renode",
            "--disable-gui",
            "--config",
            str(config_path),
            "-e",
            "include @%s" % (resc,),
            "-e",
            "logLevel 3 sysbus",
            "-e",
            "logLevel 3 rcc",
            "-e",
            "logLevel 3 nvic",
            "-e",
            "logLevel 3 usart2",
        ]
        if self.monitor_port:
            args.extend(["--hide-monitor", "--port", str(self.monitor_port)])
            stdin = subprocess.DEVNULL
        else:
            args.insert(1, "--console")
            stdin = subprocess.PIPE
        env = os.environ.copy()
        # Renode 1.16 on macOS can abort on the user's global config.lock when
        # several harnesses were interrupted. Give this fixture its own config
        # home so monitor-controlled test runs are isolated from stale locks.
        renode_home = pathlib.Path("/tmp/kalico-renode-home")
        renode_home.mkdir(parents=True, exist_ok=True)
        env["HOME"] = str(renode_home)
        self.proc = subprocess.Popen(
            args,
            cwd=str(REPO_ROOT),
            stdin=stdin,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
        )
        self._fd = self.proc.stdout.fileno()
        os.set_blocking(self._fd, False)
        if self.monitor_port:
            self._wait_for_tcp(self.monitor_port, timeout=45.0)
            self._mon = socket.create_connection(
                ("127.0.0.1", self.monitor_port), timeout=5.0
            )
            self._mon.setblocking(False)
        self._read_until_prompt(timeout=45.0)
        self.command('emulation SetGlobalQuantum "0.000001"')
        self.command("start")
        self._wait_for_tcp(self.uart_port, timeout=20.0)

    def stop(self):
        if self.proc is None:
            return
        try:
            if self.proc.poll() is None:
                try:
                    self.command("quit", timeout=2.0)
                except Exception:
                    pass
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
        finally:
            try:
                if self._mon is not None:
                    self._mon.close()
            except Exception:
                pass
            self._flush_log()

    def pause(self):
        self.command("pause")

    def advance_time(self, seconds):
        if seconds < 0:
            raise ValueError("seconds must be non-negative")
        self.command('emulation RunFor "%.9f"' % (seconds,), timeout=60.0)

    def set_gpio(self, port, pin, value):
        port = str(port).upper()
        self.command(
            "sysbus.gpioPort%s OnGPIO %d %s"
            % (port, int(pin), "true" if value else "false")
        )

    def drive_gpio_at(self, offset_seconds, port, pin, value):
        self.advance_time(offset_seconds)
        self.set_gpio(port, pin, value)

    def command(self, command, timeout=10.0):
        if self.proc is None:
            raise RenodeFixtureError("Renode process is not running")
        if self.proc.poll() is not None:
            self._flush_log()
            raise RenodeFixtureError(
                "Renode exited with rc=%s" % (self.proc.returncode,)
            )
        if self._mon is not None:
            self._mon.sendall((command + "\n").encode("utf-8"))
        elif self.proc.stdin is not None:
            self.proc.stdin.write((command + "\n").encode("utf-8"))
            self.proc.stdin.flush()
        else:
            raise RenodeFixtureError("Renode monitor transport is unavailable")
        return self._read_until_prompt(timeout=timeout)

    def _read_until_prompt(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                self._drain_stdout()
                self._flush_log()
                raise RenodeFixtureError(
                    "Renode exited before monitor prompt (rc=%s); log=%s"
                    % (self.proc.returncode, self.log_path)
                )
            self._drain_monitor()
            self._drain_stdout()
            prompt_buf = self._mon_buf if self._mon is not None else self._buf
            tail = bytes(prompt_buf[-4096:])
            if PROMPT_RE.search(tail):
                out = bytes(prompt_buf)
                prompt_buf.clear()
                return out.decode("utf-8", "replace")
            time.sleep(0.02)
        self._flush_log()
        raise RenodeFixtureError(
            "timed out waiting for Renode monitor prompt; log=%s"
            % (self.log_path,)
        )

    def _drain_stdout(self):
        while True:
            r, _, _ = select.select([self._fd], [], [], 0)
            if not r:
                return
            try:
                chunk = os.read(self._fd, 16384)
            except BlockingIOError:
                return
            if not chunk:
                return
            self._buf.extend(chunk)

    def _flush_log(self):
        try:
            self._drain_stdout()
            self.log_path.write_bytes(bytes(self._buf))
        except Exception:
            pass

    def _drain_monitor(self):
        if self._mon is None:
            return
        while True:
            r, _, _ = select.select([self._mon], [], [], 0)
            if not r:
                return
            try:
                chunk = self._mon.recv(16384)
            except (BlockingIOError, socket.timeout):
                return
            if not chunk:
                return
            self._mon_buf.extend(chunk)

    @staticmethod
    def _wait_for_tcp(port, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(
                    ("127.0.0.1", int(port)), timeout=0.2
                ):
                    return
            except OSError:
                time.sleep(0.1)
        raise RenodeFixtureError(
            "Renode UART tcp port %d did not open" % (port,)
        )


def poll_async_event(io, name, timeout=3.0):
    return io.wait_for_response(name, timeout=timeout)


def sample_gpio(io, renode, sample_id, pin_name, pull_up=0):
    io.send(
        "runtime_sim_gpio_sample sample_id=%d pin=%s pull_up=%d"
        % (int(sample_id), pin_name, int(pull_up))
    )
    renode.advance_time(0.002)
    resp = io.wait_for_response("runtime_sim_gpio_sample_response", timeout=3.0)
    event = poll_async_event(io, "runtime_sim_gpio_sample", timeout=3.0)
    return resp, event


def assert_sample(sample, expected_id, expected_value, label):
    resp, event = sample
    for source_name, msg in (("response", resp), ("async event", event)):
        if int(msg["sample_id"]) != int(expected_id):
            raise AssertionError(
                "%s %s sample_id=%s expected %s"
                % (label, source_name, msg["sample_id"], expected_id)
            )
        if int(msg["value"]) != int(expected_value):
            raise AssertionError(
                "%s %s value=%s expected %s"
                % (label, source_name, msg["value"], expected_value)
            )


def test_gpio_injection_fixture(args):
    renode = RenodeMonitor(
        uart_port=args.uart_tcp_port,
        gdb_port=args.gdb_port,
        monitor_port=args.monitor_tcp_port,
        log_path=args.renode_log,
    )
    io = None
    try:
        print("[gpio] launching Renode ...")
        renode.start()
        port_url = "socket://localhost:%d" % (args.uart_tcp_port,)
        print("[gpio] connecting host I/O on %s ..." % (port_url,))
        io = KalicoHostIO(port_url, identify_timeout=args.identify_timeout)
        parser = io.get_msgparser()
        messages = parser.get_messages()

        # `get_messages()` returns `(msgid, msgtype, msgformat)` tuples;
        # the format string is the third element.
        def _msg_fmt(m):
            if isinstance(m, str):
                return m
            if isinstance(m, (tuple, list)) and len(m) >= 3:
                return m[2] if isinstance(m[2], str) else ""
            return ""

        if not any(
            _msg_fmt(m).startswith("runtime_sim_gpio_sample ") for m in messages
        ):
            raise AssertionError(
                "runtime_sim_gpio_sample is missing from identify dict; "
                "rebuild with CONFIG_KALICO_SIM=y"
            )

        # Move to paused, deterministic monitor-controlled time. PC13 is a
        # plain input on the H743 Renode platform and is not used by USART2.
        renode.pause()
        renode.set_gpio("C", 13, False)
        low = sample_gpio(io, renode, sample_id=1, pin_name="PC13", pull_up=0)
        assert_sample(low, expected_id=1, expected_value=0, label="initial low")
        print(
            "[gpio] initial low sample observed via response and async output"
        )

        # T = 3 ms from the post-low baseline. The OnGPIO transition occurs
        # while emulation is paused immediately after the exact RunFor window.
        renode.drive_gpio_at(0.003, "C", 13, True)
        high = sample_gpio(io, renode, sample_id=2, pin_name="PC13", pull_up=0)
        assert_sample(
            high, expected_id=2, expected_value=1, label="injected high"
        )
        print("[gpio] high injection at T=3.000 ms observed")

        renode.drive_gpio_at(0.001, "C", 13, False)
        low_again = sample_gpio(
            io, renode, sample_id=3, pin_name="PC13", pull_up=0
        )
        assert_sample(
            low_again, expected_id=3, expected_value=0, label="final low"
        )
        print("[gpio] low release observed")

        print("PASS: Renode GPIO injection fixture")
        return 0
    finally:
        if io is not None:
            try:
                io.disconnect()
            except Exception:
                pass
        if not args.keep_renode:
            renode.stop()


def main():
    p = argparse.ArgumentParser(description="Renode GPIO injection fixture")
    p.add_argument("--uart-tcp-port", type=int, default=DEFAULT_UART_PORT)
    p.add_argument("--monitor-tcp-port", type=int, default=DEFAULT_MONITOR_PORT)
    p.add_argument("--gdb-port", type=int, default=DEFAULT_GDB_PORT)
    p.add_argument("--identify-timeout", type=float, default=60.0)
    p.add_argument("--renode-log", default="/tmp/kalico-renode-gpio.log")
    p.add_argument("--keep-renode", action="store_true")
    args = p.parse_args()
    return test_gpio_injection_fixture(args)


if __name__ == "__main__":
    sys.exit(main())
