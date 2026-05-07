"""Scaffolding Beacon serial stub.

Exposes a PTY at the configured path and symlinks it so klippy can open
it as a serial device. Logs all bytes received from klippy to a file so
we can iteratively reverse-engineer the protocol during Phase 7. Knows
how to:

- Accept arbitrary inbound bytes and log them.
- Emit periodic placeholder sample frames when the stream is started.
- Return canned bytes when specific opcodes are later recognised.

This is NOT a faithful beacon emulator. It is a passthrough scaffold
that the sim harness extends as we discover what the plugin actually
expects during test_boot Phase 7.

Protocol notes from survey of beacon.py
----------------------------------------
Beacon runs on top of klippy's msgproto framework, not a raw ASCII line
protocol. Commands/responses registered at init time:

  Responses the plugin registers:
    beacon_data
    beacon_status
    beacon_contact
    (accel extension)  beacon_accel_data, beacon_accel_state

  Commands the plugin looks up (outbound from klippy):
    beacon_stream en=%u
    beacon_set_threshold trigger=%u untrigger=%u
    beacon_home trsync_oid=%c trigger_reason=%c trigger_invert=%c
    beacon_stop_home
    beacon_nvm_read len=%c offset=%hu   (query — expects a reply)
    beacon_contact_home …
    beacon_contact_stop_home
    (optional) beacon_contact_set_latency_min …
    (optional) beacon_contact_set_sensitivity …

  _check_mcu_version() runs update_firmware.py as a subprocess against
  the serial port — bypassed by skip_firmware_version_check = True in
  the sim config (no PTY traffic needed for that path).

Because every command/response is msgproto-encoded, a fully faithful
stub requires the msgproto dictionary negotiated at connect time. Phase 7
will capture that dictionary from the beacon_traffic.log and extend this
stub with proper framing. Until then, we log all inbound bytes and emit
placeholder text frames so the test harness can at least observe that
klippy is attempting to communicate.
"""

import math
import os
import pty
import select
import threading
import time
from typing import Optional


class BeaconSerialStub:
    """PTY-backed scaffolding stub for the Beacon eddy-current probe.

    Lifecycle::

        stub = BeaconSerialStub("/tmp/klipper_sim_beacon", log_path="...")
        stub.start()
        # ... test runs ...
        stub.stop()

    The PTY slave is symlinked to ``pty_path`` so klippy's serial-open
    call finds it at the configured path.  All bytes received on the
    master side are logged; nothing is returned unless
    ``start_sample_stream`` has been called.
    """

    def __init__(self, pty_path: str, log_path: Optional[str] = None) -> None:
        self._pty_path = pty_path
        self._log_path = log_path
        self._z_target: float = 10.0
        self._z_noise_amp: float = 0.005  # ±5 µm as spec'd
        self._stream_rate: float = 0.0    # Hz; 0 = stream off
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._master_fd: Optional[int] = None
        self._t0: float = time.monotonic()
        # Counters exposed for tests (written only from the loop thread).
        self.rx_byte_count: int = 0
        self.tx_sample_count: int = 0

    # ------------------------------------------------------------------
    # Public API used by the test harness / orchestrator
    # ------------------------------------------------------------------

    def set_z(self, z_mm: float) -> None:
        """Update the Z position the stub will report in sample frames."""
        self._z_target = z_mm

    def start(self) -> None:
        """Open the PTY, symlink slave to pty_path, and start the I/O loop."""
        if self._thread is not None and self._thread.is_alive():
            return  # already running
        self._stop.clear()
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        # Create or replace the symlink so klippy sees a stable path.
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, self._pty_path)
        # slave_fd must stay open somewhere for the PTY to remain usable;
        # closing it here would make the master side return EIO on the
        # first read.  Keep our own reference.
        self._slave_fd = slave_fd
        self._master_fd = master_fd
        # Set master non-blocking so writes drop (EAGAIN) rather than block
        # when the kernel buffer is full.  In the real klippy scenario the
        # plugin continuously reads from the slave, so the buffer never fills;
        # in tests where nobody reads the slave we prefer a dropped frame over
        # a deadlocked loop.
        import fcntl as _fcntl
        flags = _fcntl.fcntl(master_fd, _fcntl.F_GETFL)
        _fcntl.fcntl(master_fd, _fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self._thread = threading.Thread(
            target=self._loop, name="beacon-stub", daemon=True
        )
        self._thread.start()

    def start_sample_stream(
        self, z_target_mm: float, rate_hz: float = 1600.0
    ) -> None:
        """Begin emitting placeholder sample frames at *rate_hz*.

        Calls ``start()`` implicitly if the stub has not been started yet.
        Phase 7 will replace the placeholder frame format with the real
        msgproto-encoded ``beacon_data`` message once the dictionary is
        captured.
        """
        self._z_target = z_target_mm
        self._stream_rate = rate_hz
        if self._thread is None or not self._thread.is_alive():
            self.start()

    def stop(self) -> None:
        """Signal the I/O loop to exit and clean up the PTY + symlink."""
        self._stop.set()
        # Close the master fd to unblock any select() call inside the loop.
        if self._master_fd is not None:
            try:
                os.close(self._master_fd)
            except OSError:
                pass
            self._master_fd = None
        if hasattr(self, "_slave_fd") and self._slave_fd is not None:
            try:
                os.close(self._slave_fd)
            except OSError:
                pass
            self._slave_fd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _z_sample(self) -> float:
        """Return the current simulated Z reading with ±5 µm sinusoidal noise."""
        elapsed = time.monotonic() - self._t0
        return self._z_target + self._z_noise_amp * math.sin(elapsed * 60.0)

    def _log(self, direction: str, data: bytes) -> None:
        """Append a line to the traffic log.

        Format::

            [<elapsed_s>][<direction>] <hex>  <printable_ascii>

        Both hex and printable ASCII are included so Phase-7 analysis can
        correlate msgproto bytes with human-readable content.
        """
        if self._log_path is None:
            return
        try:
            log_dir = os.path.dirname(self._log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            with open(self._log_path, "ab") as f:
                ts = f"{time.monotonic() - self._t0:.6f}"
                printable = data.decode("utf-8", errors="replace").replace(
                    "\n", "\\n"
                ).replace("\r", "\\r")
                line = f"[{ts}][{direction}] {data.hex()}  {printable}\n".encode()
                f.write(line)
        except (OSError, ValueError):
            pass

    def _loop(self) -> None:
        """Main I/O loop: drain inbound bytes, emit periodic sample frames."""
        sample_count = 0
        last_sample_time = 0.0
        master_fd = self._master_fd  # capture locally; can be None-d by stop()

        while not self._stop.is_set():
            if master_fd is None:
                break
            try:
                r, _, _ = select.select([master_fd], [], [], 0.005)
            except (ValueError, OSError):
                break  # fd closed from stop()
            if r:
                try:
                    chunk = os.read(master_fd, 256)
                except OSError:
                    break
                if chunk:
                    self._log("rx", chunk)
                    self.rx_byte_count += len(chunk)
                    # Phase 7: inspect chunk, dispatch to command handlers,
                    # send back msgproto-encoded responses.

            if self._stream_rate > 0:
                now = time.monotonic()
                if now - last_sample_time >= 1.0 / self._stream_rate:
                    last_sample_time = now
                    z = self._z_sample()
                    # Placeholder frame — extend in Phase 7 once we have the
                    # real msgproto dictionary from beacon_traffic.log.
                    msg = (
                        f"beacon_data z={z:.4f} count={sample_count}\n"
                    ).encode()
                    try:
                        os.write(master_fd, msg)
                    except BlockingIOError:
                        pass  # buffer full — klippy will drain it; just skip
                    except OSError:
                        break
                    self._log("tx", msg)
                    sample_count += 1
                    self.tx_sample_count += 1
