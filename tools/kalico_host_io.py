#!/usr/bin/env python3
# Standalone host-side I/O helper for the kalico runtime DECL_COMMAND surface.
#
# Spec §6 / Step-5 plan Task 25.5. This module is intentionally NOT built on
# klippy/reactor.py + klippy/serialhdl.py:SerialReader — those require the
# Klipper event loop to be the only owner of the serial fd, which deadlocks
# worker-thread integrations (e.g. pytest-style `wait_for_response` calls
# from the foreground thread that also need to drive the reactor).
#
# Pattern (lifted from scripts/console.py's `process_identify` flow but with
# the reactor stripped out):
#   1. open pyserial directly,
#   2. spawn a background RX thread that reads bytes and feeds them to
#      MessageParser.check_packet() / .parse(),
#   3. dispatch parsed messages to per-name `queue.Queue` instances,
#   4. on send, encode via MessageParser.create_command + encode_msgblock,
#      maintain a 4-bit sequence number ourselves.
#
# The identify handshake is done synchronously up-front, before the RX thread
# starts: this matches the Klipper protocol's wire-level guarantee that
# `identify_response` packets only flow in response to `identify` queries.
import argparse
import logging
import os
import pathlib
import queue as _queue
import struct
import sys
import threading
import time

# msgproto.py lives in klippy/.
_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "klippy"))

import msgproto  # noqa: E402

try:
    import serial  # type: ignore  # pyserial
except ImportError:  # pragma: no cover - import-time only
    serial = None  # The user installs pyserial when running on hardware.


# --- Wire-level helpers ----------------------------------------------------

# Per msgproto: messages start with [len, seq_with_dest, ...payload..., crc16, sync].
# We look for SYNC bytes to delimit packets, then ask MessageParser.check_packet
# to validate.
_SYNC = msgproto.MESSAGE_SYNC


class HostIoError(Exception):
    pass


class _RxBuffer:
    """Accumulates raw bytes; emits validated packets via MessageParser.check_packet."""

    def __init__(self, parser):
        self._buf = bytearray()
        self._parser = parser

    def feed(self, chunk):
        """Returns a list of bytes-like packets ready for `parser.parse`."""
        if not chunk:
            return []
        self._buf.extend(chunk)
        out = []
        while self._buf:
            # MessageParser.check_packet returns:
            #   0  → need more bytes
            #   -1 → corrupt — drop the leading byte and resync
            #   N>0 → valid packet of length N
            n = self._parser.check_packet(self._buf)
            if n == 0:
                break
            if n < 0:
                # Drop one byte and try to resync.
                del self._buf[0]
                continue
            pkt = bytes(self._buf[:n])
            del self._buf[:n]
            out.append(pkt)
        return out


# --- Public client class ---------------------------------------------------


class KalicoHostIO:
    """Standalone Klipper-protocol client.

    Public API (per Step-5 plan Task 25.5):
        - __init__(port, baud=250000)
        - send(cmd_str)
        - wait_for_response(name, timeout)
        - collect_responses(name, count, timeout)
        - disconnect()
    """

    IDENTIFY_CHUNK = 40

    def __init__(self, port, baud=250000, identify_timeout=5.0):
        if serial is None:
            raise HostIoError(
                "pyserial is required: `pip install pyserial` "
                "(needed for hardware bring-up; this is the Surface C path)"
            )
        self._port = port
        self._baud = baud
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._parser = msgproto.MessageParser()
        # Pre-init parser knows DefaultMessages (identify, identify_response).
        self._seq = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        # Per-response-name queues for wait_for_response / collect_responses.
        self._queues = {}
        self._queues_lock = threading.Lock()
        # Pre-handshake synchronous reader (no thread yet).
        self._rxbuf = _RxBuffer(self._parser)
        # Run identify synchronously, then start the dispatcher thread.
        self._do_identify(identify_timeout)
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="kalico-host-io-rx", daemon=True
        )
        self._rx_thread.start()

    # --- identify (synchronous, pre-thread) --------------------------------

    def _do_identify(self, timeout):
        """Pull the identify_data dictionary and load it into MessageParser."""
        deadline = time.monotonic() + timeout
        identify_data = b""
        while True:
            cmd = "identify offset=%d count=%d" % (len(identify_data), self.IDENTIFY_CHUNK)
            self._send_raw(cmd)
            params = self._wait_packet_sync("identify_response", deadline)
            if params is None:
                raise HostIoError(
                    "Timed out waiting for identify_response from %s" % (self._port,)
                )
            if params["offset"] != len(identify_data):
                # Drop and re-query (host-MCU resync).
                continue
            data = params["data"]
            if not data:
                break
            identify_data += data
        self._parser.process_identify(identify_data)

    def _wait_packet_sync(self, name, deadline):
        """Block (in the foreground thread) until a packet of `name` arrives."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._ser.timeout = max(0.05, min(remaining, 0.5))
            chunk = self._ser.read(256)
            for pkt in self._rxbuf.feed(chunk):
                params = self._parser.parse(pkt)
                if params.get("#name") == name:
                    return params
                # Pre-identify, drop everything else.

    # --- send / encode -----------------------------------------------------

    def _next_seq(self):
        # Klipper's 4-bit sequence number. Wire: (seq & 0x0F) | DEST.
        with self._lock:
            self._seq = (self._seq + 1) & msgproto.MESSAGE_SEQ_MASK
            return self._seq

    def _send_raw(self, cmd_str):
        """Encode `cmd_str` via MessageParser and write the framed bytes."""
        cmd = self._parser.create_command(cmd_str)
        if not cmd:
            return
        seq = self._next_seq()
        block = self._parser.encode_msgblock(seq, cmd)
        self._ser.write(bytes(block))
        self._ser.flush()

    def send(self, cmd_str):
        """Encode + write a command. Does not wait for a response."""
        self._send_raw(cmd_str)

    # --- response collection ----------------------------------------------

    def _ensure_queue(self, name):
        with self._queues_lock:
            q = self._queues.get(name)
            if q is None:
                q = _queue.Queue()
                self._queues[name] = q
            return q

    def wait_for_response(self, name, timeout):
        """Return the next parsed dict whose `#name` matches.

        Raises HostIoError on timeout.
        """
        q = self._ensure_queue(name)
        try:
            return q.get(timeout=timeout)
        except _queue.Empty:
            raise HostIoError(
                "Timed out after %.2fs waiting for response %r" % (timeout, name)
            )

    def collect_responses(self, name, count, timeout):
        """Collect exactly `count` responses with `#name == name`.

        Total wall-clock cap is `timeout` seconds; raises HostIoError if fewer
        than `count` responses arrived.
        """
        q = self._ensure_queue(name)
        deadline = time.monotonic() + timeout
        out = []
        while len(out) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise HostIoError(
                    "Collected %d/%d %r responses before %.2fs timeout"
                    % (len(out), count, name, timeout)
                )
            try:
                out.append(q.get(timeout=remaining))
            except _queue.Empty:
                continue
        return out

    # --- background RX dispatcher -----------------------------------------

    def _rx_loop(self):
        rxbuf = self._rxbuf  # reuse same accumulator (already drained above).
        while not self._stop.is_set():
            try:
                self._ser.timeout = 0.1
                chunk = self._ser.read(512)
            except (OSError, serial.SerialException) as exc:  # type: ignore[union-attr]
                logging.warning("kalico-host-io: serial read error: %s", exc)
                return
            if not chunk:
                continue
            try:
                packets = rxbuf.feed(chunk)
            except msgproto.error as exc:
                logging.warning("kalico-host-io: framing error: %s", exc)
                continue
            for pkt in packets:
                try:
                    params = self._parser.parse(pkt)
                except msgproto.error as exc:
                    logging.warning("kalico-host-io: parse error: %s", exc)
                    continue
                name = params.get("#name", "<noname>")
                self._ensure_queue(name).put(params)

    # --- shutdown ---------------------------------------------------------

    def disconnect(self):
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)

    # --- convenience ------------------------------------------------------

    def get_msgparser(self):
        return self._parser

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.disconnect()
        return False


# --- CLI smoke test --------------------------------------------------------


def _main():
    p = argparse.ArgumentParser(description="kalico host-io smoke test")
    p.add_argument("port", help="serial device, e.g. /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=250000)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO)
    io = KalicoHostIO(args.port, args.baud)
    try:
        msgs = io.get_msgparser().get_messages()
        print("Loaded %d messages" % (len(msgs),))
        print("App: %s" % (io.get_msgparser().get_app_info(),))
    finally:
        io.disconnect()


if __name__ == "__main__":
    _main()
