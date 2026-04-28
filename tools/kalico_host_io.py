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
import pathlib
import queue as _queue
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

    def __init__(self, port, baud=250000, identify_timeout=15.0):
        if serial is None:
            raise HostIoError(
                "pyserial is required: `pip install pyserial` "
                "(needed for hardware bring-up; this is the Surface C path)"
            )
        self._port = port
        self._baud = baud
        # pyserial routes "socket://host:port" / "loop://" / etc. through
        # serial_for_url; the bare Serial(port) constructor does not. The sim
        # bench uses socket:// to talk to Renode's USART2 server-socket.
        if "://" in port:
            self._ser = serial.serial_for_url(port, baud, timeout=0.1)
        else:
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
        """Pull the identify_data dictionary and load it into MessageParser.

        Two complications when reconnecting to a running MCU:
        1. The OS USB-CDC buffer may hold stale identify_response chunks
           from a prior klippy session; reading them naively makes us
           sync to an outdated MCU seq state.
        2. MCU's next_sequence is at whatever klippy last advanced it to
           (only resets on MCU reset).
        Strategy: drain stale RX bytes first, then send identify with
        NAK-driven seq resync. NAKs (header-only length-5 packets) carry
        the MCU's current next_sequence in their seq byte.
        """
        deadline = time.monotonic() + timeout
        # Drain any stale RX bytes from earlier klippy session.
        drain_until = time.monotonic() + 0.3
        while time.monotonic() < drain_until:
            self._ser.timeout = 0.05
            n = len(self._ser.read(4096))
            if n == 0:
                break
        self._rxbuf = _RxBuffer(self._parser)  # reset partial-packet state

        identify_data = b""
        while True:
            cmd = "identify offset=%d count=%d" % (
                len(identify_data),
                self.IDENTIFY_CHUNK,
            )
            params = None
            for attempt in range(20):
                self._send_raw(cmd)
                attempt_deadline = min(deadline, time.monotonic() + 0.15)
                params = self._wait_packet_sync(
                    "identify_response", attempt_deadline, sync_seq=True
                )
                if params is not None and params.get("offset") == len(
                    identify_data
                ):
                    break
                params = None  # stale or wrong offset; retry with synced seq
                if time.monotonic() >= deadline:
                    break
            if params is None:
                raise HostIoError(
                    "Timed out waiting for identify_response from %s"
                    % (self._port,)
                )
            data = params["data"]
            if not data:
                break
            identify_data += data
        self._parser.process_identify(identify_data)

    def _wait_packet_sync(self, name, deadline, sync_seq=False):
        """Block (in the foreground thread) until a packet of `name` arrives.

        If sync_seq is True, every received packet (including NAKs, which are
        header-only length-5 packets with no msgid) updates self._seq to the
        MCU's expected next_sequence. NAKs raise during _parser.parse because
        they have no msgid; we extract the seq directly from the packet bytes
        and swallow the parse error.
        """
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._ser.timeout = max(0.05, min(remaining, 0.5))
            chunk = self._ser.read(256)
            for pkt in self._rxbuf.feed(chunk):
                if sync_seq and len(pkt) >= 2:
                    # MCU's seq byte tells us its current next_sequence.
                    # On match: this is the response to our last send and
                    # our seq has already advanced past it. On mismatch
                    # (NAK or other in-flight): align to MCU.
                    mcu_next = pkt[1] & msgproto.MESSAGE_SEQ_MASK
                    with self._lock:
                        self._seq = mcu_next
                try:
                    params = self._parser.parse(pkt)
                except Exception:
                    # NAK packets have no msgid → parse fails. We've
                    # already extracted the seq above; drop the packet.
                    continue
                if params.get("#name") == name:
                    return params
                # Pre-identify, drop everything else.

    # --- send / encode -----------------------------------------------------

    def _next_seq(self):
        # Klipper's 4-bit sequence number. Wire: (seq & 0x0F) | DEST.
        # MCU's next_sequence starts at MESSAGE_DEST=0x10 (i.e. seq_num=0),
        # so the very first packet we send must have seq_num=0; otherwise
        # the MCU NAKs and silently drops it. Read-then-increment.
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & msgproto.MESSAGE_SEQ_MASK
            return seq

    def _send_raw(self, cmd_str):
        """Encode `cmd_str` via MessageParser and write the framed bytes."""
        cmd = self._parser.create_command(cmd_str)
        if not cmd:
            return
        seq = self._next_seq()
        # Frame the message inline. msgproto.encode_msgblock has a latent
        # bug where it `append`s the CRC as a 2-element list instead of
        # `extend`ing it (Klipper itself avoids this path — production
        # framing happens in chelper/serialqueue.c). Open-coding the
        # framing here keeps us independent of that bug.
        msglen = msgproto.MESSAGE_MIN + len(cmd)
        seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
        payload = [msglen, seq_byte] + list(cmd)
        crc = msgproto.crc16_ccitt(payload)  # returns [hi, lo]
        payload.extend(crc)
        payload.append(msgproto.MESSAGE_SYNC)
        self._ser.write(bytes(payload))
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
                "Timed out after %.2fs waiting for response %r"
                % (timeout, name)
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
            except (OSError, serial.SerialException, TypeError) as exc:  # type: ignore[union-attr]
                if self._stop.is_set():
                    return
                logging.warning("kalico-host-io: serial read error: %s", exc)
                return
            if not chunk:
                continue
            try:
                packets = rxbuf.feed(chunk)
            except Exception as exc:
                logging.warning("kalico-host-io: framing error: %s", exc)
                continue
            for pkt in packets:
                # Handler-side seq sync: every received packet's seq byte is
                # MCU's current next_sequence. Keeping our counter aligned
                # protects subsequent sends from NAK loops.
                if len(pkt) >= 2:
                    with self._lock:
                        self._seq = pkt[1] & msgproto.MESSAGE_SEQ_MASK
                try:
                    params = self._parser.parse(pkt)
                except Exception as exc:
                    # Some MCU responses (NAKs, malformed) blow up the
                    # parser. Log + skip — never let a bad packet kill
                    # the whole RX thread.
                    logging.warning(
                        "kalico-host-io: parse error on pkt %s: %s",
                        pkt.hex() if hasattr(pkt, "hex") else pkt,
                        exc,
                    )
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
