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
import zlib

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
    IDENTIFY_RESPONSE_DEADLINE = 0.050

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
        identify_decompressor = zlib.decompressobj()
        while True:
            offset = len(identify_data)
            cmd = "identify offset=%d count=%d" % (
                offset,
                self.IDENTIFY_CHUNK,
            )
            params = None
            sent_seq = None
            retransmitted_same_seq = False
            resync_attempts = 0
            while time.monotonic() < deadline:
                params = self._drain_available_sync(
                    "identify_response", True, sent_seq
                )
                if params is not None and params.get("offset") == offset:
                    break
                params = None
                if sent_seq is None:
                    sent_seq = self._send_raw(cmd)
                else:
                    self._send_raw(cmd, seq=sent_seq)
                attempt_deadline = min(
                    deadline,
                    time.monotonic() + self.IDENTIFY_RESPONSE_DEADLINE,
                )
                self._last_wait_saw_nak = False
                params = self._wait_packet_sync(
                    "identify_response",
                    attempt_deadline,
                    sync_seq=True,
                    sent_seq=sent_seq,
                )
                if params is not None:
                    if params.get("offset") == offset:
                        break
                    params = None  # stale or wrong offset; retry this query
                    continue
                if (
                    offset == 0
                    and self._last_wait_saw_nak
                    and resync_attempts < 20
                ):
                    # Reconnect to a running MCU may start with an unknown
                    # sequence. Honor the NAK's advertised next_sequence, but
                    # only before the live identify walk has made progress.
                    sent_seq = None
                    retransmitted_same_seq = False
                    resync_attempts += 1
                    continue
                if not retransmitted_same_seq:
                    retransmitted_same_seq = True
                    continue
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
            try:
                identify_decompressor.decompress(data)
            except zlib.error:
                # process_identify() will report the malformed dictionary
                # after the receive loop; don't mask that error here.
                pass
            if len(data) < self.IDENTIFY_CHUNK:
                break
            if identify_decompressor.eof:
                break
        self._parser.process_identify(identify_data)

    def _drain_available_sync(self, name=None, sync_seq=False, sent_seq=None):
        """Drain already-buffered serial bytes before writing another query."""
        found = None
        old_timeout = getattr(self._ser, "timeout", None)
        try:
            self._ser.timeout = 0
            while True:
                chunk = self._ser.read(4096)
                if not chunk:
                    break
                for pkt in self._rxbuf.feed(chunk):
                    params = self._handle_identify_sync_packet(
                        pkt, sync_seq, sent_seq, name
                    )
                    if params is not None and found is None:
                        found = params
        finally:
            self._ser.timeout = old_timeout
        return found

    def _wait_packet_sync(self, name, deadline, sync_seq=False, sent_seq=None):
        """Block (in the foreground thread) until a packet of `name` arrives.

        If sync_seq is True, data packets update self._seq to the MCU's
        expected next_sequence. Header-only length-5 packets are ACK/NAK
        frames with no msgid; classify them against sent_seq so stale ACKs
        from the previous identify query do not rewind us at 15->0 wrap.
        """
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._ser.timeout = max(0.05, min(remaining, 0.5))
            chunk = self._ser.read(256)
            for pkt in self._rxbuf.feed(chunk):
                params = self._handle_identify_sync_packet(
                    pkt, sync_seq, sent_seq, name
                )
                if params is not None:
                    return params
                # Pre-identify, drop everything else.

    def _handle_identify_sync_packet(self, pkt, sync_seq, sent_seq, name):
        # NAK/bare-ack frames are MESSAGE_MIN-length with no payload; skip
        # parse to avoid spurious "index out of range" / "Extra data" errors.
        if len(pkt) <= msgproto.MESSAGE_MIN:
            if sync_seq and len(pkt) >= 2:
                mcu_next = pkt[1] & msgproto.MESSAGE_SEQ_MASK
                if sent_seq is None:
                    self._set_seq(mcu_next)
                else:
                    sent = sent_seq & msgproto.MESSAGE_SEQ_MASK
                    ack_after_sent = (sent + 1) & msgproto.MESSAGE_SEQ_MASK
                    if mcu_next == ack_after_sent:
                        self._set_seq(mcu_next)
                    elif mcu_next != sent:
                        # Empty frame with any other seq is a NAK carrying
                        # the MCU's current expectation.
                        self._last_wait_saw_nak = True
                        self._set_seq(mcu_next)
                    # mcu_next == sent is a stale ACK for the previous command.
                    # Ignore it; accepting it rewinds seq at the 15->0 wrap.
            return None
        try:
            params = self._parser.parse(pkt)
        except Exception:
            # Defensive: malformed packet during identify. Already synced seq
            # above; drop the packet.
            return None
        if sync_seq and len(pkt) >= 2:
            self._set_seq(pkt[1] & msgproto.MESSAGE_SEQ_MASK)
        if params.get("#name") == name:
            return params
        return None

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

    def _set_seq(self, seq):
        with self._lock:
            self._seq = seq & msgproto.MESSAGE_SEQ_MASK

    def _send_raw(self, cmd_str, seq=None):
        """Encode `cmd_str` via MessageParser and write the framed bytes."""
        cmd = self._parser.create_command(cmd_str)
        if not cmd:
            return None
        if seq is None:
            seq = self._next_seq()
        else:
            seq &= msgproto.MESSAGE_SEQ_MASK
            with self._lock:
                self._seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK
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
        return seq

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
            except (
                OSError,
                serial.SerialException,
                TypeError,
                AttributeError,
            ) as exc:  # type: ignore[union-attr]
                # `AttributeError` covers pyserial's socket:// URL handler:
                # on `Serial.close()` it sets `_socket = None`, and an
                # in-flight `read()` then explodes with
                # `'NoneType' object has no attribute 'recv'` (instead of
                # raising SerialException). Treat the same as a clean shutdown.
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
                # NAK / bare-ack frames are MESSAGE_MIN-length (5 bytes:
                # 2-byte header + 2-byte CRC + 1-byte sync, zero payload).
                # Their `MESSAGE_HEADER_SIZE..len-TRAILER` window is empty;
                # `_parser.parse` would mis-decode the CRC bytes as a varint
                # msgid and either run off the end ("index out of range") or
                # leave bytes unconsumed ("Extra data at end of message").
                # We only consult the seq byte for resync purposes here.
                #
                # IMPORTANT — only NAKs (and only NAKs) advance our send-seq
                # counter. Every MCU→host message (response, output frame,
                # NAK) carries `MCU.next_sequence` in its seq byte, but only
                # NAKs warrant a forced realignment: data/output frames see
                # the seq AFTER the MCU acked previous host writes, so by
                # the time their seq arrives our `_seq` has already been
                # advanced past it via `_next_seq()`. Blindly clobbering
                # `_seq` to the received value (the original 25.5 logic)
                # creates a race where an output frame arriving between our
                # `_next_seq()` read and the MCU's processing rewinds our
                # counter behind what we already sent → MCU NAKs forever
                # because every retransmit goes out at a stale seq. Skip the
                # update on non-NAK packets entirely.
                if len(pkt) <= msgproto.MESSAGE_MIN:
                    if len(pkt) >= 2:
                        with self._lock:
                            mcu_next = pkt[1] & msgproto.MESSAGE_SEQ_MASK
                            # Only forward jumps. If MCU is asking us to
                            # rewind below where we are, there's an
                            # in-flight retransmit window we can't recover
                            # from without a real serialqueue — flag it but
                            # don't bind ourselves to a stale value.
                            mask = msgproto.MESSAGE_SEQ_MASK
                            delta = (mcu_next - self._seq) & mask
                            if delta != 0:
                                self._seq = mcu_next
                    continue
                try:
                    params = self._parser.parse(pkt)
                except Exception as exc:
                    # Real parse failure on a non-NAK packet — this is a
                    # genuine schema/wire bug worth surfacing. Log + skip.
                    logging.warning(
                        "kalico-host-io: parse error on pkt %s: %s",
                        pkt.hex() if hasattr(pkt, "hex") else pkt,
                        exc,
                    )
                    continue
                name = params.get("#name", "<noname>")
                self._ensure_queue(name).put(params)
                # Output-frame fan-out: `OutputFormat.parse` collapses every
                # `output()` emit to `#name="#output"` with the rendered
                # text in `#msg`. Tests want to wait on the *specific*
                # frame name (e.g. `kalico_status_v6`, `kalico_credit_freed`,
                # `kalico_fault`) — extract the leading whitespace-separated
                # token from `#msg` and re-publish into a per-name queue,
                # additionally promoting any `key=value` tokens to top-level
                # dict entries so consumers can `params["retired_through..."]`
                # the same way they would for a DECL_COMMAND response. The
                # legacy `#output` channel stays populated for callers that
                # want every output frame opaquely.
                if name == "#output":
                    msg = params.get("#msg", "")
                    tokens = msg.split() if msg else []
                    if tokens:
                        head, kvs = tokens[0], tokens[1:]
                        out_params = dict(params)
                        out_params["#name"] = head
                        for tok in kvs:
                            if "=" not in tok:
                                continue
                            k, v = tok.split("=", 1)
                            # Best-effort numeric coercion; fall back to str.
                            try:
                                if v.startswith("0x") or v.startswith("0X"):
                                    out_params[k] = int(v, 16)
                                else:
                                    out_params[k] = int(v)
                            except ValueError:
                                try:
                                    out_params[k] = float(v)
                                except ValueError:
                                    out_params[k] = v
                        self._ensure_queue(head).put(out_params)

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
