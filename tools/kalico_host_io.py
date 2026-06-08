#!/usr/bin/env python3
import argparse
import logging
import pathlib
import queue as _queue
import sys
import threading
import time
import zlib

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "klippy"))

import msgproto  # noqa: E402

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


_SYNC = msgproto.MESSAGE_SYNC


class HostIoError(Exception):
    pass


class _RxBuffer:
    def __init__(self, parser):
        self._buf = bytearray()
        self._parser = parser

    def feed(self, chunk):
        if not chunk:
            return []
        self._buf.extend(chunk)
        out = []
        while self._buf:
            n = self._parser.check_packet(self._buf)
            if n == 0:
                break
            if n < 0:
                del self._buf[0]
                continue
            pkt = bytes(self._buf[:n])
            del self._buf[:n]
            out.append(pkt)
        return out


class KalicoHostIO:
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
        # serial_for_url handles socket:// (the sim bench's Renode USART2
        # socket); the bare Serial() constructor does not.
        if "://" in port:
            self._ser = serial.serial_for_url(port, baud, timeout=0.1)
        else:
            self._ser = serial.Serial(port, baud, timeout=0.1)
        self._parser = msgproto.MessageParser()
        self._seq = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._queues = {}
        self._queues_lock = threading.Lock()
        self._rxbuf = _RxBuffer(self._parser)
        self._do_identify(identify_timeout)
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="kalico-host-io-rx", daemon=True
        )
        self._rx_thread.start()

    def _do_identify(self, timeout):
        # Reconnecting to a running MCU: the USB-CDC buffer may hold stale
        # identify_response chunks from a prior klippy session, and the MCU's
        # next_sequence is wherever klippy last left it (only resets on MCU
        # reset). Drain stale RX first, then resync seq from NAKs, which carry
        # the MCU's current next_sequence in their seq byte.
        deadline = time.monotonic() + timeout
        drain_until = time.monotonic() + 0.3
        while time.monotonic() < drain_until:
            self._ser.timeout = 0.05
            n = len(self._ser.read(4096))
            if n == 0:
                break
        self._rxbuf = _RxBuffer(self._parser)

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
                    params = None
                    continue
                if (
                    offset == 0
                    and self._last_wait_saw_nak
                    and resync_attempts < 20
                ):
                    # Honor the NAK's advertised next_sequence only before the
                    # identify walk has made progress (offset==0); resyncing
                    # mid-walk would discard already-collected dictionary data.
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
                # process_identify() reports the malformed dictionary after
                # the loop; raising here would mask that clearer error.
                pass
            if len(data) < self.IDENTIFY_CHUNK:
                break
            if identify_decompressor.eof:
                break
        self._parser.process_identify(identify_data)

    def _drain_available_sync(self, name=None, sync_seq=False, sent_seq=None):
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

    def _handle_identify_sync_packet(self, pkt, sync_seq, sent_seq, name):
        # NAK/bare-ack frames are MESSAGE_MIN-length with no payload; parsing
        # them mis-decodes the CRC bytes as a msgid varint.
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
                        self._last_wait_saw_nak = True
                        self._set_seq(mcu_next)
                    # mcu_next == sent is a stale ACK; resyncing to it rewinds
                    # our seq across the 15->0 wrap and stalls the handshake.
            return None
        try:
            params = self._parser.parse(pkt)
        except Exception:
            return None
        if sync_seq and len(pkt) >= 2:
            self._set_seq(pkt[1] & msgproto.MESSAGE_SEQ_MASK)
        if params.get("#name") == name:
            return params
        return None

    def _next_seq(self):
        # The MCU's next_sequence starts at seq_num=0, so our first sent packet
        # must carry seq_num=0 or the MCU NAKs and silently drops it.
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & msgproto.MESSAGE_SEQ_MASK
            return seq

    def _set_seq(self, seq):
        with self._lock:
            self._seq = seq & msgproto.MESSAGE_SEQ_MASK

    def _send_raw(self, cmd_str, seq=None):
        cmd = self._parser.create_command(cmd_str)
        if not cmd:
            return None
        if seq is None:
            seq = self._next_seq()
        else:
            seq &= msgproto.MESSAGE_SEQ_MASK
            with self._lock:
                self._seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK
        # Frame inline rather than via msgproto.encode_msgblock, which appends
        # the CRC as a 2-element list instead of extending it (a latent bug on
        # a path Klipper never exercises — it frames in chelper/serialqueue.c).
        msglen = msgproto.MESSAGE_MIN + len(cmd)
        seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
        payload = [msglen, seq_byte] + list(cmd)
        crc = msgproto.crc16_ccitt(payload)
        payload.extend(crc)
        payload.append(msgproto.MESSAGE_SYNC)
        self._ser.write(bytes(payload))
        self._ser.flush()
        return seq

    def send(self, cmd_str):
        self._send_raw(cmd_str)

    def _ensure_queue(self, name):
        with self._queues_lock:
            q = self._queues.get(name)
            if q is None:
                q = _queue.Queue()
                self._queues[name] = q
            return q

    def wait_for_response(self, name, timeout):
        q = self._ensure_queue(name)
        try:
            return q.get(timeout=timeout)
        except _queue.Empty:
            raise HostIoError(
                "Timed out after %.2fs waiting for response %r"
                % (timeout, name)
            )

    def collect_responses(self, name, count, timeout):
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

    def _rx_loop(self):
        rxbuf = self._rxbuf
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
                # AttributeError: pyserial's socket:// handler nulls _socket on
                # close(), so an in-flight read() raises "'NoneType' has no
                # attribute 'recv'" instead of SerialException on shutdown.
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
                # Only NAKs may advance our send-seq counter. Every MCU->host
                # frame carries next_sequence in its seq byte, but data/output
                # frames carry the seq from AFTER the MCU acked prior writes,
                # so _next_seq() has already moved past it; resyncing to them
                # rewinds _seq behind what we sent and the MCU NAKs forever.
                # Bare-ack/NAK frames are MESSAGE_MIN-length with no payload;
                # parsing them mis-decodes the CRC bytes as a msgid varint.
                if len(pkt) <= msgproto.MESSAGE_MIN:
                    if len(pkt) >= 2:
                        with self._lock:
                            mcu_next = pkt[1] & msgproto.MESSAGE_SEQ_MASK
                            mask = msgproto.MESSAGE_SEQ_MASK
                            delta = (mcu_next - self._seq) & mask
                            if delta != 0:
                                self._seq = mcu_next
                    continue
                try:
                    params = self._parser.parse(pkt)
                except Exception as exc:
                    logging.warning(
                        "kalico-host-io: parse error on pkt %s: %s",
                        pkt.hex() if hasattr(pkt, "hex") else pkt,
                        exc,
                    )
                    continue
                name = params.get("#name", "<noname>")
                self._ensure_queue(name).put(params)
                # OutputFormat.parse collapses every output() to #name="#output"
                # with the text in #msg; re-publish under the leading token so
                # callers can wait on a specific frame name and read its
                # key=value fields as top-level params.
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

    def disconnect(self):
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)

    def get_msgparser(self):
        return self._parser

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.disconnect()
        return False


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
