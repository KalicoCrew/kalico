"""Beacon MCU stub speaking klippy's msgproto wire protocol over a PTY."""

from __future__ import annotations

import logging
import os
import pty
import select
import struct
import threading
import time
from typing import Optional

from klippy import msgproto
from tools.sim_klippy.orchestrator.beacon_identify_dict import (
    CLOCK_FREQ,
    IDENTIFY_BLOB,
)


def _build_nvm_image() -> bytes:
    nvm = bytearray(65536 + 8)
    NVM_MODEL_F_COUNT_SENTINEL = 0xFFFFFFFF
    NVM_MODEL_ADC_COUNT_SENTINEL = 0xFFFF
    struct.pack_into(
        "<IH", nvm, 0, NVM_MODEL_F_COUNT_SENTINEL, NVM_MODEL_ADC_COUNT_SENTINEL
    )
    NVM_MODEL_VERSION_OFFSET = 12
    nvm[NVM_MODEL_VERSION_OFFSET] = 0x00
    return bytes(nvm)


NVM_IMAGE = _build_nvm_image()

DEFAULT_FREQUENCY_HZ = 5_400_000

DEFAULT_TEMP_RAW = 2048


class BeaconMcuStub:
    SAMPLE_RATE_HZ = 1600.0

    def __init__(self, pty_path: str, log_path: Optional[str] = None) -> None:
        self._pty_path = pty_path
        self._log_path = log_path
        self._z_target: float = 10.0
        self._stream_en: bool = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sample_thread: Optional[threading.Thread] = None
        self._master_fd: Optional[int] = None
        self._slave_fd: Optional[int] = None
        self._t0: float = time.monotonic()
        self._send_lock = threading.Lock()
        self._send_seq: int = 1
        self._inbuf = bytearray()
        self._parser = msgproto.MessageParser(warn_prefix="beacon-stub: ")
        self._parser.process_identify(IDENTIFY_BLOB, decompress=True)
        self._handlers = self._build_handlers()
        self.rx_byte_count: int = 0
        self.tx_sample_count: int = 0
        self.tx_frame_count: int = 0
        self._threshold_trigger: int = 0
        self._threshold_untrigger: int = 0
        self._home_active: bool = False
        self._home_trsync_oid: int = 0
        self._home_trigger_reason: int = 0
        self._home_trigger_invert: int = 0
        self._is_configured: bool = False
        self._committed_crc: int = 0
        self._trsync_oids: set = set()
        self._accel_stream_en: bool = False
        self._accel_scale_id: int = 0
        self._accel_thread: Optional[threading.Thread] = None
        self._accel_clock_at_last_emit: int = 0
        self._sample_index: int = 0
        self._clock_origin = time.monotonic()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        # Raw mode: the line discipline must not cook the binary serial wire (NL→CRLF etc.).
        import termios as _termios
        import tty as _tty

        try:
            _tty.setraw(slave_fd, _termios.TCSANOW)
        except _termios.error:
            pass
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, self._pty_path)
        self._slave_fd = slave_fd
        self._master_fd = master_fd
        import fcntl as _fcntl

        flags = _fcntl.fcntl(master_fd, _fcntl.F_GETFL)
        _fcntl.fcntl(master_fd, _fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self._thread = threading.Thread(
            target=self._reactor_loop, name="beacon-stub-rx", daemon=True
        )
        self._thread.start()

    def start_sample_stream(
        self, z_target_mm: float, rate_hz: float = SAMPLE_RATE_HZ
    ) -> None:
        self._z_target = z_target_mm
        if self._thread is None or not self._thread.is_alive():
            self.start()

    def set_z(self, z_mm: float) -> None:
        self._z_target = z_mm

    def stop(self) -> None:
        self._stop.set()
        if self._master_fd is not None:
            try:
                os.close(self._master_fd)
            except OSError:
                pass
            self._master_fd = None
        if self._slave_fd is not None:
            try:
                os.close(self._slave_fd)
            except OSError:
                pass
            self._slave_fd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sample_thread is not None:
            self._sample_thread.join(timeout=2.0)
            self._sample_thread = None
        if self._accel_thread is not None:
            self._accel_thread.join(timeout=2.0)
            self._accel_thread = None
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass

    def _now_clock(self) -> int:
        elapsed = time.monotonic() - self._clock_origin
        return int(elapsed * CLOCK_FREQ) & 0xFFFFFFFF

    def _now_clock_high(self) -> int:
        elapsed = time.monotonic() - self._clock_origin
        return (int(elapsed * CLOCK_FREQ) >> 32) & 0xFFFFFFFF

    def _send_msg(self, msgformat: str, **kwargs) -> None:
        if self._master_fd is None:
            return
        try:
            cmd = self._parser.lookup_command(msgformat).encode_by_name(
                **kwargs
            )
        except msgproto.error:
            logging.exception("beacon-stub: unknown msgformat %r", msgformat)
            return
        with self._send_lock:
            seq = self._send_seq
            seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
            payload = [msgproto.MESSAGE_MIN + len(cmd), seq_byte] + list(cmd)
            crc = msgproto.crc16_ccitt(payload)
            payload.extend(crc)
            payload.append(msgproto.MESSAGE_SYNC)
            self._send_seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK
            if self._send_seq == 0:
                self._send_seq = 1
            try:
                os.write(self._master_fd, bytes(payload))
            except (BlockingIOError, OSError):
                return
            self.tx_frame_count += 1
            self._log("tx", bytes(payload), msgformat, kwargs)

    def _reactor_loop(self) -> None:
        master_fd = self._master_fd
        while not self._stop.is_set():
            if master_fd is None:
                break
            try:
                r, _, _ = select.select([master_fd], [], [], 0.05)
            except (ValueError, OSError):
                break
            if not r:
                continue
            try:
                chunk = os.read(master_fd, 4096)
            except OSError:
                break
            if not chunk:
                continue
            self.rx_byte_count += len(chunk)
            self._inbuf.extend(chunk)
            self._drain_inbuf()

    def _drain_inbuf(self) -> None:
        while True:
            msglen = self._parser.check_packet(self._inbuf)
            if msglen == 0:
                return
            if msglen < 0:
                idx = self._inbuf.find(msgproto.MESSAGE_SYNC)
                if idx < 0:
                    self._inbuf.clear()
                    return
                del self._inbuf[: idx + 1]
                continue
            frame = list(self._inbuf[:msglen])
            del self._inbuf[:msglen]
            try:
                params = self._parser.parse(frame)
            except msgproto.error:
                logging.exception("beacon-stub: parse failed")
                continue
            self._dispatch(params, frame)

    def _dispatch(self, params: dict, frame: list) -> None:
        name = params.get("#name")
        self._log_inbound(name, params)
        handler = self._handlers.get(name)
        if handler is None:
            return
        try:
            handler(params)
        except Exception:
            logging.exception("beacon-stub: handler %r raised", name)

    def _build_handlers(self) -> dict:
        return {
            "identify": self._handle_identify,
            "get_uptime": self._handle_get_uptime,
            "get_clock": self._handle_get_clock,
            "get_config": self._handle_get_config,
            "allocate_oids": self._handle_noop,
            "finalize_config": self._handle_finalize_config,
            "emergency_stop": self._handle_noop,
            "clear_shutdown": self._handle_noop,
            "debug_nop": self._handle_noop,
            "debug_ping": self._handle_debug_ping,
            "debug_read": self._handle_debug_read,
            "debug_write": self._handle_noop,
            "beacon_stream": self._handle_beacon_stream,
            "beacon_set_threshold": self._handle_beacon_set_threshold,
            "beacon_home": self._handle_beacon_home,
            "beacon_stop_home": self._handle_beacon_stop_home,
            "beacon_nvm_read": self._handle_beacon_nvm_read,
            "beacon_contact_home": self._handle_noop,
            "beacon_contact_query": self._handle_beacon_contact_query,
            "beacon_contact_stop_home": self._handle_noop,
            "beacon_contact_set_latency_min": self._handle_noop,
            "beacon_contact_set_sensitivity": self._handle_noop,
            "config_trsync": self._handle_config_trsync,
            "trsync_start": self._handle_trsync_start,
            "trsync_set_timeout": self._handle_noop,
            "trsync_trigger": self._handle_trsync_trigger,
            "stepper_stop_on_trigger": self._handle_noop,
            "beacon_accel_stream": self._handle_beacon_accel_stream,
        }

    def _handle_noop(self, params: dict) -> None:
        return

    def _handle_identify(self, params: dict) -> None:
        offset = params["offset"]
        count = params["count"]
        if offset >= len(IDENTIFY_BLOB):
            data = b""
        else:
            data = IDENTIFY_BLOB[offset : offset + count]
        self._send_msg(
            "identify_response offset=%u data=%.*s",
            offset=offset,
            data=list(data),
        )

    def _handle_get_uptime(self, params: dict) -> None:
        self._send_msg(
            "uptime high=%u clock=%u",
            high=self._now_clock_high(),
            clock=self._now_clock(),
        )

    def _handle_get_clock(self, params: dict) -> None:
        self._send_msg("clock clock=%u", clock=self._now_clock())

    def _handle_get_config(self, params: dict) -> None:
        self._send_msg(
            "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
            is_config=1 if self._is_configured else 0,
            crc=self._committed_crc,
            is_shutdown=0,
            move_count=0,
        )

    def _handle_finalize_config(self, params: dict) -> None:
        self._committed_crc = params["crc"]
        self._is_configured = True

    def _handle_debug_ping(self, params: dict) -> None:
        self._send_msg("pong data=%*s", data=list(params["data"]))

    def _handle_debug_read(self, params: dict) -> None:
        self._send_msg("debug_result val=%u", val=0)

    def _handle_beacon_stream(self, params: dict) -> None:
        en = params["en"]
        self._stream_en = bool(en)
        if self._stream_en:
            self._start_sample_thread()
        else:
            self._stop_sample_thread()

    def _handle_beacon_set_threshold(self, params: dict) -> None:
        self._threshold_trigger = params["trigger"]
        self._threshold_untrigger = params["untrigger"]

    def _handle_beacon_home(self, params: dict) -> None:
        self._home_active = True
        self._home_trsync_oid = params["trsync_oid"]
        self._home_trigger_reason = params["trigger_reason"]
        self._home_trigger_invert = params["trigger_invert"]

    def _handle_beacon_stop_home(self, params: dict) -> None:
        self._home_active = False

    def _handle_beacon_nvm_read(self, params: dict) -> None:
        length = params["len"]
        offset = params["offset"]
        end = offset + length
        if end > len(NVM_IMAGE):
            data = NVM_IMAGE[offset:] + b"\xff" * (end - len(NVM_IMAGE))
        else:
            data = NVM_IMAGE[offset:end]
        self._send_msg(
            "beacon_nvm_data bytes=%*s offset=%hu",
            bytes=list(data),
            offset=offset,
        )

    def _handle_beacon_contact_query(self, params: dict) -> None:
        self._send_msg(
            "beacon_contact_state triggered=%c detect_clock=%u",
            triggered=0,
            detect_clock=0,
        )

    def _handle_config_trsync(self, params: dict) -> None:
        self._trsync_oids.add(params["oid"])

    def _handle_trsync_start(self, params: dict) -> None:
        self._trsync_oids.add(params["oid"])

    def _handle_trsync_trigger(self, params: dict) -> None:
        oid = params["oid"]
        reason = params["reason"]
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=oid,
            can_trigger=0,
            trigger_reason=reason,
            clock=self._now_clock(),
        )

    def _handle_beacon_accel_stream(self, params: dict) -> None:
        en = bool(params["en"])
        self._accel_scale_id = params["scale"]
        was_en = self._accel_stream_en
        self._accel_stream_en = en
        if en and not was_en:
            self._start_accel_thread()

    def _start_accel_thread(self) -> None:
        if self._accel_thread is not None and self._accel_thread.is_alive():
            return
        self._accel_thread = threading.Thread(
            target=self._accel_loop, name="beacon-stub-accel", daemon=True
        )
        self._accel_thread.start()

    def _accel_loop(self) -> None:
        SAMPLES_PER_BATCH = 6
        BATCH_PERIOD_S = 0.001
        Z_RAW_ONE_G_AT_2G_SCALE = 16384
        sample_bytes = struct.pack("<hhh", 0, 0, Z_RAW_ONE_G_AT_2G_SCALE)
        batch_payload = sample_bytes * SAMPLES_PER_BATCH
        next_tick = time.monotonic()
        last_clock = self._now_clock()
        while not self._stop.is_set() and self._accel_stream_en:
            now = time.monotonic()
            sleep_for = next_tick - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, BATCH_PERIOD_S))
                continue
            next_tick += BATCH_PERIOD_S
            cur_clock = self._now_clock()
            delta = (cur_clock - last_clock) & 0xFFFFFFFF
            self._send_msg(
                "beacon_accel_data start_clock=%u delta_clock=%u data=%*s",
                start_clock=last_clock,
                delta_clock=delta,
                data=list(batch_payload),
            )
            last_clock = cur_clock

    def _start_sample_thread(self) -> None:
        if self._sample_thread is not None and self._sample_thread.is_alive():
            return
        self._sample_thread = threading.Thread(
            target=self._sample_loop, name="beacon-stub-tx", daemon=True
        )
        self._sample_thread.start()

    def _stop_sample_thread(self) -> None:
        return

    def _sample_loop(self) -> None:
        period = 1.0 / self.SAMPLE_RATE_HZ
        next_tick = time.monotonic()
        while not self._stop.is_set() and self._stream_en:
            now = time.monotonic()
            sleep_for = next_tick - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, period))
                continue
            next_tick += period
            self._sample_index = (self._sample_index + 1) & 0x7FFFFFFF
            self._send_msg(
                "beacon_status clock=%u sample=%i frequency=%u temp=%hi",
                clock=self._now_clock(),
                sample=self._sample_index,
                frequency=DEFAULT_FREQUENCY_HZ,
                temp=DEFAULT_TEMP_RAW,
            )
            self.tx_sample_count += 1

    def _log(
        self,
        direction: str,
        data: bytes,
        msgformat: Optional[str] = None,
        kwargs: Optional[dict] = None,
    ) -> None:
        if self._log_path is None:
            return
        try:
            log_dir = os.path.dirname(self._log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            with open(self._log_path, "ab") as f:
                ts = f"{time.monotonic() - self._t0:.6f}"
                trailer = ""
                if msgformat is not None:
                    name = msgformat.split()[0]
                    args_repr = ""
                    if kwargs:
                        parts = []
                        for k, v in kwargs.items():
                            if (
                                isinstance(v, list)
                                and v
                                and isinstance(v[0], int)
                            ):
                                parts.append(f"{k}=<{len(v)} bytes>")
                            else:
                                parts.append(f"{k}={v}")
                        args_repr = " " + " ".join(parts)
                    trailer = f"  {name}{args_repr}"
                line = f"[{ts}][{direction}] {data.hex()}{trailer}\n".encode()
                f.write(line)
        except (OSError, ValueError):
            pass

    def _log_inbound(self, name: Optional[str], params: dict) -> None:
        if self._log_path is None or name is None:
            return
        try:
            log_dir = os.path.dirname(self._log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            with open(self._log_path, "ab") as f:
                ts = f"{time.monotonic() - self._t0:.6f}"
                light = {
                    k: (
                        f"<{len(v)} bytes>"
                        if isinstance(v, (bytes, bytearray))
                        else v
                    )
                    for k, v in params.items()
                    if not k.startswith("#")
                }
                line = f"[{ts}][rx-msg] {name} {light}\n".encode()
                f.write(line)
        except (OSError, ValueError):
            pass


BeaconSerialStub = BeaconMcuStub
