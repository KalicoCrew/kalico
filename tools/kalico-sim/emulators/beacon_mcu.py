"""Faithful Beacon MCU stub.

Speaks klippy's msgproto wire protocol (zlib-compressed identify dict,
length-prefixed framed messages with crc16-ccitt + sync byte) over a
PTY so the live klippy `beacon` extra plugin can complete identify
handshake, `_build_config()`, NVM reads, threshold setup, and sample
streaming without modification. The stub is faithful enough to catch
host-side beacon-API regressions of the bricking class
(`clear_homing_state("z")` from beacon's compat layer) before they
reach hardware.

Architecture summary
--------------------
* ``klippy.msgproto.MessageParser`` is loaded with our identify dict
  (same blob beacon firmware would advertise).
* A reactor thread polls the PTY master fd, drains inbound frames via
  ``parser.check_packet`` / ``parser.parse``, and dispatches by name.
* A 1.6 kHz sample-stream thread emits ``beacon_status`` frames once
  ``beacon_stream en=1`` arrives.
* Outbound seq counter follows the standard (1..15 with MESSAGE_DEST
  bit set; data messages double as acks per
  ``klippy/chelper/serialqueue.c`` ``handle_message``).

Out of scope (separate commits): G28-Z proximity-trip plumbing, contact
homing semantics, accel data path, model auto-calibration, MCU temp /
supply-voltage extended stats accuracy.

The class keeps the legacy ``BeaconSerialStub`` name as an alias so
the conftest fixture and existing scaffolding tests don't have to be
rewired in lockstep with this commit.
"""

from __future__ import annotations

import logging
import math
import os
import pty
import select
import struct
import threading
import time
from typing import Optional

from klippy import msgproto

try:
    from tools.kalico_sim.emulators.beacon_identify_dict import (
        CLOCK_FREQ,
        IDENTIFY_BLOB,
    )
except ImportError:
    try:
        from tools.sim_klippy.orchestrator.beacon_identify_dict import (
            CLOCK_FREQ,
            IDENTIFY_BLOB,
        )
    except ImportError:
        from beacon_identify_dict import CLOCK_FREQ, IDENTIFY_BLOB


# Beacon's NVM blob is a small flash region whose layout is decoded by
# `BeaconMCUTempHelper.build_with_nvm` (offset 65534, 8 bytes) and
# `BeaconModelHelper.build_with_nvm` (offset 0, 20 bytes) in beacon.py.
# We mirror the "no calibration data" sentinel layout — beacon is happy
# with this and falls back to the SAVE_CONFIG-stored model defined in
# printer.cfg.
def _build_nvm_image() -> bytes:
    """Construct a 65536-byte NVM image with sentinel "uncalibrated" markers.

    Layout (matches beacon.py reverse-engineering):

    * offset 0 .. 19 (20 bytes) — model-temp metadata read at
      `BeaconTempModelBuilder.build_with_nvm`. We pack:
        - bytes 0..3   uint32 f_count   (0xFFFFFFFF — sentinel "no data")
        - bytes 4..5   uint16 adc_count (0xFFFF — sentinel "no data")
        - bytes 6..11  reserved (zero)
        - bytes 12..15 ver=0 + 3 reserved (V0 model branch, no params)
        - bytes 16..19 reserved
      With ver=0 and the f_count/adc_count sentinels, BeaconTempModelV0
      logs "parameters not found in nvm" and returns None. beacon then
      uses the SAVE_CONFIG model from printer.cfg, which is exactly what
      we want.
    * offset 65534 .. 65541 (8 bytes) — MCU-temp helper. We pack
      lower=0 / upper=0 so `BeaconMCUTempHelper.build_with_nvm` gets
      ref_room/ref_hot=1.0 and ADC values of 0; the helper still returns
      a (non-None) instance, but the read-temperature math becomes
      degenerate. That is fine for boot — beacon doesn't gate on temp
      coming from the helper, it just falls back to thermistor math
      when the model can't compensate.
    """
    nvm = bytearray(65536 + 8)  # +8 so offset=65534 read of length 8 fits
    # Model region (offset 0): leave zeroes except for the sentinels in
    # bytes 0..5 (f_count = 0xFFFFFFFF, adc_count = 0xFFFF).
    struct.pack_into("<IH", nvm, 0, 0xFFFFFFFF, 0xFFFF)
    # ver byte at offset 12 = 0 (V0 builder branch).
    nvm[12] = 0x00
    # MCU temp helper at offset 65534: 8 bytes of zero is fine.
    return bytes(nvm)


NVM_IMAGE = _build_nvm_image()

# Beacon proximity homing trigger frequency range. Real beacon model
# calibration produces frequencies on the order of a few MHz; the
# ZL exact frequency doesn't matter for boot — we report a constant
# value inside ``[beacon].model_range`` in printer.cfg's saved model
# so beacon's downstream model.freq_to_dist() never asserts. The
# saved model in printer.cfg uses a frequency-domain of ~1.85e-7 (in
# inverse-Hz), which is ~5.4 MHz frequency-wise — but the model is
# evaluated from a `count`, not a frequency, and the count→freq
# mapping uses CLOCK_FREQ. We pick a count that yields a frequency
# inside the saved model's domain.
DEFAULT_FREQUENCY_HZ = 5_400_000  # ≈ 1/1.85e-7

# Temp reported in beacon_status is fixed-point; beacon decodes via
# `temp / temp_smooth_count * inv_adc_max`. The smooth-count is 8 and
# inv_adc_max is 1/4095. We report a constant ADC value that decodes
# to approximately 25 °C through the configured thermistor. Boot
# doesn't gate on temperature — any value inside the thermistor domain
# works, so we use mid-range (2048).
DEFAULT_TEMP_RAW = 2048


class BeaconMcuStub:
    """msgproto-speaking stub for the beacon eddy-current probe.

    Lifecycle::

        stub = BeaconMcuStub("/tmp/klipper_sim_beacon", log_path="...")
        stub.start()
        # ... klippy connects, runs identify, completes _build_config ...
        stub.stop()

    The PTY slave is symlinked to ``pty_path`` so klippy's serial-open
    finds it at the configured path. Once klippy issues
    ``beacon_stream en=1`` we begin emitting ``beacon_status`` frames at
    1.6 kHz; the orchestrator can override the reported Z-derived
    frequency via :meth:`set_z`.
    """

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
        # Outbound seq counter — bottom 4 bits cycle 1..15, 0 indicates
        # uninitialised. The seq byte is built with MESSAGE_DEST.
        self._send_seq: int = 1
        self._inbuf = bytearray()
        self._parser = msgproto.MessageParser(warn_prefix="beacon-stub: ")
        self._parser.process_identify(IDENTIFY_BLOB, decompress=True)
        # Per-command handlers, dispatched by `#name` from parser.parse.
        self._handlers = self._build_handlers()
        # Counters exposed for tests / orchestrator instrumentation.
        self.rx_byte_count: int = 0
        self.tx_sample_count: int = 0
        self.tx_frame_count: int = 0
        # Threshold + homing state — populated by inbound commands; the
        # G28-Z trip path consumes them in a follow-up commit.
        self._threshold_trigger: int = 0
        self._threshold_untrigger: int = 0
        self._home_active: bool = False
        self._home_trsync_oid: int = 0
        self._home_trigger_reason: int = 0
        self._home_trigger_invert: int = 0
        # Config state — flipped to True after we receive finalize_config.
        # get_config replies with is_config=False / crc=0 until finalize,
        # then echoes back the crc klippy committed.
        self._is_configured: bool = False
        self._committed_crc: int = 0
        # trsync — klippy registers one MCU_trsync per MCU. We track the
        # OIDs that have been configured so we can react sensibly to
        # `trsync_trigger`, but no firmware-class behaviour is emulated
        # here yet (no periodic trsync_state report, no expire-timer).
        # Boot only requires the dictionary surface + acks; the
        # proximity-trip path lights up in a follow-up commit.
        self._trsync_oids: set = set()
        self._trsync_can_trigger: dict = {}
        self._trsync_trigger_reason: dict = {}
        # Accelerometer streaming — driven on by motors_sync (via
        # APIDumpHelper batch_bulk) and off by `BeaconAccelHelper.reinit`
        # at boot (beacon.py:3466). Independent thread; the data path
        # is `beacon_accel_data start_clock delta_clock data` with
        # 6-byte (xl xh yl yh zl zh) samples per batch.
        self._accel_stream_en: bool = False
        self._accel_scale_id: int = 0
        self._accel_thread: Optional[threading.Thread] = None
        self._accel_clock_at_last_emit: int = 0
        # Synthetic sample counter and clock origin (in MCU ticks).
        self._sample_index: int = 0
        # Use a shared clock origin so beacon_status.clock and the
        # uptime/clock replies stay coherent.
        self._clock_origin = time.monotonic()

        # --- Homing trigger state ---
        # When beacon_home or beacon_contact_home is received, a timer
        # fires the trsync trigger after a configurable delay to simulate
        # the nozzle reaching the bed surface. This is the core mechanism
        # that makes G28 Z and PROBE work in the simulator.
        self._homing_trigger_delay: float = 0.5  # seconds after home cmd
        self._homing_trigger_timer: Optional[threading.Timer] = None
        self._contact_homing_active: bool = False
        self._contact_trigger_clock: int = 0
        self._contact_trigger_sample: int = 0
        self._contact_trigger_freq: int = 0
        self._contact_triggered: bool = False

        # --- Z-aware frequency model ---
        # Beacon's eddy-current frequency varies with distance to bed.
        # Model: freq = base + coeff / (z_mm + offset)
        # The saved SAVE_CONFIG model uses domain [1.8359e-7, 1.8936e-7]
        # (inverse-Hz), meaning frequencies ~5.28–5.45 MHz.
        # At z=0:  freq = 5,450,000 Hz → count = 73,153,076
        # At z=2:  freq = 5,316,667 Hz → count = 71,361,745
        # At z=5:  freq = 5,283,333 Hz → count = 70,914,218
        # At z=10: freq = 5,268,182 Hz → count = 70,710,853
        self._z_current: float = 10.0  # current simulated Z height in mm
        self._freq_base: int = 5_183_000     # ~5.18 MHz base
        self._freq_coeff: float = 763_000.0  # tuned to match model polynomial
        self._freq_offset: float = 2.857     # wider spread across 0-5mm range
        # Z tracking during proximity homing
        self._homing_approach_speed: float = 5.0  # mm/s (matches homing_speed)
        self._homing_start_z: float = 10.0
        self._homing_start_time: float = 0.0

    # ------------------------------------------------------------------
    # Public lifecycle / orchestrator API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the PTY, symlink slave, start the reactor thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        # Put the slave into raw mode so the line discipline doesn't
        # cook bytes (echo, NL→CRLF, etc.) — beacon's serial wire is
        # binary and must pass through verbatim. This mirrors the same
        # fix in `tools/kalico_host_io.py:open_pipe_with_config` for
        # the H7 USB-CDC RX path.
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
        # Non-blocking master so a slow/absent reader on the slave side
        # can't deadlock the reactor. In the live klippy case the slave
        # is being drained by serialqueue.c's read thread continuously.
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
        """Compatibility shim used by the existing conftest fixture.

        Faithful streaming actually starts when klippy sends
        ``beacon_stream en=1`` — see :meth:`_handle_beacon_stream`.
        We honour ``z_target_mm`` so the orchestrator can pre-seed the
        position the stub will report once streaming kicks in, and we
        ensure the reactor thread is running so the PTY is ready.
        """
        self._z_target = z_target_mm
        if self._thread is None or not self._thread.is_alive():
            self.start()

    def set_z(self, z_mm: float) -> None:
        """Update the simulated Z position for frequency modeling."""
        self._z_target = z_mm
        self._z_current = z_mm

    def stop(self) -> None:
        """Signal both threads to exit and unlink the PTY symlink."""
        self._stop.set()
        # Closing the master fd unblocks `select` inside the reactor.
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

    # ------------------------------------------------------------------
    # Frame I/O — pure-msgproto, no klippy chelper dependency
    # ------------------------------------------------------------------

    def _now_clock(self) -> int:
        """Return monotonic-derived MCU tick count (uint32 wrap)."""
        elapsed = time.monotonic() - self._clock_origin
        return int(elapsed * CLOCK_FREQ) & 0xFFFFFFFF

    def _now_clock_high(self) -> int:
        """Return upper-32 bits of the 64-bit MCU clock used in `uptime`."""
        elapsed = time.monotonic() - self._clock_origin
        return (int(elapsed * CLOCK_FREQ) >> 32) & 0xFFFFFFFF

    def _send_msg(self, msgformat: str, **kwargs) -> None:
        """Encode a known msgproto message and write the framed bytes.

        The framing follows ``klippy.msgproto.encode_msgblock`` but
        open-coded because that helper has a latent bug where it
        ``append``s the CRC list as a single element instead of
        ``extend``ing it (klippy itself avoids the helper — production
        framing happens in ``chelper/serialqueue.c``). See
        ``tools/kalico_host_io.py:_send_raw`` for the same pattern.
        """
        if self._master_fd is None:
            return
        try:
            cmd = self._parser.lookup_command(msgformat).encode_by_name(**kwargs)
        except msgproto.error:
            logging.exception("beacon-stub: unknown msgformat %r", msgformat)
            return
        with self._send_lock:
            seq = self._send_seq
            seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
            payload = [msgproto.MESSAGE_MIN + len(cmd), seq_byte] + list(cmd)
            crc = msgproto.crc16_ccitt(payload)  # [hi, lo]
            payload.extend(crc)
            payload.append(msgproto.MESSAGE_SYNC)
            self._send_seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK
            if self._send_seq == 0:
                # Seq 0 is reserved as "uninitialised" in serialqueue.c —
                # skip it on rollover.
                self._send_seq = 1
            try:
                os.write(self._master_fd, bytes(payload))
            except (BlockingIOError, OSError):
                # Drop on full kernel buffer — serialqueue will NAK and
                # we'll retransmit on the next dispatch tick.
                return
            self.tx_frame_count += 1
            self._log("tx", bytes(payload), msgformat, kwargs)

    # ------------------------------------------------------------------
    # Reactor + dispatch
    # ------------------------------------------------------------------

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
                return  # need more data
            if msglen < 0:
                # Resync to the next sync byte.
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
            # Ack-only — every command beacon emits should have a handler;
            # if a foreign one arrives, we silently drop. Klippy treats
            # "no response" as "command, not query" so this is correct.
            return
        try:
            handler(params)
        except Exception:
            logging.exception("beacon-stub: handler %r raised", name)

    # ------------------------------------------------------------------
    # Handlers
    # ------------------------------------------------------------------

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
            "beacon_contact_home": self._handle_beacon_contact_home,
            "beacon_contact_query": self._handle_beacon_contact_query,
            "beacon_contact_stop_home": self._handle_beacon_contact_stop_home,
            "beacon_contact_set_latency_min": self._handle_noop,
            "beacon_contact_set_sensitivity": self._handle_noop,
            # trsync — klippy instantiates an `MCU_trsync` per MCU and
            # walks through config_trsync / trsync_start /
            # trsync_set_timeout / trsync_trigger / stepper_stop_on_trigger
            # at _build_config time. None require a query response; the
            # state stream is server-pushed via `trsync_state`, which
            # the proximity-trip follow-up will use to fire the homing
            # trigger. For this commit we ack everything and emit a
            # single benign `trsync_state can_trigger=1` after start so
            # klippy's MCU_trsync._handle_trsync_state has a baseline
            # callback shape to register against.
            "config_trsync": self._handle_config_trsync,
            "trsync_start": self._handle_trsync_start,
            "trsync_set_timeout": self._handle_noop,
            "trsync_trigger": self._handle_trsync_trigger,
            "stepper_stop_on_trigger": self._handle_noop,
            # Accelerometer — `BeaconAccelHelper.reinit` immediately
            # sends `beacon_accel_stream en=0 scale=0` on connect to
            # ensure streaming is off; motors_sync flips it back on
            # via the APIDumpHelper batch_bulk callback at sync time.
            "beacon_accel_stream": self._handle_beacon_accel_stream,
        }

    def _handle_noop(self, params: dict) -> None:
        # Klippy commands (vs queries) want no reply — the seq-byte ack
        # is implicit on whatever we send next. Returning here is correct.
        return

    def _handle_identify(self, params: dict) -> None:
        offset = params["offset"]
        count = params["count"]
        if offset >= len(IDENTIFY_BLOB):
            data = b""
        else:
            data = IDENTIFY_BLOB[offset : offset + count]
        self._send_msg("identify_response offset=%u data=%.*s",
                       offset=offset, data=list(data))

    def _handle_get_uptime(self, params: dict) -> None:
        self._send_msg("uptime high=%u clock=%u",
                       high=self._now_clock_high(), clock=self._now_clock())

    def _handle_get_clock(self, params: dict) -> None:
        self._send_msg("clock clock=%u", clock=self._now_clock())

    def _handle_get_config(self, params: dict) -> None:
        # is_config=0 / crc=0 on first reply makes klippy proceed to send
        # the configuration, then issue finalize_config, then re-query.
        # After finalize_config arrives we echo back is_config=1 and the
        # committed crc — that's what klippy's MCU._connect path expects
        # to see for "configured normally" (mcu.py:1411-1420).
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
        logging.info("beacon-stub: stream en=%d", en)
        if self._stream_en:
            self._start_sample_thread()
        else:
            self._stop_sample_thread()

    def _handle_beacon_set_threshold(self, params: dict) -> None:
        self._threshold_trigger = params["trigger"]
        self._threshold_untrigger = params["untrigger"]

    def _handle_beacon_home(self, params: dict) -> None:
        self._home_trsync_oid = params["trsync_oid"]
        self._home_trigger_reason = params["trigger_reason"]
        self._home_trigger_invert = params["trigger_invert"]
        if not self._home_active:
            self._homing_start_z = self._z_current
            self._homing_start_time = time.monotonic()
            self._home_active = True
            self._start_homing_monitor()
        logging.info(
            "beacon-stub: beacon_home trsync_oid=%d "
            "z=%.2f threshold=%d",
            self._home_trsync_oid, self._z_current,
            self._threshold_trigger,
        )

    def _handle_beacon_stop_home(self, params: dict) -> None:
        self._home_active = False

    def _start_homing_monitor(self) -> None:
        """Start a background thread that monitors Z and fires the trigger.

        On real beacon hardware, the firmware monitors the coil frequency
        independently of whether beacon_data streaming is on. The sample
        loop only runs when streaming is enabled, so this separate monitor
        handles the trigger check during homing when streaming is off.
        """
        t = threading.Thread(
            target=self._homing_monitor_loop,
            name="beacon-stub-homing",
            daemon=True,
        )
        t.start()

    def _homing_monitor_loop(self) -> None:
        CHECK_HZ = 200.0
        period = 1.0 / CHECK_HZ
        iter_count = 0
        while not self._stop.is_set() and self._home_active:
            time.sleep(period)
            elapsed = time.monotonic() - self._homing_start_time
            self._z_current = max(
                0.0,
                self._homing_start_z - elapsed * self._homing_approach_speed,
            )
            freq = self._z_to_frequency(self._z_current)
            count = self._freq_to_count(freq)
            iter_count += 1
            if iter_count % 40 == 0:
                logging.info(
                    "beacon-stub: homing monitor z=%.2f freq=%d "
                    "count=%d threshold=%d",
                    self._z_current, freq, count,
                    self._threshold_trigger,
                )
            if self._threshold_trigger > 0:
                if self._home_trigger_invert:
                    triggered = count <= self._threshold_untrigger
                else:
                    triggered = count >= self._threshold_trigger
                if triggered:
                    logging.info(
                        "beacon-stub: TRIGGER z=%.2f count=%d threshold=%d",
                        self._z_current, count, self._threshold_trigger,
                    )
                    self._fire_homing_trigger()
                    return

    def _handle_beacon_nvm_read(self, params: dict) -> None:
        length = params["len"]
        offset = params["offset"]
        end = offset + length
        if end > len(NVM_IMAGE):
            # Pad with 0xFF so beacon's struct.unpack on the trailing
            # region returns sentinel "uncalibrated" values.
            data = NVM_IMAGE[offset:] + b"\xff" * (end - len(NVM_IMAGE))
        else:
            data = NVM_IMAGE[offset:end]
        self._send_msg(
            "beacon_nvm_data bytes=%*s offset=%hu",
            bytes=list(data), offset=offset,
        )

    def _handle_beacon_contact_home(self, params: dict) -> None:
        """Start contact homing — fire trsync trigger after a delay."""
        self._contact_homing_active = True
        trsync_oid = params["trsync_oid"]
        trigger_reason = params["trigger_reason"]

        def _fire():
            if not self._contact_homing_active:
                return
            self._contact_homing_active = False
            self._contact_triggered = True
            self._contact_trigger_clock = self._now_clock()
            self._contact_trigger_sample = self._sample_index
            self._contact_trigger_freq = self._z_to_frequency(0.0)
            self._send_msg(
                "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
                oid=trsync_oid, can_trigger=0,
                trigger_reason=trigger_reason,
                clock=self._contact_trigger_clock,
            )

        if self._homing_trigger_timer is not None:
            self._homing_trigger_timer.cancel()
        self._homing_trigger_timer = threading.Timer(
            self._homing_trigger_delay, _fire
        )
        self._homing_trigger_timer.daemon = True
        self._homing_trigger_timer.start()

    def _handle_beacon_contact_stop_home(self, params: dict) -> None:
        self._contact_homing_active = False
        if self._homing_trigger_timer is not None:
            self._homing_trigger_timer.cancel()
            self._homing_trigger_timer = None

    def _handle_beacon_contact_query(self, params: dict) -> None:
        self._send_msg(
            "beacon_contact_state triggered=%c detect_clock=%u",
            triggered=1 if self._contact_triggered else 0,
            detect_clock=self._contact_trigger_clock,
        )

    # -- trsync -------------------------------------------------------

    def _handle_config_trsync(self, params: dict) -> None:
        self._trsync_oids.add(params["oid"])
        self._trsync_can_trigger = {}

    def _handle_trsync_start(self, params: dict) -> None:
        self._trsync_oids.add(params["oid"])
        self._trsync_can_trigger[params["oid"]] = True
        # Reset Z to initial height so the homing monitor can simulate
        # approach from above for the next pass (fixes rehome case where
        # a previous pass left _z_current at 0).
        self._z_current = self._homing_start_z = 10.0

    def _handle_trsync_trigger(self, params: dict) -> None:
        oid = params["oid"]
        reason = params["reason"]
        if self._trsync_can_trigger.get(oid, False):
            self._trsync_can_trigger[oid] = False
            self._trsync_trigger_reason[oid] = reason
        else:
            reason = self._trsync_trigger_reason.get(oid, reason)
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=oid, can_trigger=0, trigger_reason=reason,
            clock=self._now_clock(),
        )

    # -- accelerometer ------------------------------------------------

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
        """Emit `beacon_accel_data` batches at ~1 kHz.

        Each batch carries 8 samples of 6 bytes (xl xh yl yh zl zh) for
        a printer at rest: x=0, y=0, z=+1g (in raw 16-bit signed counts
        at the configured scale). 8 samples per batch keeps the wire
        rate below the MESSAGE_PAYLOAD_MAX limit while still emitting
        ~8 kSps — comfortably above motors_sync's
        ACCEL_FILTER_THRESHOLD.
        """
        # 1g full-scale at 2g range (scale_id=0): raw_z = (1g) / (2g/2^15)
        #                                             = 2^15 / 2 = 16384.
        # We pick the value matching the default 2g scale; for other
        # scales the accelerometer would emit different counts, but
        # motors_sync's filter doesn't care about absolute magnitude
        # at config time — only that batches arrive.
        # MESSAGE_PAYLOAD_MAX = MESSAGE_MAX - MESSAGE_MIN = 59 bytes.
        # Frame overhead: msgid (≤5) + start_clock (≤5) + delta_clock (≤5)
        # + buffer length prefix (1) ≈ 16. Six 6-byte samples = 36 bytes
        # data. Total ~52 — comfortably under the limit.
        SAMPLES_PER_BATCH = 6
        BATCH_PERIOD_S = 0.001  # 1 kHz batches → 6 kSps
        # Pre-build the per-sample 6-byte payload (constant Z = +1g).
        z_raw = 16384  # int16, +1g at ±2g scale.
        sample_bytes = bytes([
            0x00, 0x00,                               # x = 0
            0x00, 0x00,                               # y = 0
            z_raw & 0xFF, (z_raw >> 8) & 0xFF,        # z
        ])
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

    # ------------------------------------------------------------------
    # Sample stream
    # ------------------------------------------------------------------

    def _start_sample_thread(self) -> None:
        if self._sample_thread is not None and self._sample_thread.is_alive():
            return
        self._sample_thread = threading.Thread(
            target=self._sample_loop, name="beacon-stub-tx", daemon=True
        )
        self._sample_thread.start()

    def _stop_sample_thread(self) -> None:
        # The loop checks ``self._stream_en`` and exits naturally when it
        # flips false; we don't have to join from here (stop() will).
        return

    def _sample_loop(self) -> None:
        """Emit beacon_data batches (frequency samples) and periodic
        beacon_status (thermal telemetry).

        beacon_data uses delta compression: each sample is either a
        2-byte delta (if small) or a 4-byte absolute value.

        beacon_status carries MCU temp, supply voltage, and coil temp
        as raw ADC readings — emitted at ~10 Hz.
        """
        BATCH_HZ = 200.0  # frequency sample batches per second
        SAMPLES_PER_BATCH = 8  # samples per batch (1600 Hz total)
        STATUS_HZ = 10.0  # thermal status rate
        batch_period = 1.0 / BATCH_HZ
        status_period = 1.0 / STATUS_HZ

        next_batch = time.monotonic()
        next_status = time.monotonic()
        last_data_value = 0
        loop_iter_count = 0

        while not self._stop.is_set() and self._stream_en:
            now = time.monotonic()
            next_event = min(next_batch, next_status)
            sleep_for = next_event - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, batch_period))
                continue

            # Emit frequency data batch
            if now >= next_batch:
                next_batch += batch_period
                start_clock = self._now_clock()

                # During proximity homing, linearly decrease Z to simulate
                # the nozzle descending toward the bed at homing_speed.
                if self._home_active:
                    elapsed = now - self._homing_start_time
                    self._z_current = max(
                        0.0,
                        self._homing_start_z - elapsed * self._homing_approach_speed,
                    )

                freq = self._z_to_frequency(self._z_current)
                # Protocol uses COUNT values, not raw Hz.
                # count = freq * 2^28 / CLOCK_FREQ
                data_value = self._freq_to_count(freq)

                # Delta-compress samples
                buf = bytearray()
                for i in range(SAMPLES_PER_BATCH):
                    delta = data_value - last_data_value
                    if -8192 <= delta <= 8191:
                        # 2-byte delta encoding
                        encoded = (delta + 8192) & 0x3FFF
                        buf.append((encoded >> 8) & 0x7F)
                        buf.append(encoded & 0xFF)
                    else:
                        # 4-byte absolute encoding
                        buf.append(0x80 | ((data_value >> 24) & 0x7F))
                        buf.append((data_value >> 16) & 0xFF)
                        buf.append((data_value >> 8) & 0xFF)
                        buf.append(data_value & 0xFF)
                    last_data_value = data_value

                delta_clock = int(CLOCK_FREQ / (BATCH_HZ * SAMPLES_PER_BATCH)
                                  ) * SAMPLES_PER_BATCH
                self._send_msg(
                    "beacon_data data=%*s samples=%c start_clock=%u delta_clock=%u",
                    data=list(buf),
                    samples=SAMPLES_PER_BATCH,
                    start_clock=start_clock,
                    delta_clock=delta_clock,
                )
                self.tx_sample_count += SAMPLES_PER_BATCH
                loop_iter_count += 1

                # Check proximity homing trigger using COUNT values.
                # _threshold_trigger / _threshold_untrigger are already in
                # counts (set by beacon_set_threshold from klippy).
                if self._home_active and self._threshold_trigger > 0:
                    count = data_value
                    if self._home_trigger_invert:
                        triggered = count <= self._threshold_untrigger
                    else:
                        triggered = count >= self._threshold_trigger
                    if loop_iter_count % 40 == 0:
                        logging.info(
                            "beacon-stub: trigger check z=%.2f "
                            "count=%d threshold=%d triggered=%s",
                            self._z_current, count,
                            self._threshold_trigger, triggered,
                        )
                    if triggered:
                        logging.info(
                            "beacon-stub: TRIGGER FIRED z=%.2f "
                            "count=%d threshold=%d",
                            self._z_current, count,
                            self._threshold_trigger,
                        )
                        self._fire_homing_trigger()

            # Emit thermal status
            if now >= next_status:
                next_status += status_period
                self._send_msg(
                    "beacon_status mcu_temp=%u supply_voltage=%u coil_temp=%u status=%u",
                    mcu_temp=2048,        # raw ADC for MCU temp
                    supply_voltage=3300,  # raw ADC for supply
                    coil_temp=143_640,    # ~25°C with BEACON_ADC_SMOOTH_COUNT=200
                    status=0,             # nominal
                )

    def _z_to_frequency(self, z_mm: float) -> int:
        """Convert Z height (mm) to eddy-current frequency (Hz).

        Model: freq = base + coeff / (z + offset)
        Tuned to match the saved beacon model polynomial at z=0,2,5mm.
        At z=0:   ~5,450,000 Hz (count ≈ 73,153,076)
        At z=2:   ~5,340,000 Hz (count ≈ 71,675,060)  — crosses trigger threshold
        At z=5:   ~5,280,000 Hz (count ≈ 70,869,319)
        At z=10:  ~5,242,000 Hz
        """
        if z_mm < 0:
            z_mm = 0
        return int(self._freq_base + self._freq_coeff / (z_mm + self._freq_offset))

    def _freq_to_count(self, freq_hz: int) -> int:
        """Convert frequency (Hz) to beacon COUNT value.

        count = freq * 2^28 / CLOCK_FREQ  (CLOCK_FREQ = 20_000_000)

        This is the inverse of beacon firmware's count→freq mapping:
        freq = count * CLOCK_FREQ / 2^28
        """
        return int(freq_hz * (2 ** 28) / CLOCK_FREQ)

    def _fire_homing_trigger(self) -> None:
        """Fire the trsync trigger to signal homing completion."""
        if not self._home_active:
            return
        self._home_active = False
        oid = self._home_trsync_oid
        reason = self._home_trigger_reason
        self._trsync_can_trigger[oid] = False
        self._trsync_trigger_reason[oid] = reason
        oid = self._home_trsync_oid
        reason = self._home_trigger_reason
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=oid, can_trigger=0, trigger_reason=reason,
            clock=self._now_clock(),
        )

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log(self, direction: str, data: bytes,
             msgformat: Optional[str] = None,
             kwargs: Optional[dict] = None) -> None:
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
                        # Keep the log compact — bytes args render as
                        # "<N bytes>" rather than dumping every value.
                        parts = []
                        for k, v in kwargs.items():
                            if isinstance(v, list) and v and isinstance(
                                v[0], int
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
                # Drop binary-heavy fields for readability.
                light = {
                    k: (f"<{len(v)} bytes>" if isinstance(v, (bytes, bytearray))
                        else v)
                    for k, v in params.items() if not k.startswith("#")
                }
                line = f"[{ts}][rx-msg] {name} {light}\n".encode()
                f.write(line)
        except (OSError, ValueError):
            pass


# Backwards-compatible alias — the conftest fixture and existing
# scaffolding tests import the old name. The new class is a strict
# superset (same lifecycle / set_z / start_sample_stream surface), so
# the alias keeps the indirection cost at zero.
BeaconSerialStub = BeaconMcuStub
