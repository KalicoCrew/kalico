# Serial port management for firmware communication
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import threading

import serial

from . import chelper, msgproto, util
from .extras.danger_options import get_danger_options


class error(Exception):
    pass


class SerialReader:
    def __init__(self, reactor, warn_prefix="", mcu=None):
        self.reactor = reactor
        self.warn_prefix = warn_prefix
        self.mcu = mcu
        # Detect motion bridge — if present, skip C serialqueue allocation
        self._use_bridge = (
            mcu is not None
            and hasattr(mcu, "_motion_bridge")
            and mcu._motion_bridge is not None
        )
        # Serial port
        self.serial_dev = None
        self.msgparser = msgproto.MessageParser(warn_prefix=warn_prefix)
        # C interface (skipped when bridge is active)
        if self._use_bridge:
            self.ffi_main = self.ffi_lib = None
            self.serialqueue = None
            self.default_cmd_queue = None
            self.stats_buf = None
        else:
            self.ffi_main, self.ffi_lib = chelper.get_ffi()
            self.serialqueue = None
            self.default_cmd_queue = self.alloc_command_queue()
            self.stats_buf = self.ffi_main.new("char[4096]")
        # Threading
        self.lock = threading.Lock()
        self.background_thread = None
        # Message handlers
        self.handlers = {}
        self.register_response(self._handle_unknown_init, "#unknown")
        self.register_response(self.handle_output, "#output")
        # Sent message notification tracking
        self.last_notify_id = 0
        self.pending_notifications = {}

    def _bg_thread(self):
        response = self.ffi_main.new("struct pull_queue_message *")
        while True:
            self.ffi_lib.serialqueue_pull(self.serialqueue, response)
            count = response.len
            if count < 0:
                break
            if response.notify_id:
                params = {
                    "#sent_time": response.sent_time,
                    "#receive_time": response.receive_time,
                }
                completion = self.pending_notifications.pop(response.notify_id)
                self.reactor.async_complete(completion, params)
                continue
            params = self.msgparser.parse(response.msg[0:count])
            params["#sent_time"] = response.sent_time
            params["#receive_time"] = response.receive_time
            hdl = (params["#name"], params.get("oid"))
            try:
                with self.lock:
                    hdl = self.handlers.get(hdl, self.handle_default)
                    hdl(params)
            except:
                logging.exception(
                    "%sException in serial callback", self.warn_prefix
                )

    def _bridge_event_poller(self, eventtime):
        """Reactor timer: drain bridge runtime events and dispatch to handlers."""
        if not self._use_bridge or self.mcu is None:
            return self.reactor.NEVER
        bridge = self.mcu._motion_bridge
        handle = self.mcu._bridge_handle
        if bridge is None or handle is None:
            return self.reactor.NEVER
        now = eventtime
        # Drain up to 32 events per tick to avoid starving the reactor.
        for _ in range(32):
            ev = bridge.take_runtime_event(handle)
            if ev is None:
                break
            ev_type = ev.get("type")
            # Map event type to a msgproto-style name so existing handlers work.
            if ev_type == "status":
                name = "kalico_status_v6"
                # Count and log first few frames so Phase-3 verification can
                # grep the log without enabling DEBUG-level logging globally.
                if not hasattr(self, "_status_frame_count"):
                    self._status_frame_count = 0
                self._status_frame_count += 1
                if self._status_frame_count <= 5 or self._status_frame_count % 50 == 0:
                    logging.info(
                        "%s[bridge-async] kalico_status_v6 frame #%d engine_status=%s",
                        self.warn_prefix,
                        self._status_frame_count,
                        ev.get("engine_status", "?"),
                    )
            elif ev_type == "credit_freed":
                name = "kalico_credit_freed"
            elif ev_type == "fault":
                name = "kalico_fault"
            elif ev_type == "endstop_tripped":
                name = "kalico_endstop_tripped"
            elif ev_type == "output":
                # #output events go to the #output handler.
                # Populate #msg from the bridge event's 'msg' field so that
                # handle_output (which reads params["#msg"]) works correctly.
                name = "#output"
                ev["#name"] = "#output"
                ev["#sent_time"] = now
                ev["#receive_time"] = now
                ev["#msg"] = ev.get("msg", "")
                with self.lock:
                    hdl = self.handlers.get(("#output", None), self.handle_default)
                try:
                    hdl(ev)
                except Exception:
                    logging.exception("%sException in bridge output callback", self.warn_prefix)
                continue
            elif ev_type == "response":
                # Klipper-protocol Response frame the firmware emitted
                # unsolicited (analog_in_state, trsync_state, stats, …).
                # Bridge owns the wire so klippy's serialqueue never receives
                # these directly — route by (name, oid) to handlers registered
                # via register_response. Fields from the firmware are spread
                # into ev by the bridge; preserve them for the callback.
                name = ev.get("name", "")
                ev["#name"] = name
                ev["#sent_time"] = now
                ev["#receive_time"] = now
                oid = ev.get("oid")
                with self.lock:
                    hdl = (
                        self.handlers.get((name, oid))
                        or self.handlers.get((name, None))
                        or self.handle_default
                    )
                try:
                    hdl(ev)
                except Exception:
                    logging.exception(
                        "%sException in bridge response callback (name=%s, oid=%s)",
                        self.warn_prefix, name, oid,
                    )
                continue
            else:
                continue
            ev["#name"] = name
            ev["#sent_time"] = now
            ev["#receive_time"] = now
            hdl_key = (name, None)
            with self.lock:
                hdl = self.handlers.get(hdl_key, None)
            if hdl is None:
                hdl = self.handle_default
            try:
                hdl(ev)
            except Exception:
                logging.exception("%sException in bridge event callback", self.warn_prefix)
        # Bridge events carry runtime credit and status for the Rust planner.
        # During motion_toolhead.wait_moves(), Python releases the GIL while
        # the planner thread blocks on producer responses, so this timer is
        # the only Klippy-side drain path for runtime events. Keep it tight
        # enough that bridge progress is not dependent on unrelated reactor
        # wakeups.
        return eventtime + 0.001

    def _error(self, msg, *params):
        raise error(self.warn_prefix + (msg % params))

    def _get_identify_data(self, eventtime):
        # Query the "data dictionary" from the micro-controller
        identify_data = b""
        while True:
            msg = "identify offset=%d count=%d" % (len(identify_data), 40)
            try:
                params = self.send_with_response(msg, "identify_response")
            except error as e:
                logging.exception(
                    "%sWait for identify_response", self.warn_prefix
                )
                return None
            if params["offset"] == len(identify_data):
                msgdata = params["data"]
                if not msgdata:
                    # Done
                    return identify_data
                identify_data += msgdata

    def _start_session(self, serial_dev, serial_fd_type=b"u", client_id=0):
        self.serial_dev = serial_dev
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(
                serial_dev.fileno(), serial_fd_type, client_id
            ),
            self.ffi_lib.serialqueue_free,
        )
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start()
        # Obtain and load the data dictionary from the firmware
        completion = self.reactor.register_callback(self._get_identify_data)
        identify_data = completion.wait(self.reactor.monotonic() + 5.0)
        if identify_data is None:
            logging.info("%sTimeout on connect", self.warn_prefix)
            self.disconnect()
            return False
        msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
        msgparser.process_identify(identify_data)
        self.msgparser = msgparser
        self.register_response(self.handle_unknown, "#unknown")
        # Setup baud adjust
        if serial_fd_type == b"c":
            wire_freq = msgparser.get_constant_float("CANBUS_FREQUENCY", None)
        else:
            wire_freq = msgparser.get_constant_float("SERIAL_BAUD", None)
        if wire_freq is not None:
            self.ffi_lib.serialqueue_set_wire_frequency(
                self.serialqueue, wire_freq
            )
        receive_window = msgparser.get_constant_int("RECEIVE_WINDOW", None)
        if receive_window is not None:
            self.ffi_lib.serialqueue_set_receive_window(
                self.serialqueue, receive_window
            )
        return True

    def check_canbus_connect(
        self, canbus_uuid, canbus_nodeid, canbus_iface="can0"
    ):
        import can  # XXX

        try:
            uuid = int(canbus_uuid, 16)
        except ValueError:
            uuid = -1
        if uuid < 0 or uuid > 0xFFFFFFFFFFFF:
            self._error("Invalid CAN uuid")

        CANBUS_ID_ADMIN = 0x3F0
        CMD_QUERY_UNASSIGNED = 0x00
        CMD_QUERY_UNASSIGNED_EXTENDED = 0x01
        RESP_NEED_NODEID = 0x20
        RESP_HAVE_NODEID = 0x21
        filters = [
            {
                "can_id": CANBUS_ID_ADMIN + 1,
                "can_mask": 0x7FF,
                "extended": False,
            }
        ]

        msg = can.Message(
            arbitration_id=CANBUS_ID_ADMIN,
            data=[CMD_QUERY_UNASSIGNED, CMD_QUERY_UNASSIGNED_EXTENDED],
            is_extended_id=False,
        )
        try:
            bus = can.interface.Bus(
                channel=canbus_iface,
                can_filters=filters,
                bustype="socketcan",
            )
            bus.send(msg)
        except (can.CanError, os.error) as e:
            logging.warning("%scan issue: %s", self.warn_prefix, e)
            return False

        start_time = curtime = self.reactor.monotonic()
        while True:
            tdiff = start_time + 1.0 - curtime
            if tdiff <= 0.0:
                break
            msg = bus.recv(tdiff)
            curtime = self.reactor.monotonic()
            if (
                msg is None
                or msg.arbitration_id != CANBUS_ID_ADMIN + 1
                or msg.dlc < 7
                or msg.data[0] not in (RESP_NEED_NODEID, RESP_HAVE_NODEID)
            ):
                continue
            found_uuid = sum(
                [v << ((5 - i) * 8) for i, v in enumerate(msg.data[1:7])]
            )
            # logging.info(f"found_uuid: {hex(found_uuid)[2:]}")
            if found_uuid == uuid:
                self.disconnect()
                bus.close = bus.shutdown  # XXX
                return True
        bus.close = bus.shutdown  # XXX
        # logging.info(f"couldn't find uuid: {hex(uuid)[2:]}")
        return False

    def connect_canbus(self, canbus_uuid, canbus_nodeid, canbus_iface="can0"):
        import can  # XXX

        txid = canbus_nodeid * 2 + 256
        filters = [{"can_id": txid + 1, "can_mask": 0x7FF, "extended": False}]
        # Prep for SET_NODEID command
        try:
            uuid = int(canbus_uuid, 16)
        except ValueError:
            uuid = -1
        if uuid < 0 or uuid > 0xFFFFFFFFFFFF:
            self._error("Invalid CAN uuid")
        uuid = [(uuid >> (40 - i * 8)) & 0xFF for i in range(6)]
        CANBUS_ID_ADMIN = 0x3F0
        CMD_SET_NODEID = 0x01
        set_id_cmd = [CMD_SET_NODEID] + uuid + [canbus_nodeid]
        set_id_msg = can.Message(
            arbitration_id=CANBUS_ID_ADMIN,
            data=set_id_cmd,
            is_extended_id=False,
        )
        # Start connection attempt
        logging.info("%sStarting CAN connect", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while True:
            if self.reactor.monotonic() > start_time + 90.0:
                self._error("Unable to connect")
            try:
                bus = can.interface.Bus(
                    channel=canbus_iface,
                    can_filters=filters,
                    bustype="socketcan",
                )
                bus.send(set_id_msg)
            except (can.CanError, os.error) as e:
                logging.warning(
                    "%sUnable to open CAN port: %s", self.warn_prefix, e
                )
                self.reactor.pause(self.reactor.monotonic() + 5.0)
                continue
            bus.close = bus.shutdown  # XXX
            ret = self._start_session(bus, b"c", txid)
            if not ret:
                continue
            # Verify correct canbus_nodeid to canbus_uuid mapping
            try:
                params = self.send_with_response("get_canbus_id", "canbus_id")
                got_uuid = bytearray(params["canbus_uuid"])
                if got_uuid == bytearray(uuid):
                    break
            except:
                logging.exception(
                    "%sError in canbus_uuid check", self.warn_prefix
                )
            logging.info(
                "%sFailed to match canbus_uuid - retrying..", self.warn_prefix
            )
            self.disconnect()

    def connect_pipe(self, filename):
        logging.info("%sStarting connect", self.warn_prefix)
        if self._use_bridge:
            # Bridge mode: Rust reactor owns the FD.  Ask the bridge to open
            # the port, run the identify handshake, and return the raw dict
            # blob so klippy's msgparser can be populated normally.
            bridge = self.mcu._motion_bridge
            # claim_mcu may not have been called yet (it normally happens in
            # _mcu_identify after connect_pipe returns). Allocate the handle
            # here so attach_serial has something to bind to; the later guard
            # in _mcu_identify will skip the second claim_mcu call.
            if self.mcu._bridge_handle is None:
                self.mcu._bridge_handle = bridge.claim_mcu(
                    self.mcu._name,
                    filename,
                    0,
                )
            handle = self.mcu._bridge_handle
            logging.info("%sbridge attach_serial %s (handle=%s)",
                         self.warn_prefix, filename, handle)
            bridge.attach_serial(handle, filename, 0, timeout_s=30.0)
            identify_data = bridge.get_identify_data(handle)
            logging.info(
                "%sbridge identify done (%d bytes)", self.warn_prefix, len(identify_data)
            )
            msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
            msgparser.process_identify(identify_data)
            self.msgparser = msgparser
            self.register_response(self.handle_unknown, "#unknown")
            # Register a reactor timer that polls runtime events from the
            # bridge and dispatches them to klippy's registered handlers.
            # This is the inbound async path for kalico_status_v6 etc.
            self.reactor.register_timer(
                self._bridge_event_poller, self.reactor.NOW
            )
            return
        start_time = self.reactor.monotonic()
        while True:
            if self.reactor.monotonic() > start_time + 90.0:
                self._error("Unable to connect")
            try:
                fd = os.open(filename, os.O_RDWR | os.O_NOCTTY)
            except OSError as e:
                logging.warning(
                    "%sUnable to open port: %s", self.warn_prefix, e
                )
                self.reactor.pause(self.reactor.monotonic() + 5.0)
                continue
            serial_dev = os.fdopen(fd, "rb+", 0)
            ret = self._start_session(serial_dev)
            if ret:
                break

    def connect_uart(self, serialport, baud, rts=True):
        # Bridge mode: redirect to bridge path regardless of baud setting.
        if self._use_bridge:
            self.connect_pipe(serialport)
            return
        # Initial connection
        logging.info("%sStarting serial connect", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while 1:
            if (
                self.serialqueue is not None
            ):  # if we're already connected, don't recon
                break
            if self.reactor.monotonic() > start_time + 90.0:
                self._error("Unable to connect")
            try:
                serial_dev = serial.Serial(
                    baudrate=baud, timeout=0, exclusive=True
                )
                serial_dev.port = serialport
                serial_dev.rts = rts
                serial_dev.open()
            except (OSError, IOError, serial.SerialException) as e:
                logging.warning(
                    "%sUnable to open serial port: %s", self.warn_prefix, e
                )
                self.reactor.pause(self.reactor.monotonic() + 5.0)
                continue
            stk500v2_leave(serial_dev, self.reactor)
            ret = self._start_session(serial_dev)
            if ret:
                break

    def check_connect(self, serialport, baud, rts=True):
        serial_dev = serial.Serial(baudrate=baud, timeout=0, exclusive=False)
        serial_dev.port = serialport
        serial_dev.rts = rts
        try:
            serial_dev.open()
        except Exception:
            return False
        serial_dev.close()
        return True

    def connect_file(self, debugoutput, dictionary, pace=False):
        self.serial_dev = debugoutput
        self.msgparser.process_identify(dictionary, decompress=False)
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(self.serial_dev.fileno(), b"f", 0),
            self.ffi_lib.serialqueue_free,
        )

    def set_clock_est(self, freq, conv_time, conv_clock, last_clock):
        if self._use_bridge:
            bridge = self.mcu._motion_bridge
            handle = self.mcu._bridge_handle
            if bridge is not None and handle is not None:
                bridge.set_clock_est(
                    handle, float(freq), float(conv_time), int(last_clock)
                )
            return
        self.ffi_lib.serialqueue_set_clock_est(
            self.serialqueue, freq, conv_time, conv_clock, last_clock
        )

    def disconnect(self):
        if self._use_bridge:
            # Bridge manages its own serial lifecycle
            for pn in self.pending_notifications.values():
                pn.complete(None)
            self.pending_notifications.clear()
            return
        if self.serialqueue is not None:
            self.ffi_lib.serialqueue_exit(self.serialqueue)
            if self.background_thread is not None:
                self.background_thread.join()
            self.background_thread = self.serialqueue = None
        if self.serial_dev is not None:
            self.serial_dev.close()
            self.serial_dev = None
        for pn in self.pending_notifications.values():
            pn.complete(None)
        self.pending_notifications.clear()

    def stats(self, eventtime):
        if self._use_bridge:
            return "bridge_mode=1"
        if self.serialqueue is None:
            return ""
        self.ffi_lib.serialqueue_get_stats(
            self.serialqueue, self.stats_buf, len(self.stats_buf)
        )
        return str(self.ffi_main.string(self.stats_buf).decode())

    def get_reactor(self):
        return self.reactor

    def get_msgparser(self):
        return self.msgparser

    def get_serialqueue(self):
        if self._use_bridge:
            return None  # Bridge manages the serial queue in Rust
        return self.serialqueue

    def get_default_command_queue(self):
        return self.default_cmd_queue

    # Serial response callbacks
    def register_response(self, callback, name, oid=None):
        with self.lock:
            if callback is None:
                del self.handlers[name, oid]
            else:
                self.handlers[name, oid] = callback

    def _check_noncritical_disconnected(self):
        if self.mcu is not None and self.mcu.non_critical_disconnected:
            self._error("non-critical MCU is disconnected")

    # Command sending
    def raw_send(self, cmd, minclock, reqclock, cmd_queue):
        self._check_noncritical_disconnected()
        if self._use_bridge:
            # Bridge mode: Rust reactor owns the wire; periodic raw_send
            # calls (e.g. get_clock from clocksync) are no-ops here.
            # Clock sync is driven by the bridge via set_clock_est callbacks.
            return
        if self.serialqueue is None:
            return
        self.ffi_lib.serialqueue_send(
            self.serialqueue, cmd_queue, cmd, len(cmd), minclock, reqclock, 0
        )

    def raw_send_wait_ack(self, cmd, minclock, reqclock, cmd_queue):
        self._check_noncritical_disconnected()
        if self._use_bridge:
            # Bridge mode: no-op; bridge owns the wire.
            return {}
        if self.serialqueue is None:
            return
        self.last_notify_id += 1
        nid = self.last_notify_id
        completion = self.reactor.completion()
        self.pending_notifications[nid] = completion
        self.ffi_lib.serialqueue_send(
            self.serialqueue, cmd_queue, cmd, len(cmd), minclock, reqclock, nid
        )
        params = completion.wait()
        if params is None:
            self._error("Serial connection closed")
        return params

    def send(self, msg, minclock=0, reqclock=0):
        if self._use_bridge:
            # Bridge mode: send fire-and-forget via the Rust reactor.
            bridge = self.mcu._motion_bridge
            handle = self.mcu._bridge_handle
            if bridge is not None and handle is not None:
                bridge.bridge_send(handle, msg)
            return
        cmd = self.msgparser.create_command(msg)
        self.raw_send(cmd, minclock, reqclock, self.default_cmd_queue)

    def send_with_response(self, msg, response):
        if self._use_bridge:
            # Route through the Rust reactor which owns the FD.
            bridge = self.mcu._motion_bridge
            handle = self.mcu._bridge_handle
            params = bridge.bridge_call(handle, msg, response)
            # Inject timing fields klippy expects (reactor monotonic as approx).
            now = self.reactor.monotonic()
            params["#sent_time"] = now
            params["#receive_time"] = now
            return params
        cmd = self.msgparser.create_command(msg)
        src = SerialRetryCommand(self, response)
        return src.get_response([cmd], self.default_cmd_queue)

    def alloc_command_queue(self):
        if self._use_bridge:
            return None  # Bridge manages its own command queues
        return self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc_commandqueue(),
            self.ffi_lib.serialqueue_free_commandqueue,
        )

    # Dumping debug lists
    def dump_debug(self):
        if self._use_bridge:
            return "SerialReader: bridge mode (no C serialqueue)"
        out = []
        out.append(
            "Dumping serial stats: %s" % (self.stats(self.reactor.monotonic()),)
        )
        sdata = self.ffi_main.new("struct pull_queue_message[1024]")
        rdata = self.ffi_main.new("struct pull_queue_message[1024]")
        scount = self.ffi_lib.serialqueue_extract_old(
            self.serialqueue, 1, sdata, len(sdata)
        )
        rcount = self.ffi_lib.serialqueue_extract_old(
            self.serialqueue, 0, rdata, len(rdata)
        )
        out.append("Dumping send queue %d messages" % (scount,))
        for i in range(scount):
            msg = sdata[i]
            cmds = self.msgparser.dump(msg.msg[0 : msg.len])
            out.append(
                "Sent %d %f %f %d: %s"
                % (i, msg.receive_time, msg.sent_time, msg.len, ", ".join(cmds))
            )
        out.append("Dumping receive queue %d messages" % (rcount,))
        for i in range(rcount):
            msg = rdata[i]
            cmds = self.msgparser.dump(msg.msg[0 : msg.len])
            out.append(
                "Receive: %d %f %f %d: %s"
                % (i, msg.receive_time, msg.sent_time, msg.len, ", ".join(cmds))
            )
        return "\n".join(out)

    # Default message handlers
    def _handle_unknown_init(self, params):
        logging.debug(
            "%sUnknown message %d (len %d) while identifying",
            self.warn_prefix,
            params["#msgid"],
            len(params["#msg"]),
        )

    def handle_unknown(self, params):
        logging.warning(
            "%sUnknown message type %d: %s",
            self.warn_prefix,
            params["#msgid"],
            repr(params["#msg"]),
        )

    def handle_output(self, params):
        logging.info(
            "%s%s: %s", self.warn_prefix, params["#name"], params["#msg"]
        )

    def handle_default(self, params):
        if get_danger_options().log_serial_reader_warnings:
            logging.warning("%s got %s", self.warn_prefix, params)


# Class to send a query command and return the received response
class SerialRetryCommand:
    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.last_params = None
        self.serial.register_response(self.handle_callback, name, oid)

    def handle_callback(self, params):
        self.last_params = params

    def get_response(self, cmds, cmd_queue, minclock=0, reqclock=0, retry=True):
        retries = 5
        retry_delay = 0.010
        if not retry:
            retries = 0
        while 1:
            for cmd in cmds[:-1]:
                self.serial.raw_send(cmd, minclock, reqclock, cmd_queue)
            self.serial.raw_send_wait_ack(
                cmds[-1], minclock, reqclock, cmd_queue
            )
            params = self.last_params
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            if retries <= 0:
                self.serial.register_response(None, self.name, self.oid)
                raise error("Unable to obtain '%s' response" % (self.name,))
            reactor = self.serial.reactor
            reactor.pause(reactor.monotonic() + retry_delay)
            retries -= 1
            retry_delay *= 2.0


# Attempt to place an AVR stk500v2 style programmer into normal mode
def stk500v2_leave(ser, reactor):
    logging.debug("Starting stk500v2 leave programmer sequence")
    util.clear_hupcl(ser.fileno())
    origbaud = ser.baudrate
    # Request a dummy speed first as this seems to help reset the port
    ser.baudrate = 2400
    ser.read(1)
    # Send stk500v2 leave programmer sequence
    ser.baudrate = 115200
    reactor.pause(reactor.monotonic() + 0.100)
    ser.read(4096)
    ser.write(b"\x1b\x01\x00\x01\x0e\x11\x04")
    reactor.pause(reactor.monotonic() + 0.050)
    res = ser.read(4096)
    logging.debug("Got %s from stk500v2", repr(res))
    ser.baudrate = origbaud


def cheetah_reset(serialport, reactor):
    # Fysetc Cheetah v1.2 boards have a weird stateful circuitry for
    # configuring the bootloader. This sequence takes care of disabling it for
    # sure.
    # Open the serial port with RTS asserted
    ser = serial.Serial(baudrate=2400, timeout=0, exclusive=True)
    ser.port = serialport
    ser.rts = True
    ser.open()
    ser.read(1)
    reactor.pause(reactor.monotonic() + 0.100)
    # Toggle DTR
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    # Deassert RTS
    reactor.pause(reactor.monotonic() + 0.100)
    ser.rts = False
    reactor.pause(reactor.monotonic() + 0.100)
    # Toggle DTR again
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    reactor.pause(reactor.monotonic() + 0.100)
    ser.close()


# Attempt an arduino style reset on a serial port
def arduino_reset(serialport, reactor):
    # First try opening the port at a different baud
    ser = serial.Serial(serialport, 2400, timeout=0, exclusive=True)
    ser.read(1)
    reactor.pause(reactor.monotonic() + 0.100)
    # Then toggle DTR
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    reactor.pause(reactor.monotonic() + 0.100)
    ser.close()
