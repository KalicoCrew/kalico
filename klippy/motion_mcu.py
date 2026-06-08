# MCU API surface that delegates to the Rust bridge instead of the C
# serialqueue path.
import logging


class MotionCommandWrapper:
    """Stand-in for mcu.CommandWrapper that routes through the bridge."""

    def __init__(self, proxy, msgformat, cq):
        self._proxy = proxy
        self._msgformat = msgformat
        self._cq = cq
        self._msgtag = hash(msgformat) & 0xFFFFFFFF

    def send(self, data=(), minclock=0, reqclock=0):
        logging.debug(
            "MotionCommandWrapper.send: %s data=%s", self._msgformat, data
        )

    def send_wait_ack(self, data=(), minclock=0, reqclock=0):
        logging.debug(
            "MotionCommandWrapper.send_wait_ack: %s data=%s",
            self._msgformat,
            data,
        )

    def get_command_tag(self):
        return self._msgtag


class MotionQueryCommandWrapper:
    """Stand-in for mcu.CommandQueryWrapper that routes through the bridge."""

    def __init__(self, proxy, msgformat, respformat, oid, cq):
        self._proxy = proxy
        self._msgformat = msgformat
        self._respformat = respformat
        self._oid = oid
        self._cq = cq

    def send(self, data=(), minclock=0, reqclock=0, retry=True):
        logging.debug(
            "MotionQueryCommandWrapper.send: %s data=%s",
            self._msgformat,
            data,
        )
        return {
            "#name": self._respformat.split()[0],
            "#sent_time": 0.0,
            "#receive_time": 0.0,
        }

    def send_with_preface(
        self,
        preface_cmd,
        preface_data=(),
        data=(),
        minclock=0,
        reqclock=0,
        retry=True,
    ):
        return self.send(data, minclock, reqclock, retry)


class MotionMcuProxy:
    """MCU public API surface, delegating to the Rust bridge. Non-motion
    peripherals flow through unchanged via the bridge's passthrough layer.
    """

    def __init__(self, bridge_wrapper, name, printer):
        self._bridge = bridge_wrapper
        self._name = name
        self._printer = printer
        self._reactor = printer.get_reactor()
        self._mcu_handle = None
        self._oid_count = 0
        self._config_callbacks = []
        self._config_cmds = []
        self._init_cmds = []
        self._restart_cmds = []
        self._constants = {}
        self._command_queue = None
        self._flush_callbacks = []
        self._stepqueues = []
        self._reserved_move_slots = 0
        self._get_status_info = {}
        self._is_shutdown = False
        self._mcu_freq = 0.0

        self.non_critical_disconnected = False
        self.is_non_critical = False

    def setup(self, serial_path, baud):
        self._mcu_handle = self._bridge.claim_mcu(self._name, serial_path, baud)
        # alias read by motion_toolhead._init_planner
        self._bridge_handle = self._mcu_handle
        self._command_queue = self._bridge.alloc_command_queue(self._mcu_handle)

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def is_fileoutput(self):
        return self._printer.get_start_args().get("debugoutput") is not None

    def is_shutdown(self):
        return self._is_shutdown

    def create_oid(self):
        oid = self._oid_count
        self._oid_count += 1
        return oid

    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        if on_restart:
            self._restart_cmds.append(cmd)
        elif is_init:
            self._init_cmds.append(cmd)
        else:
            self._config_cmds.append(cmd)

    def lookup_command(self, msgformat, cq=None):
        return MotionCommandWrapper(self, msgformat, cq)

    def lookup_query_command(
        self, msgformat, respformat, oid=None, cq=None, is_async=False
    ):
        return MotionQueryCommandWrapper(self, msgformat, respformat, oid, cq)

    def try_lookup_command(self, msgformat):
        return self.lookup_command(msgformat)

    def register_response(self, cb, msg, oid=None):
        if self._mcu_handle is not None and cb is not None:
            self._bridge.passthrough_register_handler(
                self._mcu_handle, msg, oid or 0, cb
            )

    def register_flush_callback(self, callback):
        self._flush_callbacks.append(callback)

    def alloc_command_queue(self):
        if self._mcu_handle is not None:
            return self._bridge.alloc_command_queue(self._mcu_handle)
        return None

    def get_default_command_queue(self):
        return self._command_queue

    def estimated_print_time(self, eventtime):
        return eventtime

    def print_time_to_clock(self, print_time):
        return int(print_time * self._mcu_freq) if self._mcu_freq else 0

    def clock_to_print_time(self, clock):
        return clock / self._mcu_freq if self._mcu_freq else 0.0

    def seconds_to_clock(self, seconds):
        return int(seconds * self._mcu_freq) if self._mcu_freq else 0

    def clock_to_seconds(self, clock):
        return clock / self._mcu_freq if self._mcu_freq else 0.0

    def clock32_to_clock64(self, clock32):
        return clock32

    def get_constant_float(self, name):
        val = self._constants.get(name)
        if val is None:
            raise KeyError("Unknown constant '%s'" % name)
        return float(val)

    def get_constants(self):
        return dict(self._constants)

    def get_enumerations(self):
        return {}

    def get_query_slot(self, oid):
        return 0

    def register_stepqueue(self, stepqueue):
        self._stepqueues.append(stepqueue)

    def request_move_queue_slot(self):
        self._reserved_move_slots += 1

    def min_schedule_time(self):
        return 0.100

    def max_nominal_duration(self):
        return 3.0

    def flush_moves(self, print_time, clear_history_time):
        pass

    def check_active(self, print_time, eventtime):
        pass

    def get_status(self, eventtime=None):
        return dict(self._get_status_info)

    def stats(self, eventtime):
        return False, "%s: phase_1_bridge" % (self._name,)

    def get_shutdown_clock(self):
        return 0

    def setup_pin(self, pin_type, pin_params):
        # Import here to avoid circular import at module level
        from . import mcu as mcu_mod

        pcs = {
            "digital_out": mcu_mod.MCU_digital_out,
            "pwm": mcu_mod.MCU_pwm,
            "adc": mcu_mod.MCU_adc,
        }
        if pin_type not in pcs:
            from . import pins

            raise pins.error("pin type %s not supported on mcu" % (pin_type,))
        return pcs[pin_type](self, pin_params)

    def dump_debug(self):
        return "MotionMcuProxy '%s': phase_1 bridge mode" % self._name
