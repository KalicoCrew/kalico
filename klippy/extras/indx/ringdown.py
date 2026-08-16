import logging

from .compat import float_to_u32, poll_query_until, register_response

RINGDOWN_WAVE_PATH = "/tmp/indx_ringdown_waveform.csv"

# Mirrors MCU ringdown_status / nozzle_presence enums.
RINGDOWN_STATUS = {
    0: "idle",
    1: "running",
    2: "valid",
    3: "weak_signal",
    4: "aborted",
    5: "overvoltage",
    6: "timeout",
}
RINGDOWN_STATUS_IDLE = 0
RINGDOWN_STATUS_RUNNING = 1
RINGDOWN_PRESENCE = {0: "unknown", 1: "present", 2: "absent"}
RINGDOWN_PROBE_TIMEOUT = 5.0
RINGDOWN_PROBE_POLL_INTERVAL = 0.05
RINGDOWN_EXCITE_SCALE_DEFAULT = 0.8
RINGDOWN_EXCITE_SCALE_MAX = 1.0
# DUMP CSV annotation only; not a user config and not used for classification.
RINGDOWN_DUMP_ZERO_MARGIN_V = 1.0
RINGDOWN_PRESENT_PEAK_V_DEFAULT = 87.0
RINGDOWN_ABSENT_PEAK_V_DEFAULT = 91.0
RINGDOWN_MIN_PEAK_V_DEFAULT = 20.0
RINGDOWN_RESPONSE = (
    "indx_nozzle_presence clock=%u status=%c presence=%c peak_mv=%u"
)
RINGDOWN_SET_PARAMS_CMD = (
    "indx_set_ringdown_params enable=%c heat_gate=%c present_peak_v=%u "
    "absent_peak_v=%u idle_ms=%u heat_ms=%u excite_scale=%u min_peak_v=%u"
)


class RingdownParams:
    def __init__(
        self,
        enable,
        heat_gate,
        present_peak_v,
        absent_peak_v,
        min_peak_v,
        idle_ms,
        heat_ms,
        excite_scale,
    ):
        self.enable = enable
        self.heat_gate = heat_gate
        self.present_peak_v = present_peak_v
        self.absent_peak_v = absent_peak_v
        self.min_peak_v = min_peak_v
        self.idle_ms = idle_ms
        self.heat_ms = heat_ms
        self.excite_scale = excite_scale

    @classmethod
    def from_config(cls, config):
        present_peak_v = config.getfloat(
            "ringdown_present_peak_v",
            default=RINGDOWN_PRESENT_PEAK_V_DEFAULT,
            above=0.0,
            maxval=200.0,
        )
        absent_peak_v = config.getfloat(
            "ringdown_absent_peak_v",
            default=RINGDOWN_ABSENT_PEAK_V_DEFAULT,
            above=0.0,
            maxval=200.0,
        )
        if not (present_peak_v < absent_peak_v):
            raise config.error(
                "ringdown_present_peak_v must be less than ringdown_absent_peak_v"
            )
        return cls(
            enable=config.getboolean("ringdown_enable", False),
            heat_gate=config.getboolean("ringdown_heat_gate", False),
            present_peak_v=present_peak_v,
            absent_peak_v=absent_peak_v,
            min_peak_v=config.getfloat(
                "ringdown_min_peak_v",
                default=RINGDOWN_MIN_PEAK_V_DEFAULT,
                minval=0.0,
                maxval=200.0,
            ),
            idle_ms=config.getint(
                "ringdown_idle_ms", default=500, minval=20, maxval=5000
            ),
            heat_ms=config.getint(
                "ringdown_heat_ms", default=500, minval=50, maxval=10000
            ),
            excite_scale=config.getfloat(
                "ringdown_excite_scale",
                default=RINGDOWN_EXCITE_SCALE_DEFAULT,
                above=0.05,
                maxval=RINGDOWN_EXCITE_SCALE_MAX,
            ),
        )

    def apply_gcmd(self, gcmd):
        self.enable = bool(
            gcmd.get_int("ENABLE", int(self.enable), minval=0, maxval=1)
        )
        self.heat_gate = bool(
            gcmd.get_int("HEAT_GATE", int(self.heat_gate), minval=0, maxval=1)
        )
        self.present_peak_v = gcmd.get_float(
            "PRESENT_PEAK_V",
            self.present_peak_v,
            above=0.0,
            maxval=200.0,
        )
        self.absent_peak_v = gcmd.get_float(
            "ABSENT_PEAK_V",
            self.absent_peak_v,
            above=0.0,
            maxval=200.0,
        )
        if not (self.present_peak_v < self.absent_peak_v):
            raise gcmd.error("PRESENT_PEAK_V must be less than ABSENT_PEAK_V")
        self.min_peak_v = gcmd.get_float(
            "MIN_PEAK_V",
            self.min_peak_v,
            minval=0.0,
            maxval=200.0,
        )
        self.idle_ms = gcmd.get_int(
            "IDLE_MS", self.idle_ms, minval=20, maxval=5000
        )
        self.heat_ms = gcmd.get_int(
            "HEAT_MS", self.heat_ms, minval=50, maxval=10000
        )
        self.excite_scale = gcmd.get_float(
            "EXCITE_SCALE",
            self.excite_scale,
            above=0.05,
            maxval=RINGDOWN_EXCITE_SCALE_MAX,
        )

    def mcu_args(self):
        return [
            1 if self.enable else 0,
            1 if self.heat_gate else 0,
            float_to_u32(self.present_peak_v),
            float_to_u32(self.absent_peak_v),
            self.idle_ms,
            self.heat_ms,
            float_to_u32(self.excite_scale),
            float_to_u32(self.min_peak_v),
        ]

    def config_cmd(self):
        args = self.mcu_args()
        return (
            "indx_set_ringdown_params enable=%d heat_gate=%d present_peak_v=%u "
            "absent_peak_v=%u idle_ms=%u heat_ms=%u excite_scale=%u "
            "min_peak_v=%u" % tuple(args)
        )

    def save_config(self, configfile, section):
        configfile.set(
            section, "ringdown_enable", "True" if self.enable else "False"
        )
        configfile.set(
            section,
            "ringdown_heat_gate",
            "True" if self.heat_gate else "False",
        )
        configfile.set(
            section, "ringdown_present_peak_v", "%.3f" % self.present_peak_v
        )
        configfile.set(
            section, "ringdown_absent_peak_v", "%.3f" % self.absent_peak_v
        )
        configfile.set(section, "ringdown_min_peak_v", "%.3f" % self.min_peak_v)
        configfile.set(section, "ringdown_idle_ms", "%d" % self.idle_ms)
        configfile.set(section, "ringdown_heat_ms", "%d" % self.heat_ms)
        configfile.set(
            section, "ringdown_excite_scale", "%.4f" % self.excite_scale
        )

    def status_dict(self):
        return {
            "ringdown_enable": self.enable,
            "ringdown_heat_gate": self.heat_gate,
            "ringdown_excite_scale": self.excite_scale,
            "ringdown_present_peak_v": self.present_peak_v,
            "ringdown_absent_peak_v": self.absent_peak_v,
            "ringdown_min_peak_v": self.min_peak_v,
        }


def _waveform_peaks(samples):
    """Host-side DUMP annotation: off_start index and local-maxima sample indices."""
    zero_mv = int(RINGDOWN_DUMP_ZERO_MARGIN_V * 1000.0 + 0.5)
    off_start = ""
    for index, _ns, _counts, mv in samples:
        if mv >= zero_mv:
            off_start = index
            break
    peak_indices = set()
    for i in range(1, len(samples) - 1):
        prev_mv, mv, next_mv = (
            samples[i - 1][3],
            samples[i][3],
            samples[i + 1][3],
        )
        if mv >= prev_mv and mv > next_mv:
            peak_indices.add(samples[i][0])
    if samples:
        max_i = max(range(len(samples)), key=lambda i: samples[i][3])
        if samples[max_i][3] > 0:
            peak_indices.add(samples[max_i][0])
    return off_start, peak_indices


class IndxRingdown:
    def __init__(self, heater, config):
        self.heater = heater
        self.toolboard = heater.toolboard
        self.params = RingdownParams.from_config(config)
        self.nozzle_presence = "unknown"
        self.nozzle_presence_peak_v = 0.0
        self.nozzle_presence_status = "idle"
        self.nozzle_presence_time = 0.0
        self.gcode = self.toolboard.printer.lookup_object("gcode")
        self.gcode.register_command(
            "INDX_RINGDOWN_PROBE", self.cmd_RINGDOWN_PROBE
        )
        self.gcode.register_command(
            "INDX_SET_RINGDOWN_PARAMS", self.cmd_SET_RINGDOWN_PARAMS
        )
        self.toolboard.mcu.register_config_callback(self.build_config)

    def heating_allowed(self):
        if not self.params.enable or not self.params.heat_gate:
            return True
        return self.nozzle_presence == "present"

    def get_status(self, eventtime):
        age = None
        if self.nozzle_presence_time:
            age = max(0.0, eventtime - self.nozzle_presence_time)
        status = {
            "nozzle_presence": self.nozzle_presence,
            "nozzle_presence_peak_v": self.nozzle_presence_peak_v,
            "nozzle_presence_status": self.nozzle_presence_status,
            "nozzle_presence_age": age,
        }
        status.update(self.params.status_dict())
        return status

    def build_config(self):
        mcu = self.toolboard.mcu
        cq = self.heater.cmd_queue
        register_response(mcu, self.handle_nozzle_presence, RINGDOWN_RESPONSE)
        self.ringdown_probe_cmd = mcu.lookup_command(
            "indx_ringdown_probe dump=%c", cq=cq
        )
        self.query_ringdown_cmd = mcu.lookup_query_command(
            "indx_query_ringdown",
            RINGDOWN_RESPONSE,
            cq=cq,
        )
        self.cmd_set_ringdown_params = mcu.lookup_command(
            RINGDOWN_SET_PARAMS_CMD,
            cq=cq,
        )
        mcu.add_config_cmd(self.params.config_cmd())

    def _apply_result(self, params):
        status = params["status"]
        presence = params["presence"]
        self.nozzle_presence_status = RINGDOWN_STATUS.get(status, status)
        self.nozzle_presence = RINGDOWN_PRESENCE.get(presence, "unknown")
        self.nozzle_presence_peak_v = params["peak_mv"] / 1000.0
        self.nozzle_presence_time = (
            self.toolboard.printer.get_reactor().monotonic()
        )
        # Defer: MCU response handlers must not send commands on the same queue.
        self.toolboard.printer.get_reactor().register_async_callback(
            lambda _eventtime: self._heat_gate_block_if_needed()
        )

    def _heat_gate_block_if_needed(self):
        # Soft gate: drop the host target and warn. MCU already mutes drive.
        if self.heating_allowed():
            return
        heater = self.heater.heater
        if heater is None:
            return
        eventtime = self.toolboard.printer.get_reactor().monotonic()
        _temp, target = heater.get_temp(eventtime)
        if not target:
            return
        heater.set_temp(0.0)
        msg = (
            "INDX ringdown heat gate: nozzle not present "
            "(presence=%s). Heating disabled." % (self.nozzle_presence,)
        )
        logging.warning(msg)
        self.gcode.respond_info(msg)

    def handle_nozzle_presence(self, params):
        if params["status"] == RINGDOWN_STATUS_RUNNING:
            return
        self._apply_result(params)

    def _send_params(self):
        self.cmd_set_ringdown_params.send(self.params.mcu_args())

    def cmd_RINGDOWN_PROBE(self, gcmd):
        if self.heater.coil_timings is None:
            raise gcmd.error(
                "INDX ringdown requires calibrated coil timings. "
                "Run INDX_CALIBRATE first."
            )
        dump = gcmd.get_int("DUMP", 0, minval=0, maxval=1)
        reactor = self.toolboard.printer.get_reactor()
        mcu = self.toolboard.mcu
        samples = []
        meta = {}
        wave = meta_resp = None
        if dump:
            wave = register_response(
                mcu,
                lambda p: samples.append(
                    (p["index"], p["ns"], p["counts"], p["mv"])
                ),
                "indx_ringdown_wave index=%u ns=%u counts=%u mv=%u",
            )
            meta_resp = register_response(
                mcu,
                lambda p: meta.update(p),
                "indx_ringdown_meta status=%c peak_mv=%u",
            )
        try:
            self.ringdown_probe_cmd.send([dump])
            timeout = RINGDOWN_PROBE_TIMEOUT
            if dump:
                timeout += 5.0
            result = poll_query_until(
                reactor,
                self.query_ringdown_cmd,
                "status",
                RINGDOWN_STATUS_RUNNING,
                timeout,
                RINGDOWN_PROBE_POLL_INTERVAL,
            )
            if result is None:
                raise gcmd.error("INDX ringdown probe timed out")
        finally:
            if wave is not None:
                wave.unregister()
            if meta_resp is not None:
                meta_resp.unregister()
            if dump:
                off_start, peak_indices = _waveform_peaks(samples)
                with open(RINGDOWN_WAVE_PATH, "w") as f:
                    f.write(
                        "# off_start=%s zero_mv=%s n_peaks=%s status=%s "
                        "peak_mv=%s\n"
                        % (
                            off_start,
                            int(RINGDOWN_DUMP_ZERO_MARGIN_V * 1000.0 + 0.5),
                            len(peak_indices),
                            meta.get("status", ""),
                            meta.get("peak_mv", ""),
                        )
                    )
                    f.write("index,ns,counts,mv,is_peak\n")
                    for index, ns, counts, mv in samples:
                        is_peak = 1 if index in peak_indices else 0
                        f.write(
                            "%d,%d,%d,%d,%d\n"
                            % (index, ns, counts, mv, is_peak)
                        )
                gcmd.respond_info(
                    "INDX ringdown waveform (%d samples, %d peaks) saved to %s"
                    % (len(samples), len(peak_indices), RINGDOWN_WAVE_PATH)
                )
        self._apply_result(result)
        if result["status"] == RINGDOWN_STATUS_IDLE:
            raise gcmd.error(
                "INDX ringdown probe did not start (is another probe/tune active?)"
            )
        gcmd.respond_info(
            "INDX ringdown: presence=%s status=%s peak_v=%.1f"
            % (
                self.nozzle_presence,
                self.nozzle_presence_status,
                self.nozzle_presence_peak_v,
            )
        )

    def cmd_SET_RINGDOWN_PARAMS(self, gcmd):
        self.params.apply_gcmd(gcmd)
        self._send_params()
        configfile = self.toolboard.printer.lookup_object("configfile")
        self.params.save_config(configfile, self.toolboard.name)
        gcmd.respond_info(
            "INDX ringdown params updated (enable=%s heat_gate=%s "
            "present_peak_v=%.1f absent_peak_v=%.1f min_peak_v=%.1f "
            "idle_ms=%d heat_ms=%d excite_scale=%.3f). "
            "Run SAVE_CONFIG to persist."
            % (
                self.params.enable,
                self.params.heat_gate,
                self.params.present_peak_v,
                self.params.absent_peak_v,
                self.params.min_peak_v,
                self.params.idle_ms,
                self.params.heat_ms,
                self.params.excite_scale,
            )
        )
