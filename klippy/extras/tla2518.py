# TLA2518 8-channel 12-bit ADC support
#
# Copyright (C) 2023  Lasse Dalegaard <dalegaard@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bus

TLA2518_CHANNEL_COUNT = 8
REPORT_TIME = 0.1
SAMPLE_TIME = 128 / 1e6


class TLA2518:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.ppins = config.get_printer().load_object(config, "pins")
        self.config_error = config.error
        self.reactor = self.printer.get_reactor()
        self.name = " ".join(config.get_name().split()[1:])
        self.spi = bus.MCU_SPI_from_config(config, 0, default_speed=5000000)
        self.mcu = self.spi.get_mcu()
        self.mcu.register_config_callback(self._build_config)
        self.oid = self.mcu.create_oid()
        self.mcu.register_response(
            self._handle_response, "tla2518_result", self.oid
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

        self.channels = [None for _ in range(TLA2518_CHANNEL_COUNT)]

    def _build_config(self):
        channels = 0
        for channel in self.channels:
            if channel is not None:
                channels |= channel.bit

        self.adc_channel_mask = channels
        if channels != 0:
            self.mcu.add_config_cmd(
                "config_tla2518 oid=%u spi_oid=%u channels=%u"
                % (self.oid, self.spi.get_oid(), channels)
            )

    def _chip_reset(self):
        self._reg_write(0x01, 0x1)  # Write RST to GENERAL_CFG

        # Wait for reset
        timeout = self.reactor.monotonic() + 5.0
        while True:
            if self._reg_read(0x01) & 0x01 == 0:
                break
            now = self.reactor.monotonic()
            if now > timeout:
                msg = f"Timeout trying to reset TLA2518 {self.name}"
                self.printer.invoke_shutdown(msg)
                raise
            self.reactor.pause(now + 0.1)
        if self._reg_read(0x00) != 0x81:
            msg = f"TLA2518 '{self.name}' could not be reset"
            self.printer.invoke_shutdown(msg)
            raise

    def _handle_connect(self):
        if self.adc_channel_mask == 0:
            return

        self._chip_reset()
        # DATA_CFG: APPEND_STATUS = 01b, include channel ID
        self._reg_write(0x2, 1 << 4)
        # OSR_CFG: OSR_CFG = 0111b, 128 sample oversampling
        self._reg_write(0x3, 0x7)
        # SEQUENCE_CFG: SEQ_MODE=10b, on the fly mode
        self._reg_write(0x10, 0b10)
        clock = self.mcu.get_query_slot(self.oid)
        cmd = self.mcu.lookup_command(
            "query_tla2518 oid=%c clock=%u rest_ticks=%u sample_ticks=%u"
        )
        cmd.send(
            [
                self.oid,
                clock,
                self.mcu.seconds_to_clock(REPORT_TIME),
                self.mcu.seconds_to_clock(SAMPLE_TIME),
            ]
        )

    def _handle_response(self, params):
        idx = params["channel"]
        if idx >= len(self.channels):
            return
        channel = self.channels[idx]
        if channel is not None:
            channel._handle_response(params)

    def _reg_write(self, addr, value):
        self.spi.spi_send([0x08, addr, value])

    def _reg_read(self, addr):
        params = self.spi.spi_transfer_with_preface(
            [0x10, addr, 0x00],
            [0x00, 0x00, 0x00],
        )
        return params["response"][0]

    def _register_channel(self, channel, output):
        orig = channel
        if channel.startswith("adc"):
            channel = channel[3:]
        try:
            channel = int(channel)
        except ValueError:
            raise self.config_error(f"invalid TLA2518 channel '{orig}'")
        if channel < 0 or channel >= TLA2518_CHANNEL_COUNT:
            raise self.config_error(
                f"invalid TLA2518 channel '{orig}', valid range 0 to "
                f"{TLA2518_CHANNEL_COUNT - 1}"
            )
        cur = self.channels[channel]
        if cur is None:
            cur = self.channels[channel] = TLA2518Channel(self, channel)
        cur._register_handler(output)
        return cur

    # Pins interface

    def setup_pin(self, type, params):
        if type != "adc":
            raise self.ppins.error(
                "pin type %s not supported on TLA2518" % (type,)
            )
        return TLA2518ADC(self, params)


class TLA2518Channel:
    def __init__(self, chip, idx):
        self.idx = idx
        self.bit = 1 << idx
        self.handlers = []

    def _register_handler(self, output):
        self.handlers.append(output)

    def _handle_response(self, params):
        for handler in self.handlers:
            handler._handle_response(params)


class TLA2518ADC:
    def __init__(self, chip, params):
        self.channel = chip._register_channel(params["pin"], self)
        self.chip = chip
        self._callback = None

    def _handle_response(self, params):
        if self._callback:
            read_time = self.chip.mcu.clock_to_print_time(params["clock"])
            self._callback(read_time, params["value"] / 65535)

    # MCU_adc interface

    def setup_adc_callback(self, _report_time, callback):
        self._callback = callback

    def setup_minmax(
        self,
        sample_time,
        sample_count,
        minval=0.0,
        maxval=1.0,
        range_check_count=0,
    ):
        pass


def load_config_prefix(config):
    obj = TLA2518(config)
    chip_name = "tla2518_" + obj.name
    obj.ppins.register_chip(chip_name, obj)
