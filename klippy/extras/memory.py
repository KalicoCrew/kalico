"""
Support for Serial I2C Memory (EEPROM, EERAM, FRAM, etc)
"""

from __future__ import annotations

import typing
import logging

from . import bus

if typing.TYPE_CHECKING:
    from ..configfile import ConfigWrapper
    from ..gcode import GCodeCommand, GCodeDispatch
    from ..printer import Printer

DEFAULT_I2C_ADDR = 0b01010000  # 0x50
DEFAULT_I2C_SPEED = 400000  # 400KHz, more stable than 1MHz


def hexdump(data: bytearray, offset: int = 0) -> str:
    # format a byte array as a hexdump
    # [    addr] .. .. .. .. .. .. .. ..  .. .. .. .. .. .. .. .. [ascii]
    # [end addr]

    res = []
    for i in range(0, len(data), 16):
        bs = bytearray(data[i : i + 16])
        line = "{:08x}  {:23}  {:23}  |{:16}|".format(
            offset + i,
            " ".join("{:02x}".format(b) for b in bs[:8]),
            " ".join("{:02x}".format(b) for b in bs[8:]),
            "".join((chr(b) if 32 <= b < 127 else ".") for b in bs),
        )
        res.append(line)
    res.append("{:08x}".format(offset + len(data)))
    return "\n".join(res)


class Memory:
    printer: Printer
    gcode: GCodeDispatch

    i2c: bus.MCU_I2C

    def __init__(self, config: ConfigWrapper):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=DEFAULT_I2C_ADDR, default_speed=400000
        )
        self.capacity = config.getint("capacity", minval=0x0001, maxval=0x10000)
        self.read_only = config.getboolean("read_only", False)

        ## TODO: Implement paged writing
        # self.page_size = config.getint(
        #     "page_size",
        #     default=self.capacity,
        #     minval=32,
        #     maxval=0x10000,
        # )

        self.current_address = 0x0
        self.last_result = None

        if config.getboolean("enable_gcode", False):
            self.gcode.register_mux_command(
                "MEMORY_READ", "NAME", self.name, self.cmd_READ
            )
            if not self.read_only:
                self.gcode.register_mux_command(
                    "MEMORY_WRITE", "NAME", self.name, self.cmd_WRITE
                )

    def write(self, address: int, value: bytearray):
        assert 0 <= address and address + len(value) <= self.capacity
        assert self.i2c._configured
        if value and self.read_only:
            raise self.printer.command_error(f"Memory {self.name} is read only")

        # Chunk write into 32 byte pages to fit klippy serial packets
        while value:
            chunk, value = value[:32], value[32:]
            hi, lo = address.to_bytes(2)
            logging.debug(
                f"[memory {self.name}] Writing {len(value)}B to {address} ({chunk!r})"
            )
            self.i2c.i2c_write([hi, lo, *list(chunk)])
            address += len(chunk)

    def read(self, address: int, length: int = 1) -> bytearray:
        assert self.i2c._configured

        logging.debug(f"[memory {self.name}] Dummy write for address {address}")
        hi, lo = address.to_bytes(2)
        self.i2c.i2c_write([hi, lo])
        response = bytearray()
        while length:
            chunk = min(length, 32)
            length -= chunk
            params = self.i2c.i2c_read([], chunk)
            logging.debug(
                f"[memory {self.name}] Read {chunk}B: {params['response']}"
            )
            response += bytearray(params["response"])
        return response

    def cmd_READ(self, gcmd: GCodeCommand):
        "Read data from a series"
        address = gcmd.get_int("ADDRESS", minval=0x0000, maxval=self.capacity)
        length = gcmd.get_int(
            "LENGTH", minval=1, maxval=(self.capacity) - address
        )

        data = self.read(address, length)
        gcmd.respond_info(hexdump(data, address))

    def cmd_WRITE(self, gcmd: GCodeCommand):
        """
        MEMORY_WRITE NAME= ADDRESS= [DATA=] [TEXT=]
        Write the value to memory at the specified address
        """
        address = gcmd.get_int("ADDRESS", minval=0, maxval=self.capacity - 1)
        if data := gcmd.get("DATA", None):
            val = bytes.fromhex(data)
        elif text := gcmd.get("TEXT", None):
            val = text.encode()
        else:
            raise gcmd.error("MEMORY_WRITE requires either DATA=")
        if address + len(val) > self.capacity:
            raise gcmd.error(
                f"Writing {len(val)}B at ${address:04x} would exceed the memory capacity ({self.capacity}B)"
            )

        self.write(address, val)

    def get_status(self, _):
        return {
            "address": self.current_address,
            "last_result": self.last_result,
        }


def load_config_prefix(config: ConfigWrapper):
    return Memory(config)
