"""
Support for on-toolhead memory
"""

from __future__ import annotations

import copy
import dataclasses
import datetime
import struct
import uuid
from typing import TYPE_CHECKING, ClassVar

import msgpack

from ...mcu import error as mcu_error
from ...msgproto import crc16_ccitt
from ..memory import Memory

if TYPE_CHECKING:
    from ...configfile import ConfigWrapper
    from ...gcode import GCodeDispatch
    from ...printer import Printer
    from ...reactor import SelectReactor
    from .toolhead import CocoaToolheadControl


NULL_CRC16 = b"\xff\xff"  # crc16(b'')


class HeaderError(Exception): ...


class InvalidChecksum(HeaderError): ...


class InvalidMagic(HeaderError): ...


class MemoryError(Exception): ...


class MemoryNotConnected(MemoryError): ...


def timestamp_utc() -> int:
    return int(datetime.datetime.now(tz=datetime.timezone.utc).timestamp())


def datetime_from_utc_timestamp(ts: int) -> datetime.datetime:
    return datetime.datetime.fromtimestamp(ts, tz=datetime.timezone.utc)


@dataclasses.dataclass(frozen=True)
class Header:
    """
    On-toolhead memory data header format
    32 bytes long, to fit on a single eeprom page
    4 padding bytes could be used in the future to encode extra information
    Changes to the header *must* remain backwards compatible
    """

    struct: ClassVar = struct.Struct(
        "<"  # byte order
        "3s"  # magic, 3 bytes
        "B"  # version, 1 byte
        "16s"  # uuid, 16 bytes
        "xx"  # padding, 2 bytes
        "I"  # timestamp, 4 bytes
        "H"  # data length, 2 bytes
        "2s"  # data crc16, 2 bytes
        "2s"  # header crc16, 2 bytes
    )
    size: ClassVar = struct.size

    # constants
    # 0x00
    magic: bytes = dataclasses.field(default=b"\xc0\xc0\xa0", repr=False)
    # 0x03
    version: int = dataclasses.field(default=1, repr=False)
    # static
    # 0x04
    uid: uuid.UUID = dataclasses.field(default_factory=uuid.uuid1)
    # changing
    timestamp: int = dataclasses.field(default_factory=timestamp_utc)
    data_length: int = dataclasses.field(default=0)
    data_checksum: bytes = dataclasses.field(default=NULL_CRC16)

    @classmethod
    def from_bytes(cls, v: bytes):
        if v[:3] != cls.magic:
            raise InvalidMagic()

        if bytes(crc16_ccitt(v[:-2])) != v[-2:]:
            raise InvalidChecksum()

        (
            magic,
            version,
            uid_bytes,
            timestamp,
            data_length,
            data_checksum,
            _header_checksum,
        ) = cls.struct.unpack(v)

        return cls(
            magic,
            version,
            uuid.UUID(bytes=uid_bytes),
            timestamp,
            data_length,
            data_checksum,
        )

    def to_bytes(self) -> bytes:
        dat = self.struct.pack(
            self.magic,
            self.version,
            self.uid.bytes,
            self.timestamp,
            self.data_length,
            self.data_checksum,
            NULL_CRC16,
        )
        return dat[:-2] + bytes(crc16_ccitt(dat[:-2]))

    def update(self, *, data: bytes):
        "Return a new Header updated for the provided data"
        return dataclasses.replace(
            self,
            timestamp=timestamp_utc(),
            data_length=len(data),
            data_checksum=bytes(crc16_ccitt(data)),
        )


class CocoaMemory:
    printer: Printer
    reactor: SelectReactor
    config: ConfigWrapper
    gcode: GCodeDispatch

    name: str
    memory: Memory
    connected: bool
    header: Header
    config: dict
    _last_header: Header
    _last_config: dict

    def __init__(
        self, cocoa_toolhead: CocoaToolheadControl, config: ConfigWrapper
    ):
        self.config = config
        self.name = cocoa_toolhead.name
        self.logger = cocoa_toolhead.logger.getChild("memory")

        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        self.memory = Memory(config)
        self.backup_address = self.memory.capacity // 2

        self.connected = False
        self.header = self._last_header = None
        self.config = self._last_config = None

        self.printer.register_event_handler(
            "cocoa_toolhead:attached", self._on_attach
        )
        self.printer.register_event_handler(
            "cocoa_toolhead:detached", self._on_detach
        )

    def _on_attach(self, name):
        if self.name != name:
            return

        self.reactor.register_callback(self._attached)

    def _attached(self, _eventtime):
        try:
            header_data = self.memory.read(0, Header.size)
        except mcu_error as e:
            self.connected = False
            self.logger.exception(
                f"cocoa_memory[{self.name}] Unable to read memory"
            )
            return

        self.connected = True

        try:
            self.header = Header.from_bytes(header_data)
        except HeaderError:
            self.logger.debug(f"cocoa_memory[{self.name}] initializing")
            self._last_header = self.header = Header()
            self.memory.write(0, self.header.to_bytes())

        self.config = {}

        if self.header.data_length > 0:
            if self.header == self._last_header and self._last_config:
                self.logger.debug(
                    f"cocoa_memory[{self.name}] config unchanged from last attachment"
                )

            else:
                data = self.memory.read(
                    Header.size,
                    self.header.data_length,
                )
                self._last_config = msgpack.loads(data)

            self.config = copy.deepcopy(self._last_config)

        self.gcode.respond_info(
            f"cocoa_memory[{self.name}] {self.header}\n{self.config=}"
        )
        self.printer.send_event(
            "cocoa_memory:connected", self.name, self.config
        )

    def _on_detach(self, name):
        if self.name != name:
            return

        self.connected = False
        self.header = None
        self.config = None
        self.printer.send_event("cocoa_memory:disconnected", self.name)

    def has_changes(self):
        return self.config != self._last_config

    def save(self):
        if not self.has_changes():
            self.logger.debug(
                f"cocoa_memory[{self.name}] unchanged, nothing to save"
            )
            return

        # config = json.dumps(self.config, separators=(",", ":")).encode()
        config = msgpack.dumps(self.config)

        new_header = self.header.update(data=config)
        self.memory.write(new_header.size, config)
        self.memory.write(0, new_header.to_bytes())
        self.header = new_header

    def set(self, key: str, val):
        if not self.connected:
            raise MemoryNotConnected()
        self.config[key] = val

    def get(self, key: str, default=...):
        if not self.connected:
            raise MemoryNotConnected()
        val = self.config.get(key, default)
        if val is ...:
            raise KeyError(f"no such key {key!r} in cocoa_memory")
        return val

    def get_status(self, eventtime):
        return {
            # **self.memory.get_status(eventtime),
            "connected": self.connected,
            "uid": str(self.header.uid) if self.header else None,
            "timestamp": self.header.timestamp if self.header else None,
            "config": self.config,
            "changes_pending": self._last_config != self.config,
        }
