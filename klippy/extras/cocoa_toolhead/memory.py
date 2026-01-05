"""
Support for on-toolhead memory
"""

from __future__ import annotations

import copy
import dataclasses
import datetime
import struct
import uuid
from typing import TYPE_CHECKING, ClassVar, TypeVar

import msgpack

from ...gcode import CommandError
from ...msgproto import crc16_ccitt
from ..memory import Memory

if TYPE_CHECKING:
    from ...configfile import ConfigWrapper
    from ...gcode import GCodeDispatch
    from ...printer import Printer
    from ...reactor import SelectReactor
    from .toolhead import CocoaToolheadControl

    T = TypeVar("T")

AUTOSAVE_INTERVAL = 1.0
NULL_CRC16 = b"\xff\xff"  # crc16(b'')


class HeaderError(CommandError): ...


class InvalidChecksum(HeaderError): ...


class InvalidVersion(HeaderError): ...


class InvalidMagic(HeaderError): ...


class MemoryError(CommandError): ...


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
        "2s"  # magic, 2 bytes
        "B"  # version, 1 byte
        "16s"  # uuid, 16 bytes
        "xx"  # padding, 2 bytes
        "I"  # timestamp, 4 bytes
        "B"  # data page, 1 byte
        "H"  # data length, 2 bytes
        "2s"  # data crc16, 2 bytes
        "2s"  # header crc16, 2 bytes
    )
    size: ClassVar = struct.size

    # constants
    magic: bytes = dataclasses.field(default=b"\xc0\xc0", repr=False)
    version: int = dataclasses.field(default=1, repr=False)
    # static
    # 0x04
    uid: uuid.UUID = dataclasses.field(default_factory=uuid.uuid1)
    # changing
    timestamp: int = dataclasses.field(default_factory=timestamp_utc)
    data_page: int = dataclasses.field(default=0)
    data_length: int = dataclasses.field(default=0)
    data_checksum: bytes = dataclasses.field(default=NULL_CRC16)

    @classmethod
    def from_bytes(cls, v: bytes | bytearray):
        if v[:2] != cls.magic:
            raise InvalidMagic()
        if v[2] != cls.version:
            raise InvalidVersion()
        if bytes(crc16_ccitt(v[:-2])) != v[-2:]:
            raise InvalidChecksum()

        (
            magic,
            version,
            uid_bytes,
            timestamp,
            data_page,
            data_length,
            data_checksum,
            _header_checksum,
        ) = cls.struct.unpack(v)

        return cls(
            magic,
            version,
            uuid.UUID(bytes=uid_bytes),
            timestamp,
            data_page,
            data_length,
            data_checksum,
        )

    def to_bytes(self) -> bytes:
        dat = self.struct.pack(
            self.magic,
            self.version,
            self.uid.bytes,
            self.timestamp,
            self.data_page,
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

    def get_data_address(self):
        return 256 * (self.data_page + 1)


class CocoaMemory:
    printer: Printer
    reactor: SelectReactor
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
        self.name = cocoa_toolhead.name
        self.mux_name = cocoa_toolhead.mux_name
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

        self._save_timer = self.reactor.register_timer(
            self._autosave, self.reactor.NEVER
        )

    def _on_attach(self, name):
        if self.name != name:
            return

        self.reactor.register_callback(self._attached)

    def _attached(self, _eventtime):
        try:
            header_data = self.memory.read(0, Header.size)
        except CommandError:
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
                offset = 256 * (self.header.data_page + 1)
                data = self.memory.read(offset, self.header.data_length)
                self._last_config = msgpack.loads(data)

            self.config = copy.deepcopy(self._last_config)

        self.reactor.update_timer(
            self._save_timer, self.reactor.monotonic() + AUTOSAVE_INTERVAL
        )
        self.gcode.respond_info(
            f"cocoa_memory[{self.name}] {self.config.get('name', self.header.uid)} attached"
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
        self.reactor.update_timer(self._save_timer, self.reactor.NEVER)
        self.printer.send_event("cocoa_memory:disconnected", self.name)

    def _autosave(self, eventtime):
        if self.has_changes():
            self.save()
        return eventtime + AUTOSAVE_INTERVAL

    def has_changes(self):
        return self.config != self._last_config

    def save(self):
        self.logger.info(f"cocoa_memory[{self.name}] Saving {self.config=}")
        config = msgpack.dumps(self.config)

        new_header = self.header.update(data=config)
        self.memory.write(new_header.get_data_address(), config)
        self.memory.write(0, new_header.to_bytes())
        self.header = new_header
        self._last_config = copy.deepcopy(self.config)

    def has(self, key: str):
        if not self.connected:
            raise MemoryNotConnected()
        return key in self.config

    def get(self, key: str, default=...):
        if not self.connected:
            raise MemoryNotConnected()
        val = self.config.get(key, default)
        if val is ...:
            raise KeyError(f"no such key {key!r} in cocoa_memory")
        return val

    def set(self, key: str, val):
        if not self.connected:
            raise MemoryNotConnected()
        self.config[key] = val

    def setdefault(self, key: str, default: T) -> T:
        if not self.connected:
            raise MemoryNotConnected()
        return self.config.setdefault(key, default)

    def delete(self, key: str):
        if not self.connected:
            raise MemoryNotConnected()
        if key in self.config:
            del self.config[key]

    def get_status(self, eventtime):
        return {
            # **self.memory.get_status(eventtime),
            "connected": self.connected,
            "uid": str(self.header.uid) if self.header else None,
            "timestamp": self.header.timestamp if self.header else None,
            "config": self.config,
            "changes_pending": self._last_config != self.config,
        }
