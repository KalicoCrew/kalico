from __future__ import annotations

import typing

if typing.TYPE_CHECKING:
    from . import ConfigurationSection


class MCUConfigProto(typing.Protocol):
    @typing.overload
    def __call__(
        self,
        name: str = None,
        *,
        canbus_uuid: str,
        canbus_interface: str = "can0",
        restart_method: typing.Literal[
            None, "arduino", "cheetah", "command", "rpi_usb"
        ] = None,
        max_stepper_error: float = 0.000025,
        reconnect_interval: float = 2.0,
    ) -> ConfigurationSection: ...

    @typing.overload
    def __call__(
        self,
        name: str = None,
        *,
        serial: str,
        baud: int = 250000,
        restart_method: typing.Literal[
            None, "arduino", "cheetah", "command", "rpi_usb"
        ] = None,
        max_stepper_error: float = 0.000025,
        is_non_critical: bool = False,
        reconnect_interval: float = 2.0,
    ) -> ConfigurationSection: ...
