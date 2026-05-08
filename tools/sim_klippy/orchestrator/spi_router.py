"""Dispatcher that routes (cs, payload) frames to per-CS chip emulators.

Used as the framed-mode handler for ``ChipSocketServer`` when a single
sim SPI bus carries multiple chips. Each chip emulator registers itself
on the CS value reported by the firmware (the chardev gpio offset of the
chip's CS pin); transfers arriving on an unmapped CS raise ``KeyError``
so a misconfigured wiring blows up loudly rather than silently
mis-dispatching.
"""
from typing import Callable, Dict


ChipHandler = Callable[[bytes], bytes]


class SpiRouter:
    """Per-CS dispatcher for framed-mode ChipSocketServer.

    Usage::

        router = SpiRouter()
        router.attach(5, tmc_x.transfer)   # PC7 → chardev offset 5
        router.attach(40, max31865.transfer)  # PF8 → chardev offset 40
        server = ChipSocketServer(path, router, framed=True)
    """

    def __init__(self) -> None:
        self._chips: Dict[int, ChipHandler] = {}

    def attach(self, cs: int, handler: ChipHandler) -> None:
        if cs in self._chips:
            raise ValueError(f"sim spi: CS {cs} already attached")
        self._chips[cs] = handler

    def __call__(self, cs: int, payload: bytes) -> bytes:
        handler = self._chips.get(cs)
        if handler is None:
            attached = sorted(self._chips.keys())
            raise KeyError(
                f"sim spi: no chip on CS {cs} "
                f"(attached={attached}, payload_len={len(payload)})"
            )
        return handler(payload)
