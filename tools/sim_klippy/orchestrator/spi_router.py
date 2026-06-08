from typing import Callable, Dict

ChipHandler = Callable[[bytes], bytes]


class SpiRouter:
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
