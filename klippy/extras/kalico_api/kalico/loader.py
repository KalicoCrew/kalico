# This sucks, but it gives a way to share a global within the kalico module
from __future__ import annotations

import typing

if typing.TYPE_CHECKING:
    from .. import MacroLoader


class _LoadContext:
    def set_loader(self, loader: MacroLoader):
        self.loader = loader


load_context = _LoadContext()

__all__ = ()
