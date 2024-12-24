"""
Kalico compatibility for legacy Klipper extras and plugins

This hook provides two features:

1. Rewrite non-package imports, e.g. `import mcu` effectively becomes `import klippy.mcu as mcu`
2. Provide aliases in sys.modules to prevent double-execution of modules.

The second is especially important as `is` and `isinstance` checks will fail between two seperately
imported copies of the same module.
"""

import functools
import importlib.machinery
import pathlib
import sys

ROOT = pathlib.Path(__file__).parent


class KlippyPathFinder(importlib.machinery.PathFinder):
    @classmethod
    def install_hook(cls):
        # Place the KlippyMetaPathFinder *before* `PathFinder`
        sys.meta_path.insert(-1, KlippyPathFinder())

        # Setup aliases for already loaded modules
        for name in list(sys.modules.keys()):
            if name.startswith("klippy."):
                sys.modules.setdefault(
                    name.removeprefix("klippy."),
                    sys.modules[name],
                )

    @classmethod
    def patch_loader(cls, spec, *names):
        "Patch a spec loader to add aliases for a module after import"

        orig_exec_module = spec.loader.exec_module

        @functools.wraps(orig_exec_module)
        def aliased(module):
            try:
                return orig_exec_module(module)
            finally:
                for name in names:
                    if name not in sys.modules:
                        sys.modules[name] = module

        spec.loader.exec_module = aliased

    @classmethod
    def find_spec(cls, fullname, path=None, target=None):
        if not fullname.startswith("klippy."):
            parts = fullname.split(".")
            klippy_path = ROOT.joinpath(*parts)

            if (
                klippy_path.with_suffix(".py").is_file()
                or (klippy_path / "__init__.py").is_file()
            ):
                fullname = "klippy." + fullname

        spec = super().find_spec(fullname, path, target)

        if spec and spec.name.startswith("klippy."):
            cls.patch_loader(spec, spec.name.removeprefix("klippy."))

        return spec


def install():
    KlippyPathFinder.install_hook()
