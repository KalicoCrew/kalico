import sys


def hotpatch_modules():
    """
    This is a compatibility shim for legacy external modules
     to fix
    Redirect legacy `import x` to `import klippy.x`

    """

    for module_name, module in list(sys.modules.items()):
        if not module_name.startswith("klippy."):
            continue

        hotpatched_name = module_name.lstrip("klippy.")
        if hotpatched_name in sys.modules:
            continue

        sys.modules[hotpatched_name] = module
