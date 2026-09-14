#!/usr/bin/env python3

# Copyright (C) 2026  Igor Baranov <iibaranov@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import argparse
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "lib" / "kconfiglib"))

import kconfiglib  # noqa: E402
import menuconfig  # noqa: E402


def usb_identity(kconf):
    return tuple(kconf.syms[name].str_value for name in ("MCU", "USB_PRODUCT"))


def update_automatic_usb_product(kconf, previous):
    previous_mcu, previous_product = previous
    current_mcu, current_product = usb_identity(kconf)
    if (
        previous_product == previous_mcu
        and current_mcu != previous_mcu
        and current_product == previous_product
    ):
        kconf.syms["USB_PRODUCT"].unset_value()
        return True
    return False


def run_menuconfig(kconfig):
    config = os.environ.get("KCONFIG_CONFIG", ".config")
    before = kconfiglib.Kconfig(kconfig, suppress_traceback=True)
    before.load_config(config)
    previous = usb_identity(before)

    menuconfig.menuconfig(kconfiglib.Kconfig(kconfig, suppress_traceback=True))

    # Reload the file so declining menuconfig's save prompt cannot persist
    # changes from its in-memory configuration.
    saved = kconfiglib.Kconfig(kconfig, suppress_traceback=True)
    saved.load_config(config)
    if update_automatic_usb_product(saved, previous):
        saved.write_config(config, save_old=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("menuconfig",))
    parser.add_argument("kconfig")
    args = parser.parse_args()
    if args.command == "menuconfig":
        run_menuconfig(args.kconfig)


if __name__ == "__main__":
    main()
