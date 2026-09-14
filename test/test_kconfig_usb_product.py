import os
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "lib" / "kconfiglib"))

import kconfiglib  # noqa: E402

from scripts import kconfig as kconfig_script  # noqa: E402

KCONFIG = str(ROOT / "src" / "Kconfig")


def stm32_usb_config():
    kconf = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
    kconf.load_config(str(ROOT / "test" / "configs" / "stm32f446.config"))
    kconf.syms["LOW_LEVEL_OPTIONS"].set_value(2)
    kconf.syms["STM32_USB_PA11_PA12"].set_value(2)
    return kconf


def switch_to_rp2040(kconf):
    kconf.syms["MACH_RPXXXX"].set_value(2)
    kconf.syms["RPXXXX_USB"].set_value(2)


class USBProductTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.previous_cwd = os.getcwd()
        os.chdir(ROOT)
        subprocess.run(
            ["bash", str(ROOT / "scripts" / "find-firmware-extras.sh")],
            cwd=ROOT,
            check=True,
        )

    @classmethod
    def tearDownClass(cls):
        os.chdir(cls.previous_cwd)

    def saved_config(self, kconf):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        config = pathlib.Path(temporary.name) / ".config"
        kconf.write_config(str(config), save_old=False)
        return config

    def test_automatic_usb_product_follows_changed_mcu(self):
        config = self.saved_config(stm32_usb_config())
        changed = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
        changed.load_config(str(config))
        previous = kconfig_script.usb_identity(changed)
        switch_to_rp2040(changed)

        self.assertEqual(
            kconfig_script.usb_identity(changed),
            ("rp2040", "stm32f446xx"),
        )
        self.assertTrue(
            kconfig_script.update_automatic_usb_product(changed, previous)
        )
        self.assertEqual(
            kconfig_script.usb_identity(changed), ("rp2040", "rp2040")
        )

    def test_custom_usb_product_survives_changed_mcu(self):
        initial = stm32_usb_config()
        initial.syms["USB_PRODUCT"].set_value("My Printer")
        config = self.saved_config(initial)
        changed = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
        changed.load_config(str(config))
        previous = kconfig_script.usb_identity(changed)
        switch_to_rp2040(changed)

        self.assertFalse(
            kconfig_script.update_automatic_usb_product(changed, previous)
        )
        self.assertEqual(
            kconfig_script.usb_identity(changed), ("rp2040", "My Printer")
        )

    def test_usb_product_changed_during_switch_is_preserved(self):
        config = self.saved_config(stm32_usb_config())
        changed = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
        changed.load_config(str(config))
        previous = kconfig_script.usb_identity(changed)
        switch_to_rp2040(changed)
        changed.syms["USB_PRODUCT"].set_value("My RP2040")

        self.assertFalse(
            kconfig_script.update_automatic_usb_product(changed, previous)
        )
        self.assertEqual(
            kconfig_script.usb_identity(changed), ("rp2040", "My RP2040")
        )

    def test_menuconfig_refreshes_the_saved_automatic_product(self):
        config = self.saved_config(stm32_usb_config())

        def save_architecture_switch(kconf):
            kconf.load_config(str(config))
            switch_to_rp2040(kconf)
            kconf.write_config(str(config), save_old=False)

        with mock.patch.dict(os.environ, {"KCONFIG_CONFIG": str(config)}):
            with mock.patch.object(
                kconfig_script.menuconfig,
                "menuconfig",
                save_architecture_switch,
            ):
                kconfig_script.run_menuconfig(KCONFIG)

        saved = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
        saved.load_config(str(config))
        self.assertEqual(
            kconfig_script.usb_identity(saved), ("rp2040", "rp2040")
        )

    def test_menuconfig_does_not_persist_an_unsaved_switch(self):
        config = self.saved_config(stm32_usb_config())

        def abandon_architecture_switch(kconf):
            kconf.load_config(str(config))
            switch_to_rp2040(kconf)

        with mock.patch.dict(os.environ, {"KCONFIG_CONFIG": str(config)}):
            with mock.patch.object(
                kconfig_script.menuconfig,
                "menuconfig",
                abandon_architecture_switch,
            ):
                kconfig_script.run_menuconfig(KCONFIG)

        saved = kconfiglib.Kconfig(KCONFIG, suppress_traceback=True)
        saved.load_config(str(config))
        self.assertEqual(
            kconfig_script.usb_identity(saved),
            ("stm32f446xx", "stm32f446xx"),
        )


if __name__ == "__main__":
    unittest.main()
