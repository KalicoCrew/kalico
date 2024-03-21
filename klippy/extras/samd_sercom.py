# SAMD Sercom configuration
#
# Copyright (C) 2019  Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class SamdSERCOM:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.sercom = config.get("sercom")
        self.tx_pin = config.get("tx_pin")
        self.rx_pin = config.get("rx_pin", None)
        self.clk_pin = config.get("clk_pin")

        ppins = self.printer.lookup_object("pins")
        tx_pin_params = ppins.lookup_pin(self.tx_pin)
        self.tx_pin_params = tx_pin_params
        self.mcu = tx_pin_params["chip"]

        clk_pin_params = ppins.lookup_pin(self.clk_pin)
        self.clk_pin_params = clk_pin_params
        if self.mcu is not clk_pin_params["chip"]:
            raise ppins.error(
                "%s: SERCOM pins must be on same mcu" % (config.get_name(),)
            )

        if self.rx_pin:
            rx_pin_params = ppins.lookup_pin(self.rx_pin)
            self.rx_pin_params = rx_pin_params
            if self.mcu is not rx_pin_params["chip"]:
                raise ppins.error(
                    "%s: SERCOM pins must be on same mcu" % (config.get_name(),)
                )

        self.mcu.register_config_callback()

    def _build_config(self):
        self.mcu.add_config_cmd(
            "set_sercom_pin bus=%s sercom_pin_type=tx pin=%s"
            % (self.sercom, self.tx_pin_params["pin"])
        )
        self.mcu.add_config_cmd(
            "set_sercom_pin bus=%s sercom_pin_type=clk pin=%s"
            % (self.sercom, self.clk_pin_params["pin"])
        )
        if self.rx_pin:
            self.mcu.add_config_cmd(
                "set_sercom_pin bus=%s sercom_pin_type=rx pin=%s"
                % (self.sercom, self.rx_pin_params["pin"])
            )


def load_config_prefix(config):
    return SamdSERCOM(config)
