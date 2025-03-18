# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy.extras.load_cell.load_cell_probe import LoadCellPrinterProbe


def load_config(config):
    return LoadCellPrinterProbe(config)
