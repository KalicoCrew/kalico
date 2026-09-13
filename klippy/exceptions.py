# Exceptions that are broadly used in the klippy codebase
#
# Copyright (C) 2026  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class CommandError(Exception):
    pass


class WaitInterruption(CommandError):
    pass
