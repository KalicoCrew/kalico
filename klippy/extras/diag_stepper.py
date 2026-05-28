# Minimal-firmware control test for stepper hardware.
#
# Sends an MCU-side `diag_stepper_buzz` command that bypasses the Rust
# motion engine entirely — just toggles step pin N times with a fixed
# period. If this moves the motor but the engine path doesn't, the engine
# is the only suspect.
#
# 2026-05-21: written to break the "engine clean but motors silent"
# debugging dead-end after two weeks of motion-broken bench.

import logging


class DiagStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "DIAG_STEPPER_BUZZ",
            self.cmd_DIAG_STEPPER_BUZZ,
            desc="Bypass the motion engine; toggle a stepper's step pin"
            " N times directly.",
        )

    def cmd_DIAG_STEPPER_BUZZ(self, gcmd):
        stepper_name = gcmd.get("STEPPER")
        steps = gcmd.get_int("STEPS", 200, minval=1, maxval=2000)
        period_us = gcmd.get_int("PERIOD_US", 1000, minval=100, maxval=100000)
        direction = gcmd.get_int("DIR", 1, minval=0, maxval=1)

        force_move = self.printer.lookup_object("force_move", None)
        stepper = None
        if force_move is not None:
            stepper = force_move.lookup_stepper(stepper_name)
        if stepper is None:
            for s in (
                self.printer.lookup_object("toolhead")
                .get_kinematics()
                .get_steppers()
            ):
                if s.get_name() == stepper_name:
                    stepper = s
                    break
        if stepper is None:
            raise gcmd.error(
                "stepper '%s' not found; try one of: stepper_x stepper_y"
                " stepper_z stepper_z1 ..." % (stepper_name,)
            )

        mcu_stepper = (
            stepper
            if hasattr(stepper, "get_oid")
            else stepper.get_mcu_stepper()
        )
        mcu = mcu_stepper.get_mcu()
        oid = mcu_stepper.get_oid()
        period_ticks = max(1, int(mcu.seconds_to_clock(period_us * 1e-6)))

        # Plain formatted-string MCU command (clocksync.py pattern). The
        # CommandWrapper.encode() path used by mainline Klipper doesn't
        # exist on this fork's serial layer — send_with_response handles
        # the formatted string directly through the bridge call.
        msg_str = (
            "diag_stepper_buzz oid=%d dir=%d step_count=%d period_ticks=%d"
            % (oid, direction, steps, period_ticks)
        )

        logging.info(
            "DIAG_STEPPER_BUZZ stepper=%s oid=%d steps=%d period_us=%d"
            " period_ticks=%d dir=%d cmd=%s",
            stepper_name,
            oid,
            steps,
            period_us,
            period_ticks,
            direction,
            msg_str,
        )

        try:
            response = mcu._serial.send_with_response(
                msg_str, "diag_stepper_buzz_response"
            )
            gcmd.respond_info(
                "DIAG_STEPPER_BUZZ stepper=%s steps=%d period_us=%d dir=%d:"
                " response=%s"
                % (stepper_name, steps, period_us, direction, response)
            )
        except Exception as e:
            raise gcmd.error("DIAG_STEPPER_BUZZ failed: %s" % (e,))


def load_config(config):
    return DiagStepper(config)
