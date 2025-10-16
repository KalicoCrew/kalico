# Calibration of heater PID settings
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import heaters


class PIDCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "PID_CALIBRATE",
            self.cmd_PID_CALIBRATE,
            desc=self.cmd_PID_CALIBRATE_help,
        )

    def _calibrate(
        self,
        pheaters,
        heater,
        target,
        tolerance,
        write_file,
        gcmd,
        calibrate_secondary,
    ):
        if isinstance(heater.control, heaters.ControlDualLoopPID):
            if calibrate_secondary:
                delta = heater.control.secondary_max_temp - target
                if delta <= 15.0:
                    gcmd.respond_raw(
                        "!! Target calibration temperature of %d°C is <= 15°C below `secondary_max_temp`."
                        % target
                    )
                    gcmd.respond_raw(
                        "!! Depending on the thermal mass this might prevent successful calibration."
                    )

                gcmd.respond_info(
                    "Calibrating secondary pid loop (target=%.1f)"
                    % (heater.control.secondary_max_temp)
                )
                calibrate = ControlAutoTune(
                    heater,
                    heater.control.secondary_max_temp,
                    tolerance,
                    calibrate_secondary=calibrate_secondary,
                )
            else:
                gcmd.respond_info(
                    "Calibrating primary pid loop (target=%.1f)" % (target)
                )
                calibrate = ControlAutoTune(
                    heater,
                    target,
                    tolerance,
                    calibrate_secondary=calibrate_secondary,
                )
                max_temp = max(0.0, calibrate.temp_low - 5.0)
                gcmd.respond_info(
                    "Waiting for heater %s to cool down to %.1f for calibration."
                    % (heater.get_name(), target)
                )
                pheaters.set_temperature(heater, max_temp, True)
        else:
            calibrate = ControlAutoTune(heater, target, tolerance)

        old_control = heater.set_control(calibrate)
        try:
            pheaters.set_temperature(heater, target, True)
        except self.printer.command_error as e:
            heater.set_control(old_control, False)
            raise
        heater.set_control(old_control, False)

        if write_file:
            fname = "/tmp/heattest.csv"
            if calibrate_secondary:
                fname = "tmp/heattest_secondary.txt"
            calibrate.write_file(fname)

        if calibrate.check_busy(0.0, 0.0, 0.0):
            raise gcmd.error("pid_calibrate interrupted")
        # Log and report results
        kp, ki, kd = calibrate.calc_pid()
        logging.info(
            "Autotune: final: Kp=%f Ki=%f Kd=%f, secondary=%s",
            kp,
            ki,
            kd,
            calibrate_secondary,
        )
        return kp, ki, kd, old_control

    cmd_PID_CALIBRATE_help = "Run PID calibration test"

    def cmd_PID_CALIBRATE(self, gcmd):
        heater_name = gcmd.get("HEATER")
        target = gcmd.get_float("TARGET")
        write_file = gcmd.get_int("WRITE_FILE", 0)
        tolerance = gcmd.get_float("TOLERANCE", TUNE_PID_TOL, above=0.0)
        profile_name = gcmd.get("PROFILE", "default")
        pheaters = self.printer.lookup_object("heaters")
        try:
            heater = pheaters.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))
        self.printer.lookup_object("toolhead").get_last_move_time()

        if isinstance(heater.control, heaters.ControlDualLoopPID):
            # Calibrate the inner (secondary) loop
            kp_s, ki_s, kd_s, _ = self._calibrate(
                pheaters,
                heater,
                target,
                tolerance,
                write_file,
                gcmd,
                calibrate_secondary=True,
            )

            logging.info("PID_CALIBRATE (primary) target=%.3f", target)
            # Calibrate the outer (primary) loop1
            kp_p, ki_p, kd_p, _ = self._calibrate(
                pheaters,
                heater,
                target,
                tolerance,
                write_file,
                gcmd,
                calibrate_secondary=False,
            )

            gcmd.respond_info(
                "PID parameters for %.2f\xb0C: "
                "Primary: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
                "Secondary: inner_pid_Kp=%.3f inner_pid_Ki=%.3f inner_pid_Kd=%.3f\n"
                "Heater: %s\n"
                "Tolerance: %.4f\n"
                "Profile: %s\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with these parameters and restart the printer."
                % (
                    target,
                    kp_p,
                    ki_p,
                    kd_p,
                    kp_s,
                    ki_s,
                    kd_s,
                    heater_name,
                    tolerance,
                    profile_name,
                )
            )

            profile = {
                "pid_target": target,
                "pid_tolerance": tolerance,
                "control": "dual_loop_pid",
                "pid_kp": kp_p,
                "pid_ki": ki_p,
                "pid_kd": kd_p,
                "inner_pid_kp": kp_s,
                "inner_pid_ki": ki_s,
                "inner_pid_kd": kd_s,
                "smooth_time": None,
                "name": profile_name,
            }
        else:
            Kp, Ki, Kd, old_control = self._calibrate(
                pheaters,
                heater,
                target,
                tolerance,
                write_file,
                gcmd,
                calibrate_secondary=False,
            )

            logging.info("Autotune: final: Kp=%f Ki=%f Kd=%f", Kp, Ki, Kd)
            gcmd.respond_info(
                "PID parameters for %.2f\xb0C: "
                "pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
                "Heater: %s\n"
                "Tolerance: %.4f\n"
                "Profile: %s\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with these parameters and restart the printer."
                % (target, Kp, Ki, Kd, heater_name, tolerance, profile_name)
            )
            control = "pid_v" if old_control.get_type() == "pid_v" else "pid"

            profile = {
                "pid_target": target,
                "pid_tolerance": tolerance,
                "control": control,
                "pid_kp": Kp,
                "pid_ki": Ki,
                "pid_kd": Kd,
                "smooth_time": None,
                "name": profile_name,
            }

        heater.set_control(heater.lookup_control(profile, True), False)
        heater.pmgr.save_profile(profile_name=profile_name, verbose=False)


TUNE_PID_DELTA = 5.0
TUNE_PID_TOL = 0.02
TUNE_PID_SAMPLES = 3
TUNE_PID_MAX_PEAKS = 60


class ControlAutoTune:
    def __init__(self, heater, target, tolerance, calibrate_secondary=False):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        # store the reference so we can push messages if needed
        self.gcode = heater.printer.lookup_object("gcode")
        # holds the various max power settings used during the test.
        self.powers = [self.heater_max_power]
        # holds the times the power setting was changed.
        self.times = []
        # the target temperature to tune for
        self.target = target
        # the tolerance that determines if the system has converged to an
        # acceptable level
        self.tolerance = tolerance
        # the the temp that determines when to turn the heater off
        self.temp_high = target + TUNE_PID_DELTA / 2.0
        # the the temp that determines when to turn the heater on
        self.temp_low = target - TUNE_PID_DELTA / 2.0
        # is the heater on
        self.heating = False
        # the current potential peak value
        self.peak = self.target
        # the time values associated with the current potential peak
        self.peak_times = []
        # known peaks and their associated time values
        self.peaks = []
        # has the target temp been crossed at-least once
        self.target_crossed = False
        # has the tuning processed finished
        self.done = False
        # has the tuning processed started
        self.started = False
        # did an error happen
        self.errored = False
        # data from the test that can be optionally written to a file
        self.data = []
        # Control for dual loop pid
        self.control = heater.control
        self.calibrate_secondary = calibrate_secondary

    def temperature_update(
        self,
        read_time,
        primary_temp,
        target_temp,
        secondary_temp=None,
    ):
        if self.calibrate_secondary:
            temp = secondary_temp
        else:
            temp = primary_temp

        # tuning is done, so don't do any more calculations
        if self.done:
            return
        # store test data
        self.data.append(
            (read_time, temp, self.heater.last_pwm_value, self.target)
        )
        # ensure the starting temp is low enough to run the test.
        if not self.started and temp >= self.temp_low:
            self.errored = True
            self.finish(read_time)
            self.gcode.respond_info(
                "temperature is too high to start calibration"
            )
            return
        else:
            self.started = True
        # ensure the test doesn't run too long
        if float(len(self.peaks)) > TUNE_PID_MAX_PEAKS:
            self.errored = True
            self.finish(read_time)
            self.gcode.respond_info("calibration did not finish in time")
            return
        # indicate that the target temp has been crossed at-least once
        if temp > self.target and self.target_crossed is False:
            self.target_crossed = True
        # only do work if the target temp has been crossed at-least once
        if self.target_crossed:
            # check for a new peak value
            if temp > self.temp_high or temp < self.temp_low:
                self.check_peak(read_time, temp)
            # it's time to calculate and store a high peak
            if self.peak > self.temp_high and temp < self.target:
                self.store_peak()
            # it's time to calculate and store a low peak
            if self.peak < self.temp_low and temp > self.target:
                self.store_peak()
            # check if the conditions are right to evaluate a new sample
            peaks = float(len(self.peaks)) - 1.0
            powers = float(len(self.powers))
            if (peaks % 2.0) == 0.0 and (powers * 2.0) == peaks:
                self.log_info()
                # check for convergence
                if self.converged():
                    self.finish(read_time)
                    return
                self.set_power()
        # turn the heater off
        if self.heating and temp >= self.temp_high:
            self.heating = False
            self.times.append(read_time)
            self.heater.alter_target(self.temp_low)
        # turn the heater on
        if not self.heating and temp <= self.temp_low:
            self.heating = True
            self.times.append(read_time)
            self.heater.alter_target(self.temp_high)
        # set the pwm output based on the heater state
        if self.heating:
            if not self.calibrate_secondary:
                pid = self.control.secondary_pid
                secondary_target_temp = self.control.secondary_max_temp
                _, bounded_co = pid.calculate_output(
                    read_time, secondary_temp, secondary_target_temp
                )
                bounded_co = min(bounded_co, self.powers[-1])
                self.heater.set_pwm(read_time, bounded_co)
            else:
                self.heater.set_pwm(read_time, self.powers[-1])
        else:
            self.heater.set_pwm(read_time, 0)

    def check_peak(self, time, temp):
        # deal with duplicate temps
        if temp == self.peak:
            self.peak_times.append(time)
        # deal with storing high peak values
        if temp > self.target and temp > self.peak:
            self.peak = temp
            self.peak_times = [time]
        # deal with storing low peak values
        if temp < self.target and temp < self.peak:
            self.peak = temp
            self.peak_times = [time]

    def store_peak(self):
        time = sum(self.peak_times) / float(len(self.peak_times))
        self.peaks.append((time, self.peak))
        self.peak = self.target
        self.peak_times = []

    def log_info(self):
        # provide some useful info to the user
        sample = len(self.powers)
        pwm = self.powers[-1]
        asymmetry = (self.peaks[-2][1] + self.peaks[-1][1]) / 2.0 - self.target
        tolerance = self.get_sample_tolerance()
        if tolerance is False:
            fmt = "sample:%d pwm:%.4f asymmetry:%.4f tolerance:n/a\n"
            data = (sample, pwm, asymmetry)
            self.gcode.respond_info(fmt % data)
        else:
            fmt = "sample:%d pwm:%.4f asymmetry:%.4f tolerance:%.4f\n"
            data = (sample, pwm, asymmetry, tolerance)
            self.gcode.respond_info(fmt % data)

    def get_sample_tolerance(self):
        powers = len(self.powers)
        if powers < TUNE_PID_SAMPLES + 1:
            return False
        powers = self.powers[-1 * (TUNE_PID_SAMPLES + 1) :]
        return max(powers) - min(powers)

    def converged(self):
        tolerance = self.get_sample_tolerance()
        if tolerance is False:
            return False
        if tolerance <= self.tolerance:
            return True
        return False

    def set_power(self):
        peak_low = self.peaks[-2][1]
        peak_high = self.peaks[-1][1]
        power = self.powers[-1]
        mid = power * ((self.target - peak_low) / (peak_high - peak_low))
        if mid * 2.0 > self.heater_max_power:
            # the new power is to high so just return max power
            self.powers.append(self.heater_max_power)
            return
        self.powers.append(mid * 2.0)

    def finish(self, time):
        self.heater.set_pwm(time, 0)
        self.heater.alter_target(0)
        self.done = True
        self.heating = False

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        if eventtime == 0.0 and smoothed_temp == 0.0 and target_temp == 0.0:
            if self.errored:
                return True
            return False
        if self.done:
            return False
        return True

    def write_file(self, filename):
        f = open(filename, "w")
        f.write("time, temp, pwm, target\n")
        data = [
            "%.5f, %.5f, %.5f, %.5f" % (time, temp, pwm, target)
            for time, temp, pwm, target in self.data
        ]
        f.write("\n".join(data))
        peaks = self.peaks[:]
        powers = self.powers[:]
        # pop off the
        peaks.pop(0)
        samples = []
        for i in range(len(powers)):
            samples.append(
                (
                    i,
                    peaks[i * 2][0],
                    peaks[i * 2][1],
                    peaks[i * 2 + 1][0],
                    peaks[i * 2 + 1][1],
                    powers[i],
                )
            )
        f.write("\nsample, low time, low, high time, high, max power\n")
        data = [
            "%.5f, %.5f, %.5f, %.5f, %.5f, %.5f"
            % (sample, low_time, low, high_time, high, max_power)
            for sample, low_time, low, high_time, high, max_power in samples
        ]
        f.write("\n".join(data))
        f.close()

    def calc_pid(self):
        temp_diff = 0.0
        time_diff = 0.0
        theta = 0.0
        for i in range(1, TUNE_PID_SAMPLES * 2, 2):
            temp_diff = temp_diff + self.peaks[-i][1] - self.peaks[-i - 1][1]
            time_diff = time_diff + self.peaks[-i][0] - self.peaks[-i - 2][0]
            theta = theta + self.peaks[-i][0] - self.times[-i]
        temp_diff = temp_diff / float(TUNE_PID_SAMPLES)
        time_diff = time_diff / float(TUNE_PID_SAMPLES)
        theta = theta / float(TUNE_PID_SAMPLES)
        amplitude = 0.5 * abs(temp_diff)
        power = self.powers[-1 * (TUNE_PID_SAMPLES) :]
        power = sum(power) / float(len(power))
        # calculate the various parameters
        Ku = 4.0 * power / (math.pi * amplitude)
        Tu = time_diff
        Wu = (2.0 * math.pi) / Tu
        tau = math.tan(math.pi - theta * Wu) / Wu
        Km = -math.sqrt(tau**2 * Wu**2 + 1.0) / Ku
        # log the extra details
        logging.info("Ziegler-Nichols constants: Ku=%f Tu=%f", Ku, Tu)
        logging.info(
            "Cohen-Coon constants: Km=%f Theta=%f Tau=%f", Km, theta, tau
        )
        # Use Ziegler-Nichols method to generate PID parameters
        Ti = 0.5 * Tu
        Td = 0.125 * Tu
        Kp = 0.6 * Ku * heaters.PID_PARAM_BASE
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd

    def update_smooth_time(self, write_to_profile):
        return

    def get_profile(self):
        return {"name": "autotune"}

    def get_type(self):
        return "autotune"


def load_config(config):
    return PIDCalibrate(config)
