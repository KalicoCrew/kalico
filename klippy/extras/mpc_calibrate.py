import math
import logging

PIN_MIN_TIME = 0.100


class MPCCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "MPC_CALIBRATE",
            self.cmd_MPC_CALIBRATE,
            desc=self.cmd_MPC_CALIBRATE_help,
        )

    cmd_MPC_CALIBRATE_help = "Run MPC calibration"

    def cmd_MPC_CALIBRATE(self, gcmd):
        heater_name = gcmd.get("HEATER")
        pheaters = self.printer.lookup_object("heaters")
        try:
            heater = pheaters.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))
        cal = MpcCalibrate(self.printer, heater, self.config)
        cal.run(gcmd)


class MpcCalibrate:
    def __init__(self, printer, heater, config):
        self.printer = printer
        self.config = config
        self.heater = heater
        self.pmgr = heater.pmgr
        self.orig_control = heater.get_control()
        self.ambient_sensor_name = self.config.get("ambient_temp_sensor", None)

    def run(self, gcmd):
        profile_name = gcmd.get("PROFILE", "default")
        use_analytic = gcmd.get("USE_DELTA", None) is not None
        ambient_max_measure_time = gcmd.get_float(
            "AMBIENT_MAX_MEASURE_TIME", 20.0, above=0.0
        )
        ambient_measure_sample_time = gcmd.get_float(
            "AMBIENT_MEASURE_SAMPLE_TIME", 5.0, below=ambient_max_measure_time
        )
        fan_breakpoints = gcmd.get_int("FAN_BREAKPOINTS", 3, minval=2)
        target_temp = gcmd.get_float("TARGET", 200.0, minval=90.0)
        threshold_temp = gcmd.get_float(
            "THRESHOLD", max(50.0, min(100, target_temp - 100.0))
        )
        ambient_sensor_name = gcmd.get("AMBIENT_TEMP_SENSOR", self.ambient_sensor_name)
        ambient_sensor = None
        if ambient_sensor_name is not None:
            try:
                ambient_sensor = self.printer.lookup_object(ambient_sensor_name)
            except Exception:
                raise self.config.error(
                    f"Unknown ambient_temp_sensor '{ambient_sensor_name}' " f"specified"
                )

        control = TuningControl(self.heater)
        old_control = self.heater.set_control(control)
        try:
            ambient_temp = self.await_ambient(
                gcmd, control, threshold_temp, ambient_sensor
            )
            samples = self.heatup_test(gcmd, target_temp, control)
            first_res = self.process_first_pass(
                samples,
                self.orig_control.heater_max_power,
                ambient_temp,
                threshold_temp,
                use_analytic,
            )
            logging.info("First pass: %s", first_res)

            profile = dict(self.orig_control.profile)
            for key in [
                "block_heat_capacity",
                "ambient_transfer",
                "sensor_responsiveness",
            ]:
                profile[key] = first_res[key]
            new_control = self.heater.lookup_control(profile, True)
            new_control.state_block_temp = first_res["post_block_temp"]
            new_control.state_sensor_temp = first_res["post_sensor_temp"]
            new_control.state_ambient_temp = ambient_temp
            self.heater.set_control(new_control)

            transfer_res = self.transfer_test(
                gcmd,
                ambient_max_measure_time,
                ambient_measure_sample_time,
                fan_breakpoints,
                new_control,
                first_res,
            )
            second_res = self.process_second_pass(
                first_res,
                transfer_res,
                ambient_temp,
                self.orig_control.heater_max_power,
            )
            logging.info("Second pass: %s", second_res)

            block_heat_capacity = (
                second_res["block_heat_capacity"]
                if use_analytic
                else first_res["block_heat_capacity"]
            )
            sensor_responsiveness = (
                second_res["sensor_responsiveness"]
                if use_analytic
                else first_res["sensor_responsiveness"]
            )
            ambient_transfer = second_res["ambient_transfer"]
            fan_ambient_transfer = ", ".join(
                [f"{p:.6g}" for p in second_res["fan_ambient_transfer"]]
            )

            for key in [
                "block_heat_capacity",
                "ambient_transfer",
                "fan_ambient_transfer",
                "sensor_responsiveness",
            ]:
                profile[key] = first_res[key]

            new_control = self.heater.lookup_control(profile, True)
            self.heater.set_control(new_control, False)

            gcmd.respond_info(
                f"Finished MPC calibration of heater '{self.heater.short_name}'\n"
                "Measured:\n "
                f"  block_heat_capacity={block_heat_capacity:#.6g} [J/K]\n"
                f"  sensor_responsiveness={sensor_responsiveness:#.6g} [K/s/K]\n"
                f"  ambient_transfer={ambient_transfer:#.6g} [W/K]\n"
                f"  fan_ambient_transfer={fan_ambient_transfer} [W/K]\n"
            )
            self.heater.pmgr.save_profile(profile_name=profile_name, verbose=False)

        except self.printer.command_error as e:
            raise gcmd.error("%s failed: %s" % (gcmd.get_command(), e))
        finally:
            self.heater.set_control(old_control)
            self.heater.alter_target(0.0)

    def wait_stable(self, cycles=5):
        """
        We wait for the extruder to cycle x amount of times above and below the target
        doing this should ensure the temperature is stable enough to give a good result
        as a fallback if it stays within 0.1 degree for ~30 seconds it is also accepted
        """

        below_target = True
        above_target = 0
        on_target = 0
        starttime = self.printer.reactor.monotonic()

        def process(eventtime):
            nonlocal below_target, above_target, on_target
            temp, target = self.heater.get_temp(eventtime)
            if below_target and temp > target + 0.015:
                above_target += 1
                below_target = False
            elif not below_target and temp < target - 0.015:
                below_target = True
            if (
                above_target >= cycles
                and (self.printer.reactor.monotonic() - starttime) > 30.0
            ):
                return False
            if above_target > 0 and abs(target - temp) < 0.1:
                on_target += 1
            else:
                on_target = 0
            if on_target >= 150:  # in case the heating is super consistent
                return False
            return True

        self.printer.wait_while(process, True, 0.2)

    def wait_settle(self, max_rate):
        last_temp = None
        next_check = None
        samples = []

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            samples.append((eventtime, temp))
            while samples[0][0] < eventtime - 10.0:
                samples.pop(0)
            dT = samples[-1][1] - samples[0][1]
            dt = samples[-1][0] - samples[0][0]
            if dt < 8.0:
                return True
            rate = abs(dT / dt)
            return not rate < max_rate

        self.printer.wait_while(process)
        return samples[-1][1]

    def await_ambient(self, gcmd, control, minimum_temp, ambient_sensor):
        self.heater.alter_target(1.0)  # Turn on fan to increase settling speed

        if ambient_sensor is not None:
            # If we have an ambient sensor we won't waste time waiting for ambient.
            # We do however need to wait for sub minimum_temp(we pick -5 C relative).
            reported = [False]
            target = minimum_temp - 5

            def process(eventtime):
                temp, _ = self.heater.get_temp(eventtime)
                ret = temp > target
                if ret and not reported[0]:
                    gcmd.respond_info(
                        f"Waiting for heater to drop below {target} degrees celcius"
                    )
                    reported[0] = True
                return ret

            self.printer.wait_while(process)
            self.heater.alter_target(0.0)
            return self.orig_control.ambient_sensor.get_temp(
                self.heater.reactor.monotonic()
            )[0]

        gcmd.respond_info("Waiting for heater to settle at ambient temperature")
        ambient_temp = self.wait_settle(0.01)
        self.heater.alter_target(0.0)
        return ambient_temp

    def heatup_test(self, gcmd, target_temp, control):
        gcmd.respond_info(
            "Performing heatup test, target is %.1f degrees" % (target_temp,)
        )
        control.set_output(self.heater.get_max_power(), target_temp)

        control.logging = True

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            return temp < target_temp

        self.printer.wait_while(process)
        control.logging = False
        self.heater.alter_target(0.0)

        log = control.log
        control.log = []
        return log

    def transfer_test(
        self,
        gcmd,
        ambient_max_measure_time,
        ambient_measure_sample_time,
        fan_breakpoints,
        control,
        first_pass_results,
    ):
        target_temp = round(first_pass_results["post_block_temp"])
        self.heater.set_temp(target_temp)
        gcmd.respond_info(
            "Performing ambient transfer tests, target is %.1f degrees" % (target_temp,)
        )

        self.wait_stable(5)

        fan = self.orig_control.cooling_fan

        fan_powers = []
        if fan is None:
            power_base = self.measure_power(
                ambient_max_measure_time, ambient_measure_sample_time
            )
            gcmd.respond_info(f"Average stable power: {power_base} W")
        else:
            if fan is not None:
                for idx in range(0, fan_breakpoints):
                    speed = idx / (fan_breakpoints - 1)
                    curtime = self.heater.reactor.monotonic()
                    print_time = fan.get_mcu().estimated_print_time(curtime)
                    fan.set_speed(print_time + PIN_MIN_TIME, speed)
                    gcmd.respond_info("Waiting for temperature to stabilize")
                    self.wait_stable(3)
                    gcmd.respond_info(
                        f"Temperature stable, measuring power usage with {speed*100.:.0f}% fan speed"
                    )
                    power = self.measure_power(
                        ambient_max_measure_time, ambient_measure_sample_time
                    )
                    gcmd.respond_info(
                        f"{speed*100.:.0f}% fan average power: {power:.2f} W"
                    )
                    fan_powers.append((speed, power))
                curtime = self.heater.reactor.monotonic()
                print_time = fan.get_mcu().estimated_print_time(curtime)
                fan.set_speed(print_time + PIN_MIN_TIME, 0.0)
            power_base = fan_powers[0][1]

        return {
            "target_temp": target_temp,
            "base_power": power_base,
            "fan_powers": fan_powers,
        }

    def measure_power(self, max_time, sample_time):
        samples = []
        time = [0]
        last_time = [None]

        def process(eventtime):
            dt = eventtime - (last_time[0] if last_time[0] is not None else eventtime)
            last_time[0] = eventtime
            status = self.heater.get_status(eventtime)
            samples.append((dt, status["control_stats"]["power"] * dt))
            time[0] += dt
            return time[0] < max_time

        self.printer.wait_while(process)

        total_energy = 0
        total_time = 0
        for dt, energy in reversed(samples):
            total_energy += energy
            total_time += dt
            if total_time > sample_time:
                break

        return total_energy / total_time

    def fastest_rate(self, samples):
        best = [-1, 0, 0]
        base_t = samples[0][0]
        for idx in range(2, len(samples)):
            dT = samples[idx][1] - samples[idx - 2][1]
            dt = samples[idx][0] - samples[idx - 2][0]
            rate = dT / dt
            if rate > best[0]:
                sample = samples[idx - 1]
                best = [sample[0] - base_t, sample[1], rate]
        return best

    def process_first_pass(
        self,
        all_samples,
        heater_power,
        ambient_temp,
        threshold_temp,
        use_analytic,
    ):
        # Find a continous segment of samples that all lie in the threshold.. range
        best_lower = None
        for idx in range(0, len(all_samples)):
            if all_samples[idx][1] > threshold_temp and best_lower is None:
                best_lower = idx
            elif all_samples[idx][1] < threshold_temp:
                best_lower = None

        t1_time = all_samples[best_lower][0] - all_samples[0][0]

        samples = all_samples[best_lower:]
        pitch = math.floor((len(samples) - 1) / 2)
        # We pick samples 0, pitch, and 2pitch, ensuring matching time spacing
        dt = samples[pitch][0] - samples[0][0]
        t1 = samples[0][1]
        t2 = samples[pitch][1]
        t3 = samples[2 * pitch][1]

        asymp_T = (t2 * t2 - t1 * t3) / (2.0 * t2 - t1 - t3)
        block_responsiveness = -math.log((t2 - asymp_T) / (t1 - asymp_T)) / dt
        ambient_transfer = heater_power / (asymp_T - ambient_temp)

        block_heat_capacity = -1.0
        sensor_responsiveness = -1.0
        start_temp = all_samples[0][1]

        # Asymptotic method
        if use_analytic:
            block_heat_capacity = ambient_transfer / block_responsiveness
            sensor_responsiveness = block_responsiveness / (
                1.0
                - (start_temp - asymp_T)
                * math.exp(-block_responsiveness * t1_time)
                / (t1 - asymp_T)
            )

        # Differential method
        if not use_analytic or block_heat_capacity < 0 or sensor_responsiveness < 0:
            fastest_rate = self.fastest_rate(samples)
            block_heat_capacity = heater_power / fastest_rate[2]
            sensor_responsiveness = fastest_rate[2] / (
                fastest_rate[2] * fastest_rate[0] + ambient_temp - fastest_rate[0]
            )

        heat_time = all_samples[-1][0] - all_samples[0][0]
        post_block_temp = asymp_T + (start_temp - asymp_T) * math.exp(
            -block_responsiveness * heat_time
        )
        post_sensor_temp = all_samples[-1][1]

        return {
            "post_block_temp": post_block_temp,
            "post_sensor_temp": post_sensor_temp,
            "block_responsiveness": block_responsiveness,
            "ambient_transfer": ambient_transfer,
            "block_heat_capacity": block_heat_capacity,
            "sensor_responsiveness": sensor_responsiveness,
            "asymp_temp": asymp_T,
            "t1": t1,
            "t1_time": t1_time,
            "t2": t2,
            "start_temp": start_temp,
            "dt": dt,
        }

    def process_second_pass(self, first_res, transfer_res, ambient_temp, heater_power):
        target_ambient_temp = transfer_res["target_temp"] - ambient_temp
        ambient_transfer = transfer_res["base_power"] / target_ambient_temp
        asymp_T = ambient_temp + heater_power / ambient_transfer
        block_responsiveness = (
            -math.log((first_res["t2"] - asymp_T) / (first_res["t1"] - asymp_T))
            / first_res["dt"]
        )
        block_heat_capacity = ambient_transfer / block_responsiveness
        sensor_responsiveness = block_responsiveness / (
            1.0
            - (first_res["start_temp"] - asymp_T)
            * math.exp(-block_responsiveness * first_res["t1_time"])
            / (first_res["t1"] - asymp_T)
        )

        fan_ambient_transfer = [
            power / target_ambient_temp
            for (_speed, power) in transfer_res["fan_powers"]
        ]

        return {
            "ambient_transfer": ambient_transfer,
            "block_responsiveness": block_responsiveness,
            "block_heat_capacity": block_heat_capacity,
            "sensor_responsiveness": sensor_responsiveness,
            "asymp_temp": asymp_T,
            "fan_ambient_transfer": fan_ambient_transfer,
        }


class TuningControl:
    def __init__(self, heater):
        self.value = 0.0
        self.target = None
        self.heater = heater
        self.log = []
        self.logging = False

    def temperature_update(self, read_time, temp, target_temp):
        if self.logging:
            self.log.append((read_time, temp))
        self.heater.set_pwm(read_time, self.value)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.value != 0.0 or self.target != 0

    def set_output(self, value, target):
        self.value = value
        self.target = target
        self.heater.set_temp(target)

    def get_profile(self):
        return {"name": "autotune"}

    def get_type(self):
        return "autotune"


def load_config(config):
    return MPCCalibrate(config)
