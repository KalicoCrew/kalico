import math
import logging

AMBIENT_TEMP = 25.0
PIN_MIN_TIME = 0.100

FILAMENT_TEMP_SRC_AMBIENT = "ambient"
FILAMENT_TEMP_SRC_FIXED = "fixed"
FILAMENT_TEMP_SRC_SENSOR = "sensor"


class ControlMPC:
    def __init__(self, profile, heater, load_clean=False, register=True):
        self.profile = profile
        self._load_profile()
        self.heater = heater
        self.heater_max_power = heater.get_max_power() * self.const_heater_power

        self.want_ambient_refresh = self.ambient_sensor is not None
        self.state_block_temp = (
            AMBIENT_TEMP if load_clean else self._heater_temp()
        )
        self.state_sensor_temp = self.state_block_temp
        self.state_ambient_temp = AMBIENT_TEMP

        self.last_power = 0.0
        self.last_loss_ambient = 0.0
        self.last_loss_filament = 0.0
        self.last_time = 0.0
        self.last_temp_time = 0.0

        self.printer = heater.printer
        self.toolhead = None

        if not register:
            return

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "MPC_CALIBRATE",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_CALIBRATE,
            desc=self.cmd_MPC_CALIBRATE_help,
        )
        gcode.register_mux_command(
            "MPC_SET",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_SET,
            desc=self.cmd_MPC_SET_help,
        )

    cmd_MPC_SET_help = "Set MPC parameter"

    def cmd_MPC_SET(self, gcmd):
        self.const_filament_diameter = gcmd.get_float(
            "FILAMENT_DIAMETER", self.const_filament_diameter
        )
        self.const_filament_density = gcmd.get_float(
            "FILAMENT_DENSITY", self.const_filament_density
        )
        self.const_filament_heat_capacity = gcmd.get_float(
            "FILAMENT_HEAT_CAPACITY", self.const_filament_heat_capacity
        )

        self.const_block_heat_capacity = gcmd.get_float(
            "BLOCK_HEAT_CAPACITY", self.const_block_heat_capacity
        )
        self.const_sensor_responsiveness = gcmd.get_float(
            "SENSOR_RESPONSIVENESS", self.const_sensor_responsiveness
        )
        self.const_ambient_transfer = gcmd.get_float(
            "AMBIENT_TRANSFER", self.const_ambient_transfer
        )

        if gcmd.get("FAN_AMBIENT_TRANSFER", None):
            try:
                self.const_fan_ambient_transfer = [
                    float(v)
                    for v in gcmd.get("FAN_AMBIENT_TRANSFER").split(",")
                ]
            except ValueError:
                raise gcmd.error(
                    f"Error on '{gcmd._commandline}': unable to parse FAN_AMBIENT_TRANSFER\n"
                    "Must be a comma-separated list of values ('0.05,0.07,0.08')"
                )

        temp = gcmd.get("FILAMENT_TEMP", None)
        if temp is not None:
            temp = temp.lower().strip()
            if temp == "sensor":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_SENSOR,)
            elif temp == "ambient":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_AMBIENT,)
            else:
                try:
                    value = float(temp)
                except ValueError:
                    raise gcmd.error(
                        f"Error on '{gcmd._commandline}': unable to parse FILAMENT_TEMP\n"
                        "Valid options are 'sensor', 'ambient', or number."
                    )
                self.filament_temp_src = (FILAMENT_TEMP_SRC_FIXED, value)

        self._update_filament_const()

    cmd_MPC_CALIBRATE_help = "Run MPC calibration"

    def cmd_MPC_CALIBRATE(self, gcmd):
        cal = MpcCalibrate(self.printer, self.heater, self)
        cal.run(gcmd)

    # Helpers

    def _heater_temp(self):
        return self.heater.get_temp(self.heater.reactor.monotonic())[0]

    def _load_profile(self):
        self.const_block_heat_capacity = self.profile["block_heat_capacity"]
        self.const_ambient_transfer = self.profile["ambient_transfer"]
        self.const_target_reach_time = self.profile["target_reach_time"]
        self.const_heater_power = self.profile["heater_power"]
        self.const_smoothing = self.profile["smoothing"]
        self.const_sensor_responsiveness = self.profile["sensor_responsiveness"]
        self.const_min_ambient_change = self.profile["min_ambient_change"]
        self.const_steady_state_rate = self.profile["steady_state_rate"]
        self.const_filament_diameter = self.profile["filament_diameter"]
        self.const_filament_density = self.profile["filament_density"]
        self.const_filament_heat_capacity = self.profile[
            "filament_heat_capacity"
        ]
        self.const_maximum_retract = self.profile["maximum_retract"]
        self.filament_temp_src = self.profile["filament_temp_src"]
        self._update_filament_const()
        self.ambient_sensor = self.profile["ambient_temp_sensor"]
        self.cooling_fan = self.profile["cooling_fan"]
        self.const_fan_ambient_transfer = self.profile["fan_ambient_transfer"]

    def is_valid(self):
        return (
            self.const_block_heat_capacity is not None
            and self.const_ambient_transfer is not None
            and self.const_sensor_responsiveness is not None
        )

    def check_valid(self):
        if self.is_valid():
            return
        name = self.heater.get_name()
        raise self.printer.command_error(
            f"Cannot activate '{name}' as MPC control is not fully configured.\n\n"
            f"Run 'MPC_CALIBRATE' or ensure 'block_heat_capacity', 'sensor_responsiveness', and "
            f"'ambient_transfer' settings are defined for '{name}'."
        )

    def _update_filament_const(self):
        radius = self.const_filament_diameter / 2.0
        self.const_filament_cross_section_heat_capacity = (
            (radius * radius)  # mm^2
            * math.pi  # 1
            / 1000.0  # mm^3 => cm^3
            * self.const_filament_density  # g/cm^3
            * self.const_filament_heat_capacity  # J/g/K
        )

    # Control interface

    def temperature_update(self, read_time, temp, target_temp):
        if not self.is_valid():
            self.heater.set_pwm(read_time, 0.0)
            return

        dt = read_time - self.last_temp_time
        if self.last_temp_time == 0.0 or dt < 0.0 or dt > 1.0:
            dt = 0.1

        # Extruder position
        extrude_speed_prev = 0.0
        extrude_speed_next = 0.0
        if target_temp != 0.0:
            if self.toolhead is None:
                self.toolhead = self.printer.lookup_object("toolhead")
            if self.toolhead is not None:
                extruder = self.toolhead.get_extruder()
                if (
                    hasattr(extruder, "find_past_position")
                    and extruder.get_heater() == self.heater
                ):
                    pos = extruder.find_past_position(read_time)

                    pos_prev = extruder.find_past_position(read_time - dt)
                    pos_moved = max(-self.const_maximum_retract, pos - pos_prev)
                    extrude_speed_prev = pos_moved / dt

                    pos_next = extruder.find_past_position(read_time + dt)
                    pos_move = max(-self.const_maximum_retract, pos_next - pos)
                    extrude_speed_next = pos_move / dt

        # Modulate ambient transfer coefficient with fan speed
        ambient_transfer = self.const_ambient_transfer
        if self.cooling_fan and len(self.const_fan_ambient_transfer) > 1:
            fan_speed = max(
                0.0, min(1.0, self.cooling_fan.get_status(read_time)["speed"])
            )
            fan_break = fan_speed * (len(self.const_fan_ambient_transfer) - 1)
            below = self.const_fan_ambient_transfer[math.floor(fan_break)]
            above = self.const_fan_ambient_transfer[math.ceil(fan_break)]
            if below != above:
                frac = fan_break % 1.0
                ambient_transfer = below * (1 - frac) + frac * above
            else:
                ambient_transfer = below

        # Simulate

        # Expected power by heating at last power setting
        expected_heating = self.last_power
        # Expected power from block to ambient
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        expected_ambient_transfer = block_ambient_delta * ambient_transfer
        expected_filament_transfer = (
            block_ambient_delta
            * extrude_speed_prev
            * self.const_filament_cross_section_heat_capacity
        )

        # Expected block dT since last period
        expected_block_dT = (
            (
                expected_heating
                - expected_ambient_transfer
                - expected_filament_transfer
            )
            * dt
            / self.const_block_heat_capacity
        )
        self.state_block_temp += expected_block_dT

        # Expected sensor dT since last period
        expected_sensor_dT = (
            (self.state_block_temp - self.state_sensor_temp)
            * self.const_sensor_responsiveness
            * dt
        )
        self.state_sensor_temp += expected_sensor_dT

        # Correct

        smoothing = 1 - (1 - self.const_smoothing) ** dt
        adjustment_dT = (temp - self.state_sensor_temp) * smoothing
        self.state_block_temp += adjustment_dT
        self.state_sensor_temp += adjustment_dT

        if self.want_ambient_refresh:
            temp = self.ambient_sensor.get_temp(read_time)[0]
            if temp != 0.0:
                self.state_ambient_temp = temp
                self.want_ambient_refresh = False
        if (self.last_power > 0 and self.last_power < 1.0) or abs(
            expected_block_dT + adjustment_dT
        ) < self.const_steady_state_rate * dt:
            if adjustment_dT > 0.0:
                ambient_delta = max(
                    adjustment_dT, self.const_min_ambient_change * dt
                )
            else:
                ambient_delta = min(
                    adjustment_dT, -self.const_min_ambient_change * dt
                )
            self.state_ambient_temp += ambient_delta

        # Output

        # Amount of power needed to reach the target temperature in the desired time

        heating_power = (
            (target_temp - self.state_block_temp)
            * self.const_block_heat_capacity
            / self.const_target_reach_time
        )
        # Losses (+ = lost from block, - = gained to block)
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        loss_ambient = block_ambient_delta * ambient_transfer
        block_filament_delta = self.state_block_temp - self.filament_temp(
            read_time, self.state_ambient_temp
        )
        loss_filament = (
            block_filament_delta
            * extrude_speed_next
            * self.const_filament_cross_section_heat_capacity
        )

        if target_temp != 0.0:
            # The required power is the desired heating power + compensation for all the losses
            power = max(
                0.0,
                min(
                    self.heater_max_power,
                    heating_power + loss_ambient + loss_filament,
                ),
            )
        else:
            power = 0

        duty = power / self.const_heater_power

        # logging.info(
        #     "mpc: [%.3f/%.3f] %.2f => %.2f / %.2f / %.2f = %.2f[%.2f+%.2f+%.2f] / %.2f, dT %.2f, E %.2f=>%.2f",
        #     dt,
        #     smoothing,
        #     temp,
        #     self.state_block_temp,
        #     self.state_sensor_temp,
        #     self.state_ambient_temp,
        #     power,
        #     heating_power,
        #     loss_ambient,
        #     loss_filament,
        #     duty,
        #     adjustment_dT,
        #     extrude_speed_prev,
        #     extrude_speed_next,
        # )

        self.last_power = power
        self.last_loss_ambient = loss_ambient
        self.last_loss_filament = loss_filament
        self.last_temp_time = read_time
        self.heater.set_pwm(read_time, duty)

    def filament_temp(self, read_time, ambient_temp):
        src = self.filament_temp_src
        if src[0] == FILAMENT_TEMP_SRC_FIXED:
            return src[1]
        elif (
            src[0] == FILAMENT_TEMP_SRC_SENSOR
            and self.ambient_sensor is not None
        ):
            return self.ambient_sensor.get_temp(read_time)[0]
        else:
            return ambient_temp

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(target_temp - smoothed_temp) > 1.0

    def update_smooth_time(self):
        pass

    def get_profile(self):
        return self.profile

    def get_type(self):
        return "mpc"

    def get_status(self, eventtime):
        return {
            "temp_block": self.state_block_temp,
            "temp_sensor": self.state_sensor_temp,
            "temp_ambient": self.state_ambient_temp,
            "power": self.last_power,
            "loss_ambient": self.last_loss_ambient,
            "loss_filament": self.last_loss_filament,
            "filament_temp": self.filament_temp_src,
            "filament_heat_capacity": self.const_filament_heat_capacity,
            "filament_density": self.const_filament_density,
        }


class MpcCalibrate:
    def __init__(self, printer, heater, orig_control):
        self.printer = printer
        self.heater = heater
        self.orig_control = orig_control

    def run(self, gcmd):
        use_analytic = gcmd.get("USE_DELTA", None) is not None
        ambient_max_measure_time = gcmd.get_float(
            "AMBIENT_MAX_MEASURE_TIME", 20.0, above=0.0
        )
        ambient_measure_sample_time = gcmd.get_float(
            "AMBIENT_MEASURE_SAMPLE_TIME", 5.0, below=ambient_max_measure_time
        )
        fan_breakpoints = gcmd.get_int("FAN_BREAKPOINTS", 3, minval=2)
        default_target_temp = (
            90.0 if self.heater.get_name() == "heater_bed" else 200.0
        )
        target_temp = gcmd.get_float("TARGET", default_target_temp, minval=60.0)
        threshold_temp = gcmd.get_float(
            "THRESHOLD", max(50.0, min(100, target_temp - 100.0))
        )

        control = TuningControl(self.heater)
        old_control = self.heater.set_control(control)
        try:
            ambient_temp = self.await_ambient(gcmd, control, threshold_temp)
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
            new_control = ControlMPC(profile, self.heater, False, False)
            new_control.state_block_temp = first_res["post_block_temp"]
            new_control.state_sensor_temp = first_res["post_sensor_temp"]
            new_control.state_ambient_temp = ambient_temp
            self.heater.set_control(new_control)

            transfer_res = self.transfer_test(
                gcmd,
                ambient_max_measure_time,
                ambient_measure_sample_time,
                fan_breakpoints,
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

            cfgname = self.heater.get_name()
            gcmd.respond_info(
                f"Finished MPC calibration of heater '{cfgname}'\n"
                "Measured:\n "
                f"  block_heat_capacity={block_heat_capacity:#.6g} [J/K]\n"
                f"  sensor_responsiveness={sensor_responsiveness:#.6g} [K/s/K]\n"
                f"  ambient_transfer={ambient_transfer:#.6g} [W/K]\n"
                f"  fan_ambient_transfer={fan_ambient_transfer} [W/K]\n"
            )

            configfile = self.heater.printer.lookup_object("configfile")
            configfile.set(cfgname, "control", "mpc")
            configfile.set(
                cfgname, "block_heat_capacity", f"{block_heat_capacity:#.6g}"
            )
            configfile.set(
                cfgname,
                "sensor_responsiveness",
                f"{sensor_responsiveness:#.6g}",
            )
            configfile.set(
                cfgname, "ambient_transfer", f"{ambient_transfer:#.6g}"
            )
            configfile.set(
                cfgname,
                "fan_ambient_transfer",
                fan_ambient_transfer,
            )

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

    def await_ambient(self, gcmd, control, minimum_temp):
        self.heater.alter_target(1.0)  # Turn on fan to increase settling speed
        if self.orig_control.ambient_sensor is not None:
            # If we have an ambient sensor we won't waste time waiting for ambient.
            # We do however need to wait for sub minimum_temp(we pick -5 C relative).
            reported = [False]
            target = minimum_temp - 5

            def process(eventtime):
                temp, _ = self.heater.get_temp(eventtime)
                ret = temp > target
                if ret and not reported[0]:
                    gcmd.respond_info(
                        f"Waiting for heater to drop below {target} degrees Celsius"
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
        first_pass_results,
    ):
        target_temp = round(first_pass_results["post_block_temp"])
        self.heater.set_temp(target_temp)
        gcmd.respond_info(
            "Performing ambient transfer tests, target is %.1f degrees"
            % (target_temp,)
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
            for idx in range(0, fan_breakpoints):
                speed = idx / (fan_breakpoints - 1)
                curtime = self.heater.reactor.monotonic()
                print_time = fan.get_mcu().estimated_print_time(curtime)
                fan.set_speed(print_time + PIN_MIN_TIME, speed)
                gcmd.respond_info("Waiting for temperature to stabilize")
                self.wait_stable(3)
                gcmd.respond_info(
                    f"Temperature stable, measuring power usage with {speed * 100.0:.0f}% fan speed"
                )
                power = self.measure_power(
                    ambient_max_measure_time, ambient_measure_sample_time
                )
                gcmd.respond_info(
                    f"{speed * 100.0:.0f}% fan average power: {power:.2f} W"
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
            dt = eventtime - (
                last_time[0] if last_time[0] is not None else eventtime
            )
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
        if (
            not use_analytic
            or block_heat_capacity < 0
            or sensor_responsiveness < 0
        ):
            fastest_rate = self.fastest_rate(samples)
            block_heat_capacity = heater_power / fastest_rate[2]
            sensor_responsiveness = fastest_rate[2] / (
                fastest_rate[2] * fastest_rate[0]
                + ambient_temp
                - fastest_rate[0]
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

    def process_second_pass(
        self, first_res, transfer_res, ambient_temp, heater_power
    ):
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
        return {"name": "tuning"}

    def get_type(self):
        return "tuning"
