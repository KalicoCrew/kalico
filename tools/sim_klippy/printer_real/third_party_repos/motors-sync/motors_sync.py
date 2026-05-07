# Motors synchronization script
#
# Copyright (C) 2024-2025  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, time, itertools
from datetime import datetime
import numpy as np
import chelper
from . import z_tilt

PLOT_PATH = '~/printer_data/config/adxl_results/motors_sync'
PIN_MIN_TIME = 0.010            # Minimum wait time to enable hardware pin
MOTOR_STALL_TIME = 0.100        # Minimum wait time to enable motor pin
LEVELING_KINEMATICS = (         # Kinematics with interconnected axes
    ['corexy', 'limited_corexy'])

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]

MATH_MODELS = {
    "polynomial": lambda fx, coeffs:
        max(np.roots([*coeffs[:-1], coeffs[-1] - fx]).real),
    "power": lambda fx, coeffs:
        (fx / coeffs[0]) ** (1 / coeffs[1]),
    "root": lambda fx, coeffs:
        (fx**2 - 2*coeffs[1]*fx + coeffs[1]**2) / coeffs[0]**2,
    "hyperbolic": lambda fx, coeffs:
        coeffs[0] / (fx - coeffs[1]),
    "exponential": lambda fx, coeffs:
        np.log((fx - coeffs[2]) / coeffs[0]) / coeffs[1],
    "enc_auto": lambda fx, coeffs: (fx / 1e3 / coeffs[0])
}

# Plug class for the Klipper version before add "motion_queuing"
# module, and probably also for Kalico. May the gods forgive me.
class DummyPrinterMotionQueuing:
    def __init__(self, config):
        self.printer = config.get_printer()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def allocate_trapq(self):
        return self.trapq

    def lookup_trapq_append(self):
        return self.trapq_append

    def note_mcu_movequeue_activity(self, ptime):
        ctime = ptime + 99999.9
        self.trapq_finalize_moves(self.trapq, ctime, ctime)
        self.toolhead.note_mcu_movequeue_activity(ptime)

    def wipe_trapq(self, trapq):
        return


class StepperManualMove:
    from . import force_move
    calc_move_time = staticmethod(force_move.calc_move_time)
    del force_move
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.stepper_en = printer.load_object(config, 'stepper_enable')
        self.motion_queuing = \
            printer.load_object(config, 'motion_queuing', None)
        if self.motion_queuing is None:
            self.motion_queuing = DummyPrinterMotionQueuing(config)
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.stepper_kin = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = min(self.toolhead.max_velocity, 100)
        self.travel_accel = min(self.toolhead.max_accel, 5000)

    def stepper_enable(self, stepper_name, mode, ontime, offtime):
        self.toolhead.dwell(ontime)
        print_time = self.toolhead.get_last_move_time()
        el = self.stepper_en.enable_lines[stepper_name]
        el.motor_enable(print_time) if mode \
            else el.motor_disable(print_time)
        self.toolhead.dwell(offtime)

    def manual_move(self, mcu_stepper, moves):
        self.toolhead.flush_step_generation()
        prev_sk = mcu_stepper.set_stepper_kinematics(self.stepper_kin)
        prev_trapq = mcu_stepper.set_trapq(self.trapq)
        mcu_stepper.set_position((0., 0., 0.))
        ptime = start_ptime = self.toolhead.get_last_move_time()
        last_pos = 0.0
        for move in moves:
            if abs(move) < 0.00001:
                continue
            axis_r, accel_t, cruise_t, cruise_v = self.calc_move_time(
                move, self.travel_speed, self.travel_accel)
            self.trapq_append(
                self.trapq, ptime, accel_t, cruise_t, accel_t, last_pos,
                0., 0., axis_r, 0., 0., 0., cruise_v, self.travel_accel)
            ptime = ptime + accel_t + cruise_t + accel_t
            last_pos += move
        if hasattr(mcu_stepper, 'generate_steps'):
            mcu_stepper.generate_steps(ptime)
        self.motion_queuing.note_mcu_movequeue_activity(ptime)
        self.toolhead.dwell(ptime - start_ptime)
        self.toolhead.flush_step_generation()
        mcu_stepper.set_trapq(prev_trapq)
        mcu_stepper.set_stepper_kinematics(prev_sk)
        self.motion_queuing.wipe_trapq(self.trapq)


class BaseSensorHelper:
    def __init__(self, axis, chip_name):
        self.axis = axis
        self.chip_name = chip_name
        self.dim_type = ''
        self.sync = axis.sync
        self.printer = axis.printer
        self.stepper_move = axis.stepper_move
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.chip_config = None
        self.is_finished = False
        self.samples = []
        self.request_start_time = None
        self.request_end_time = None
        self._run_on_connect(self._handle_connect)
        self._run_on_connect(self._init_chip_config)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def _init_chip_config(self):
        self.chip_config = self.printer.lookup_object(self.chip_name)

    def _run_on_connect(self, task):
        _, state = self.printer.get_state_message()
        if state == "ready":
            task()
        else:
            self.printer.register_event_handler("klippy:connect", task)

    def flush_data(self):
        self.is_finished = False
        self.samples.clear()
        self.request_start_time = None
        self.request_end_time = None

    def _handle_batch(self, batch):
        if self.is_finished:
            return False
        samples = batch['data']
        self.samples.extend(samples)
        return True

    def update_start_time(self):
        self.request_start_time = self.toolhead.get_last_move_time()

    def update_end_time(self):
        self.request_end_time = self.toolhead.get_last_move_time()

    def start_measurements(self):
        self.flush_data()
        self.chip_config.batch_bulk.add_client(self._handle_batch)

    def finish_measurements(self):
        self.toolhead.wait_moves()
        self.is_finished = True

    def _wait_samples(self):
        lim = self.reactor.monotonic() + 5.
        while True:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            if self.samples and self.request_end_time:
                last_mcu_time = self.samples[-1][0]
                if last_mcu_time > self.request_end_time:
                    return True
                elif now > lim:
                    raise Exception(
                        'motors_sync: No data from sensor')

    def _get_samples(self):
        self._wait_samples()
        raw_data = np.array(self.samples)
        start_idx = np.searchsorted(raw_data[:, 0],
                    self.request_start_time, side='left')
        end_idx = np.searchsorted(raw_data[:, 0],
                    self.request_end_time, side='right')
        return raw_data[start_idx:end_idx][:, 1:]

    def measure_deviation(self):
        # Measure the impact
        self.axis.buzz(25)
        self.flush_data()
        self.axis.toggle_main_stepper(1, (PIN_MIN_TIME,))
        self.axis.toggle_main_stepper(0, (PIN_MIN_TIME,))
        self.update_start_time()
        self.axis.toggle_main_stepper(1)
        self.update_end_time()
        self.axis.buzz(5)
        return self.calc_deviation()

    def calc_deviation(self):
        raise

    def detect_move_dir(self):
        raise


class AccelHelper(BaseSensorHelper):
    ACCEL_FILTER_THRESHOLD = 3000
    def __init__(self, axis, chip_name):
        self.chip_filter = None
        super().__init__(axis, chip_name)
        self.dim_type = 'magnitude'
        self._init_chip_filter(only_read=True)

    def _init_chip_filter(self, only_read=False):
        config = self.sync.config
        filters = {m: m for m in ['default', 'median', 'kalman']}
        filter = config.getchoice(f'chip_filter_{self.axis.name}',
                                  filters, 'default').lower()
        if filter == 'default':
            filter = config.getchoice('chip_filter',
                                      filters, 'median').lower()
        if filter == 'median':
            window = config.getint(f'median_size_{self.axis.name}',
                                   '', minval=3, maxval=9)
            if not window:
                window = config.getint('median_size', default=3,
                                       minval=3, maxval=9)
            if window % 2 == 0: raise config.error(
                f"motors_sync: parameter 'median_size' cannot be even")
            if only_read:
                return
            self.chip_filter = lambda samples, w=window: (
                np.median([samples[i - w:i + w + 1]
                           for i in range(w, len(samples) - w)], axis=1))
        elif filter == 'kalman':
            coeffs = config.getfloatlist(
                f'kalman_coeffs_{self.axis.name}',
                default=tuple('' for _ in range(6)), count=6)
            if not all(coeffs):
                coeffs = config.getfloatlist('kalman_coeffs',
                    default=tuple((1.1, 1., 1e-1, 1e-2, .5, 1.)), count=6)
                if only_read:
                    return
            self.chip_filter = KalmanLiteFilter(*coeffs).process_samples

    def _init_chip_config(self):
        self.chip_config = self.printer.lookup_object(self.chip_name)
        if self.chip_config.data_rate > self.ACCEL_FILTER_THRESHOLD:
            self._init_chip_filter()
        else:
            self.chip_filter = lambda data: data

    def calc_deviation(self):
        # Calculate impact magnitude
        vects = self._get_samples()
        vects_len = vects.shape[0]
        # Kalman filter may distort the first values, or in some
        # cases there may be residual values of toolhead inertia.
        # It is better to take a shifted zone from zero.
        static_zone = range(vects_len // 5, vects_len // 3)
        z_cut_zone = vects[static_zone, :]
        z_axis = np.mean(np.abs(z_cut_zone), axis=0).argmax()
        xy_mask = np.arange(vects.shape[1]) != z_axis
        magnitudes = np.linalg.norm(vects[:, xy_mask], axis=1)
        # Add median, Kalman or none filter
        magnitudes = self.chip_filter(magnitudes)
        # Calculate static noise
        static = np.mean(magnitudes[static_zone])
        # Return avg of 5 max magnitudes with deduction static
        magnitude = np.mean(np.sort(magnitudes)[-5:])
        magnitude = np.around(magnitude - static, 2)
        self.axis.update_log(int(magnitude))
        return magnitude

    def detect_move_dir(self):
        # Determine axis movement direction
        self.axis.set_move_dir(0)
        self.axis.single_move()
        self.axis.new_magnitude = self.measure_deviation()
        self.sync.msg_helper.stepped_msg(self.axis)
        if self.axis.new_magnitude > self.axis.magnitude:
            self.axis.set_move_dir(-1)
        else:
            self.axis.set_move_dir(1)
        self.sync.msg_helper.direction_msg(self.axis)
        self.axis.magnitude = self.axis.new_magnitude


class BeaconAccelHelper(AccelHelper):
    def __init__(self, axis, chip_name):
        super().__init__(axis, chip_name)

    def _init_beacon_config(self):
        beacon = self.printer.lookup_object(self.chip_name)
        # Beacon "chip_config" is "beacon.accel_helper"
        self.chip_config = beacon.accel_helper
        if self.chip_config is None:
            msg = "This Beacon has no accelerometer"
            raise self.printer.config_error(msg)
        # Beacon use self-written "batch_bulk" named as "_api_dump"
        self.chip_config.batch_bulk = self.chip_config._api_dump
        # Beacon module doesn't have "data_rate" attribute
        # Beacon adxl345 sampling rate > ACCEL_FILTER_THRESHOLD
        self._init_chip_filter()

    def _init_chip_config(self):
        # Move to the end of the klippy:connect queue due beacon
        # create BeaconAccelHelper in the klippy:connect stage
        self._run_on_connect(self._init_beacon_config)

    def _handle_batch(self, batches):
        hb = super()._handle_batch
        for batch in batches:
            res = hb(batch)
        return res


class EncoderHelper(BaseSensorHelper):
    MIN_SAMPLE_PERIOD = 0.000400
    def __init__(self, axis, chip_name):
        super().__init__(axis, chip_name)
        self.dim_type = 'deviation'
        self.last_raw_deviation = 0

    def _init_chip_config(self):
        self.chip_config = self.printer.lookup_object(self.chip_name)
        self._check_sample_rate()
        self._check_encoder_place()

    def _check_sample_rate(self):
        per = self.chip_config.sample_period
        if per > self.MIN_SAMPLE_PERIOD:
            raise self.printer.config_error(
                f'motors_sync: Encoder sample rate too '
                f'low: {per} < {self.MIN_SAMPLE_PERIOD}')

    def _check_encoder_place(self):
        # Swap duties between motors depending on encoder place
        binded_stepper = self.chip_config.calibration.stepper_name
        axis_steppers = [s.get_name() for s in self.axis.get_steppers()]
        if binded_stepper not in axis_steppers:
            raise self.printer.config_error(
                f"motors_sync: Encoder '{self.chip_name}' stepper: "
                f"'{binded_stepper}' not in '{self.axis.name}' "
                f"axis steppers: '{', '.join(axis_steppers)}'")
        if binded_stepper != axis_steppers[0]:
            self.axis.swap_steppers()

    def _normalize_encoder_pos(self, pos):
        angle = (1 << 16) / pos
        length = self.axis.rd / angle
        return angle, length

    def calc_deviation(self):
        # Calculate impact position on encoder in µm
        positions = self._get_samples()[:, 0]
        poss_len = positions.shape[0]
        static_zone = range(poss_len // 5, poss_len // 3)
        # Calculate static position
        static = np.mean(positions[static_zone])
        # Return avg of 5 max positions with deduction static
        deviations = positions - static
        top_dev_ids = np.argsort(np.abs(deviations))[-5:]
        deviation = np.mean(deviations[top_dev_ids])
        dev_norm = self._normalize_encoder_pos(deviation)
        deviation = np.around(dev_norm[1] * 1e3, 2)
        abs_deviation = abs(deviation)
        self.axis.update_log(int(abs_deviation))
        self.last_raw_deviation = deviation
        return abs_deviation

    def detect_move_dir(self):
        # Determine axis movement direction
        if self.last_raw_deviation < 0:
            self.axis.set_move_dir(-1)
        else:
            self.axis.set_move_dir(1)
        self.sync.msg_helper.direction_msg(self.axis)
        self.axis.new_magnitude = self.axis.magnitude


class FanController:
    def __init__(self, fan, printer):
        self.fan = fan
        self.printer = printer
        self.is_hold = False

    def _hold(self):
        raise Exception("Internal error in motors_sync")

    def _resume(self):
        raise Exception("Internal error in motors_sync")

    def set_state(self, state):
        if not state:
            if self.is_hold:
                return
            self.is_hold = True
            self._hold()
        else:
            if not self.is_hold:
                return
            self.is_hold = False
            self._resume()

    def _set_fan_speed(self, value):
        now = self.printer.get_reactor().monotonic()
        print_time = self.fan.fan.get_mcu().estimated_print_time(now)
        self.fan.fan.set_speed(value=value,
            print_time=print_time + PIN_MIN_TIME)


class ControllerFanController(FanController):
    def __init__(self, fan, printer):
        super().__init__(fan, printer)
        self.fan_speed = None
        self.idle_speed = None
        self.last_speed = None

    def _hold(self):
        self.fan_speed = self.fan.fan_speed
        self.idle_speed = self.fan.idle_speed
        self.last_speed = self.fan.last_speed
        self.fan.fan_speed = 0
        self.fan.idle_speed = 0
        self.fan.last_speed = 0
        self._set_fan_speed(0)

    def _resume(self):
        self.fan.fan_speed = self.fan_speed
        self.fan.idle_speed = self.idle_speed
        self.fan.last_speed = self.last_speed
        self._set_fan_speed(self.last_speed)
        self.fan_speed = None
        self.idle_speed = None
        self.last_speed = None


class HeaterFanController(FanController):
    def __init__(self, fan, printer):
        super().__init__(fan, printer)
        self.fan_speed = fan.fan_speed

    def _hold(self):
        self.fan.fan_speed = self.fan.last_speed = 0.0
        self._set_fan_speed(0.0)

    def _resume(self):
        self.fan.fan_speed = self.fan.last_speed = self.fan_speed
        self._set_fan_speed(self.fan_speed)


class TemperatureFanController(FanController):
    def __init__(self, fan, printer):
        super().__init__(fan, printer)
        self.target_temp = None
        self.min_speed = None
        self.max_speed = None

    def _hold(self):
        _, self.target_temp = self.fan.get_temp(0)
        self.min_speed = self.fan.get_min_speed()
        self.max_speed = self.fan.get_max_speed()
        self.fan.set_temp(0)
        self.fan.set_min_speed(0)
        self.fan.set_max_speed(0)
        self._set_fan_speed(0)

    def _resume(self):
        self.fan.set_temp(self.target_temp)
        self.fan.set_min_speed(self.min_speed)
        self.fan.set_max_speed(self.max_speed)
        self.target_temp = None
        self.min_speed = None
        self.max_speed = None


class FanProxy:
    def __init__(self, manager, axis_name, fan_names):
        self._manager = manager
        self._axis_name = axis_name
        self._fan_names = fan_names

    def toggle(self, state):
        for name in self._fan_names:
            self._manager._request_toggle(name, self._axis_name, state)


class FanManager:
    from . import controller_fan, heater_fan, temperature_fan
    FAN_METHODS = {
        controller_fan.ControllerFan: ControllerFanController,
        heater_fan.PrinterHeaterFan: HeaterFanController,
        temperature_fan.TemperatureFan: TemperatureFanController,
    }
    del controller_fan, heater_fan, temperature_fan
    def __init__(self, config):
        self.printer = config.get_printer()
        self._fans = {}
        self._fan_off_requests = {}

    def register_fans(self, axis_name, fan_names):
        for name in fan_names:
            if name in self._fans:
                continue
            self._register_fan(name)
        return FanProxy(self, axis_name, fan_names)

    def _register_fan(self, fan_name):
        try:
            fan_obj = self.printer.lookup_object(fan_name)
        except:
            if len(fan_name.split()) != 1:
                raise self.printer.config_error(
                    f"motors_sync: Unknown fan '{fan_name}'")
            raise self.printer.config_error(
                f"motors_sync: Provide full fan '{fan_name}' name "
                f"in 'head_fan' option, e.g. 'heater_fan {fan_name}'")
        controller_cls = None
        for fan_type, ctrl_type in self.FAN_METHODS.items():
            if isinstance(fan_obj, fan_type):
                controller_cls = ctrl_type
                break
        if not controller_cls:
            raise self.printer.config_error(
                f"motors_sync: Unknown fan type '{fan_name}' "
                f"--> '{type(fan_obj).__name__}' object")
        self._fans[fan_name] = controller_cls(fan_obj, self.printer)
        self._fan_off_requests[fan_name] = set()

    def _request_toggle(self, fan_name, axis_id, state):
        off_set = self._fan_off_requests[fan_name]
        if state:
            off_set.discard(axis_id)
        else:
            off_set.add(axis_id)
        controller = self._fans[fan_name]
        should_be_on = len(off_set) == 0
        controller.set_state(should_be_on)


class MotionAxisMsgHelper:
    def __init__(self, config):
        self.gcode = config.get_printer().lookup_object('gcode')

    def start_msg(self, ax):
        msg = (f"{ax.name_prefixed}-Initial "
               f"{ax.chip_helper.dim_type}: {ax.init_magnitude}")
        self.gcode.respond_info(msg)

    def done_msg(self, ax):
        msg = (f"{ax.name_prefixed}-Motors adjusted by "
               f"{ax.actual_msteps}/{ax.microsteps} step, "
               f"{ax.chip_helper.dim_type} "
               f"{ax.init_magnitude} --> {ax.magnitude}")
        self.gcode.respond_info(msg)

    def in_tolerance_msg(self, axes):
        retry_tols = ", ".join([
            f"{ax.name_prefixed}: {ax.retry_tolerance}"
            for ax in axes])
        msg = f"Motors magnitudes are in tolerance: {retry_tols}"
        self.gcode.respond_info(msg)

    def static_msg(self, ax):
        msg = (f"{ax.name_prefixed}-New {ax.chip_helper.dim_type}: "
               f"{ax.new_magnitude}")
        self.gcode.respond_info(msg)

    def stepped_msg(self, ax):
        msteps = ax.move_msteps * ax.move_dir[0]
        msg = (f"{ax.name_prefixed}-New {ax.chip_helper.dim_type}: "
               f"{ax.new_magnitude} on {msteps}/{ax.microsteps} "
               f"step move")
        self.gcode.respond_info(msg)

    def direction_msg(self, ax):
        msg = f"{ax.name_prefixed}-Movement direction: {ax.move_dir[1]}"
        self.gcode.respond_info(msg)

    def retry_msg(self, ax):
        msg = (f"{ax.name_prefixed}-Retries: {ax.curr_retry}/"
               f"{ax.max_retries} Back to last "
               f"{ax.chip_helper.dim_type}: {ax.magnitude} on "
               f"{ax.actual_msteps}/{ax.microsteps} step "
               f"to reach {ax.retry_tolerance}")
        self.gcode.respond_info(msg)


class MotionAxis:
    VALID_MSTEPS = [256, 128, 64, 32, 16, 8, 0]
    def __init__(self, sync, name, jx, ph_off):
        self.sync = sync
        self.name = name
        self.joint_axes = jx.get(name, [])
        self.phase_offset = ph_off.get(name, None)
        self.config = sync.config
        self.stepper_move = sync.stepper_move
        self.printer = self.config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.move_dir = []
        self.set_move_dir(0)
        self.move_msteps = 2
        self.actual_msteps = 0
        self.check_msteps = 0
        self.backup_msteps = 0
        self.init_magnitude = 0.
        self.magnitude = 0.
        self.new_magnitude = 0.
        self.curr_retry = 0
        self.is_finished = False
        self.chip_helper = None
        self.motion_log = []
        stepper = 'stepper_' + name
        st_section = self.config.getsection(stepper)
        min_pos = st_section.getfloat('position_min', 0)
        max_pos = st_section.getfloat('position_max')
        self.rd = st_section.getfloat('rotation_distance')
        fspr = st_section.getint('full_steps_per_rotation', 200)
        self.limits = (min_pos + 10, max_pos - 10, (min_pos + max_pos) / 2)
        self.buzz_moves = {}
        self.rel_buzz_d = self.rd / fspr * 5
        msteps_dict = {m: m for m in self.VALID_MSTEPS}
        self.microsteps = self.config.getchoice(
            f'microsteps_{name}', msteps_dict, default=0)
        if not self.microsteps:
            self.microsteps = self.config.getchoice(
                'microsteps', msteps_dict, default=16)
        self.move_d = self.rd / fspr / self.microsteps
        sync.add_connect_task(self._init_steppers)
        sync.add_connect_task(self._init_tmc_drivers)
        self._init_chip_helper()
        conf_fans = self.config.getlist(f'head_fan_{name}', '')
        if not conf_fans:
            conf_fans = self.config.getlist('head_fan', [])
        sync.add_connect_task(lambda: self._init_fan(conf_fans))
        msmax = self.microsteps / 2
        self.max_step_size = self.config.getint(
            f'max_step_size_{name}', default=0, minval=1, maxval=msmax)
        if not self.max_step_size:
            self.max_step_size = self.config.getint(
                'max_step_size', default=3, minval=1, maxval=msmax)
        self.axes_steps_diff = self.config.getint(
            f'axes_steps_diff_{name}', default=0, minval=1)
        if not self.axes_steps_diff:
            self.axes_steps_diff = self.config.getint(
                'axes_steps_diff', self.max_step_size + 1, minval=1)
        rmin = self.move_d * 1e3
        self.retry_tolerance = self.config.getfloat(
            f'retry_tolerance_{name}', default=0, above=rmin)
        if not self.retry_tolerance:
            self.retry_tolerance = self.config.getfloat(
                'retry_tolerance', default=0, above=rmin)
        self.max_retries = self.config.getint(
            f'retries_{name}', default=0, minval=0, maxval=10)
        if not self.max_retries:
            self.max_retries = self.config.getint(
                'retries', default=0, minval=0, maxval=10)
        self.name_prefixed = name.upper()
        if name_prefix := self.config.get(f'axis_prefix_{name}', None):
            self.name_prefixed = f'{name_prefix} {self.name_prefixed}'

    def flush_motion_data(self):
        self.set_move_dir(0)
        self.move_msteps = 2
        self.actual_msteps = 0
        self.check_msteps = 0
        self.backup_msteps = 0
        self.init_magnitude = 0.
        self.magnitude = 0.
        self.new_magnitude = 0.
        self.curr_retry = 0
        self.is_finished = False
        self.motion_log = []

    def swap_steppers(self):
        self.steppers.reverse()
        self.tmcs.reverse()

    def get_steppers(self):
        return self.steppers

    def get_phase(self, tmc):
        field_name = "mscnt"
        if tmc.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "mstep"
        reg = tmc.fields.lookup_register(field_name)
        phase = tmc.mcu_tmc.get_register(reg)
        phase = tmc.fields.get_field(field_name, phase)
        stepper = self.steppers[self.tmcs.index(tmc)]
        if not stepper.get_dir_inverted()[0]:
            return 1023 - phase
        return phase

    def get_phase_offset(self):
        p1, p2 = (self.get_phase(t) for t in self.tmcs)
        return p2 - p1

    def update_phase_offset(self):
        self.phase_offset = self.get_phase_offset()

    def set_phase_offset(self):
        # Set phase offset by <axis>1 stepper
        if self.get_phase_offset() != 0:
            return
        if self.phase_offset is None:
            return
        move_d = self.phase_offset / 256 * self.move_d * self.microsteps
        if abs(move_d) < self.move_d * 2:
            return
        self.toggle_main_stepper(0, (PIN_MIN_TIME, PIN_MIN_TIME))
        self.stepper_move.manual_move(self.steppers[1], [move_d])
        msteps = int(move_d // self.move_d)
        self.gcode.respond_info(
            f'{self.name.upper()}-Restore previous '
            f'position: {msteps}/{self.microsteps} step')

    def toggle_main_stepper(self, mode, times=None):
        if times is None:
            times = (MOTOR_STALL_TIME, MOTOR_STALL_TIME)
        elif len(times) < 2:
            times = (*times, MOTOR_STALL_TIME)
        name = self.steppers[0].get_name()
        self.stepper_move.stepper_enable(name, mode, *times)

    def toggle_steppers(self, mode):
        for st in self.steppers:
            self.stepper_move.stepper_enable(
                st.get_name(), mode, PIN_MIN_TIME, PIN_MIN_TIME)

    def toggle_joint_axes(self, mode):
        for name in self.joint_axes:
            self.sync.motion[name].toggle_steppers(mode)

    def single_move(self, dir=1):
        # Move <axis>1 stepper motor by default
        mcu_stepper = self.steppers[1]
        move_msteps = self.move_msteps * self.move_dir[0] * dir
        dist = self.move_d * move_msteps
        self.actual_msteps += move_msteps
        self.check_msteps += move_msteps
        self.stepper_move.manual_move(mcu_stepper, [dist])

    def _gen_buzz_moves(self, rel_moves):
        moves = []
        last_abs_pos = 0
        for osc in reversed(range(0, rel_moves)):
            abs_pos = self.rel_buzz_d * (osc / rel_moves)
            for inv in [1, -1]:
                abs_pos *= inv
                dist = (abs_pos - last_abs_pos)
                moves.append(dist)
                last_abs_pos = abs_pos
        self.buzz_moves[rel_moves] = moves
        return moves

    def _get_buzz_moves(self, rel_moves):
        moves = self.buzz_moves.get(rel_moves)
        if moves is None:
            moves = self._gen_buzz_moves(rel_moves)
        return moves

    def buzz(self, rel_moves=25):
        # Fading oscillations by <axis>1 stepper
        mcu_stepper = self.steppers[1]
        moves = self._get_buzz_moves(rel_moves)
        self.toggle_main_stepper(0, (PIN_MIN_TIME,)*2)
        self.stepper_move.manual_move(mcu_stepper, moves)

    def measure_deviation(self):
        return self.chip_helper.measure_deviation()

    def detect_move_dir(self):
        return self.chip_helper.detect_move_dir()

    def update_log(self, deviation):
        self.motion_log.append([int(deviation), self.actual_msteps])

    def write_log(self, state):
        self.sync.stats_helper.write_axis_stats_log(self, state)

    def set_move_dir(self, dir):
        if dir == 1:
            self.move_dir = [1, 'Forward']
        elif dir == -1:
            self.move_dir = [-1, 'Backward']
        elif dir == 0:
            self.move_dir = [1, 'unknown']

    def is_move_dir_unknown(self):
        return True if self.move_dir[1] == 'unknown' else False

    def on_start(self):
        self.flush_motion_data()
        self.fan.toggle(False)
        self.chip_helper.start_measurements()

    def on_done(self):
        self.fan.toggle(True)
        self.chip_helper.finish_measurements()
        self.toggle_main_stepper(1, (PIN_MIN_TIME, PIN_MIN_TIME))
        self.update_phase_offset()
        self.write_log(True)

    def on_error(self):
        self.fan.toggle(True)
        self.chip_helper.finish_measurements()
        self.toggle_steppers(1)
        self.write_log(False)

    def _init_steppers(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        belt_steppers = [s for s in kin.get_steppers()
                         if 'stepper_' + self.name in s.get_name()]
        if len(belt_steppers) not in (2,):
            raise self.config.error(f"motors_sync: Not supported "
                                    f"'{len(belt_steppers)}' count of motors")
        for steppers in belt_steppers:
            st_section = self.config.getsection(steppers.get_name())
            st_msteps = st_section.getint('microsteps')
            if self.microsteps > st_msteps:
                raise self.config.error(
                    f'motors_sync: Invalid microsteps count, cannot be '
                    f'more than steppers, {self.microsteps} vs {st_msteps}')
        self.steppers = belt_steppers

    def _init_tmc_drivers(self):
        self.tmcs = []
        for stepper in self.steppers:
            for driver in TRINAMIC_DRIVERS:
                driver_name = f"{driver} {stepper.get_name()}"
                module = self.printer.lookup_object(driver_name, None)
                if module is not None:
                    self.tmcs.append(module)
                    break
            else:
                raise self.config.error(f"Unable to find TMC driver for "
                                        f"'{stepper.get_name()}' stepper")

    def _init_steps_models(self, def_model):
        # todo: rewrite all func logic
        models = {
            'linear': {'ct': 2, 'a': None, 'f': MATH_MODELS['polynomial']},
            'quadratic': {'ct': 3, 'a': None, 'f': MATH_MODELS['polynomial']},
            'power': {'ct': 2, 'a': None, 'f': MATH_MODELS['power']},
            'root': {'ct': 2, 'a': 0, 'f': MATH_MODELS['root']},
            'hyperbolic': {'ct': 2, 'a': 0, 'f': MATH_MODELS['hyperbolic']},
            'exponential': {'ct': 3, 'a': 0, 'f': MATH_MODELS['exponential']},
            'enc_auto': {'ct': 1, 'a': -1, 'f': MATH_MODELS['enc_auto']},
        }
        model = self.config.getlist(f'steps_model_{self.name}', None)
        if model is None:
            model = self.config.getlist('steps_model', def_model)
        model_name = model[0]
        coeffs_vals = list(map(float, model[1:]))
        coeffs_args = [chr(97 + i) for i in range(len(coeffs_vals) + 1)]
        model_coeffs = {arg: float(val)
                        for arg, val in zip(coeffs_args, coeffs_vals)}
        model_config = models.get(model_name, None)
        if model_config is None:
            raise self.config.error(
                f"motors_sync: Invalid steps model '{model_name}'")
        if len(model_coeffs) != model_config['ct']:
            raise self.config.error(
                f"motors_sync: Steps model '{model_name}' "
                f"requires {model_config['ct']} coefficients")
        if model_coeffs['a'] == model_config['a']:
            raise self.config.error(
                f"motors_sync: Coefficient 'a' cannot be "
                f"{model_coeffs['a']} for a '{model_name}' model")
        self.model_name = model_name
        self.model_coeffs = tuple(model_coeffs.values())
        model_scale = self.microsteps / 16
        if model_name == 'enc_auto':
            model_scale = 1
        def model_solve(fx=None):
            if fx is None:
                fx = self.new_magnitude
            res = model_config['f'](fx, self.model_coeffs)
            if np.isnan(res):
                raise Exception(
                    f"Microsteps calculation returned NaN "
                    f"for {self.name_prefixed} axis")
            return res * model_scale
        self.model_solve = model_solve

    def init_accel_chip_helper(self, accel_chip_name):
        if isinstance(self.chip_helper, BaseSensorHelper):
            self.chip_helper.finish_measurements()
        if accel_chip_name == 'beacon':
            self.chip_helper = BeaconAccelHelper(self, accel_chip_name)
        else:
            self.chip_helper = AccelHelper(self, accel_chip_name)

    def _init_chip_helper(self):
        accel_chip_name = self.config.get(f'accel_chip_{self.name}', '')
        enc_chip_name = self.config.get(f'encoder_chip_{self.name}', '')
        if accel_chip_name and enc_chip_name:
            raise self.config.error(f"motors_sync: Only 1 sensor "
                                    f"type can be selected")
        if not accel_chip_name and not enc_chip_name:
            accel_chip_name = self.config.get('accel_chip', '')
        if not accel_chip_name and not enc_chip_name:
            raise self.config.error(
                f"motors_sync: Sensors type 'accel_chip' or "
                f"'encoder_chip_<axis>' must be provided")
        if accel_chip_name:
            self.init_accel_chip_helper(accel_chip_name)
            def_steps_model = ['linear', 20000, 0]
        elif enc_chip_name:
            self.chip_helper = EncoderHelper(self, enc_chip_name)
            def_steps_model = ['enc_auto', self.move_d]
        else:
            raise
        self._init_steps_models(def_steps_model)

    def _init_fan(self, fans):
        self.fan = self.sync.fan_manager.register_fans(self.name, fans)


class MotorsSync:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.status = z_tilt.ZAdjustStatus(self.printer)
        self.connect_tasks = []
        self.stepper_move = StepperManualMove(config)
        self.fan_manager = FanManager(config)
        self.stats_helper = MotionAxesStats(config)
        self.msg_helper = MotionAxisMsgHelper(config)
        # Read config
        self._init_axes()
        self._init_sync_method()
        # Register commands
        self.gcode.register_command('SYNC_MOTORS', self.cmd_SYNC_MOTORS,
                                    desc=self.cmd_SYNC_MOTORS_help)
        self.gcode.register_command('SYNC_MOTORS_CALIBRATE',
                                    self.cmd_SYNC_MOTORS_CALIBRATE,
                                    desc=self.cmd_SYNC_MOTORS_CALIBRATE_help)
        # Variables
        self.reactor = self.printer.get_reactor()

    def add_connect_task(self, task):
        self.connect_tasks.append(task)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = min(self.toolhead.max_accel, 5000)
        self.kin = self.toolhead.get_kinematics()
        for task in self.connect_tasks: task()
        self.connect_tasks.clear()

    def _check_common_attr(self):
        # Apply restrictions for LEVELING_KINEMATICS kinematics
        common_attr = ['microsteps', 'model_name', 'model_coeffs',
                       'max_step_size', 'axes_steps_diff']
        for attr in common_attr:
            diff = set([getattr(cls, attr) for cls in self.motion.values()])
            if len(diff) < 2:
                continue
            params_str = ', '.join(f"'{attr}: {v}'" for v in diff)
            raise self.config.error(
                f"motors_sync: Options {params_str} cannot be "
                f"different for a '{self.conf_kin}' kinematics")

    def _init_axes(self):
        valid_axes = ['x', 'y']
        printer_section = self.config.getsection('printer')
        self.conf_kin = printer_section.get('kinematics')
        if self.conf_kin in LEVELING_KINEMATICS:
            self.do_level = True
            axes = [a.lower() for a in self.config.getlist(
                'axes', count=2, default=['x', 'y'])]
            joint_ax = {'x': ['y'], 'y': ['x']}
        elif self.conf_kin == 'cartesian':
            self.do_level = False
            axes = [a.lower() for a in self.config.getlist('axes')]
            joint_ax = {}
        else:
            raise self.config.error(f"motors_sync: Not supported "
                                    f"kinematics '{self.conf_kin}'")
        if any(axis not in valid_axes for axis in axes):
            raise self.config.error(f"motors_sync: Invalid axes "
                                    f"parameter '{','.join(axes)}'")
        ph_off = self.stats_helper.get_axes_phase_offsets(axes)
        logging.info(f'motors_sync_phase_offset: {ph_off}')
        self.motion = {ax: MotionAxis(self, ax, joint_ax, ph_off)
                       for ax in axes}
        if self.conf_kin in LEVELING_KINEMATICS:
            self._check_common_attr()

    def _init_sync_method(self):
        methods = ['sequential', 'alternately', 'synchronous', 'default']
        _sync_method = self.config.get(
            'sync_method', {m: m for m in methods}, 'default')
        self.config.deprecate('sync_method')
        if self.conf_kin in LEVELING_KINEMATICS:
            self.sync_method = 'synchronous'
        else:
            self.sync_method = 'sequential'

    def gsend(self, params):
        self.gcode.run_script_from_command(params)

    def homing(self):
        # Homing and going to center
        now = self.reactor.monotonic()
        axes_names = list(self.motion.keys())
        homed_axes = self.kin.get_status(now)['homed_axes']
        if any(a not in homed_axes for a in axes_names):
            self.gsend(f"G28 {' '.join(axes_names)}")
        center_pos = (f'{a}{c.limits[2]}' for a, c in self.motion.items())
        self.gsend(f"G0 {' '.join(center_pos)} F{self.travel_speed * 60}")
        self.toolhead.dwell(MOTOR_STALL_TIME)
        self.toolhead.flush_step_generation()

    def _single_sync(self, m):
        # "m" is a main axis, just single axis
        if m.is_move_dir_unknown():
            if not m.actual_msteps or m.curr_retry:
                m.new_magnitude = m.measure_deviation()
                m.magnitude = m.new_magnitude
                self.msg_helper.static_msg(m)
            if (not m.actual_msteps
                    and m.retry_tolerance
                    and m.new_magnitude < m.retry_tolerance):
                m.is_finished = True
                return
            m.detect_move_dir()
        m.move_msteps = min(max(
            int(m.model_solve()), 1), m.max_step_size)
        m.single_move()
        m.new_magnitude = m.measure_deviation()
        self.msg_helper.stepped_msg(m)
        if m.new_magnitude > m.magnitude:
            m.single_move(dir=-1)
            if m.retry_tolerance and m.magnitude > m.retry_tolerance:
                m.curr_retry += 1
                if m.curr_retry > m.max_retries:
                    raise Exception('Too many retries')
                m.set_move_dir(0)
                self.msg_helper.retry_msg(m)
                return
            m.is_finished = True
            return
        m.magnitude = m.new_magnitude

    def check_axis_drift(self, m, s):
        # Note: m.axes_steps_diff == s.axes_steps_diff
        steps_diff = abs(abs(m.check_msteps) - abs(s.check_msteps))
        if steps_diff >= m.axes_steps_diff:
            m.new_magnitude = m.magnitude = m.measure_deviation()
            self.msg_helper.static_msg(m)
            m.check_msteps, s.check_msteps = 0, 0
            return True

    def _run_sync(self):
        # Axes synchronization
        if self.sync_method == 'synchronous' and len(self.axes) > 1:
            cycling = itertools.cycle(self.axes)
            max_ax = [c for c in sorted(
                self.motion.values(), key=lambda i: i.init_magnitude)][-1]
            max_ax.detect_move_dir()
            while True:
                axis = next(cycling)
                cycling, cycle = itertools.tee(cycling)
                m = self.motion[axis]
                sec = next(cycle)
                s = self.motion[sec]
                if m.is_finished:
                    if all(self.motion[ax].is_finished for ax in self.axes):
                        break
                    continue
                if m.magnitude < s.magnitude and not s.is_finished:
                    self.check_axis_drift(m, s)
                    continue
                self._single_sync(m)
        elif self.sync_method == 'sequential' or len(self.axes) == 1:
            for axis in self.axes:
                m = self.motion[axis]
                # To skip measure_deviation() in _single_sync()
                m.detect_move_dir()
                while True:
                    if m.is_finished:
                        if all(self.motion[ax].is_finished for ax in self.axes):
                            return
                        break
                    self._single_sync(m)
        else:
            raise Exception('Error in sync methods!')

    cmd_SYNC_MOTORS_help = 'Start motors synchronization'
    def cmd_SYNC_MOTORS(self, gcmd, force_run=False):
        # Live variables
        axes_from_gcmd = gcmd.get('AXES', '')
        if axes_from_gcmd:
            axes_from_gcmd = axes_from_gcmd.split(',')
            if any([axis not in self.motion.keys()
                    for axis in axes_from_gcmd]):
                raise self.gcode.error(f'Invalid axes parameter')
            self.axes = [axis for axis in axes_from_gcmd]
        else:
            self.axes = list(self.motion.keys())
        chip = gcmd.get(f'ACCEL_CHIP', '')
        for axis in self.axes:
            m = self.motion[axis]
            if not isinstance(m.chip_helper, AccelHelper):
                continue
            ax_chip = gcmd.get(f'ACCEL_CHIP_{axis.upper()}', chip)
            if ax_chip and ax_chip != m.chip_helper.chip_name:
                try:
                    m.init_accel_chip_helper(ax_chip)
                except Exception as e:
                    raise self.gcode.error(e)
            retry_tol = gcmd.get_int(f'RETRY_TOLERANCE_{axis.upper()}', 0)
            if not retry_tol:
                retry_tol = gcmd.get_int(f'RETRY_TOLERANCE', 0)
            if retry_tol:
                m.retry_tolerance = retry_tol
            retries = gcmd.get_int(f'RETRIES_{axis.upper()}', 0)
            if not retries:
                retries = gcmd.get_int(f'RETRIES', 0)
            if retries:
                m.max_retries = retries
        # Run
        self.status.reset()
        self.homing()
        self.gcode.respond_info('Motors synchronization started', True)
        try:
            # Try restore last sync position on cold start
            for axis in self.axes:
                self.motion[axis].set_phase_offset()
            # Init axes magnitudes
            for axis in self.axes:
                m = self.motion[axis]
                m.on_start()
                m.init_magnitude = m.magnitude = m.measure_deviation()
                self.msg_helper.start_msg(m)
            # Check if all axes in tolerance
            mot_axes_to_sync = [self.motion[ax] for ax in self.axes]
            if (not force_run and all(
                    m.init_magnitude < m.retry_tolerance
                    for m in mot_axes_to_sync)):
                for m in mot_axes_to_sync:
                    m.on_done()
                self.msg_helper.in_tolerance_msg(mot_axes_to_sync)
            else:
                self._run_sync()
                # Info
                for axis in self.axes:
                    m = self.motion[axis]
                    m.on_done()
                    self.msg_helper.done_msg(m)
            self.status.check_retry_result('done')
        except Exception as e:
            for axis in self.axes:
                try:
                    self.motion[axis].on_error()
                except:
                    raise
            raise self.gcode.error(str(e))

    cmd_SYNC_MOTORS_CALIBRATE_help = 'Calibrate synchronization process model'
    def cmd_SYNC_MOTORS_CALIBRATE(self, gcmd):
        # Calibrate sync model and model coeffs
        if not hasattr(self, 'sync_calibrate_helper'):
            self.sync_calibrate_helper = MotorsSyncCalibrate(self)
        self.sync_calibrate_helper.run_calibrate(gcmd)
        self.status.reset()

    def get_status(self, eventtime):
        return self.status.get_status(eventtime)


class MotorsSyncCalibrate:
    def __init__(self, sync):
        self.sync = sync
        self.reactor = sync.printer.get_reactor()
        self.gcode = sync.printer.lookup_object('gcode')
        try:
            self._load_modules()
        except ImportError as e:
            raise self.gcode.error(f'motors_sync: Could not import: {e}')
        self.path = os.path.expanduser(PLOT_PATH)
        self.check_export_path()

    @staticmethod
    def _load_modules():
        globals().update({
            'wrap': __import__('textwrap', fromlist=['wrap']).wrap,
            'multiprocessing': __import__('multiprocessing'),
            'plt': __import__('matplotlib.pyplot', fromlist=['']),
            'ticker': __import__('matplotlib.ticker', fromlist=['']),
            'curve_fit': __import__(
                'scipy.optimize', fromlist=['curve_fit']).curve_fit
        })

    def check_export_path(self):
        if os.path.exists(self.path):
            return
        try:
            os.makedirs(self.path)
        except OSError as e:
            raise self.gcode.error(
                f'Error generate path {self.path}: {e}')

    math_models = {
        'linear': (lambda x, a, b: a*x + b, '-.', '#DF8816'),
        'quadratic': (lambda x, a, b, c: a*x**2 + b*x + c, '--', 'green'),
        'power': (lambda x, a, b: a * np.power(x, b), ':', 'cyan'),
        'root': (lambda x, a, b: a * np.sqrt(x) + b, '--', 'magenta'),
        'hyperbolic': (lambda x, a, b: a / x + b, '-.', 'purple'),
        'exponential': (lambda x, a, b, c: a * np.exp(b*x) + c, ':', 'blue')
    }

    def find_best_func(self, x_data, y_data, x_prio_len=0, maxfev=999999999):
        x_len = len(x_data)
        if x_prio_len == 0 or x_prio_len == x_len:
            x_prio_len = x_len
            sigma = None
        else:
            weights = np.ones(x_len)
            weights[x_prio_len:] = np.linspace(1.0, 0.01, x_len - x_prio_len)
            sigma = 1 / weights
        funcs = []
        with np.errstate(over='raise', divide='raise', invalid='raise'):
            for name, (math_model, *params) in self.math_models.items():
                try:
                    coeffs, _ = curve_fit(math_model, x_data, y_data,
                                          sigma=sigma, maxfev=maxfev)
                    y_pred = math_model(x_data, *coeffs)
                    diff = y_data[:x_prio_len] - y_pred[:x_prio_len]
                    rmse = np.sqrt(np.mean(diff ** 2))
                    funcs.append({'name': name, 'rmse': rmse,
                                  'coeffs': coeffs})
                except BaseException as e:
                    logging.warning(f"motors_sync_calibrate: "
                                    f"error on {name} - {e}")
        funcs = sorted(funcs, key=lambda f: f['rmse'])
        info = ['Functions RMSE and coefficients']
        for f in funcs:
            c_str = ','.join([f'{c:.10f}' for c in f['coeffs']])
            info.append(f"{f['name']}: RMSE {f['rmse']:.2f} coeffs: {c_str}")
        return info, (x_data, y_data, funcs)

    def plotter(self, x_data, y_data, funcs, axis, accel_chip,
                peak_msteps, fullstep, rmse_lim=20000):
        # Plotting
        pow_lim = (-2, 2)
        fig, ax = plt.subplots()
        ax.scatter(x_data, y_data, label='Samples',
                   color='red', zorder=2, s=10)
        x_fit = np.linspace(min(x_data), max(x_data), 200)
        for func in funcs:
            rmse = func['rmse']
            if rmse < rmse_lim:
                upname = func['name'].capitalize()
                model = self.math_models[func['name']]
                _func = model[0](x_fit, *func['coeffs'])
                c_str = ','.join([f'{c:.3f}' for c in func['coeffs']])
                func_str = f"{upname} RMSE: {rmse:.2f} coeffs: {c_str}"
                linestyle = model[1]
                color = model[2]
                ax.plot(x_fit, _func, label=func_str,
                        linestyle=linestyle, linewidth=1, color=color)
        ax.legend(loc='lower right', fontsize=6, framealpha=1, ncol=1)
        accel_chip = accel_chip.replace(' ', '-')
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        lognames = (f'calibrate_plot_{axis}_{peak_msteps}-'
                    f'{fullstep}_{accel_chip}_{now}.png')
        title = (f"Dependency of desynchronization "
                 f"and functions ({''.join(lognames)})")
        ax.set_title('\n'.join(wrap(title, 66)), fontsize=10)
        ax.set_xlabel(f'Microsteps: 1/{fullstep}')
        ax.set_xticks(np.arange(0, max(x_data) + 2.5, 2.5))
        ax.xaxis.set_minor_locator(ticker.MultipleLocator(2.5))
        ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.set_ylabel('Magnitude')
        ax.ticklabel_format(axis='y', style='scientific', scilimits=pow_lim)
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax.grid(which='major', color='grey')
        ax.grid(which='minor', color='lightgrey')
        png_path = os.path.join(self.path, lognames)
        plt.savefig(png_path, dpi=1000)
        return f'Access to interactive plot at: {png_path}'

    def save_config(self, axis, func):
        configfile = self.sync.printer.lookup_object("configfile")
        axes = [axis] + [a for a in self.sync.motion[axis].joint_axes]
        for ax in axes:
            pname = f'steps_model_{ax}'
            func_name = func['name']
            func_coeffs = list(map(str, func['coeffs']))
            block = func_name + ',\n  ' + ',\n  '.join(func_coeffs)
            configfile.set('motors_sync', pname, block)
            msg = f"{pname}: {func_name}, {','.join(func_coeffs)}"
            self.gcode.respond_info(msg)
        self.gcode.respond_info(
            f"Motors sync model for '{', '.join(axes)}' axis "
             "has been calibrated.\nThe SAVE_CONFIG command "
             "will update the printer config\n file with new "
             "parameters and restart the printer.")

    def run_calibrate(self, gcmd, fullstep=16):
        # "m" is a main axis, just single axis
        repeats = gcmd.get_int('REPEATS', 2, minval=2, maxval=100)
        axis = gcmd.get('AXIS').lower()
        m = self.sync.motion.get(axis, None)
        if m is None:
            raise self.gcode.error(f'Invalid axis: {axis.upper()}')
        peak_mstep = gcmd.get_int('DISTANCE', fullstep,
                                  minval=2, maxval=fullstep*2)
        need_plot = False
        need_plot_str = gcmd.get('PLOT', 'True').lower()
        if need_plot_str == 'true' or need_plot_str == '1':
            need_plot = True
        self.gcode.respond_info(
            f'Calibration started on {axis.upper()} axis with '
            f'{repeats} repeats, move to +-{peak_mstep}/16 microstep')
        self.gcode.respond_info('Synchronizing before calibration...')
        self.sync.cmd_SYNC_MOTORS(gcmd, force_run=True)
        max_steps = 0
        invs = [1, -1, -1, 1]
        y_samples = [-1,]
        mcu_stepper1 = m.get_steppers()[1]
        looped_pos = itertools.cycle([m.rd, -m.rd, -m.rd, m.rd])
        m.on_start()
        # Scale calibration steps to 1/16
        m.move_msteps = m.microsteps // fullstep
        emul_peak_mstep = peak_mstep * m.move_msteps
        m.toggle_main_stepper(0)
        for r in range(1, repeats + 1):
            self.gcode.respond_info(
                f'Repeats: {r}/{repeats} Move to +-'
                f'{emul_peak_mstep}/{m.microsteps} microstep')
            self.sync.stepper_move.manual_move(
                mcu_stepper1, [next(looped_pos)])
            for inv in invs:
                m.set_move_dir(inv)
                for _ in range(peak_mstep):
                    m.single_move()
                    m.new_magnitude = m.measure_deviation()
                    self.sync.msg_helper.stepped_msg(m)
                    if m.new_magnitude > max(y_samples):
                        max_steps += 1
                    y_samples.append(m.new_magnitude)
        m.magnitude = m.new_magnitude
        m.on_done()
        # To array with removed first sample
        y_samples = np.sort(np.array(y_samples[1:]))
        x_samples = np.linspace(0.01, max_steps, len(y_samples))
        div = 1. if peak_mstep < 5 else peak_mstep / 5
        x_prio_len = int(x_samples.shape[0] // div)
        x_samples_str = ', '.join([f'{i:.2f}' for i in x_samples])
        y_samples_str = ', '.join([str(i) for i in y_samples])
        logging.info(f"motors_sync_calibrate: x = [{x_samples_str}]")
        logging.info(f"motors_sync_calibrate: y = [{y_samples_str}]")
        self.gcode.respond_info('Data processing...', True)
        def samples_processing():
            try:
                os.nice(10)
            except:
                pass
            try:
                info, data = self.find_best_func(x_samples, y_samples,
                                                 x_prio_len)
                msg = None
                if need_plot:
                    msg = self.plotter(*data, axis, m.chip_helper \
                                        .chip_name, peak_mstep, fullstep)
                c_conn.send((False, (msg, data)))
                c_conn.close()
            except Exception as e:
                c_conn.send((True, (e,)))
                c_conn.close()
        # Run plotter
        p_conn, c_conn = multiprocessing.Pipe()
        proc = multiprocessing.Process(target=samples_processing)
        proc.daemon = True
        proc.start()
        lim_t = 60.
        now = start_t = last_report_time = self.reactor.monotonic()
        while proc.is_alive():
            if now > last_report_time + 5.:
                last_report_time = now
                self.gcode.respond_info('Data processing...', True)
                if now > start_t + lim_t:
                    raise self.gcode.error(f'Data processing stuck!')
            now = self.reactor.pause(now + .1)
        err, res = p_conn.recv()
        if err:
            raise self.gcode.error(f'Data processing finished '
                                   f'with error: {res[0]}')
        if res[0]:
            self.gcode.respond_info(str(res[0]))
        self.save_config(axis, res[1][-1][0])


class KalmanLiteFilter:
    def __init__(self, A, H, Q, R, P0, x0):
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.P = self.st_p = P0
        self.x = self.st_x = x0
        self.I = 1

    def flush_data(self):
        self.x = self.st_x
        self.P = self.st_p

    def predict(self):
        self.x = self.A * self.x
        self.P = self.A * self.P * self.A + self.Q

    def update(self, z):
        self.predict()
        y = z - (self.H * self.x)
        S = self.H * self.P * self.H + self.R
        K = self.P * self.H * S
        self.x += K * y
        self.P = (self.I - K * self.H) * self.P
        return self.x

    def process_samples(self, samples):
        self.flush_data()
        return np.array(
            [self.update(z) for z in samples]).reshape(-1)


class LogFileHelper:
    def __init__(self, file_name, format, max_size=1000):
        self._load_modules()
        self.format = format
        self.max_size = max_size
        # Variables
        self.home_dir = os.path.dirname(os.path.realpath(__file__))
        self.log_path = os.path.join(self.home_dir, file_name)
        self.error = ''
        # Checks
        self.check_log()

    @staticmethod
    def _load_modules():
        for module in ['csv', 'ast']:
            globals()[module] = __import__(module)

    def check_log(self):
        if os.path.exists(self.log_path):
            header, raw_log = self.read_log(True)
            if ','.join(header) != self.format:
                self.clear_log()
            elif raw_log.shape[0] > self.max_size:
                self.trim_log(header, raw_log, self.max_size)
        else:
            try:
                self.write_log(self.format.split(','))
            except Exception as e:
                self.error = str(e)

    def read_log(self, with_header=False):
        with open(self.log_path, mode='r', newline='') as f:
            data = list(csv.reader(f, delimiter=','))
        if with_header:
            return data[0], np.array(data[1:])
        return np.array(data[1:])

    def write_log(self, line):
        with open(self.log_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(line)

    def trim_log(self, header, raw_log, max_size):
        last_rows = raw_log[-max_size:]
        with open(self.log_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(last_rows)

    def clear_log(self):
        os.remove(self.log_path)
        self.error = ''
        self.check_log()

    def parse_raw_log(self, log):
        converted = []
        for line in log:
            converted_line = []
            for element in line:
                element = str(element).strip("'\"")
                try:
                    converted_el = ast.literal_eval(element)
                except (ValueError, SyntaxError):
                    converted_el = element
                if isinstance(converted_el, tuple):
                    converted_el = np.array(converted_el)
                converted_line.append(converted_el)
            converted.append(converted_line)
        return np.array(converted, dtype=object)


class MotionAxesStats:
    def __init__(self, config):
        self.gcode = config.get_printer().lookup_object('gcode')
        # Register commands
        self.gcode.register_command('SYNC_MOTORS_STATS', self.cmd_GET_STATS,
                                    desc=self.cmd_GET_STATS_help)
        # Variables
        file_name = 'sync_stats.csv'
        format = 'axis,status,magnitudes,steps,msteps,retries,offset,date,'
        self.log_helper = LogFileHelper(file_name, format)

    def get_axes_phase_offsets(self, axes):
        raw_log = self.log_helper.read_log()
        if raw_log.size == 0:
            return {}
        log = self.log_helper.parse_raw_log(raw_log)
        a = {}
        for p in log[::-1]:
            if all(ax in a for ax in axes):
                return a
            if p[0] in a:
                continue
            if not p[1]:
                continue
            a[p[0]] = p[6]
        return a

    def write_axis_stats_log(self, axis, state):
        if self.log_helper.error:
            return
        if not axis.actual_msteps:
            return
        name = axis.name
        magnitudes, pos = zip(*axis.motion_log)
        msteps = axis.microsteps
        retries = axis.curr_retry
        offset = axis.get_phase_offset()
        date = datetime.now().strftime('%Y-%m-%d')
        self.log_helper.write_log([name, state, magnitudes, pos,
                                   msteps, retries, offset, date])

    def respond_stats(self, log):
        a = {}
        for p in log:
            a.setdefault(p[0], {
                'count': 0,
                'success': 0,
                'msteps': 0,
                'magnitudes': [0., 0., 0., 999999.],
                'retries': 0,
            })
            a[p[0]]['count'] += 1
            if p[1]:
                a[p[0]]['success'] += 1
            a[p[0]]['magnitudes'][:2] = (np.add(
                a[p[0]]['magnitudes'][:2], (p[2][-2], p[2][0])))
            if p[2].max() > a[p[0]]['magnitudes'][2]:
                a[p[0]]['magnitudes'][2] = p[2].max()
            if p[2].min() < a[p[0]]['magnitudes'][3]:
                a[p[0]]['magnitudes'][3] = p[2].min()
            a[p[0]]['msteps'] += abs(p[3][-2] / (p[4] / 16))
            a[p[0]]['retries'] += p[5]
        for axis, a in a.items():
            cf_microsteps = 16
            st_microsteps = a['msteps'] / a['count'] * (cf_microsteps / 16)
            msg = (f"""
            --------------------------------
            {axis.upper()} axis statistics:
            Successfully synced:     {a['success'] / a['count'] * 100:.2f}%
            Average start magnitude: {a['magnitudes'][1] / a['count']:.2f}
            Average end magnitude:   {a['magnitudes'][0] / a['count']:.2f}
            Average msteps count:    {st_microsteps:.0f}/{cf_microsteps}
            Average retries count:   {a['retries'] / a['count']:.2f}
            Min detected magnitude:  {a['magnitudes'][3]:.2f}
            Max detected magnitude:  {a['magnitudes'][2]:.2f}
            Synchronization count:   {a['count']}
            """)
            self.gcode.respond_info(msg)

    cmd_GET_STATS_help = 'Show statistics'
    def cmd_GET_STATS(self, gcmd):
        do_clear = gcmd.get('CLEAR', '').lower()
        if do_clear in ['true', '1']:
            self.log_helper.clear_log()
            self.gcode.respond_info('Statistics was cleared')
            return
        if self.log_helper.error:
            self.gcode.respond_info(f'Statistics collection is disabled '
                                    f'due:\n{self.log_helper.error}')
            return
        raw_log = self.log_helper.read_log()
        if raw_log.size == 0:
            self.gcode.respond_info('Logs are empty')
            return
        log = self.log_helper.parse_raw_log(raw_log)
        self.respond_stats(log)


def load_config(config):
    return MotorsSync(config)
