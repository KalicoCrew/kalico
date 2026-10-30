#Copied and stripped down from https://github.com/Snapmaker/u1-klipper/blob/93cb12faebf6e74c51bbc5f0be66e95124af14c8/klippy/extras/homing_precise_corexy.py

import math, logging
import stepper
from . import homing

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"]

class HomingPreciseCorexy:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        if config.getsection('printer').get('kinematics') != 'corexy':
            raise config.error("homing_precise_corexy: kinematics must be corexy!")
        self.xy_back_offset = config.getfloat('xy_back_offset', 5., above=0.)
        self.probe_before_delay = config.getfloat('diagonal_probe_before_delay', 0.1, minval=0)
        self.probe_tolerance = config.getfloat('diagonal_probe_tolerance', 0.0, minval=0)
        self.probe_tolerance_retries = config.getint('diagonal_probe_tolerance_retries', 10, minval=0)
        self.probe_accel = config.getfloat('diagonal_probe_accel', None, above=0.)
        self.probe_speed = config.getfloat('diagonal_probe_speed', above=0.)
        self.probe_retract_speed = config.getfloat('diagonal_probe_retract_speed', None, above=0.)
        if self.probe_retract_speed is None:
            self.probe_retract_speed = self.probe_speed
        # Currently only supports diagonal_move_rail = 1 (B-motor), diagonal_move_rail = 0 (A-motor) not yet adapted
        self.move_rail = 1
        sconfig = config.getsection('stepper_x')
        rotation_dist, steps_per_rotation = stepper.parse_step_distance(sconfig)
        self.x_step_dist = rotation_dist / steps_per_rotation
        self.x_phases = sconfig.getint("microsteps", note_valid=False) * 4
        sconfig = config.getsection('stepper_y')
        rotation_dist, steps_per_rotation = stepper.parse_step_distance(sconfig)
        self.y_step_dist = rotation_dist / steps_per_rotation
        self.y_phases = sconfig.getint("microsteps", note_valid=False) * 4
        self.x_tmc_module = None
        self.y_tmc_module = None
        self.c_a_full_phase = None
        self.c_b_full_phase = None

        self.printer.register_event_handler("klippy:connect", self.lookup_tmc)
        self.gcode.register_command('HOMING_PRECISE_COREXY', self.cmd_HOMING_PRECISE_COREXY)

    def lookup_tmc(self):
        for driver in TRINAMIC_DRIVERS:
            x_driver_name = "%s stepper_x" % (driver)
            y_driver_name = "%s stepper_y" % (driver)
            module_x = self.printer.lookup_object(x_driver_name, None)
            module_y = self.printer.lookup_object(y_driver_name, None)
            if module_x is not None:
                self.x_tmc_module = module_x
            if module_y is not None:
                self.y_tmc_module = module_y

    def check_xy_is_homed(self):
        toolhead = self.printer.lookup_object('toolhead')
        homed_axes = toolhead.get_kinematics().get_status(0)['homed_axes']
        if 'x' not in homed_axes or 'y' not in homed_axes:
            error = '{"coded": "0002-0528-0000-0010", "msg":"%s", "action": "pause"}' % ("Corexy precise homing aborted: Must home x and y axis first")
            raise self.printer.command_error(error)

    def cal_backoff_position(self, move_offset_x, move_offset_y):
        toolhead = self.printer.lookup_object('toolhead')
        corexy_rails = toolhead.get_kinematics().rails
        x_homing_info = corexy_rails[0].get_homing_info()
        y_homing_info = corexy_rails[1].get_homing_info()
        start_park_position_x = x_homing_info.position_endstop + [1, -1][x_homing_info.positive_dir == True]*move_offset_x
        start_park_position_y = y_homing_info.position_endstop + [1, -1][y_homing_info.positive_dir == True]*move_offset_y
        return start_park_position_x, start_park_position_y

    def calc_position(self, a_dist, b_dist):
        return (a_dist+b_dist)*0.5, (a_dist-b_dist)*0.5

    def phase_backoff_steps(self, corexy_rails):
        if self.x_tmc_module is None or self.y_tmc_module is None:
            raise self.printer.command_error("stepper_x, stepper_y must use the specified tmc devices")
        # Getting an effective direction of travel
        x_away_from_endstop_dir = not corexy_rails[0].get_homing_info().positive_dir
        y_away_from_endstop_dir = not corexy_rails[1].get_homing_info().positive_dir
        x_stepper_dir_inverted = corexy_rails[0].get_steppers()[0].get_dir_inverted()[0]
        y_stepper_dir_inverted = corexy_rails[1].get_steppers()[0].get_dir_inverted()[0]
        x_backoff_dir = [1, -1][x_away_from_endstop_dir == False] * [-1, 1][x_stepper_dir_inverted]
        y_backoff_dir = [1, -1][y_away_from_endstop_dir == False] * [-1, 1][y_stepper_dir_inverted]
        # Calculate the step to be moved for tmc phase alignment
        x_backoff_step = (([1, -1][x_backoff_dir == 1]*self.x_tmc_module.query_phase()) % 1024) / (1024 / self.x_tmc_module.get_phase_offset()[1])
        y_backoff_step = (([1, -1][y_backoff_dir == 1]*self.y_tmc_module.query_phase()) % 1024) / (1024 / self.y_tmc_module.get_phase_offset()[1])
        return x_backoff_step, y_backoff_step

    def diagonal_probe(self, endstops, movepos):
        toolhead = self.printer.lookup_object('toolhead')
        if self.probe_before_delay:
            toolhead.dwell(self.probe_before_delay)
            toolhead.wait_moves()
        hmove = homing.HomingMove(self.printer, endstops)
        # start diagonal probe
        epos = hmove.homing_move(movepos, self.probe_speed, True)
        toolhead.flush_step_generation()
        trigger_mcu_pos = {sp.stepper_name: sp.trig_pos
                                for sp in hmove.stepper_positions}
        return epos, trigger_mcu_pos

    def cal_diagonal_dist(self, m_steps):
        d = (abs(m_steps[0][0]) + abs(m_steps[0][1]) + abs(m_steps[1][0]) + abs(m_steps[1][1])) // 2
        c_dist_a = (d + self.x_phases) // (2 * self.x_phases)
        d_y = d - (abs(m_steps[0][0]) + abs(m_steps[0][1]))
        d_y2 = abs(d_y) + self.y_phases
        if d_y < 0:
            d_y2 = -d_y2
        c_dist_b = d_y2 // (2 * self.y_phases)

        c_a_raw = (d) / (2 * self.x_phases)
        c_b_raw = d_y / (2*self.y_phases)
        return c_dist_a, c_dist_b, c_a_raw, c_b_raw

    def round_to_nearest_multiple(self, value, multiple):
        return round(value / multiple) * multiple

    def round_steps_to_multiples(self, m_steps):
        aligned_steps = []
        for i, step_group in enumerate(m_steps):
            aligned_group = []
            for step_value in step_group:
                phase_size = (self.x_phases/4) if self.move_rail == 0 else (self.y_phases/4)
                aligned_value = self.round_to_nearest_multiple(step_value, phase_size)
                aligned_group.append(aligned_value)
            aligned_steps.append(aligned_group)
        return aligned_steps

    def is_point_unstable(self, c_dist, origin=None):
        threshold = 1.0 / 4
        if origin is None:
            origin = [0.0, 0.0]
        for i in range(2):
            diff = c_dist[i] - origin[i]
            diff = math.fmod(diff, 1.0)
            if abs(diff - 0.5) < threshold:
                self.gcode.respond_info("Point is unstable, c_dist={}, origin={}".format(c_dist, origin))
                return True
        return False

    def translate_to_ab_grid(self, c_dist, origin=None):
        c_ab = [0, 0]
        if origin is None:
                origin = [0.0, 0.0]
        for i in range(2):
            o_int = int(round(origin[i]))
            c_ab[i] = int(round(c_dist[i] - origin[i])) + o_int
        return c_ab

    def perform_diagonal_probing(self, probe_start_x, probe_start_y, xy_back_offset=None):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        corexy_rails = toolhead.get_kinematics().rails
        if xy_back_offset is None:
            xy_back_offset = self.xy_back_offset
        try:
            toolhead.manual_move([probe_start_x, probe_start_y, None], self.probe_retract_speed)
            toolhead.wait_moves()
            kin.set_ignore_check_move_limit(True)
            probe_cnt, sample, recv_steps, recv_dists = 0, 0, [[], []], [[], []]
            park_step = corexy_rails[1].get_steppers()[0].get_mcu_position()
            probe_success = False
            while probe_cnt <= self.probe_tolerance_retries:
                x_move_position, y_move_position = self.calc_position(0, [-1, 1][sample % 2 == 0]*xy_back_offset*4)
                toolhead.wait_moves()

                pos = toolhead.get_position()
                pos[0] = pos[0] + x_move_position
                pos[1] = pos[1] + y_move_position
                epos, trigger_mcu_pos = self.diagonal_probe(corexy_rails[1].get_endstops(), pos)
                recv_steps[[0, 1][sample % 2 == 1]].append(park_step - trigger_mcu_pos['stepper_y'])
                recv_dists[[0, 1][sample % 2 == 1]].append((round(epos[0]-probe_start_x, 5), round(epos[1]-probe_start_y, 5)))
                toolhead.manual_move([probe_start_x, probe_start_y, None], self.probe_retract_speed)
                toolhead.wait_moves()
                sample += 1
                if len(recv_steps[1]) >= 2:
                    if (((abs(recv_steps[0][0] - recv_steps[0][1])*self.y_step_dist) > self.probe_tolerance) or
                        (abs((recv_steps[1][0] - recv_steps[1][1])*self.y_step_dist) > self.probe_tolerance)):
                        sample = 0
                        probe_cnt += 1
                        self.gcode.respond_info("###{}, {}, {}####".format(abs((abs(recv_steps[0][0]) - abs(recv_steps[0][1]))*self.y_step_dist), abs((abs(recv_steps[1][0]) - abs(recv_steps[1][1]))*self.y_step_dist), recv_steps))
                        recv_steps = [[], []]
                        probe_success = False
                    else:
                        probe_success = True
                        recv_steps = self.round_steps_to_multiples(recv_steps)
                        break
            return recv_steps, recv_dists, probe_success
        except Exception as e:
            raise
        finally:
            kin.set_ignore_check_move_limit(False)

    def generate_and_save_results(self, c_dist_a, c_dist_b, c_dist_raw_a, c_dist_raw_b,
                                probe_start_x, probe_start_y, recv_steps, recv_dists):
        x, y = self.calc_position(c_dist_a*self.x_phases*self.x_step_dist,
                                c_dist_b*self.y_phases*self.y_step_dist)
        cal_x_position, cal_y_position = self.cal_backoff_position(x, y)
        results = [
                "%.5f" % probe_start_x, "%.5f" % probe_start_y,
                "%.5f" % cal_x_position, "%.5f" % cal_y_position,
                str(recv_steps),
                ]
        results.extend(["%.5f" % c_dist_raw_a, "%.5f" % c_dist_raw_b])
        results.extend([str(c_dist_a), str(c_dist_b)])
        data_line = ", ".join(results) + "\n"
        self.gcode.respond_info(data_line)
        # self.save_calibration_data(data_line)
        return cal_x_position, cal_y_position

    def precise_corexy_coord(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        # homing_xyz_override = self.printer.lookup_object('homing_xyz_override', None)
        kin = toolhead.get_kinematics()
        cur_max_accel = None
        home_unstable = False
        try:
            # Ensure that x and y have go home before diagonally moving the probe.
            self.check_xy_is_homed()
            corexy_rails = toolhead.get_kinematics().rails
            # Move to the specified position, manual_move detects if the target position is out of range
            x_position, y_position = self.cal_backoff_position(self.xy_back_offset, self.xy_back_offset)
            toolhead.manual_move([x_position, y_position, None], self.probe_retract_speed)
            toolhead.wait_moves()
            self.c_a_full_phase = None
            self.c_b_full_phase = None
            # full phase alignment
            x_step, y_step = self.phase_backoff_steps(corexy_rails)
            x_dist, y_dist = self.calc_position(x_step*self.x_step_dist, y_step*self.y_step_dist)
            pos = toolhead.get_position()
            probe_start_x, probe_start_y = pos[0]+x_dist, pos[1]+y_dist
            self.gcode.respond_info("probe_start_x: {:.5f}, probe_start_y: {:.5f}".format(probe_start_x, probe_start_y))
            if self.probe_accel is not None:
                cur_max_accel = toolhead.max_accel
                toolhead.set_accel(self.probe_accel)

            recv_steps, recv_dists, probe_success = self.perform_diagonal_probing(probe_start_x, probe_start_y)
            if probe_success == True:
                c_dist_a, c_dist_b, c_a_raw, c_b_raw = self.cal_diagonal_dist(recv_steps)
                self.c_a_full_phase = round(c_a_raw, 5)
                self.c_b_full_phase = round(c_b_raw, 5)
                if self.is_point_unstable([c_dist_a, c_dist_b]):
                    home_unstable = True
                    if home_unstable:
                        error = '{"coded": "0003-0530-0000-0027", "msg":"%s", "action": "cancel"}' % ("Corexy home is unstable")
                        raise self.printer.command_error(error)
                    else:
                        self.gcode.respond_info("Corexy home point is unstable")
                c_ab = self.translate_to_ab_grid([c_dist_a, c_dist_b])#, self.calibrated_origin)
                cal_x_position, cal_y_position = self.generate_and_save_results(
                                                c_ab[0], c_ab[1], c_a_raw, c_b_raw,
                                                probe_start_x, probe_start_y, recv_steps, recv_dists)

                # Check if calculated data is reasonable
                if abs(cal_x_position - probe_start_x) > self.x_step_dist*self.x_phases*2 or abs(cal_y_position - probe_start_y) > self.y_step_dist*self.y_phases*2:
                    error = '{"coded": "0002-0528-0000-0011", "msg":"%s", "action": "pause"}' % ("Corexy precise homing computed coordinate anomaly")
                    logging.info(f"Corexy precise homing: cal_x: {cal_x_position}, cal_y: {cal_y_position}, probe_x: {probe_start_x}, probe_y: {probe_start_y}")
                    logging.info(f"x_step_dist: {self.x_step_dist}, x_phases: {self.x_phases}, y_step_dist: {self.y_step_dist}, y_phases: {self.y_phases}")
                    raise self.printer.command_error(error)
                toolhead.wait_moves()
                pos = toolhead.get_position()
                pos[0], pos[1] = cal_x_position, cal_y_position
                toolhead.set_position(pos)
            else:
                error = '{"coded": "0002-0528-0000-0012", "msg":"%s", "action": "pause"}' % ("Corexy precise homing failed")
                raise self.printer.command_error(error)

        except Exception as e:
            can_motor_off = True
            if ("z" in kin.get_status(0)['homed_axes']):
                machine_state_manager = self.printer.lookup_object('machine_state_manager', None)
                if machine_state_manager is not None:
                    cur_sta = machine_state_manager.get_status()
                    if str(cur_sta["main_state"]) == "PRINTING":
                        can_motor_off = False

            if can_motor_off:
                self.printer.lookup_object('stepper_enable').motor_off()
            else:
                if hasattr(kin, "note_x_not_homed"):
                    kin.note_x_not_homed()
                if hasattr(kin, "note_y_not_homed"):
                    kin.note_y_not_homed()
            raise
        finally:
            if cur_max_accel is not None:
                toolhead.set_accel(cur_max_accel)

    def cmd_HOMING_PRECISE_COREXY(self, gcmd):
        self.precise_corexy_coord(gcmd)


    def get_status(self, eventtime):
        return {
            'c_a_full_phase': self.c_a_full_phase,
            'c_b_full_phase': self.c_b_full_phase
        }

def load_config(config):
    return HomingPreciseCorexy(config)
