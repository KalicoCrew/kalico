import collections

HomingInfo = collections.namedtuple(
    "HomingInfo",
    [
        "speed",
        "position_endstop",
        "retract_speed",
        "retract_dist",
        "positive_dir",
        "second_homing_speed",
        "use_sensorless_homing",
        "min_home_dist",
        "accel",
    ],
)


class BaseRail:
    def __init__(self):
        self.second_homing_speed = 0.0
        self.use_sensorless_homing = False
        self.min_home_dist = 0.0
        self.homing_accel = None

    def _parse_position_range(self, config):
        self.position_min = config.getfloat("position_min", 0.0)
        self.position_max = config.getfloat(
            "position_max", above=self.position_min
        )

    def _parse_homing_speeds(self, config):
        self.homing_speed = config.getfloat("homing_speed", 5.0, above=0.0)
        self.homing_retract_dist = config.getfloat(
            "homing_retract_dist", 5.0, minval=0.0
        )
        self.homing_retract_speed = config.getfloat(
            "homing_retract_speed", self.homing_speed, above=0.0
        )

    def get_range(self):
        return self.position_min, self.position_max

    def get_homing_info(self):
        return HomingInfo(
            speed=self.homing_speed,
            position_endstop=self.position_endstop,
            retract_speed=self.homing_retract_speed,
            retract_dist=self.homing_retract_dist,
            positive_dir=self.homing_positive_dir,
            second_homing_speed=self.second_homing_speed,
            use_sensorless_homing=self.use_sensorless_homing,
            min_home_dist=self.min_home_dist,
            accel=self.homing_accel,
        )
