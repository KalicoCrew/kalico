import collections


class ProfileManager:
    def __init__(self, outer_instance, control_types):
        self.outer_instance = outer_instance
        self.control_types = control_types
        self.profiles = {}
        self.incompatible_profiles = []
        # Fetch stored profiles from Config
        stored_profs = self.outer_instance.config.get_prefix_sections(
            "pid_profile %s" % self.outer_instance.short_name
        )
        for profile in stored_profs:
            self._init_profile(
                profile, profile.get_name().split(" ", 2)[-1]
            )

    def _init_profile(self, config_section, name):
        control = self._check_value_config("control", config_section, str, False)
        if control in self.control_types.keys():
            temp_profile = self.control_types[control].init_profile(
                config_section, name, self
            )
            if temp_profile is not None:
                temp_profile["control"] = control
                temp_profile["name"] = name
                self.profiles[name] = temp_profile
            return temp_profile
        else:
            raise self.outer_instance.printer.config_error(
                "Unknown control type '%s' "
                "in [pid_profile %s %s]."
                % (control, self.outer_instance.name, name)
            )

    def _check_value_config(self, key, config_section, type, can_be_none):
        if type is int:
            value = config_section.getint(key, None)
        elif type is float:
            value = config_section.getfloat(key, None)
        else:
            value = config_section.get(key, None)
        if not can_be_none and value is None:
            raise self.outer_instance.gcode.error(
                "pid_profile: '%s' has to be "
                "specified in [pid_profile %s %s]."
                % (
                    key,
                    self.outer_instance.name,
                    config_section.get_name(),
                )
            )
        return value

    def _compute_section_name(self, profile_name):
        return (
            self.outer_instance.name
            if profile_name == "default"
            else (
                "pid_profile " + self.outer_instance.name + " " + profile_name
            )
        )

    def _check_value_gcmd(
        self,
        name,
        default,
        gcmd,
        type,
        can_be_none,
        minval=None,
        maxval=None,
    ):
        if type is int:
            value = gcmd.get_int(name, default, minval=minval, maxval=maxval)
        elif type is float:
            value = gcmd.get_float(name, default, minval=minval, maxval=maxval)
        else:
            value = gcmd.get(name, default)
        if not can_be_none and value is None:
            raise self.outer_instance.gcode.error(
                "pid_profile: '%s' has to be specified." % name
            )
        return value.lower() if type == "lower" else value

    def init_default_profile(self):
        return self._init_profile(self.outer_instance.config, "default")

    def set_values(self, profile_name, gcmd, verbose):
        current_profile = self.outer_instance.get_control().get_profile()
        target = self._check_value_gcmd("TARGET", None, gcmd, float, False)
        tolerance = self._check_value_gcmd(
            "TOLERANCE",
            current_profile["pid_tolerance"],
            gcmd,
            float,
            False,
        )
        control = self._check_value_gcmd(
            "CONTROL", current_profile["control"], gcmd, "lower", False
        )
        kp = self._check_value_gcmd("KP", None, gcmd, float, False)
        ki = self._check_value_gcmd("KI", None, gcmd, float, False)
        kd = self._check_value_gcmd("KD", None, gcmd, float, False)
        smooth_time = self._check_value_gcmd("SMOOTH_TIME", None, gcmd, float, True)
        smoothing_elements = self._check_value_gcmd(
            "SMOOTHING_ELEMENTS", None, gcmd, int, True
        )
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        temp_profile = {
            "pid_target": target,
            "pid_tolerance": tolerance,
            "control": control,
            "smooth_time": smooth_time,
            "smoothing_elements": smoothing_elements,
            "pid_kp": kp,
            "pid_ki": ki,
            "pid_kd": kd,
        }
        temp_control = self.outer_instance.lookup_control(temp_profile, load_clean)
        self.outer_instance.set_control(temp_control, keep_target)
        msg = (
            "PID Parameters:\n"
            "Target: %.2f,\n"
            "Tolerance: %.4f\n"
            "Control: %s\n" % (target, tolerance, control)
        )
        if smooth_time is not None:
            msg += "Smooth Time: %.3f\n" % smooth_time
        msg += (
            "pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "have been set as current profile." % (kp, ki, kd)
        )
        self.outer_instance.gcode.respond_info(msg)
        self.save_profile(profile_name=profile_name, verbose=True)

    def get_values(self, profile_name, gcmd, verbose):
        temp_profile = self.outer_instance.get_control().get_profile()
        target = temp_profile["pid_target"]
        tolerance = temp_profile["pid_tolerance"]
        control = temp_profile["control"]
        kp = temp_profile["pid_kp"]
        ki = temp_profile["pid_ki"]
        kd = temp_profile["pid_kd"]
        smooth_time = (
            self.outer_instance.get_smooth_time()
            if temp_profile["smooth_time"] is None
            else temp_profile["smooth_time"]
        )
        smoothing_elements = (
            self.outer_instance.get_smoothing_elements()
            if temp_profile["smoothing_elements"] is None
            else temp_profile["smoothing_elements"]
        )
        name = temp_profile["name"]
        self.outer_instance.gcode.respond_info(
            "PID Parameters:\n"
            "Target: %.2f,\n"
            "Tolerance: %.4f\n"
            "Control: %s\n"
            "Smooth Time: %.3f\n"
            "Smoothing Elements: %d\n"
            "pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "name: %s"
            % (
                target,
                tolerance,
                control,
                smooth_time,
                smoothing_elements,
                kp,
                ki,
                kd,
                name,
            )
        )

    def save_profile(self, profile_name=None, gcmd=None, verbose=True):
        temp_profile = self.outer_instance.get_control().get_profile()
        self.control_types[temp_profile["control"]].save_profile(
            pmgr=self, profile_name=profile_name, gcmd=gcmd, verbose=verbose
        )

    def load_profile(self, profile_name, gcmd, verbose):
        verbose = self._check_value_gcmd("VERBOSE", "low", gcmd, "lower", True)
        load_clean = self._check_value_gcmd(
            "LOAD_CLEAN", 0, gcmd, int, True, minval=0, maxval=1
        )
        if (
            profile_name == self.outer_instance.get_control().get_profile()["name"]
            and not load_clean
        ):
            if verbose == "high" or verbose == "low":
                self.outer_instance.gcode.respond_info(
                    "Heater Profile [%s] already loaded for heater [%s]."
                    % (profile_name, self.outer_instance.name)
                )
            return
        keep_target = self._check_value_gcmd(
            "KEEP_TARGET", 0, gcmd, int, True, minval=0, maxval=1
        )
        profile = self.profiles.get(profile_name, None)
        defaulted = False
        default = gcmd.get("DEFAULT", None)
        if profile is None:
            if default is None:
                raise self.outer_instance.gcode.error(
                    "pid_profile: Unknown profile [%s] for heater [%s]."
                    % (profile_name, self.outer_instance.name)
                )
            profile = self.profiles.get(default, None)
            defaulted = True
            if profile is None:
                raise self.outer_instance.gcode.error(
                    "pid_profile: Unknown default "
                    "profile [%s] for heater [%s]."
                    % (default, self.outer_instance.name)
                )
        control = self.outer_instance.lookup_control(profile, load_clean)
        self.outer_instance.set_control(control, keep_target)

        if verbose != "high" and verbose != "low":
            return
        if defaulted:
            self.outer_instance.gcode.respond_info(
                "Couldn't find profile "
                "[%s] for heater [%s]"
                ", defaulted to [%s]."
                % (profile_name, self.outer_instance.name, default)
            )
        self.outer_instance.gcode.respond_info(
            "Heater Profile [%s] loaded for heater [%s].\n"
            % (profile["name"], self.outer_instance.name)
        )
        if verbose == "high":
            smooth_time = (
                self.outer_instance.get_smooth_time()
                if profile["smooth_time"] is None
                else profile["smooth_time"]
            )
            smoothing_elements = (
                self.outer_instance.get_smoothing_elements()
                if profile["smoothing_elements"] is None
                else profile["smoothing_elements"]
            )
            msg = (
                "Target: %.2f\n"
                "Tolerance: %.4f\n"
                "Control: %s\n"
                % (
                    profile["pid_target"],
                    profile["pid_tolerance"],
                    profile["control"],
                )
            )
            if smooth_time is not None:
                msg += "Smooth Time: %.3f\n" % smooth_time
            if smoothing_elements is not None:
                msg += "Smoothing Elements: %d\n" % smoothing_elements
            msg += "PID Parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n" % (
                profile["pid_kp"],
                profile["pid_ki"],
                profile["pid_kd"],
            )
            self.outer_instance.gcode.respond_info(msg)

    def remove_profile(self, profile_name, gcmd, verbose):
        if profile_name in self.profiles:
            section_name = self._compute_section_name(profile_name)
            self.outer_instance.configfile.remove_section(section_name)
            profiles = dict(self.profiles)
            del profiles[profile_name]
            self.profiles = profiles
            self.outer_instance.gcode.respond_info(
                "Profile [%s] for heater [%s] "
                "removed from storage for this session.\n"
                "The SAVE_CONFIG command will update the printer\n"
                "configuration and restart the printer"
                % (profile_name, self.outer_instance.name)
            )
        else:
            self.outer_instance.gcode.respond_info(
                "No profile named [%s] to remove" % profile_name
            )

    cmd_PID_PROFILE_help = "Heater Profile Persistent Storage management"

    def cmd_PID_PROFILE(self, gcmd):
        options = collections.OrderedDict(
            {
                "LOAD": self.load_profile,
                "SAVE": self.save_profile,
                "GET_VALUES": self.get_values,
                "SET_VALUES": self.set_values,
                "REMOVE": self.remove_profile,
            }
        )
        for key in options:
            profile_name = gcmd.get(key, None)
            if profile_name is not None:
                if not profile_name.strip():
                    raise self.outer_instance.gcode.error(
                        "pid_profile: Profile must be specified"
                    )
                options[key](profile_name, gcmd, True)
                return
        raise self.outer_instance.gcode.error(
            "pid_profile: Invalid syntax '%s'" % (gcmd.get_commandline(),)
        )
