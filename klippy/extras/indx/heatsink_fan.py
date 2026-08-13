from klippy.extras import fan

from .compat import ConfigWrapper

BLOCKED_CHECK_SECONDS = 5


class IndxHeatsinkFan:
    def __init__(self, toolboard):
        self.toolboard = toolboard

        mcu_name = self.toolboard.mcu.get_name()
        config = ConfigWrapper(
            self.toolboard.printer,
            f"fan {self.toolboard.name}_heatsink",
            {
                "shutdown_speed": 1.0,
                "pin": f"{mcu_name}:fan0_out",
                "tachometer_pin": f"{mcu_name}:fan0_tacho",
                "tachometer_poll_interval": 30.0 / (2.0 * 25000.0),
            },
        )
        self.fan = fan.Fan(config)

        self.toolboard.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )
        self.toolboard.printer.register_event_handler(
            "klippy:shutdown", self.handle_shutdown
        )
        self.check_timer = None
        self.active_stopped_count = 0
        self.is_active = 0.0
        self.steppers = []
        self.heaters = []

    def handle_connect(self):
        reactor = self.toolboard.printer.get_reactor()
        self.check_timer = reactor.register_timer(self.check_event, reactor.NOW)

        stepper_enable = self.toolboard.printer.lookup_object("stepper_enable")
        steppers = [
            stepper_enable.lookup_enable(s)
            for s in stepper_enable.get_steppers()
        ]
        self.steppers = [
            s
            for s in steppers
            if s.stepper.get_mcu().get_name() == self.toolboard.mcu.get_name()
        ]

    def handle_shutdown(self):
        if self.check_timer is not None:
            reactor = self.toolboard.printer.get_reactor()
            reactor.update_timer(self.check_timer, reactor.NEVER)

    def add_heater(self, heater):
        self.heaters.append(heater)

    def check_event(self, eventtime):
        activate = False

        for heater in self.heaters:
            current_temp, target_temp = heater.get_temp(eventtime)
            activate |= bool(target_temp != 0.0)
            activate |= current_temp >= 40.0
        for s in self.steppers:
            activate |= s.is_motor_enabled()

        fan_status = self.fan.get_status(eventtime)
        rpm = fan_status["rpm"]

        if self.is_active and activate:
            if rpm < 1000:
                self.active_stopped_count += 1
                if self.active_stopped_count >= BLOCKED_CHECK_SECONDS:
                    self.toolboard.printer.invoke_shutdown(
                        "INDX Heatsink fan block detected"
                    )
                    return self.toolboard.printer.get_reactor().NEVER
        else:
            self.active_stopped_count = 0

        if not self.is_active and activate:
            self.is_active = True
            self.fan.set_speed(1.0)
        elif self.is_active and not activate:
            self.is_active = False
            self.fan.set_speed(0.0)

        return eventtime + 1.0
