"""Simulated ADC + heater feedback.

For each thermistor we maintain a HeaterModel that ramps its
temperature toward a target at a configured rate. The orchestrator
calls step(dt) periodically and pushes the resulting temperature
through temp_to_adc() into the firmware via runtime_sim_adc_set.

For the success bar (boot + G28 + small print), the model is crude —
no real thermal mass, no PID overshoot. Heater PWM writes are
ignored; we ramp directly toward the user-set target. The real
heater dynamics aren't relevant for the unknown-unknown bug-catching
goal of the sim.
"""
import math


class HeaterModel:
    def __init__(self, initial_temp_c: float, ramp_rate_c_per_s: float):
        self.temp_c = float(initial_temp_c)
        self.target_c = float(initial_temp_c)
        self._rate = float(ramp_rate_c_per_s)

    def set_target(self, target_c: float) -> None:
        self.target_c = float(target_c)

    def step(self, dt_s: float) -> None:
        delta = self.target_c - self.temp_c
        max_step = self._rate * dt_s
        if abs(delta) <= max_step:
            self.temp_c = self.target_c
        else:
            self.temp_c += max_step if delta > 0 else -max_step


def temp_to_adc(temp_c: float) -> int:
    """Convert a temperature in °C to a 12-bit ADC reading for a
    10 kΩ NTC thermistor with a 4700 Ω pull-up to 3.3 V.

    R(T) = R0 * exp(B * (1/T - 1/T0))
    V_adc = 3.3 * R / (R + pull_up)
    adc   = int(V_adc / 3.3 * 4095)
    """
    B = 3950.0
    T0 = 298.15  # 25 °C in Kelvin
    R0 = 10000.0
    T = float(temp_c) + 273.15
    R = R0 * math.exp(B * (1.0 / T - 1.0 / T0))
    pull_up = 4700.0
    v_adc = 3.3 * R / (R + pull_up)
    adc = int(v_adc / 3.3 * 4095)
    return max(1, min(4094, adc))
