#include "pid.h"
#include "generic/misc.h"
extern "C" {
#include "command.h"
}

#include <cmath>

float
pid_controller::step(float current_temperature, float dt) {

    if (!this->set_point) {
        this->integrator = this->params.kp * current_temperature;
        this->output = 0.0f;
        return this->output;
    }

    auto sp = *this->set_point;

    auto params = &this->params;

    auto p = params->kp * (params->b * sp - current_temperature);

    float i = 0.0;
    if (params->ti >= 0.000001) {
        float t = 0.0;
        if (params->tt >= 0.000001) {
            auto error_sat =
                std::max(0.0f, std::min(1.0f, this->output)) - this->output;
            t = error_sat / params->tt;
        }
        this->integrator +=
            (params->kp * (sp - current_temperature) / params->ti + t) * dt;
        if (this->integrator > 1.0f)
            this->integrator = 1.0f;
        else if (this->integrator < 0.0)
            this->integrator = 0.0;
        i = this->integrator;
    }

    float d = 0.0;

    this->output = p + i + d;

    auto clamped_output = std::max(0.0f, std::min(1.0f, this->output));
    return clamped_output;
}
