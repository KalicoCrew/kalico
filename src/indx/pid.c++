#include "pid.h"
#include "generic/misc.h"
extern "C" {
#include "command.h"
}

#include <cmath>

float
pid_controller::step(float current_temperature, float dt) {

    if (!this->set_point_) {
        this->output = 0.0f;
        this->integrator = 0.0f;
        return this->output;
    }

    auto sp = *this->set_point_;

    auto params = &this->params;

    auto p = params->kp * (params->b * sp - current_temperature);

    auto error = sp - current_temperature;
    if (params->ti < 0.000001 || fabsf(error) > params->i_window) {
        this->integrator = 0.0f;
    } else {
        this->integrator = std::max(
            0.0f,
            std::min(params->i_limit,
                     this->integrator + params->kp * error / params->ti * dt));
    }

    float d = 0.0;

    this->output = p + this->integrator + d;

    auto clamped_output = std::max(0.0f, std::min(1.0f, this->output));
    return clamped_output;
}
