#ifndef __INDX_PID_H__
#define __INDX_PID_H__

#include <optional>

struct pid_params {
    static pid_params
    zero() {
        return pid_params{
            .kp = 0.0,
            .ti = 0.0,
            .td = 0.0,
            .b = 0.0,
            .tt = 0.0,
        };
    }

    float kp;
    float ti;
    float td;
    float b;
    float tt;
};

struct pid_controller {
    pid_controller(pid_params params) : params(params) {}

    pid_params params;
    std::optional<float> set_point{std::nullopt};

    float output{0.0};
    float integrator{0.0};

    float
    step(float current_temperature, float dt);
};

#endif
