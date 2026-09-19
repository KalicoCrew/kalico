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
            .i_window = 0.0,
            .i_limit = 0.0,
        };
    }

    float kp;
    float ti;
    float td;
    float b;
    float tt;
    float i_window;
    float i_limit;
};

struct pid_controller {
    pid_controller(pid_params params) : params(params) {}

    void
    update_params(pid_params params) {
        this->params = params;
        this->integrator = 0.0f;
    }

    void
    update_set_point(std::optional<float> set_point) {
        if (this->set_point_ != set_point)
            this->integrator = 0.0f;
        this->set_point_ = set_point;
    }

    std::optional<float>
    set_point() const {
        return this->set_point_;
    }

    float
    step(float current_temperature, float dt);

  private:
    pid_params params;
    std::optional<float> set_point_{std::nullopt};

    float output{0.0};
    float integrator{0.0};
};

#endif
