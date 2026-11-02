// RMCS 对照：保留按拍计算的 PID 和独立算法类，积分使用此前各拍的误差和。
// 首拍不计算微分，积分分离与限幅留到后续阶段。
#pragma once

#include <optional>

namespace mvp_core::controller::pid {

class PidCalculator {
public:
    PidCalculator(double kp, double ki, double kd)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd) {}

    void set_gains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    double update(double error) {
        double control = kp_ * error + ki_ * integral_;
        if (previous_error_)
            control += kd_ * (error - *previous_error_);
        integral_ += error;
        previous_error_ = error;
        return control;
    }

private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    std::optional<double> previous_error_;
};

} // namespace mvp_core::controller::pid
