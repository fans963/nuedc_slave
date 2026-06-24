#pragma once

#include "peripheral/qdec.hpp"
#include "util/pid.hpp"
#include <algorithm>
#include <cstdint>
#include <zephyr/drivers/pwm.h>

/// DC motor closed-loop control: PID velocity → PWM duty cycle.
/// 1000Hz control loop via k_timer ISR.
class Motor {
public:
    struct Config {
        uint32_t pwm_period_ns; // PWM 周期 (e.g. 50000 = 20kHz)
        float output_min;       // PID 输出下限 (负值 = 反转)
        float output_max;       // PID 输出上限
        float vel_min;          // 目标速度下限 (rad/s)
        float vel_max;          // 目标速度上限 (rad/s)
    };

    Motor(const struct device* pwm_dev, uint8_t pwm_channel, Qdec& encoder, const Config& cfg);

    int init();

    /// Set PID gains.
    void set_pid(float kp, float ki, float kd);

    /// Set PID output and integral limits.
    void set_limits(float out_min, float out_max, float int_min, float int_max);

    /// Set target velocity in rad/s (from host MotorCommandPack).
    void set_target(float vel_rad_s);

    /// Current target velocity.
    float target() const { return target_; }

    /// Current measured velocity (from encoder).
    float measured() const { return encoder_.velocity(); }

    /// Current PID output (raw, before clamping).
    float output() { return pid_.compute(target_, encoder_.velocity()); }

private:
    const struct device* pwm_dev_;
    uint8_t pwm_channel_;
    Qdec& encoder_;
    Config cfg_;

    util::PidF pid_;
    float target_ = 0.0f;

    struct k_timer timer_;

    static void timer_callback(struct k_timer* timer);
    void on_timer();

    void set_pwm(float duty_ratio);
};
