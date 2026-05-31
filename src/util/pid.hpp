#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>

namespace util {

/// Compile-time configurable PID controller.
/// T: float or double
/// Anti-windup: clamping + conditional integration
/// Derivative: optional low-pass filter on D term
template <std::floating_point T = float>
class Pid {
public:
    struct Gains {
        T kp{};
        T ki{};
        T kd{};
    };

    struct Limits {
        T out_min{T(-1e30)};
        T out_max{T(1e30)};
        T integral_min{T(-1e30)};
        T integral_max{T(1e30)};
    };

    struct Config {
        Gains gains{};
        Limits limits{};
        T dt{};
        T d_filter_alpha{T(1)}; // 1 = no filter, 0 = filter off
    };

    constexpr explicit Pid() = default;
    constexpr explicit Pid(const Config &cfg) : cfg_(cfg) {}

    // ── Configuration ──────────────────────────────────────────────────

    constexpr void set_gains(T kp, T ki, T kd) {
        cfg_.gains = {kp, ki, kd};
    }

    constexpr void set_limits(T out_min, T out_max,
                              T int_min, T int_max) {
        cfg_.limits = {out_min, out_max, int_min, int_max};
    }

    constexpr void set_dt(T dt) { cfg_.dt = dt; }
    constexpr void set_d_filter(T alpha) { cfg_.d_filter_alpha = alpha; }

    constexpr const Config &config() const { return cfg_; }
    constexpr const Gains &gains() const { return cfg_.gains; }

    // ── Core computation ───────────────────────────────────────────────

    /// Compute PID output. `setpoint` and `measurement` are raw values.
    constexpr T compute(T setpoint, T measurement) {
        T error = setpoint - measurement;
        return compute_error(error);
    }

    /// Compute PID output from pre-computed error.
    constexpr T compute_error(T error) {
        // Proportional
        T p = cfg_.gains.kp * error;

        // Integral with conditional anti-windup
        integral_ += cfg_.gains.ki * error * cfg_.dt;
        integral_ = std::clamp(integral_, cfg_.limits.integral_min, cfg_.limits.integral_max);

        // Derivative with optional low-pass filter
        T raw_d = (error - prev_error_) / cfg_.dt;
        d_filtered_ = cfg_.d_filter_alpha * raw_d
                     + (T(1) - cfg_.d_filter_alpha) * d_filtered_;
        T d = cfg_.gains.kd * d_filtered_;

        prev_error_ = error;

        // Output with clamping
        T output = p + integral_ + d;
        output = std::clamp(output, cfg_.limits.out_min, cfg_.limits.out_max);

        // Conditional anti-windup: stop integrating if output is saturated
        // and error would make it worse
        if ((output >= cfg_.limits.out_max && error > T(0)) ||
            (output <= cfg_.limits.out_min && error < T(0))) {
            integral_ -= cfg_.gains.ki * error * cfg_.dt;
        }

        return output;
    }

    // ── State management ───────────────────────────────────────────────

    constexpr void reset() {
        integral_ = T(0);
        prev_error_ = T(0);
        d_filtered_ = T(0);
    }

    constexpr T integral() const { return integral_; }
    constexpr T prev_error() const { return prev_error_; }
    constexpr T d_filtered() const { return d_filtered_; }

private:
    Config cfg_{};
    T integral_{};
    T prev_error_{};
    T d_filtered_{};
};

// ── Convenience aliases ────────────────────────────────────────────────

using PidF = Pid<float>;
using PidD = Pid<double>;

/// Create a PID from gains only (other params use defaults).
constexpr PidF make_pid(float kp, float ki, float kd, float dt,
                        float out_min = -1e30f, float out_max = 1e30f)
{
    PidF::Config cfg{
        .gains = {kp, ki, kd},
        .limits = {out_min, out_max, out_min, out_max},
        .dt = dt,
    };
    return PidF{cfg};
}

} // namespace util
