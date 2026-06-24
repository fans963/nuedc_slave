#include "peripheral/motor.hpp"

Motor::Motor(const struct device* pwm_dev, uint8_t pwm_channel, Qdec& encoder, const Config& cfg)
    : pwm_dev_(pwm_dev)
    , pwm_channel_(pwm_channel)
    , encoder_(encoder)
    , cfg_(cfg)
    , pid_(util::PidF::Config {
          .gains  = { 0, 0, 0 },
          .limits = { cfg.output_min, cfg.output_max, cfg.output_min, cfg.output_max },
          .dt     = 0.001f,
      }) { }

int Motor::init() {
    if (!device_is_ready(pwm_dev_)) return -EIO;

    // Start at zero duty
    set_pwm(0.0f);

    // Setup 1000Hz control loop timer
    k_timer_init(&timer_, timer_callback, nullptr);
    timer_.user_data = this;
    k_timer_start(&timer_, K_USEC(1000), K_USEC(1000));

    return 0;
}

void Motor::set_pid(float kp, float ki, float kd) { pid_.set_gains(kp, ki, kd); }

void Motor::set_limits(float out_min, float out_max, float int_min, float int_max) {
    pid_.set_limits(out_min, out_max, int_min, int_max);
    cfg_.output_min = out_min;
    cfg_.output_max = out_max;
}

void Motor::set_target(float vel_rad_s) {
    target_ = std::clamp(vel_rad_s, cfg_.vel_min, cfg_.vel_max);
}

void Motor::timer_callback(struct k_timer* timer) {
    auto* self = static_cast<Motor*>(timer->user_data);
    self->on_timer();
}

void Motor::on_timer() {
    // PID: target velocity → output
    float vel = encoder_.velocity();
    float out = pid_.compute(target_, vel);

    // Map PID output to PWM duty ratio
    // output range [output_min, output_max] → duty [0, 1]
    // Negative output = reverse direction (for H-bridge: swap pins or use complementary output)
    // Here we clamp to [0, 1] for single-direction, or handle sign for bidirectional
    float range = cfg_.output_max - cfg_.output_min;
    float duty  = (out - cfg_.output_min) / range;
    duty        = std::clamp(duty, 0.0f, 1.0f);

    set_pwm(duty);
}

void Motor::set_pwm(float duty_ratio) {
    uint32_t pulse_ns = static_cast<uint32_t>(duty_ratio * cfg_.pwm_period_ns);
    pwm_set(pwm_dev_, pwm_channel_, cfg_.pwm_period_ns, pulse_ns, PWM_POLARITY_NORMAL);
}
