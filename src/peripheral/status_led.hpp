#pragma once

#include <zephyr/drivers/led_strip.h>
#include <cstdint>

/// WS2812 status LED — indicates system state via color.
/// Blue = initializing, Green = running, Red = error (ring buffer full)
class StatusLed {
public:
    enum class State : uint8_t {
        Init,       // 蓝色 — 初始化中
        Running,    // 绿色 — 正常运行
        Error,      // 红色 — ring buffer 满
    };

    int init();

    /// Set state (only updates LED if state changed).
    void set(State s);

    /// Report ring buffer fullness — if any is full, state = Error.
    /// Call from main loop with current buffer usage.
    void report_ringbuf_status(bool any_full);

    State state() const { return state_; }

private:
    const struct device *strip_ = nullptr;
    State state_ = State::Init;
    struct led_rgb color_{};

    void apply();
    static constexpr led_rgb COLOR_INIT    = {0, 0, 20};    // 蓝
    static constexpr led_rgb COLOR_RUNNING = {0, 20, 0};    // 绿
    static constexpr led_rgb COLOR_ERROR   = {20, 0, 0};    // 红
};
