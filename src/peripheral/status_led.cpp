#include "peripheral/status_led.hpp"

int StatusLed::init() {
    strip_ = DEVICE_DT_GET(DT_CHOSEN(zephyr_led_strip));
    if (!device_is_ready(strip_)) return -ENODEV;

    state_ = State::Init;
    apply();
    return 0;
}

void StatusLed::set(State s) {
    if (s == state_) return;
    state_ = s;
    apply();
}

void StatusLed::report_ringbuf_status(bool any_full) {
    if (any_full && state_ != State::Error) {
        set(State::Error);
    } else if (!any_full && state_ == State::Error) {
        set(State::Running);
    }
}

void StatusLed::apply() {
    switch (state_) {
    case State::Init:
        color_ = COLOR_INIT;
        break;
    case State::Running:
        color_ = COLOR_RUNNING;
        break;
    case State::Error:
        color_ = COLOR_ERROR;
        break;
    }
    led_strip_update_rgb(strip_, &color_, 1);
}
