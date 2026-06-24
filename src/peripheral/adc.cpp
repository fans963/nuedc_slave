#include "peripheral/adc.hpp"

Adc::Adc(const struct device* dev, uint8_t idx)
    : dev_(dev)
    , idx_(idx) { }

int Adc::init() {
    if (!device_is_ready(dev_)) {
        return -ENODEV;
    }

    seq_.channels    = BIT(0) | BIT(1) | BIT(2) | BIT(3);
    seq_.resolution  = 12;
    seq_.buffer      = samples_;
    seq_.buffer_size = sizeof(samples_);

    return 0;
}

void Adc::poll() {
    int64_t now = k_uptime_get();
    if (now - last_poll_ms_ < POLL_INTERVAL_MS) return;
    last_poll_ms_ = now;

    if (adc_read(dev_, &seq_) < 0) return;

    msg::encode_adc(uplink_writer_, samples_, NUM_CHANNELS);
}
