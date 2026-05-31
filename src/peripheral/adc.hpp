#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include "protocol/ring_buffer_writer.hpp"
#include "protocol/message_handler.hpp"
#include <cstdint>

class Adc {
public:
    Adc(const struct device *dev, uint8_t idx);

    int init();

    /// Call from main loop: read all configured channels and encode.
    void poll();

    protocol::RingBufferWriter<256> &uplink_writer() { return uplink_writer_; }

    uint8_t idx() const { return idx_; }

private:
    const struct device *dev_;
    uint8_t idx_;
    struct adc_sequence seq_{};

    static constexpr uint8_t NUM_CHANNELS = 4;
    uint16_t samples_[NUM_CHANNELS]{};

    protocol::RingBufferWriter<256> uplink_writer_;

    int64_t last_poll_ms_ = 0;
    static constexpr int64_t POLL_INTERVAL_MS = 10;
};
