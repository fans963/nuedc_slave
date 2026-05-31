#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "protocol/ring_buffer_writer.hpp"
#include <cstdint>

/// Quadrature encoder — 1000Hz timer ISR-driven sampling.
/// Uses raw encoder counts, converts to rad/s via lines_per_rev from host.
class Qdec {
public:
    static constexpr int SAMPLE_PERIOD_US = 1000;

    Qdec(const struct device *dev, uint8_t idx);

    int init();

    /// Set encoder lines per revolution (from host EncoderConfigPack).
    /// Actual counts per rev = lines_per_rev * 4 (x4 mode).
    void set_lines_per_rev(uint16_t lines) {
        lines_per_rev_ = lines;
        counts_per_rev_ = static_cast<uint32_t>(lines) * 4;
        configured_ = true;
    }

    float velocity() const { return velocity_; }

    protocol::RingBufferWriter<256> &uplink_writer() { return uplink_writer_; }

    uint8_t idx() const { return idx_; }

private:
    const struct device *dev_;
    uint8_t idx_;

    uint16_t lines_per_rev_ = 0;
    uint32_t counts_per_rev_ = 0;
    int32_t last_count_ = 0;
    float velocity_ = 0.0f;
    volatile bool configured_ = false;

    protocol::RingBufferWriter<256> uplink_writer_;

    struct k_timer timer_;

    static void timer_callback(struct k_timer *timer);
    void on_timer();
};
