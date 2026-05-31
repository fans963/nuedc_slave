#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "protocol/ring_buffer_writer.hpp"
#include "protocol/message_handler.hpp"
#include <cstdint>

/// BMI088 via Zephyr sensor API (DT-driven).
class IMU {
public:
    IMU(const struct device *accel_dev, const struct device *gyro_dev);

    int init();

    /// Call from main loop: fetch + encode IMU data.
    void poll();

    protocol::RingBufferWriter<512> &uplink_writer() { return uplink_writer_; }

private:
    const struct device *accel_dev_;
    const struct device *gyro_dev_;

    protocol::RingBufferWriter<512> uplink_writer_;

    int64_t last_poll_ms_ = 0;
    static constexpr int64_t POLL_INTERVAL_MS = 5;
};
