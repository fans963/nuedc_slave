#pragma once

#include "protocol/ring_buffer_writer.hpp"
#include <zephyr/drivers/sensor.h>

/// BMI088 via Zephyr sensor API — interrupt-driven (data-ready trigger).
class IMU {
public:
    IMU(const struct device* accel_dev, const struct device* gyro_dev);

    int init();

    protocol::RingBufferWriter<512>& uplink_writer() { return uplink_writer_; }

private:
    static void accel_trigger_handler(const struct device* dev, const struct sensor_trigger* trig);

    void on_data_ready();

    const struct device* accel_dev_;
    const struct device* gyro_dev_;

    protocol::RingBufferWriter<512> uplink_writer_;

    struct sensor_trigger accel_trig_;

    // 全局实例指针（用于静态回调）
    static IMU* instance_;
};
