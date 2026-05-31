#include "peripheral/imu.hpp"
#include <zephyr/drivers/sensor.h>

IMU::IMU(const struct device *accel_dev, const struct device *gyro_dev)
    : accel_dev_(accel_dev), gyro_dev_(gyro_dev) {}

int IMU::init()
{
    if (!device_is_ready(accel_dev_)) return -ENODEV;
    if (!device_is_ready(gyro_dev_)) return -ENODEV;
    return 0;
}

void IMU::poll()
{
    int64_t now = k_uptime_get();
    if (now - last_poll_ms_ < POLL_INTERVAL_MS) return;
    last_poll_ms_ = now;

    // Fetch accel
    if (sensor_sample_fetch(accel_dev_) < 0) return;
    struct sensor_value accel[3];
    if (sensor_channel_get(accel_dev_, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) return;

    // Fetch gyro
    if (sensor_sample_fetch(gyro_dev_) < 0) return;
    struct sensor_value gyro[3];
    if (sensor_channel_get(gyro_dev_, SENSOR_CHAN_GYRO_XYZ, gyro) < 0) return;

    msg::encode_imu(uplink_writer_,
                    sensor_value_to_double(&accel[0]),
                    sensor_value_to_double(&accel[1]),
                    sensor_value_to_double(&accel[2]),
                    sensor_value_to_double(&gyro[0]),
                    sensor_value_to_double(&gyro[1]),
                    sensor_value_to_double(&gyro[2]));
}
