#include "peripheral/imu.hpp"
#include "protocol/message_handler.hpp"
#include <zephyr/drivers/sensor.h>

IMU* IMU::instance_ = nullptr;

IMU::IMU(const struct device* accel_dev, const struct device* gyro_dev)
    : accel_dev_(accel_dev)
    , gyro_dev_(gyro_dev) { }

int IMU::init() {
    if (!device_is_ready(accel_dev_)) return -ENODEV;
    if (!device_is_ready(gyro_dev_)) return -ENODEV;

    instance_ = this;

    accel_trig_.type = SENSOR_TRIG_DATA_READY;
    accel_trig_.chan = SENSOR_CHAN_ACCEL_XYZ;

    int ret = sensor_trigger_set(accel_dev_, &accel_trig_, accel_trigger_handler);
    if (ret < 0) return ret;

    return 0;
}

void IMU::accel_trigger_handler(const struct device* dev, const struct sensor_trigger* trig) {
    ARG_UNUSED(dev);
    ARG_UNUSED(trig);
    if (instance_) instance_->on_data_ready();
}

void IMU::on_data_ready() {
    // Fetch accel
    if (sensor_sample_fetch(accel_dev_) < 0) return;
    struct sensor_value accel[3];
    if (sensor_channel_get(accel_dev_, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) return;

    // Fetch gyro
    if (sensor_sample_fetch(gyro_dev_) < 0) return;
    struct sensor_value gyro[3];
    if (sensor_channel_get(gyro_dev_, SENSOR_CHAN_GYRO_XYZ, gyro) < 0) return;

    msg::encode_imu(uplink_writer_, sensor_value_to_float(&accel[0]),
        sensor_value_to_float(&accel[1]), sensor_value_to_float(&accel[2]),
        sensor_value_to_float(&gyro[0]), sensor_value_to_float(&gyro[1]),
        sensor_value_to_float(&gyro[2]));
}
