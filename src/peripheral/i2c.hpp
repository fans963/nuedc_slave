#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "protocol/ring_buffer_writer.hpp"
#include <cstdint>

/// Generic I2C wrapper with zero-copy read/write via DT.
class I2c {
public:
    I2c(const struct device *dev, uint8_t idx);

    int init();

    /// Write data to device at address `addr`.
    int write(uint16_t addr, const uint8_t *data, size_t len);

    /// Read data from device at address `addr`.
    int read(uint16_t addr, uint8_t *data, size_t len);

    /// Write then read (no stop between).
    int write_read(uint16_t addr, const uint8_t *tx, size_t tx_len,
                   uint8_t *rx, size_t rx_len);

    /// Write a register, then read back.
    int reg_read(uint16_t addr, uint8_t reg, uint8_t *data, size_t len);

    /// Write a register value.
    int reg_write(uint16_t addr, uint8_t reg, uint8_t value);

    protocol::RingBufferWriter<256> &uplink_writer() { return uplink_writer_; }

    const struct device *dev() const { return dev_; }
    uint8_t idx() const { return idx_; }

private:
    const struct device *dev_;
    uint8_t idx_;
    protocol::RingBufferWriter<256> uplink_writer_;
};
