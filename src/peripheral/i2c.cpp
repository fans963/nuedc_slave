#include "peripheral/i2c.hpp"

I2c::I2c(const struct device *dev, uint8_t idx)
    : dev_(dev), idx_(idx) {}

int I2c::init()
{
    if (!device_is_ready(dev_)) return -ENODEV;
    return 0;
}

int I2c::write(uint16_t addr, const uint8_t *data, size_t len)
{
    return i2c_write(dev_, data, len, addr);
}

int I2c::read(uint16_t addr, uint8_t *data, size_t len)
{
    return i2c_read(dev_, data, len, addr);
}

int I2c::write_read(uint16_t addr, const uint8_t *tx, size_t tx_len,
                    uint8_t *rx, size_t rx_len)
{
    return i2c_write_read(dev_, addr, tx, tx_len, rx, rx_len);
}

int I2c::reg_read(uint16_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_burst_read(dev_, addr, reg, data, len);
}

int I2c::reg_write(uint16_t addr, uint8_t reg, uint8_t value)
{
    return i2c_reg_write_byte(dev_, addr, reg, value);
}
