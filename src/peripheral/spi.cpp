#include "peripheral/spi.hpp"

Spi::Spi(const struct device* dev, uint8_t idx)
    : dev_(dev)
    , idx_(idx) { }

int Spi::init() {
    if (!device_is_ready(dev_)) return -ENODEV;

    cfg_.frequency = 10000000U; // 10MHz default
    cfg_.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;
    cfg_.slave     = 0;

    return 0;
}

int Spi::transfer(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
    struct spi_buf tx_buf     = { .buf = const_cast<uint8_t*>(tx), .len = tx_len };
    struct spi_buf rx_buf     = { .buf = rx, .len = rx_len };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    return spi_transceive(dev_, &cfg_, &tx_set, &rx_set);
}

int Spi::write(const uint8_t* data, size_t len) {
    struct spi_buf buf     = { .buf = const_cast<uint8_t*>(data), .len = len };
    struct spi_buf_set set = { .buffers = &buf, .count = 1 };
    return spi_write(dev_, &cfg_, &set);
}

int Spi::read(uint8_t* data, size_t len) {
    struct spi_buf buf     = { .buf = data, .len = len };
    struct spi_buf_set set = { .buffers = &buf, .count = 1 };
    return spi_read(dev_, &cfg_, &set);
}
