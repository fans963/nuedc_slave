#pragma once

#include "protocol/ring_buffer_writer.hpp"
#include <cstdint>
#include <cstring>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/// Generic raw SPI wrapper for arbitrary SPI devices.
/// Provides blocking read/write with CS management via DT.
class Spi {
public:
  Spi(const struct device *dev, uint8_t idx);

  int init();

  /// Blocking SPI transfer (write then read).
  int transfer(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

  /// Blocking write only.
  int write(const uint8_t *data, size_t len);

  /// Blocking read only.
  int read(uint8_t *data, size_t len);

  protocol::RingBufferWriter<256> &uplink_writer() { return uplink_writer_; }

  const struct device *dev() const { return dev_; }
  uint8_t idx() const { return idx_; }

private:
  const struct device *dev_;
  uint8_t idx_;
  struct spi_config cfg_{};

  protocol::RingBufferWriter<256> uplink_writer_;
};
