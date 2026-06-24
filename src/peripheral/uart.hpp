#pragma once

#include "protocol/message_handler.hpp"
#include "protocol/ring_buffer_writer.hpp"
#include "util/byte_ring_buffer.hpp"
#include <cstddef>
#include <cstdint>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

/// Zephyr-based UART peripheral.
/// TX: data from downlink ring buffer → uart_tx()
/// RX: ISR callback → staging buffer → FlatBuffer encode → uplink ring buffer
class Uart {
public:
    Uart(const struct device* dev, uint8_t idx);

    int init();

    protocol::RingBufferWriter<512>& uplink_writer() { return uplink_writer_; }
    util::ByteRingBuffer<256>& downlink_buf() { return downlink_buf_; }

    /// Call from main loop: drain downlink buffer to hardware TX.
    bool try_transmit();

    uint8_t idx() const { return idx_; }

private:
    const struct device* dev_;
    uint8_t idx_;

    protocol::RingBufferWriter<512> uplink_writer_;
    util::ByteRingBuffer<256> downlink_buf_;

    // ISR RX staging
    static constexpr size_t RX_STAGING_SIZE = 128;
    uint8_t rx_staging_[RX_STAGING_SIZE] { };

    // TX double-buffering
    static constexpr size_t TX_BUF_SIZE = 256;
    uint8_t tx_buf_[TX_BUF_SIZE] { };
    atomic_t tx_busy_ = ATOMIC_INIT(0);

    static void uart_callback(const struct device* dev, struct uart_event* evt, void* user_data);
    void on_rx(const uint8_t* data, size_t len);
};
