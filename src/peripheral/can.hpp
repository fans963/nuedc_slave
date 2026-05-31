#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include "protocol/ring_buffer_writer.hpp"
#include "protocol/message_handler.hpp"
#include "util/byte_ring_buffer.hpp"
#include <cstdint>

class Can {
public:
    Can(const struct device *dev, uint8_t idx);

    int init();

    protocol::RingBufferWriter<1024> &uplink_writer() { return uplink_writer_; }
    util::ByteRingBuffer<512> &downlink_buf() { return downlink_buf_; }

    /// Call from main loop: drain downlink buffer → CAN TX.
    bool try_transmit();

    uint8_t idx() const { return idx_; }

private:
    const struct device *dev_;
    uint8_t idx_;

    protocol::RingBufferWriter<1024> uplink_writer_;
    util::ByteRingBuffer<512> downlink_buf_;

    static void rx_callback(const struct device *dev, struct can_frame *frame, void *user_data);
    static void tx_callback(const struct device *dev, int error, void *user_data);
    atomic_t tx_busy_ = ATOMIC_INIT(0);
};
