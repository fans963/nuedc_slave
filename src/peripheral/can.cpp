#include "peripheral/can.hpp"
#include <cstring>

Can::Can(const struct device *dev, uint8_t idx)
    : dev_(dev), idx_(idx) {}

int Can::init()
{
    if (!device_is_ready(dev_)) {
        return -ENODEV;
    }

    // Set bitrate: 1Mbps
    int ret = can_set_bitrate(dev_, 1000000);
    if (ret < 0) return ret;

    // Add RX filter: accept all frames
    struct can_filter filter = {
        .id = 0,
        .mask = 0,
        .flags = 0,
    };
    can_add_rx_filter(dev_, rx_callback, this, &filter);

    // Start CAN controller
    ret = can_start(dev_);
    if (ret < 0) return ret;

    return 0;
}

void Can::rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    auto *self = static_cast<Can *>(user_data);

    bool is_extended = (frame->flags & CAN_FRAME_IDE) != 0;
    bool is_rtr = (frame->flags & CAN_FRAME_RTR) != 0;
    uint32_t can_id = frame->id;
    uint8_t data_len = can_dlc_to_bytes(frame->dlc);

    msg::encode_can_rx(
        self->uplink_writer_,
        self->idx_,
        can_id,
        data_len,
        is_extended,
        is_rtr,
        frame->data,
        data_len);
}

void Can::tx_callback(const struct device *dev, int error, void *user_data)
{
    auto *self = static_cast<Can *>(user_data);
    atomic_set(&self->tx_busy_, 0);
}

bool Can::try_transmit()
{
    if (atomic_get(&tx_busy_)) return false;
    if (downlink_buf_.size() < sizeof(msg::CanTxData)) return false;

    // Read one CAN TX data struct from downlink buffer (zero-copy)
    auto chunk = downlink_buf_.get_claim(sizeof(msg::CanTxData));
    if (chunk.size() < sizeof(msg::CanTxData)) {
        downlink_buf_.get_finish(0);
        return false;
    }

    const auto *tx = reinterpret_cast<const msg::CanTxData *>(chunk.data());

    struct can_frame frame{};
    frame.id = tx->can_id & 0x1FFFFFFF;
    frame.flags = 0;
    if (tx->is_extended) {
        frame.flags |= CAN_FRAME_IDE;
    }
    if (tx->is_rtr) {
        frame.flags |= CAN_FRAME_RTR;
    }
    frame.dlc = can_bytes_to_dlc(tx->can_dlc);
    std::memcpy(frame.data, tx->data, 8);

    downlink_buf_.get_finish(sizeof(msg::CanTxData));

    atomic_set(&tx_busy_, 1);
    int ret = can_send(dev_, &frame, K_NO_WAIT, tx_callback, this);
    if (ret < 0) {
        atomic_set(&tx_busy_, 0);
        return false;
    }
    return true;
}
