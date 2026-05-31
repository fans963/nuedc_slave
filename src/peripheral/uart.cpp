#include "peripheral/uart.hpp"

Uart::Uart(const struct device *dev, uint8_t idx)
    : dev_(dev), idx_(idx) {}

int Uart::init()
{
    if (!device_is_ready(dev_)) {
        return -ENODEV;
    }

    // Register async callback
    int ret = uart_callback_set(dev_, uart_callback, this);
    if (ret < 0) return ret;

    // Start RX with the staging buffer as receive buffer
    return uart_rx_enable(dev_, rx_staging_, RX_STAGING_SIZE, 1000 /* 1ms timeout */);
}

void Uart::uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    auto *self = static_cast<Uart *>(user_data);

    switch (evt->type) {
    case UART_RX_RDY:
        // New data in rx buffer at offset+buf
        self->on_rx(evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
        break;

    case UART_RX_DISABLED:
        // RX buffer full or timeout — re-enable
        uart_rx_enable(dev, self->rx_staging_, RX_STAGING_SIZE, 1000);
        break;

    case UART_TX_DONE:
        atomic_set(&self->tx_busy_, 0);
        break;

    default:
        break;
    }
}

void Uart::on_rx(const uint8_t *data, size_t len)
{
    // Encode received bytes into uplink FlatBuffer
    msg::encode_uart_rx(uplink_writer_, idx_, data, len);
}

bool Uart::try_transmit()
{
    if (atomic_get(&tx_busy_)) return false;
    if (downlink_buf_.empty()) return false;

    // Drain downlink buffer into tx buffer
    size_t total = 0;
    downlink_buf_.drain([this, &total](std::span<const uint8_t> chunk) {
        size_t n = std::min(chunk.size(), TX_BUF_SIZE - total);
        std::memcpy(tx_buf_ + total, chunk.data(), n);
        total += n;
    }, TX_BUF_SIZE);

    if (total == 0) return false;

    atomic_set(&tx_busy_, 1);
    int ret = uart_tx(dev_, tx_buf_, total, 100 /* 100µs timeout */);
    if (ret < 0) {
        atomic_set(&tx_busy_, 0);
        return false;
    }
    return true;
}
