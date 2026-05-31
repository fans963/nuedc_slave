#pragma once

#include "peripheral/imu.hpp"
#include "protocol/protocol.hpp"
#include "protocol/message_handler.hpp"
#include "protocol/ring_buffer_writer.hpp"
#include "util/byte_ring_buffer.hpp"
#include <array>
#include <cstdint>
#include <cstddef>
#include <span>

class Can;
class Uart;
class Spi;
class I2c;
class IMU;
class Adc;
class Qdec;
class Motor;

/// USB bulk device — bridges host ↔ device peripherals via FlatBuffers protocol.
/// Uplink: collects from all peripheral uplink_writers → USB bulk IN
/// Downlink: USB bulk OUT → protocol decode → dispatch to peripheral downlink_bufs
class BulkDevice {
public:
    BulkDevice();

    int init();
    void try_transmit();

    /// Register peripherals (call before init)
    void set_cans(std::span<Can *const> cans);
    void set_uarts(std::span<Uart *const> uarts);
    void set_spis(std::span<Spi *const> spis);
    void set_i2cs(std::span<I2c *const> i2cs);
    void set_imus(std::span<IMU *const> imus);
    void set_adcs(std::span<Adc *const> adcs);
    void set_qdecs(std::span<Qdec *const> qdecs);
    void set_motors(std::span<Motor *const> motors);

    /// USB callbacks (public for C linkage)
    void on_rx_data(const uint8_t *data, size_t len);
    void on_tx_done();

    /// Protocol frame handler (called by decoder)
    void on_frame(const uint8_t *data, size_t len);

private:
    static constexpr size_t TX_BUF_SIZE = 1024;
    static constexpr size_t MAX_CAN = 2;
    static constexpr size_t MAX_UART = 2;
    static constexpr size_t MAX_SPI = 2;
    static constexpr size_t MAX_I2C = 2;
    static constexpr size_t MAX_IMU = 1;
    static constexpr size_t MAX_ADC = 2;
    static constexpr size_t MAX_QDEC = 2;
    static constexpr size_t MAX_MOTOR = 2;

    protocol::ProtocolDecoder<BulkDevice> decoder_;

    // Peripheral arrays (nullptr = not registered)
    std::array<Can *, MAX_CAN> cans_{};
    std::array<Uart *, MAX_UART> uarts_{};
    std::array<Spi *, MAX_SPI> spis_{};
    std::array<I2c *, MAX_I2C> i2cs_{};
    std::array<IMU *, MAX_IMU> imus_{};
    std::array<Adc *, MAX_ADC> adcs_{};
    std::array<Qdec *, MAX_QDEC> qdecs_{};
    std::array<Motor *, MAX_MOTOR> motors_{};

    // TX double-buffering
    uint8_t tx_buf_a_[TX_BUF_SIZE]{};
    uint8_t tx_buf_b_[TX_BUF_SIZE]{};
    uint8_t *tx_fill_buf_ = tx_buf_a_;
    uint8_t *tx_send_buf_ = tx_buf_b_;
    size_t tx_fill_len_ = 0;
    size_t tx_send_len_ = 0;
    atomic_t tx_busy_ = ATOMIC_INIT(0);

    void swap_tx_buffers();
};
