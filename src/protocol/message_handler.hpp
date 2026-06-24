#pragma once

#include "protocol/protocol.hpp"
#include <flatbuffers/flatbuffers.h>
#include <host_to_slave_generated.h>
#include <slave_to_host_generated.h>

#include <cstdint>
#include <cstring>
#include <array>

namespace msg {

// ── Static allocator for FlatBufferBuilder (no malloc) ──────────────────
template <size_t N>
class StaticAllocator : public flatbuffers::Allocator {
public:
    uint8_t *allocate(size_t size) override {
        if (size > N) return nullptr;
        return buf_.data();
    }
    void deallocate(uint8_t *p, size_t) override {
        // static buffer, nothing to free
    }
    uint8_t *reallocate_downward(uint8_t *old_p, size_t old_size,
                                  size_t new_size, size_t in_use_back,
                                  size_t in_use_front) override {
        if (new_size > N) return nullptr;
        // buffer is static and large enough, just return it
        // copy existing data to the end (downward growth)
        uint8_t *new_p = buf_.data();
        if (old_p && old_p != new_p) {
            std::memmove(new_p + new_size - in_use_back,
                        old_p + old_size - in_use_back, in_use_back);
            std::memmove(new_p, old_p, in_use_front);
        }
        return new_p;
    }

private:
    std::array<uint8_t, N> buf_{};
};

// ── CAN TX data layout (fixed-size, written into ByteRingBuffer) ──────
struct CanTxData {
    uint8_t can_idx;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t is_extended;
    uint8_t is_rtr;
    uint8_t data[8];
};

// ── Encode helpers ────────────────────────────────────────────────────
// Each function uses a static FlatBufferBuilder + StaticAllocator
// to avoid any malloc. Each builder is only called from one context
// (one ISR or main loop), so no reentrancy issue.

template <typename W>
void encode_can_rx(W& writer, uint8_t can_idx, uint32_t can_id, uint8_t dlc, bool is_extended,
    bool is_rtr, const uint8_t* data, size_t data_len) {
    static StaticAllocator<256> alloc;
    static flatbuffers::FlatBufferBuilder fbb(256, &alloc);
    fbb.Clear();
    auto rx_data  = fbb.CreateVector(data, data_len);
    auto can_pack = Protocol::SlaveToHost::CreateCanPack(
        fbb, can_idx, can_id, dlc, is_extended, is_rtr, rx_data);
    auto frame = Protocol::SlaveToHost::CreateSlaveToHostFrame(
        fbb, Protocol::SlaveToHost::MsgPayload::CanPack, can_pack.Union());
    fbb.FinishSizePrefixed(frame);

    protocol::protocol_encode(writer, fbb.GetBufferPointer(), fbb.GetSize());
}

template <typename W>
void encode_imu(W& writer, float ax, float ay, float az, float gx, float gy, float gz) {
    static StaticAllocator<256> alloc;
    static flatbuffers::FlatBufferBuilder fbb(256, &alloc);
    fbb.Clear();
    auto frame = Protocol::SlaveToHost::CreateSlaveToHostFrame(fbb,
        Protocol::SlaveToHost::MsgPayload::ImuPack,
        Protocol::SlaveToHost::CreateImuPack(fbb, ax, ay, az, gx, gy, gz).Union());
    fbb.FinishSizePrefixed(frame);

    protocol::protocol_encode(writer, fbb.GetBufferPointer(), fbb.GetSize());
}

template <typename W>
void encode_uart_rx(W& writer, uint8_t uart_idx, const uint8_t* data, size_t len) {
    static StaticAllocator<256> alloc;
    static flatbuffers::FlatBufferBuilder fbb(256, &alloc);
    fbb.Clear();
    auto rx_data   = fbb.CreateVector(data, len);
    auto uart_pack = Protocol::SlaveToHost::CreateUartPack(fbb, uart_idx, rx_data);
    auto frame     = Protocol::SlaveToHost::CreateSlaveToHostFrame(
        fbb, Protocol::SlaveToHost::MsgPayload::UartPack, uart_pack.Union());
    fbb.FinishSizePrefixed(frame);

    protocol::protocol_encode(writer, fbb.GetBufferPointer(), fbb.GetSize());
}

template <typename W>
void encode_encoder(W& writer, uint8_t encoder_id, float velocity_rad_s) {
    static StaticAllocator<128> alloc;
    static flatbuffers::FlatBufferBuilder fbb(128, &alloc);
    fbb.Clear();
    auto frame = Protocol::SlaveToHost::CreateSlaveToHostFrame(fbb,
        Protocol::SlaveToHost::MsgPayload::EncoderPack,
        Protocol::SlaveToHost::CreateEncoderPack(fbb, encoder_id, velocity_rad_s).Union());
    fbb.FinishSizePrefixed(frame);

    protocol::protocol_encode(writer, fbb.GetBufferPointer(), fbb.GetSize());
}

template <typename W>
void encode_adc(W& writer, const uint16_t* values, size_t count) {
    static StaticAllocator<256> alloc;
    static flatbuffers::FlatBufferBuilder fbb(256, &alloc);
    fbb.Clear();
    auto data     = fbb.CreateVector(values, count);
    auto adc_pack = Protocol::SlaveToHost::CreateAdcPack(fbb, 0, data);
    auto frame    = Protocol::SlaveToHost::CreateSlaveToHostFrame(
        fbb, Protocol::SlaveToHost::MsgPayload::AdcPack, adc_pack.Union());
    fbb.FinishSizePrefixed(frame);

    protocol::protocol_encode(writer, fbb.GetBufferPointer(), fbb.GetSize());
}

// ── Decode + dispatch (called from USB bulk-out callback) ─────────────

template <typename CanDownlink, typename UartDownlink, typename EncoderConfig,
    typename MotorCommand, typename PidConfig>
void dispatch_downlink(const uint8_t* flatbuf, size_t len, CanDownlink& can0_dl,
    CanDownlink& can1_dl, UartDownlink& uart0_dl, UartDownlink& uart1_dl,
    EncoderConfig&& on_encoder_config, MotorCommand&& on_motor_cmd, PidConfig&& on_pid_config) {
    auto verifier = flatbuffers::Verifier(flatbuf, len);
    if (!verifier.VerifySizePrefixedBuffer<Protocol::HostToSlave::HostToSlaveFrame>(nullptr))
        return;

    auto frame = flatbuffers::GetSizePrefixedRoot<Protocol::HostToSlave::HostToSlaveFrame>(flatbuf);
    if (!frame || !frame->payload()) return;

    switch (frame->payload_type()) {
    case Protocol::HostToSlave::MsgPayload::CanPack: {
        auto can = frame->payload_as_CanPack();
        if (!can) break;
        CanTxData tx { };
        tx.can_idx     = can->can_idx();
        tx.can_id      = can->can_id();
        tx.can_dlc     = can->can_dlc();
        tx.is_extended = can->is_extended() ? 1 : 0;
        tx.is_rtr      = can->is_rtr() ? 1 : 0;
        auto data_vec  = can->tx_data();
        if (data_vec) {
            size_t n = data_vec->size();
            if (n > 8) n = 8;
            std::memcpy(tx.data, data_vec->data(), n);
        }
        auto& dl = (can->can_idx() == 0) ? can0_dl : can1_dl;
        dl.write(reinterpret_cast<const uint8_t*>(&tx), sizeof(tx));
        break;
    }
    case Protocol::HostToSlave::MsgPayload::UartPack: {
        auto uart = frame->payload_as_UartPack();
        if (!uart) break;
        auto data_vec = uart->tx_data();
        if (!data_vec) break;
        auto& dl = (uart->uart_idx() == 0) ? uart0_dl : uart1_dl;
        dl.write(data_vec->data(), data_vec->size());
        break;
    }
    case Protocol::HostToSlave::MsgPayload::MotorCommandPack: {
        auto motor = frame->payload_as_MotorCommandPack();
        if (!motor) break;
        on_motor_cmd(motor->motor_id(), motor->target_speed_rad_s());
        break;
    }
    case Protocol::HostToSlave::MsgPayload::PidConfigPack: {
        auto pid = frame->payload_as_PidConfigPack();
        if (!pid) break;
        on_pid_config(pid->controller_id(), pid->kp(), pid->ki(), pid->kd(), pid->output_min(),
            pid->output_max(), pid->integral_min(), pid->integral_max(), pid->reset());
        break;
    }
    case Protocol::HostToSlave::MsgPayload::EncoderConfigPack: {
        auto enc = frame->payload_as_EncoderConfigPack();
        if (!enc) break;
        on_encoder_config(enc->encoder_id(), enc->lines_per_rev());
        break;
    }
    default:
        break;
    }
}

} // namespace msg
