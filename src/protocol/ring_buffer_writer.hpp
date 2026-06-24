#pragma once

#include "util/byte_ring_buffer.hpp"

namespace protocol {

/// Adapts ByteRingBuffer to satisfy the FrameWriter concept.
/// protocol_encode() writes the complete frame into the ring buffer.
/// A separate drain call (from main loop or TX-complete ISR) pushes bytes to hardware.
template <size_t N>
class RingBufferWriter {
public:
    void write(const uint8_t* data, size_t len) { buf_.write(data, len); }

    /// Zero-copy write: claim space, fill, finish.
    [[nodiscard]] auto put_claim(size_t len) { return buf_.put_claim(len); }
    void put_finish(size_t count) { buf_.put_finish(count); }

    /// Push readable bytes to hardware via callback(span<const uint8_t>).
    template <typename F>
    size_t drain(F&& hw_write) {
        return buf_.drain(std::forward<F>(hw_write));
    }

    size_t readable() const { return buf_.size(); }

private:
    util::ByteRingBuffer<N> buf_;
};

} // namespace protocol
