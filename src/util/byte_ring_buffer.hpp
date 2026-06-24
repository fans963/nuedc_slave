#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <zephyr/sys/ring_buffer.h>

namespace util {

/// Modern C++ wrapper around Zephyr's ring_buf.
/// Zero-copy claim/finish API + convenience write/read.
/// N: capacity in bytes (must be power of two).
template <size_t N>
    requires(N >= 2 && std::has_single_bit(N))
class ByteRingBuffer {
public:
    ByteRingBuffer() { ring_buf_init(&rb_, N, buf_); }

    // ── Capacity ────────────────────────────────────────────────────────

    size_t size() const { return ring_buf_size_get(&rb_); }
    size_t capacity() const { return N; }
    bool empty() const { return size() == 0; }

    // ── Zero-copy write (producer side) ─────────────────────────────────

    /// Claim up to `len` bytes for writing. Returns a writable span.
    /// Call put_finish() after filling the data.
    [[nodiscard]] std::span<uint8_t> put_claim(size_t len) {
        uint8_t* ptr = nullptr;
        auto granted = ring_buf_put_claim(&rb_, &ptr, len);
        return { ptr, granted };
    }

    /// Confirm `count` bytes from the last put_claim().
    void put_finish(size_t count) { ring_buf_put_finish(&rb_, count); }

    /// Convenience: copy `data` into the ring buffer.
    size_t write(const uint8_t* data, size_t len) { return ring_buf_put(&rb_, data, len); }

    // ── Zero-copy read (consumer side) ──────────────────────────────────

    /// Claim up to `len` readable bytes. Returns a readable span.
    /// Call get_finish() after consuming the data.
    [[nodiscard]] std::span<const uint8_t> get_claim(size_t len) {
        uint8_t* ptr = nullptr;
        auto granted = ring_buf_get_claim(&rb_, &ptr, len);
        return { ptr, granted };
    }

    /// Confirm `count` bytes from the last get_claim().
    void get_finish(size_t count) { ring_buf_get_finish(&rb_, count); }

    /// Convenience: copy data out of the ring buffer.
    size_t read(uint8_t* dst, size_t len) { return ring_buf_get(&rb_, dst, len); }

    // ── Drain (zero-copy callback) ──────────────────────────────────────

    /// Consume readable bytes through callback(span<const uint8_t>).
    /// Returns total bytes consumed.
    template <typename F>
    size_t drain(F&& callback, size_t max = N) {
        size_t total = 0;
        while (total < max) {
            auto chunk = get_claim(max - total);
            if (chunk.empty()) break;
            callback(chunk);
            get_finish(chunk.size());
            total += chunk.size();
        }
        return total;
    }

private:
    struct ring_buf rb_ { };
    uint8_t buf_[N] { };
};

} // namespace util
