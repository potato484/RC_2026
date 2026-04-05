#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "rc26_serial/protocol.hpp"

namespace rc26_serial {

class RingParser {
public:
    static constexpr size_t CAPACITY = 4096;

    using DeliverFn = std::function<void(uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen)>;

    struct Stats {
        uint32_t frames_ok = 0;
        uint32_t len_invalid = 0;
        uint32_t tail_bad = 0;
        uint32_t crc_bad = 0;
        uint32_t head_drop = 0;
        uint32_t overflow_drop = 0;
    };

    void push(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            if (size_ >= CAPACITY) {
                drop(1);
                ++stats_.overflow_drop;
            }
            const size_t tail = (head_ + size_) % CAPACITY;
            buf_[tail] = data[i];
            ++size_;
        }
    }

    void parse(const DeliverFn& deliver) {
        constexpr size_t kMinFrameSize = 12;
        constexpr size_t kFrameOverhead = 11;

        while (size() >= kMinFrameSize) {
            if (at(0) != rc26_decision::FRAME_HEAD_0 || at(1) != rc26_decision::FRAME_HEAD_1) {
                drop(1);
                ++stats_.head_drop;
                continue;
            }

            const uint8_t len = at(3);
            if (len < 1U || len > (rc26_decision::MAX_PAYLOAD_SIZE + 1U)) {
                drop(1);
                ++stats_.len_invalid;
                continue;
            }

            const size_t frame_size = static_cast<size_t>(len) + kFrameOverhead;
            if (frame_size > size()) {
                return;
            }

            if (at(frame_size - 2) != rc26_decision::FRAME_TAIL_0 ||
                at(frame_size - 1) != rc26_decision::FRAME_TAIL_1) {
                drop(1);
                ++stats_.tail_bad;
                continue;
            }

            constexpr size_t kCrcInputMax = 1 + 1 + 1 + (rc26_decision::MAX_PAYLOAD_SIZE + 1);
            std::array<uint8_t, kCrcInputMax> crc_input{};
            const size_t crc_len = 1 + 1 + 1 + static_cast<size_t>(len);
            for (size_t i = 0; i < crc_len; ++i) {
                crc_input[i] = at(2 + i);
            }

            const uint32_t expected_crc = rc26_decision::crc32_mpeg2_calculate(crc_input.data(), crc_len);
            const uint32_t actual_crc = static_cast<uint32_t>(at(frame_size - 6)) |
                                        (static_cast<uint32_t>(at(frame_size - 5)) << 8) |
                                        (static_cast<uint32_t>(at(frame_size - 4)) << 16) |
                                        (static_cast<uint32_t>(at(frame_size - 3)) << 24);

            if (expected_crc != actual_crc) {
                drop(1);
                ++stats_.crc_bad;
                continue;
            }

            const uint8_t seq = at(2);
            const uint8_t cmd = at(5);
            const size_t payload_len = static_cast<size_t>(len - 1U);
            std::array<uint8_t, rc26_decision::MAX_PAYLOAD_SIZE> payload{};
            for (size_t i = 0; i < payload_len; ++i) {
                payload[i] = at(6 + i);
            }

            deliver(seq, cmd, payload.data(), payload_len);
            drop(frame_size);
            ++stats_.frames_ok;
        }
    }

    void reset() {
        head_ = 0;
        size_ = 0;
        stats_ = {};
    }

    Stats consumeStats() {
        const Stats delta = stats_;
        stats_ = {};
        return delta;
    }

private:
    std::array<uint8_t, CAPACITY> buf_{};
    size_t head_{0};
    size_t size_{0};
    Stats stats_{};

    size_t size() const {
        return size_;
    }

    uint8_t at(size_t i) const {
        return buf_[(head_ + i) % CAPACITY];
    }

    void drop(size_t n) {
        const size_t consumed = (n > size_) ? size_ : n;
        head_ = (head_ + consumed) % CAPACITY;
        size_ -= consumed;
    }
};

}  // namespace rc26_serial
