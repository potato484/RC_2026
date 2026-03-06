#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "rc26_serial/protocol.hpp"
#include "rc26_serial/ring_parser.hpp"

namespace {

std::vector<uint8_t> buildFrame(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload, uint8_t retry = 0x00) {
    const uint8_t len = static_cast<uint8_t>(1U + payload.size());
    const size_t frame_size = 2 + 1 + 1 + 1 + 1 + payload.size() + 4 + 2;
    std::vector<uint8_t> frame(frame_size);

    size_t idx = 0;
    frame[idx++] = rc26_decision::FRAME_HEAD_0;
    frame[idx++] = rc26_decision::FRAME_HEAD_1;
    frame[idx++] = seq;
    frame[idx++] = len;
    frame[idx++] = retry;
    frame[idx++] = cmd;
    for (uint8_t b : payload) {
        frame[idx++] = b;
    }
    const uint32_t crc = rc26_decision::crc32_mpeg2_calculate(&frame[2], 1 + 1 + 1 + static_cast<size_t>(len));
    frame[idx++] = static_cast<uint8_t>(crc & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc >> 24) & 0xFF);
    frame[idx++] = rc26_decision::FRAME_TAIL_0;
    frame[idx++] = rc26_decision::FRAME_TAIL_1;
    return frame;
}

struct DeliveredFrame {
    uint8_t seq = 0;
    uint8_t cmd = 0;
    std::vector<uint8_t> payload;
};

}  // namespace

TEST(RingParser, DeliversSingleFrameAndFragmentedFrames) {
    rc26_serial::RingParser parser;
    std::vector<DeliveredFrame> delivered;

    const auto frame1 = buildFrame(1, 0x21, {0x11, 0x22});
    const auto frame2 = buildFrame(2, 0x22, {0x33});
    const auto frame3 = buildFrame(3, 0x23, {});

    parser.push(frame1.data(), frame1.size() / 2U);
    parser.parse([&](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
        delivered.push_back({seq, cmd, std::vector<uint8_t>(payload, payload + plen)});
    });
    EXPECT_TRUE(delivered.empty());

    parser.push(frame1.data() + frame1.size() / 2U, frame1.size() - frame1.size() / 2U);
    parser.parse([&](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
        delivered.push_back({seq, cmd, std::vector<uint8_t>(payload, payload + plen)});
    });
    ASSERT_EQ(delivered.size(), 1U);
    EXPECT_EQ(delivered[0].seq, 1U);
    EXPECT_EQ(delivered[0].cmd, 0x21U);
    EXPECT_EQ(delivered[0].payload, (std::vector<uint8_t>{0x11, 0x22}));

    std::vector<uint8_t> multi = frame2;
    multi.insert(multi.end(), frame3.begin(), frame3.end());
    parser.push(multi.data(), multi.size());
    parser.parse([&](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
        delivered.push_back({seq, cmd, std::vector<uint8_t>(payload, payload + plen)});
    });
    ASSERT_EQ(delivered.size(), 3U);
    EXPECT_EQ(delivered[1].seq, 2U);
    EXPECT_EQ(delivered[2].seq, 3U);

    const auto stats = parser.consumeStats();
    EXPECT_EQ(stats.frames_ok, 3U);
    EXPECT_EQ(stats.len_invalid, 0U);
    EXPECT_EQ(stats.tail_bad, 0U);
    EXPECT_EQ(stats.crc_bad, 0U);
}

TEST(RingParser, RecoversFromNoiseAndInvalidFramesWithDropOne) {
    rc26_serial::RingParser parser;
    std::vector<DeliveredFrame> delivered;

    std::vector<uint8_t> invalid_len = {
        rc26_decision::FRAME_HEAD_0, rc26_decision::FRAME_HEAD_1, 0x10, 0xF0, 0x00, 0x55,
        0xAA,                         0xBB,                         0xCC, 0xDD, 0xEE, 0xFF};

    auto bad_tail = buildFrame(4, 0x44, {0x01, 0x02, 0x03});
    bad_tail[bad_tail.size() - 2] = 0x00;

    auto bad_crc = buildFrame(5, 0x45, {0x04, 0x05});
    bad_crc[bad_crc.size() - 6] ^= 0xFF;

    const auto good = buildFrame(6, 0x46, {0x77});
    const std::vector<uint8_t> noise = {0x00, 0xAA, 0xAA, 0x55, 0x13};

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), noise.begin(), noise.end());
    stream.insert(stream.end(), invalid_len.begin(), invalid_len.end());
    stream.insert(stream.end(), bad_tail.begin(), bad_tail.end());
    stream.insert(stream.end(), bad_crc.begin(), bad_crc.end());
    stream.insert(stream.end(), good.begin(), good.end());

    parser.push(stream.data(), stream.size());
    parser.parse([&](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
        delivered.push_back({seq, cmd, std::vector<uint8_t>(payload, payload + plen)});
    });

    ASSERT_EQ(delivered.size(), 1U);
    EXPECT_EQ(delivered[0].seq, 6U);
    EXPECT_EQ(delivered[0].cmd, 0x46U);
    EXPECT_EQ(delivered[0].payload, (std::vector<uint8_t>{0x77}));

    const auto stats = parser.consumeStats();
    EXPECT_EQ(stats.frames_ok, 1U);
    EXPECT_GT(stats.len_invalid, 0U);
    EXPECT_GT(stats.tail_bad, 0U);
    EXPECT_GT(stats.crc_bad, 0U);
    EXPECT_GT(stats.head_drop, 0U);
}

TEST(RingParser, HandlesOverflowAndCanContinueParsing) {
    rc26_serial::RingParser parser;
    std::vector<DeliveredFrame> delivered;

    std::vector<uint8_t> noise(rc26_serial::RingParser::CAPACITY + 128U, 0xFF);
    parser.push(noise.data(), noise.size());
    parser.parse([&](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
        delivered.push_back({seq, cmd, std::vector<uint8_t>(payload, payload + plen)});
    });

    const auto good = buildFrame(7, 0x47, {0x10, 0x20, 0x30});
    parser.push(good.data(), good.size());
    parser.parse([&](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
        delivered.push_back({seq, cmd, std::vector<uint8_t>(payload, payload + plen)});
    });

    ASSERT_EQ(delivered.size(), 1U);
    EXPECT_EQ(delivered[0].seq, 7U);
    EXPECT_EQ(delivered[0].cmd, 0x47U);

    const auto stats = parser.consumeStats();
    EXPECT_EQ(stats.frames_ok, 1U);
    EXPECT_GT(stats.overflow_drop, 0U);
}
