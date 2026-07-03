#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include "rc26_serial/protocol.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace {

constexpr uint8_t kMcuError = static_cast<uint8_t>(rc26_serial::FeedbackID::MCU_ERROR);
constexpr uint8_t kAck = static_cast<uint8_t>(rc26_serial::FeedbackID::ACK);
constexpr uint8_t kArmRaiseDone = static_cast<uint8_t>(rc26_serial::FeedbackID::ARM_RAISE_DONE);

std::vector<uint8_t> buildFrame(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload,
                                uint8_t retry = 0x00) {
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

struct TxFrame {
    uint8_t seq = 0;
    uint8_t retry = 0;
    uint8_t cmd = 0;
    std::vector<uint8_t> payload;
};

struct ResponseFrame {
    uint8_t seq = 0;
    uint8_t feedback_id = 0;
    std::vector<uint8_t> payload;
};

class PseudoMcu {
public:
    using ResponseFn = std::function<std::vector<ResponseFrame>(const TxFrame&, size_t)>;

    explicit PseudoMcu(ResponseFn response_fn) : response_fn_(std::move(response_fn)) {
        master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master_fd_ < 0) {
            throw std::runtime_error("posix_openpt failed");
        }
        if (::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
            ::close(master_fd_);
            master_fd_ = -1;
            throw std::runtime_error("grantpt/unlockpt failed");
        }
        char* slave_name = ::ptsname(master_fd_);
        if (slave_name == nullptr) {
            ::close(master_fd_);
            master_fd_ = -1;
            throw std::runtime_error("ptsname failed");
        }
        slave_path_ = slave_name;
        worker_ = std::thread(&PseudoMcu::run, this);
    }

    ~PseudoMcu() {
        stop_.store(true, std::memory_order_release);
        if (worker_.joinable()) {
            worker_.join();
        }
        if (master_fd_ >= 0) {
            ::close(master_fd_);
            master_fd_ = -1;
        }
    }

    PseudoMcu(const PseudoMcu&) = delete;
    PseudoMcu& operator=(const PseudoMcu&) = delete;

    const std::string& slavePath() const { return slave_path_; }

    std::vector<TxFrame> receivedFrames() const {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        return received_frames_;
    }

private:
    void run() {
        std::vector<uint8_t> buffer;
        while (!stop_.load(std::memory_order_acquire)) {
            pollfd pfd {};
            pfd.fd = master_fd_;
            pfd.events = POLLIN;
            const int poll_ret = ::poll(&pfd, 1, 20);
            if (poll_ret <= 0) {
                continue;
            }
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                break;
            }
            if (!(pfd.revents & POLLIN)) {
                continue;
            }

            uint8_t tmp[256];
            const ssize_t n = ::read(master_fd_, tmp, sizeof(tmp));
            if (n <= 0) {
                continue;
            }
            buffer.insert(buffer.end(), tmp, tmp + n);
            parse(buffer);
        }
    }

    void parse(std::vector<uint8_t>& buffer) {
        while (buffer.size() >= 10U) {
            auto head = std::search(buffer.begin(), buffer.end(), kHead.begin(), kHead.end());
            if (head == buffer.end()) {
                buffer.clear();
                return;
            }
            if (head != buffer.begin()) {
                buffer.erase(buffer.begin(), head);
            }
            if (buffer.size() < 10U) {
                return;
            }

            const uint8_t len = buffer[3];
            if (len == 0U || len > 1U + rc26_decision::MAX_PAYLOAD_SIZE) {
                buffer.erase(buffer.begin());
                continue;
            }
            const size_t frame_size = 2 + 1 + 1 + 1 + static_cast<size_t>(len) + 4 + 2;
            if (buffer.size() < frame_size) {
                return;
            }
            if (buffer[frame_size - 2] != rc26_decision::FRAME_TAIL_0 ||
                buffer[frame_size - 1] != rc26_decision::FRAME_TAIL_1) {
                buffer.erase(buffer.begin());
                continue;
            }
            const uint32_t expected_crc =
                rc26_decision::crc32_mpeg2_calculate(&buffer[2], 1 + 1 + 1 + static_cast<size_t>(len));
            const uint32_t actual_crc = static_cast<uint32_t>(buffer[frame_size - 6]) |
                                        (static_cast<uint32_t>(buffer[frame_size - 5]) << 8) |
                                        (static_cast<uint32_t>(buffer[frame_size - 4]) << 16) |
                                        (static_cast<uint32_t>(buffer[frame_size - 3]) << 24);
            if (actual_crc != expected_crc) {
                buffer.erase(buffer.begin());
                continue;
            }

            TxFrame frame;
            frame.seq = buffer[2];
            frame.retry = buffer[4];
            frame.cmd = buffer[5];
            frame.payload.assign(buffer.begin() + 6, buffer.begin() + 6 + (len - 1U));
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(frame_size));

            size_t index = 0;
            {
                std::lock_guard<std::mutex> lock(frames_mutex_);
                index = received_frames_.size();
                received_frames_.push_back(frame);
            }

            for (const auto& response : response_fn_(frame, index)) {
                const auto response_frame = buildFrame(response.seq, response.feedback_id, response.payload);
                writeAll(response_frame);
            }
        }
    }

    void writeAll(const std::vector<uint8_t>& frame) {
        size_t written = 0;
        while (written < frame.size()) {
            const ssize_t n = ::write(master_fd_, frame.data() + written, frame.size() - written);
            if (n <= 0) {
                return;
            }
            written += static_cast<size_t>(n);
        }
    }

    static constexpr std::array<uint8_t, 2> kHead{rc26_decision::FRAME_HEAD_0, rc26_decision::FRAME_HEAD_1};

    int master_fd_ = -1;
    std::string slave_path_;
    ResponseFn response_fn_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    mutable std::mutex frames_mutex_;
    std::vector<TxFrame> received_frames_;
};

}  // namespace

TEST(SerialDriverMcuError, RetriesAfterMcuErrorAndSucceedsOnAck) {
    PseudoMcu mcu([](const TxFrame& frame, size_t index) {
        return std::vector<ResponseFrame>{{frame.seq, index < 2U ? kMcuError : kAck, {}}};
    });

    rc26_decision::SerialDriver driver;
    ASSERT_TRUE(driver.open(mcu.slavePath(), rc26_decision::UART_BAUDRATE)) << driver.lastError();

    uint8_t seq = 0;
    EXPECT_TRUE(driver.sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::ARM_RAISE), {}, seq))
        << driver.lastError();
    driver.close();

    const auto frames = mcu.receivedFrames();
    ASSERT_GE(frames.size(), 3U);
    EXPECT_EQ(frames[0].seq, seq);
    EXPECT_EQ(frames[1].seq, seq);
    EXPECT_EQ(frames[2].seq, seq);
    EXPECT_EQ(frames[0].retry, 0x00U);
    EXPECT_EQ(frames[1].retry, 0x01U);
    EXPECT_EQ(frames[2].retry, 0x02U);
    EXPECT_EQ(driver.commHealth().mcu_error_responses.load(), 2U);
}

TEST(SerialDriverMcuError, SameSeqBusinessFeedbackAfterAckIsDeliveredAfterSendCommandReturns) {
    PseudoMcu mcu([](const TxFrame& frame, size_t) {
        return std::vector<ResponseFrame>{{frame.seq, kAck, {}}, {frame.seq, kArmRaiseDone, {}}};
    });

    rc26_decision::SerialDriver driver;
    ASSERT_TRUE(driver.open(mcu.slavePath(), rc26_decision::UART_BAUDRATE)) << driver.lastError();

    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<bool> send_returned{false};
    bool done_received = false;
    bool done_after_return = false;
    uint8_t done_seq = 0;

    driver.setReceiveCallback([&](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>&) {
        if (cmd != kArmRaiseDone) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex);
        done_received = true;
        done_after_return = send_returned.load(std::memory_order_acquire);
        done_seq = seq;
        cv.notify_one();
    });

    uint8_t seq = 0;
    const bool sent = driver.sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::ARM_RAISE), {}, seq);
    send_returned.store(true, std::memory_order_release);
    ASSERT_TRUE(sent) << driver.lastError();

    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(1), [&]() { return done_received; }));
        EXPECT_TRUE(done_after_return);
        EXPECT_EQ(done_seq, seq);
    }

    driver.close();

    const auto frames = mcu.receivedFrames();
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].seq, seq);
    EXPECT_EQ(frames[0].retry, 0x00U);
}

TEST(SerialDriverMcuError, TwoByteMcuErrorPayloadIsDeliveredAsBusinessFeedback) {
    PseudoMcu mcu([](const TxFrame& frame, size_t) {
        return std::vector<ResponseFrame>{
            {frame.seq, kAck, {}},
            {frame.seq, kMcuError,
             {static_cast<uint8_t>(rc26_serial::CommandID::ARM_LOWER),
              static_cast<uint8_t>(rc26_serial::PlanarArmFailCode::BUSY)}}};
    });

    rc26_decision::SerialDriver driver;
    ASSERT_TRUE(driver.open(mcu.slavePath(), rc26_decision::UART_BAUDRATE)) << driver.lastError();

    std::mutex mutex;
    std::condition_variable cv;
    bool error_received = false;
    uint8_t error_seq = 0;
    std::vector<uint8_t> error_payload;

    driver.setReceiveCallback([&](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
        if (cmd != kMcuError) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        error_received = true;
        error_seq = seq;
        error_payload = payload;
        cv.notify_one();
    });

    uint8_t seq = 0;
    ASSERT_TRUE(driver.sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::ARM_LOWER), {}, seq))
        << driver.lastError();

    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(1), [&]() { return error_received; }));
        EXPECT_EQ(error_seq, seq);
        ASSERT_EQ(error_payload.size(), 2U);
        EXPECT_EQ(error_payload[0], static_cast<uint8_t>(rc26_serial::CommandID::ARM_LOWER));
        EXPECT_EQ(error_payload[1], static_cast<uint8_t>(rc26_serial::PlanarArmFailCode::BUSY));
    }

    driver.close();
    EXPECT_EQ(driver.commHealth().mcu_error_responses.load(), 0U);
}

TEST(SerialDriverMcuError, FailsWithLowerMachineReasonAfterRetryExhaustion) {
    PseudoMcu mcu([](const TxFrame& frame, size_t) {
        return std::vector<ResponseFrame>{{frame.seq, kMcuError, {}}};
    });

    rc26_decision::SerialDriver driver;
    ASSERT_TRUE(driver.open(mcu.slavePath(), rc26_decision::UART_BAUDRATE)) << driver.lastError();

    uint8_t seq = 0;
    EXPECT_FALSE(driver.sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::ARM_LOWER), {}, seq));
    const std::string last_error = driver.lastError();
    driver.close();

    EXPECT_NE(last_error.find("0xFE"), std::string::npos);
    EXPECT_NE(last_error.find("下位机"), std::string::npos);
    EXPECT_EQ(driver.commHealth().mcu_error_responses.load(), 10U);

    const auto frames = mcu.receivedFrames();
    ASSERT_EQ(frames.size(), 10U);
    for (size_t i = 0; i < frames.size(); ++i) {
        EXPECT_EQ(frames[i].seq, seq);
        EXPECT_EQ(frames[i].retry, static_cast<uint8_t>(i));
    }
}

TEST(SerialDriverMcuError, UnmatchedMcuErrorDoesNotSatisfyAckWait) {
    PseudoMcu mcu([](const TxFrame& frame, size_t index) {
        if (index == 0U) {
            return std::vector<ResponseFrame>{{static_cast<uint8_t>(frame.seq + 1U), kMcuError, {}}};
        }
        return std::vector<ResponseFrame>{{frame.seq, kAck, {}}};
    });

    rc26_decision::SerialDriver driver;
    ASSERT_TRUE(driver.open(mcu.slavePath(), rc26_decision::UART_BAUDRATE)) << driver.lastError();

    uint8_t seq = 0;
    EXPECT_TRUE(driver.sendCommand(static_cast<uint8_t>(rc26_serial::CommandID::PLACE_KFS_GRID), {}, seq))
        << driver.lastError();
    driver.close();

    const auto frames = mcu.receivedFrames();
    ASSERT_GE(frames.size(), 2U);
    EXPECT_EQ(frames[0].seq, seq);
    EXPECT_EQ(frames[1].seq, seq);
    EXPECT_EQ(frames[0].retry, 0x00U);
    EXPECT_EQ(frames[1].retry, 0x01U);
    EXPECT_EQ(driver.commHealth().mcu_error_responses.load(), 1U);
}
