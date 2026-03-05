#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rc26_serial/adaptive_timeout.hpp"
#include "rc26_serial/protocol.hpp"
#include "rc26_serial/ring_parser.hpp"
#include "rc26_serial/sliding_counter.hpp"

namespace rc26_decision {

class SerialDriver {
public:
    using ReceiveCallback = std::function<void(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)>;
    using HeartbeatFailureCallback = std::function<void()>;
    using DebugCallback = std::function<void(bool is_tx, const std::vector<uint8_t>& data)>;

    struct CommHealth {
        std::atomic<uint32_t> total_frames{0};
        std::atomic<uint32_t> parse_errors{0};
        std::atomic<uint32_t> ack_timeouts{0};
        std::atomic<uint32_t> reconnect_count{0};
        std::atomic<uint32_t> heartbeat_failures{0};

        rc26_serial::SlidingCounter<1000> parse_window;
        rc26_serial::SlidingCounter<200> ack_window;

        static constexpr uint32_t RECONNECT_RING_SIZE = 8;
        mutable std::mutex reconnect_ring_mutex_;
        std::array<std::chrono::steady_clock::time_point, RECONNECT_RING_SIZE> reconnect_times_{};
        uint32_t reconnect_ring_pos_{0};

        enum class Level : uint8_t { HEALTHY = 0, DEGRADED = 1, CRITICAL = 2, FAILED = 3 };

        Level level() const {
            const float parse_rate = parse_window.errorRate();
            const float ack_rate = ack_window.errorRate();
            {
                std::lock_guard<std::mutex> lock(reconnect_ring_mutex_);
                const auto now = std::chrono::steady_clock::now();
                uint32_t recent_reconnects = 0;
                for (const auto& t : reconnect_times_) {
                    if (t.time_since_epoch().count() > 0 &&
                        std::chrono::duration<double>(now - t).count() <= 60.0) {
                        ++recent_reconnects;
                    }
                }
                if (recent_reconnects > 3U) {
                    return Level::FAILED;
                }
            }
            if (parse_rate > 0.05F || ack_rate > 0.025F) {
                return Level::CRITICAL;
            }
            if (parse_rate > 0.01F || ack_rate > 0.01F) {
                return Level::DEGRADED;
            }
            return Level::HEALTHY;
        }

        void recordReconnect() {
            reconnect_count.fetch_add(1, std::memory_order_relaxed);
            std::lock_guard<std::mutex> lock(reconnect_ring_mutex_);
            reconnect_times_[reconnect_ring_pos_++ % RECONNECT_RING_SIZE] = std::chrono::steady_clock::now();
        }

        void resetWindows() {
            parse_window.reset();
            ack_window.reset();
            std::lock_guard<std::mutex> lock(reconnect_ring_mutex_);
            reconnect_times_.fill({});
            reconnect_ring_pos_ = 0;
        }
    };

    SerialDriver();
    ~SerialDriver();

    SerialDriver(const SerialDriver&) = delete;
    SerialDriver& operator=(const SerialDriver&) = delete;

    // ========================================================================
    // 连接管理
    // ========================================================================
    bool open(const std::string& port, int baudrate = UART_BAUDRATE);
    void close();
    bool isOpen() const { return fd_ >= 0 && running_.load(); }
    std::string lastError() const;

    // ========================================================================
    // 发送接口（用户只需传入命令和载荷）
    // ========================================================================
    bool sendCommand(uint8_t cmd, const std::vector<uint8_t>& payload = {});
    bool sendCommand(CommandID cmd, const std::vector<uint8_t>& payload = {});
    bool sendCommand(uint8_t cmd, const std::vector<uint8_t>& payload, uint8_t& out_seq);
    bool sendCommand(CommandID cmd, const std::vector<uint8_t>& payload, uint8_t& out_seq);
    uint8_t lastSentSeq() const { return seq_.load(std::memory_order_relaxed); }
    const CommHealth& commHealth() const { return comm_health_; }
    float avgRttMs() const;

    // ========================================================================
    // 便捷接口
    // ========================================================================
    bool sendPose(CommandID cmd, float vx, float vy, float wz, uint8_t& out_seq);
    bool sendPose(CommandID cmd, float vx, float vy, float wz);
    bool sendStop();
    bool sendHeartbeat();

    // ========================================================================
    // 回调设置
    // ========================================================================
    // CONSTRAINT: 回调内禁止调用 sendCommand()/close()/open()，否则 recv 线程无法递送 ACK 而死锁
    void setReceiveCallback(ReceiveCallback callback);
    void setHeartbeatFailureCallback(HeartbeatFailureCallback callback);
    using ReconnectCallback = std::function<void()>;
    void setReconnectCallback(ReconnectCallback callback);
    using ReconnectStartCallback = std::function<void()>;
    void setReconnectStartCallback(ReconnectStartCallback callback);
    using ReconnectFailedCallback = std::function<void()>;
    void setReconnectFailedCallback(ReconnectFailedCallback callback);
    void setDebugCallback(DebugCallback callback);

private:
    int fd_ = -1;
    std::string port_;
    int baudrate_ = UART_BAUDRATE;
    std::atomic<uint8_t> seq_{0};

    std::atomic<bool> running_{false};
    std::thread recv_thread_;
    std::mutex send_mutex_;
    std::mutex ack_command_mutex_;

    ReceiveCallback recv_callback_;
    HeartbeatFailureCallback heartbeat_failure_callback_;
    DebugCallback debug_callback_;
    mutable std::mutex callback_mutex_;

    mutable std::mutex last_error_mutex_;
    std::string last_error_;

    std::mutex ack_mutex_;
    std::condition_variable ack_cv_;
    bool waiting_for_ack_{false};
    uint8_t waiting_seq_{0};
    uint8_t waiting_cmd_{0};
    std::atomic<uint32_t> link_epoch_{0};
    uint32_t waiting_epoch_{0};  // 受 ack_mutex_ 保护
    enum class AckWaitResult : uint8_t { kReceived, kTimeout, kLinkDown };
    bool ack_response_received_{false};
    bool ack_success_{false};
    rc26_serial::RingParser ring_parser_;

    uint8_t nextSeq();
    std::vector<uint8_t> buildFrame(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload,
                                    uint8_t retry = 0x00);
    bool writeAll(const uint8_t* data, size_t size);
    void setLastError(std::string message);

    bool sendCommandNoAck(uint8_t cmd, const std::vector<uint8_t>& payload);

    void beginWaitAck(uint8_t seq, uint8_t cmd);
    void endWaitAck();
    AckWaitResult waitForAck(std::chrono::milliseconds timeout, bool& success);
    void notifyAck(uint8_t seq, uint8_t cmd);

    void notifyHeartbeatFailure();
    void notifyReconnect();
    void notifyReconnectFailed();

    bool reconnect();
    void reconnectThreadFunc();
    void requestReconnect(const char* reason);

    void recvThreadFunc();
    void dispatchFrame(uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen);

    bool write_error_active_{false};
    bool recv_error_active_{false};
    CommHealth comm_health_;
    mutable std::mutex timeout_mutex_;
    rc26_serial::AdaptiveTimeout adaptive_timeout_;

    ReconnectCallback reconnect_callback_;
    ReconnectStartCallback reconnect_start_callback_;
    ReconnectFailedCallback reconnect_failed_callback_;
    std::atomic<uint8_t> heartbeat_failure_count_{0};
    std::atomic<bool> reconnecting_{false};

    // 异步重连线程（生命周期绑定 SerialDriver 对象）
    std::atomic<bool> reconnect_thread_running_{false};
    std::thread reconnect_thread_;
    std::mutex reconnect_cv_mutex_;
    std::condition_variable reconnect_cv_;
    std::atomic<bool> reconnect_requested_{false};
};

}  // namespace rc26_decision
