#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

class SerialDriver {
public:
    using ReceiveCallback = std::function<void(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload)>;
    using HeartbeatFailureCallback = std::function<void()>;
    using DebugCallback = std::function<void(bool is_tx, const std::vector<uint8_t>& data)>;

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

    // ========================================================================
    // 便捷接口
    // ========================================================================
    bool sendPose(float vx, float vy, float wx, float wy, float wz, float roll, float pitch, float yaw);
    bool sendPose(CommandID cmd, float vx, float vy, float wz);
    bool sendStop();
    bool sendHeartbeat();

    // ========================================================================
    // 回调设置
    // ========================================================================
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
    bool ack_response_received_{false};
    bool ack_success_{false};

    static constexpr size_t RECV_BUFFER_SIZE = 256;
    uint8_t recv_buffer_[RECV_BUFFER_SIZE];
    size_t recv_len_ = 0;

    uint8_t nextSeq();
    std::vector<uint8_t> buildFrame(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload,
                                    uint8_t retry = 0x00);
    bool writeAll(const uint8_t* data, size_t size);
    void setLastError(std::string message);

    bool sendCommandNoAck(uint8_t cmd, const std::vector<uint8_t>& payload);

    void beginWaitAck(uint8_t seq, uint8_t cmd);
    void endWaitAck();
    bool waitForAck(std::chrono::milliseconds timeout, bool& success);
    void notifyAck(uint8_t seq, uint8_t cmd);

    void notifyHeartbeatFailure();
    void notifyReconnect();
    void notifyReconnectFailed();

    bool reconnect();

    void recvThreadFunc();
    void parseReceivedData();

    bool write_error_active_{false};
    bool recv_error_active_{false};
    uint32_t parse_error_count_{0};

    ReconnectCallback reconnect_callback_;
    ReconnectStartCallback reconnect_start_callback_;
    ReconnectFailedCallback reconnect_failed_callback_;
    std::atomic<uint8_t> heartbeat_failure_count_{0};
    std::atomic<bool> reconnecting_{false};
};

}  // namespace rc26_decision
