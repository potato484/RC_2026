// RC2026 串口驱动实现
#include "rc26_serial/serial_driver.hpp"

#include <cassert>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#ifdef __linux__
#include <pthread.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

namespace {
rclcpp::Logger serialLogger() {
    return rclcpp::get_logger("rc26_decision.serial_driver");
}

bool toTermiosBaudrate(int baudrate, speed_t& baud) {
    switch (baudrate) {
    case 9600:
        baud = B9600;
        return true;
    case 19200:
        baud = B19200;
        return true;
    case 38400:
        baud = B38400;
        return true;
    case 57600:
        baud = B57600;
        return true;
    case 115200:
        baud = B115200;
        return true;
    case 230400:
        baud = B230400;
        return true;
    case 460800:
        baud = B460800;
        return true;
    case 500000:
        baud = B500000;
        return true;
    case 576000:
        baud = B576000;
        return true;
    case 921600:
        baud = B921600;
        return true;
    case 1000000:
        baud = B1000000;
        return true;
    case 1152000:
        baud = B1152000;
        return true;
    case 1500000:
        baud = B1500000;
        return true;
    case 2000000:
        baud = B2000000;
        return true;
    case 2500000:
        baud = B2500000;
        return true;
    case 3000000:
        baud = B3000000;
        return true;
    case 3500000:
        baud = B3500000;
        return true;
    case 4000000:
        baud = B4000000;
        return true;
    default:
        return false;
    }
}

std::string errnoText(int err) {
    return std::to_string(err) + " (" + std::string(std::strerror(err)) + ")";
}

thread_local bool tl_in_recv_callback = false;
}  // namespace

SerialDriver::SerialDriver() {
    reconnect_thread_running_ = true;
    reconnect_thread_ = std::thread(&SerialDriver::reconnectThreadFunc, this);
}

SerialDriver::~SerialDriver() {
    reconnect_thread_running_ = false;
    reconnect_cv_.notify_one();
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
    close();
}

std::string SerialDriver::lastError() const {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    return last_error_;
}

void SerialDriver::setLastError(std::string message) {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    last_error_ = std::move(message);
}

bool SerialDriver::open(const std::string& port, int baudrate) {
    const bool cold_start = port_.empty();

    if (fd_ >= 0) {
        close();
    }

    baudrate_ = baudrate;

    if (port.empty()) {
        setLastError("serial_port 不能为空");
        RCLCPP_ERROR(serialLogger(), "打开串口失败：serial_port 为空");
        return false;
    }

    speed_t baud;
    if (!toTermiosBaudrate(baudrate, baud)) {
        setLastError("不支持的 baudrate: " + std::to_string(baudrate));
        RCLCPP_ERROR(serialLogger(),
                     "打开串口失败：不支持的 baudrate=%d（支持: 9600/19200/38400/57600/115200/230400/460800）",
                     baudrate);
        return false;
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        const int err = errno;
        setLastError("open() 失败: errno=" + errnoText(err) + ", port=" + port);
        RCLCPP_ERROR(serialLogger(), "打开串口失败：open(%s) errno=%d(%s)", port.c_str(), err, std::strerror(err));
        return false;
    }

    port_ = port;

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        const int err = errno;
        setLastError("tcgetattr() 失败: errno=" + errnoText(err) + ", port=" + port_);
        RCLCPP_ERROR(serialLogger(), "配置串口失败：tcgetattr(%s) errno=%d(%s)", port_.c_str(), err,
                     std::strerror(err));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
        const int err = errno;
        setLastError("cfsetispeed/cfsetospeed 失败: errno=" + errnoText(err) + ", port=" + port_);
        RCLCPP_ERROR(serialLogger(), "配置串口失败：设置波特率失败(%s @ %d) errno=%d(%s)", port_.c_str(), baudrate, err,
                     std::strerror(err));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 8N1, 无流控
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // 非阻塞读取
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms 超时

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const int err = errno;
        setLastError("tcsetattr() 失败: errno=" + errnoText(err) + ", port=" + port_);
        RCLCPP_ERROR(serialLogger(), "配置串口失败：tcsetattr(%s) errno=%d(%s)", port_.c_str(), err,
                     std::strerror(err));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    recv_len_ = 0;
    parse_error_count_ = 0;
    if (cold_start) {
        comm_health_.total_frames.store(0, std::memory_order_relaxed);
        comm_health_.parse_errors.store(0, std::memory_order_relaxed);
        comm_health_.ack_timeouts.store(0, std::memory_order_relaxed);
        comm_health_.reconnect_count.store(0, std::memory_order_relaxed);
        comm_health_.heartbeat_failures.store(0, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> timeout_lock(timeout_mutex_);
            adaptive_timeout_ = rc26_serial::AdaptiveTimeout();
        }
    }
    recv_error_active_ = false;
    {
        std::lock_guard<std::mutex> lock(send_mutex_);
        write_error_active_ = false;
    }

    try {
        running_ = true;
        recv_thread_ = std::thread(&SerialDriver::recvThreadFunc, this);
#ifdef __linux__
        {
            struct sched_param sp {};
            sp.sched_priority = 60;
            if (pthread_setschedparam(recv_thread_.native_handle(), SCHED_FIFO, &sp) != 0) {
                RCLCPP_WARN(serialLogger(), "设置接收线程实时优先级失败（需 CAP_SYS_NICE 或 root）");
            }
        }
#endif
    } catch (const std::exception& e) {
        running_ = false;
        setLastError(std::string("启动接收线程失败: ") + e.what());
        RCLCPP_ERROR(serialLogger(), "启动接收线程失败：%s", e.what());
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    setLastError({});
    heartbeat_failure_count_ = 0;
    comm_health_.heartbeat_failures.store(0, std::memory_order_relaxed);
    RCLCPP_DEBUG(serialLogger(), "串口打开成功：%s @ %d", port_.c_str(), baudrate);

    return true;
}

void SerialDriver::close() {
    running_ = false;

    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    if (fd_ >= 0) {
        RCLCPP_DEBUG(serialLogger(), "关闭串口：%s", port_.c_str());
        ::close(fd_);
        fd_ = -1;
    }

    recv_len_ = 0;
    parse_error_count_ = 0;
}

uint8_t SerialDriver::nextSeq() {
    return seq_.fetch_add(1, std::memory_order_relaxed);
}

void SerialDriver::notifyHeartbeatFailure() {
    HeartbeatFailureCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = heartbeat_failure_callback_;
    }
    if (cb) {
        cb();
    }
}

void SerialDriver::notifyReconnect() {
    ReconnectCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = reconnect_callback_;
    }
    if (cb) {
        cb();
    }
}

void SerialDriver::notifyReconnectFailed() {
    ReconnectFailedCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = reconnect_failed_callback_;
    }
    if (cb) {
        cb();
    }
}

void SerialDriver::setReconnectCallback(ReconnectCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    reconnect_callback_ = std::move(callback);
}

void SerialDriver::setReconnectStartCallback(ReconnectStartCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    reconnect_start_callback_ = std::move(callback);
}

void SerialDriver::setReconnectFailedCallback(ReconnectFailedCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    reconnect_failed_callback_ = std::move(callback);
}

void SerialDriver::reconnectThreadFunc() {
    while (reconnect_thread_running_) {
        {
            std::unique_lock<std::mutex> lock(reconnect_cv_mutex_);
            reconnect_cv_.wait(lock, [this]() {
                return !reconnect_thread_running_.load() || reconnect_requested_.load();
            });
        }

        if (!reconnect_thread_running_) {
            break;
        }
        if (reconnect_requested_.exchange(false)) {
            reconnect();
        }
    }
}

void SerialDriver::requestReconnect(const char* reason) {
    RCLCPP_WARN(serialLogger(), "请求重连: %s", reason);
    reconnect_requested_.store(true);
    reconnect_cv_.notify_one();
}

bool SerialDriver::reconnect() {
    // 防止并发重连
    bool expected = false;
    if (!reconnecting_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(serialLogger(), "重连已在进行中，跳过本次重连请求");
        return false;
    }

    comm_health_.reconnect_count.fetch_add(1, std::memory_order_relaxed);

    RCLCPP_WARN(serialLogger(), "开始串口重连：%s @ %d", port_.c_str(), baudrate_);

    {
        ReconnectStartCallback cb;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            cb = reconnect_start_callback_;
        }
        if (cb) {
            cb();
        }
    }

    for (uint8_t attempt = 1; attempt <= MAX_RECONNECT_ATTEMPTS; ++attempt) {
        close();
        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_INTERVAL_MS));

        if (open(port_, baudrate_)) {
            RCLCPP_INFO(serialLogger(), "串口重连成功：%s（第%u次尝试）", port_.c_str(), attempt);
            notifyReconnect();
            reconnecting_ = false;
            return true;
        }

        RCLCPP_WARN(serialLogger(), "串口重连失败（第%u/%u次），%ums后重试：%s", attempt, MAX_RECONNECT_ATTEMPTS,
                    RECONNECT_INTERVAL_MS, port_.c_str());
    }

    // 所有重连尝试失败
    RCLCPP_ERROR(serialLogger(), "串口重连失败，已达最大尝试次数(%u)：%s", MAX_RECONNECT_ATTEMPTS, port_.c_str());
    notifyReconnectFailed();
    reconnecting_ = false;
    return false;
}

bool SerialDriver::writeAll(const uint8_t* data, size_t size) {
    size_t offset = 0;
    while (offset < size) {
        ssize_t written = ::write(fd_, data + offset, size - offset);
        if (written > 0) {
            offset += static_cast<size_t>(written);
            continue;
        }

        if (written < 0 && errno == EINTR) {
            continue;
        }

        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }

        if (!write_error_active_) {
            const int err = errno;
            write_error_active_ = true;
            setLastError("write() 失败: errno=" + errnoText(err) + ", port=" + port_);
            RCLCPP_ERROR(serialLogger(), "串口写入失败：port=%s, size=%zu, errno=%d(%s)", port_.c_str(), size, err,
                         std::strerror(err));
        }
        return false;
    }

    if (write_error_active_) {
        write_error_active_ = false;
        RCLCPP_DEBUG(serialLogger(), "串口写入已恢复：%s", port_.c_str());
    }
    return true;
}

std::vector<uint8_t> SerialDriver::buildFrame(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload,
                                              uint8_t retry) {
    // payload 长度校验
    if (payload.size() > MAX_PAYLOAD_SIZE) {
        setLastError("payload 超出最大长度限制: " + std::to_string(payload.size()) + " > " +
                     std::to_string(MAX_PAYLOAD_SIZE));
        RCLCPP_ERROR(serialLogger(), "构建帧失败：payload 长度 %zu 超出最大限制 %u", payload.size(), MAX_PAYLOAD_SIZE);
        return {};
    }

    // HEAD(2) + SEQ(1) + LEN(1) + RETRY(1) + CMD(1) + PAYLOAD(N) + CRC32(4) + TAIL(2)
    size_t frame_size = 2 + 1 + 1 + 1 + 1 + payload.size() + 4 + 2;
    std::vector<uint8_t> frame(frame_size);

    size_t idx = 0;

    frame[idx++] = FRAME_HEAD_0;
    frame[idx++] = FRAME_HEAD_1;

    frame[idx++] = seq;

    // LEN: 1字节，len = cmd(1) + payload(N)
    uint8_t len = static_cast<uint8_t>(1 + payload.size());
    frame[idx++] = len;

    // RETRY: 重发次数字段
    frame[idx++] = retry;

    frame[idx++] = cmd;

    for (uint8_t byte : payload) {
        frame[idx++] = byte;
    }

    // CRC32校验: SEQ(1) + LEN(1) + RETRY(1) + CMD(1) + PAYLOAD(N)
    size_t crc_start = 2;
    size_t crc_len = 1 + 1 + 1 + 1 + payload.size();
    uint32_t crc = crc32_mpeg2_calculate(&frame[crc_start], crc_len);
    frame[idx++] = static_cast<uint8_t>(crc & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    frame[idx++] = static_cast<uint8_t>((crc >> 24) & 0xFF);

    frame[idx++] = FRAME_TAIL_0;
    frame[idx++] = FRAME_TAIL_1;

    return frame;
}

void SerialDriver::beginWaitAck(uint8_t seq, uint8_t cmd) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    waiting_for_ack_ = true;
    waiting_seq_ = seq;
    waiting_cmd_ = cmd;
    ack_response_received_ = false;
    ack_success_ = false;
}

void SerialDriver::endWaitAck() {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    waiting_for_ack_ = false;
    ack_response_received_ = false;
    ack_success_ = false;
}

bool SerialDriver::waitForAck(std::chrono::milliseconds timeout, bool& success) {
    std::unique_lock<std::mutex> lock(ack_mutex_);
    bool ok = ack_cv_.wait_for(lock, timeout, [this]() { return ack_response_received_; });
    if (!ok) {
        success = false;
        return false;
    }

    success = ack_success_;
    return true;
}

void SerialDriver::notifyAck(uint8_t seq, uint8_t cmd) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    if (!waiting_for_ack_ || seq != waiting_seq_) {
        return;
    }

    if (cmd == static_cast<uint8_t>(FeedbackID::ACK)) {
        ack_response_received_ = true;
        ack_success_ = true;
        ack_cv_.notify_all();
        RCLCPP_DEBUG(serialLogger(), "收到 ACK：seq=%u", seq);
        return;
    }
    if (cmd == static_cast<uint8_t>(FeedbackID::NACK)) {
        ack_response_received_ = true;
        ack_success_ = false;
        ack_cv_.notify_all();
        RCLCPP_DEBUG(serialLogger(), "收到 NACK：seq=%u", seq);
        return;
    }

    if (waiting_cmd_ == static_cast<uint8_t>(CommandID::HEARTBEAT) &&
        cmd == static_cast<uint8_t>(FeedbackID::HEARTBEAT_ACK)) {
        ack_response_received_ = true;
        ack_success_ = true;
        ack_cv_.notify_all();
        RCLCPP_DEBUG(serialLogger(), "收到 HEARTBEAT_ACK：seq=%u", seq);
        return;
    }

    if (cmd == static_cast<uint8_t>(FeedbackID::ACTION_FAIL) ||
        cmd == static_cast<uint8_t>(FeedbackID::ERROR)) {
        ack_response_received_ = true;
        ack_success_ = false;
        ack_cv_.notify_all();
        RCLCPP_DEBUG(serialLogger(), "ACTION_FAIL/ERROR: seq=%u cmd=0x%02X", seq, cmd);
        return;
    }
}

bool SerialDriver::sendCommandNoAck(uint8_t cmd, const std::vector<uint8_t>& payload) {
    if (fd_ < 0) {
        setLastError("串口未打开");
        return false;
    }

    uint8_t seq = nextSeq();
    std::vector<uint8_t> frame = buildFrame(seq, cmd, payload, 0x00);
    if (frame.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    bool ok = writeAll(frame.data(), frame.size());
    if (ok) {
        DebugCallback cb;
        {
            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            cb = debug_callback_;
        }
        if (cb) {
            cb(true, frame);
        }
    }
    return ok;
}

bool SerialDriver::sendCommand(uint8_t cmd, const std::vector<uint8_t>& payload) {
    uint8_t ignored_seq = 0;
    return sendCommand(cmd, payload, ignored_seq);
}

bool SerialDriver::sendCommand(uint8_t cmd, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    if (tl_in_recv_callback) {
        setLastError("sendCommand() 在接收回调上下文中被调用");
        RCLCPP_ERROR(serialLogger(), "sendCommand() 在接收回调内被调用（会死锁），立即返回失败");
        return false;
    }

    if (fd_ < 0) {
        setLastError("串口未打开");
        return false;
    }

    std::lock_guard<std::mutex> cmd_lock(ack_command_mutex_);

    // 重发机制：retry从0x00开始，步长1，每100ms重发一次
    // 每3次为1轮，达到0x09后触发重连
    out_seq = nextSeq();
    for (uint8_t retry = 0x00; retry <= MAX_RETRY_VALUE; ++retry) {
        std::vector<uint8_t> frame = buildFrame(out_seq, cmd, payload, retry);
        if (frame.empty()) {
            return false;
        }

        beginWaitAck(out_seq, cmd);

        {
            std::lock_guard<std::mutex> lock(send_mutex_);
            if (!writeAll(frame.data(), frame.size())) {
                endWaitAck();
                return false;
            }
            DebugCallback cb;
            {
                std::lock_guard<std::mutex> cb_lock(callback_mutex_);
                cb = debug_callback_;
            }
            if (cb) {
                cb(true, frame);
            }
        }

        std::chrono::milliseconds ack_timeout{ACK_TIMEOUT_MS};
        {
            std::lock_guard<std::mutex> lock(timeout_mutex_);
            ack_timeout = adaptive_timeout_.get();
        }

        const auto wait_start = std::chrono::steady_clock::now();
        bool ack_success = false;
        bool got_response = waitForAck(ack_timeout, ack_success);
        const auto measured_ms =
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - wait_start).count();
        if (got_response && ack_success) {
            {
                std::lock_guard<std::mutex> lock(timeout_mutex_);
                adaptive_timeout_.update(measured_ms);
            }
            endWaitAck();
            return true;
        }

        endWaitAck();
        if (!got_response) {
            comm_health_.ack_timeouts.fetch_add(1, std::memory_order_relaxed);
        }

        uint8_t round = retry / RETRIES_PER_ROUND + 1;
        if (got_response) {
            RCLCPP_WARN(serialLogger(), "指令收到 NACK (retry=0x%02X, 第%u轮)：cmd=0x%02X, seq=%u", retry, round, cmd,
                        out_seq);
        } else {
            RCLCPP_WARN(serialLogger(),
                        "指令等待超时 (timeout=%lldms, retry=0x%02X, 第%u轮)：cmd=0x%02X, seq=%u",
                        static_cast<long long>(ack_timeout.count()), retry, round, cmd, out_seq);
        }
    }

    // 所有重试失败（0x00-0x09共10次），触发串口重连
    RCLCPP_ERROR(serialLogger(), "指令重发失败 cmd=0x%02X, 已达最大重试次数(0x09)，触发串口重连", cmd);

    requestReconnect("ack_retry_exhausted");

    setLastError("等待 ACK 超时：cmd=" + std::to_string(static_cast<int>(cmd)) + ", 已重试至0x09并触发重连");
    return false;
}

bool SerialDriver::sendCommand(CommandID cmd, const std::vector<uint8_t>& payload) {
    uint8_t ignored_seq = 0;
    return sendCommand(cmd, payload, ignored_seq);
}

bool SerialDriver::sendCommand(CommandID cmd, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    return sendCommand(static_cast<uint8_t>(cmd), payload, out_seq);
}

float SerialDriver::avgRttMs() const {
    std::lock_guard<std::mutex> lock(timeout_mutex_);
    return adaptive_timeout_.ewmaMs();
}

bool SerialDriver::sendPose(CommandID cmd, float vx, float vy, float wz) {
    std::vector<uint8_t> payload(12);
    std::memcpy(&payload[0], &vx, sizeof(float));
    std::memcpy(&payload[4], &vy, sizeof(float));
    std::memcpy(&payload[8], &wz, sizeof(float));
    return sendCommandNoAck(static_cast<uint8_t>(cmd), payload);
}

bool SerialDriver::sendStop() {
    return sendCommand(CommandID::STOP);
}

bool SerialDriver::sendHeartbeat() {
    if (fd_ < 0) {
        setLastError("串口未打开");
        return false;
    }

    // 心跳也需要受 ack_command_mutex_ 保护，防止与 sendCommand 并发覆盖 ACK 状态
    std::lock_guard<std::mutex> cmd_lock(ack_command_mutex_);

    uint8_t cmd = static_cast<uint8_t>(CommandID::HEARTBEAT);
    uint8_t seq = nextSeq();
    std::vector<uint8_t> frame = buildFrame(seq, cmd, {}, 0x00);
    if (frame.empty()) {
        return false;
    }

    beginWaitAck(seq, cmd);

    {
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (!writeAll(frame.data(), frame.size())) {
            endWaitAck();
            return false;
        }
        DebugCallback cb;
        {
            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            cb = debug_callback_;
        }
        if (cb) {
            cb(true, frame);
        }
    }

    std::chrono::milliseconds ack_timeout{ACK_TIMEOUT_MS};
    {
        std::lock_guard<std::mutex> lock(timeout_mutex_);
        ack_timeout = adaptive_timeout_.get();
    }

    const auto wait_start = std::chrono::steady_clock::now();
    bool ack_success = false;
    bool got_response = waitForAck(ack_timeout, ack_success);
    const auto measured_ms =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - wait_start).count();
    endWaitAck();

    if (got_response && ack_success) {
        {
            std::lock_guard<std::mutex> lock(timeout_mutex_);
            adaptive_timeout_.update(measured_ms);
        }
        heartbeat_failure_count_ = 0;
        RCLCPP_DEBUG(serialLogger(), "心跳成功：seq=%u", seq);
        return true;
    }

    if (!got_response) {
        comm_health_.ack_timeouts.fetch_add(1, std::memory_order_relaxed);
    }
    uint8_t failures = ++heartbeat_failure_count_;
    comm_health_.heartbeat_failures.store(failures, std::memory_order_relaxed);
    RCLCPP_WARN(serialLogger(), "心跳失败（连续%u次）：seq=%u", failures, seq);

    if (failures >= MAX_HEARTBEAT_FAILURES) {
        RCLCPP_ERROR(serialLogger(), "心跳连续失败%u次，触发串口重连", failures);
        notifyHeartbeatFailure();
        requestReconnect("heartbeat_failure");
    }

    return false;
}

void SerialDriver::setReceiveCallback(ReceiveCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    recv_callback_ = std::move(callback);
}

void SerialDriver::setHeartbeatFailureCallback(HeartbeatFailureCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    heartbeat_failure_callback_ = std::move(callback);
}

void SerialDriver::setDebugCallback(DebugCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    debug_callback_ = std::move(callback);
}

void SerialDriver::recvThreadFunc() {
    RCLCPP_DEBUG(serialLogger(), "接收线程启动：%s", port_.c_str());

    while (running_) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;  // 50ms

        int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            const int err = errno;
            if (!recv_error_active_) {
                recv_error_active_ = true;
                setLastError("select() 失败: errno=" + errnoText(err) + ", port=" + port_);
                RCLCPP_ERROR(serialLogger(), "串口接收失败：select(%s) errno=%d(%s)", port_.c_str(), err,
                             std::strerror(err));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (err == EBADF) {
                running_ = false;
                break;
            }
            continue;
        }
        if (ret == 0) {
            continue;
        }

        if (FD_ISSET(fd_, &read_fds)) {
            size_t available = RECV_BUFFER_SIZE - recv_len_;
            if (available > 0) {
                ssize_t n = ::read(fd_, &recv_buffer_[recv_len_], available);
                if (n > 0) {
                    if (recv_error_active_) {
                        recv_error_active_ = false;
                        RCLCPP_DEBUG(serialLogger(), "串口接收已恢复：%s", port_.c_str());
                    }
                    recv_len_ += static_cast<size_t>(n);
                    try {
                        parseReceivedData();
                    } catch (const std::exception& e) {
                        recv_len_ = 0;
                        setLastError(std::string("解析接收数据异常: ") + e.what());
                        RCLCPP_ERROR(serialLogger(), "解析接收数据异常：%s", e.what());
                    } catch (...) {
                        recv_len_ = 0;
                        setLastError("解析接收数据异常: 未知异常");
                        RCLCPP_ERROR(serialLogger(), "解析接收数据异常：未知异常");
                    }
                    continue;
                }

                if (n == 0) {
                    if (!recv_error_active_) {
                        recv_error_active_ = true;
                        setLastError("read() 返回 0：可能设备断开");
                        RCLCPP_WARN(serialLogger(), "串口读取返回 0：可能设备断开(%s)，触发重连", port_.c_str());
                    }
                    // 设备断开，退出接收线程并由外部触发重连
                    // 注意：不在此处直接调用 reconnect()，避免线程生命周期问题
                    requestReconnect("read_eof");
                    running_ = false;
                    break;
                }

                if (errno == EINTR) {
                    continue;
                }

                const int err = errno;
                if (!recv_error_active_) {
                    recv_error_active_ = true;
                    setLastError("read() 失败: errno=" + errnoText(err) + ", port=" + port_);
                    RCLCPP_ERROR(serialLogger(), "串口读取失败：port=%s, errno=%d(%s)", port_.c_str(), err,
                                 std::strerror(err));
                }
            }
        }
    }

    RCLCPP_DEBUG(serialLogger(), "接收线程退出：%s", port_.c_str());
}

void SerialDriver::parseReceivedData() {
    // HEAD(2) + SEQ(1) + LEN(1) + RETRY(1) + CMD(1) + CRC32(4) + TAIL(2) = 12 (最小帧，无payload)
    constexpr size_t MIN_FRAME_SIZE = 12;
    // frame_size = 11 + len，所以 FRAME_OVERHEAD = 11
    constexpr size_t FRAME_OVERHEAD = 11;

    while (recv_len_ >= MIN_FRAME_SIZE) {
        size_t head_pos = 0;
        bool found = false;
        for (size_t i = 0; i + 1 < recv_len_; ++i) {
            if (recv_buffer_[i] == FRAME_HEAD_0 && recv_buffer_[i + 1] == FRAME_HEAD_1) {
                head_pos = i;
                found = true;
                break;
            }
        }

        if (!found) {
            if (recv_len_ > 1) {
                recv_buffer_[0] = recv_buffer_[recv_len_ - 1];
                recv_len_ = 1;
            }

            ++parse_error_count_;
            comm_health_.parse_errors.fetch_add(1, std::memory_order_relaxed);
            if (parse_error_count_ <= 5 || (parse_error_count_ % 100 == 0)) {
                RCLCPP_DEBUG(serialLogger(), "解析失败：未找到帧头，已累计丢弃 %u 次", parse_error_count_);
            }
            return;
        }

        if (head_pos > 0) {
            std::memmove(recv_buffer_, &recv_buffer_[head_pos], recv_len_ - head_pos);
            recv_len_ -= head_pos;
        }

        // 需要至少4字节才能读取LEN字段(1字节)
        if (recv_len_ < 4) {
            return;
        }

        // LEN: 1字节，位置3
        uint8_t len = recv_buffer_[3];

        if (len < 1 || (static_cast<size_t>(len) + FRAME_OVERHEAD) > RECV_BUFFER_SIZE) {
            ++parse_error_count_;
            comm_health_.parse_errors.fetch_add(1, std::memory_order_relaxed);
            if (parse_error_count_ <= 5 || (parse_error_count_ % 100 == 0)) {
                RCLCPP_DEBUG(serialLogger(), "解析失败：长度字段异常 len=%u，已累计丢弃 %u 次", len,
                             parse_error_count_);
            }
            std::memmove(recv_buffer_, &recv_buffer_[2], recv_len_ - 2);
            recv_len_ -= 2;
            continue;
        }

        size_t frame_size = static_cast<size_t>(len) + FRAME_OVERHEAD;

        if (recv_len_ < frame_size) {
            return;
        }

        if (recv_buffer_[frame_size - 2] != FRAME_TAIL_0 || recv_buffer_[frame_size - 1] != FRAME_TAIL_1) {
            ++parse_error_count_;
            comm_health_.parse_errors.fetch_add(1, std::memory_order_relaxed);
            if (parse_error_count_ <= 5 || (parse_error_count_ % 100 == 0)) {
                RCLCPP_DEBUG(serialLogger(), "解析失败：帧尾错误，已累计丢弃 %u 次", parse_error_count_);
            }
            std::memmove(recv_buffer_, &recv_buffer_[2], recv_len_ - 2);
            recv_len_ -= 2;
            continue;
        }

        // CRC32校验: SEQ(1) + LEN(1) + RETRY(1) + CMD(1) + PAYLOAD(N)
        size_t crc_start = 2;
        size_t crc_len = 1 + 1 + 1 + len;
        uint32_t expected_crc = crc32_mpeg2_calculate(&recv_buffer_[crc_start], crc_len);
        uint32_t actual_crc = static_cast<uint32_t>(recv_buffer_[frame_size - 6]) |
                              (static_cast<uint32_t>(recv_buffer_[frame_size - 5]) << 8) |
                              (static_cast<uint32_t>(recv_buffer_[frame_size - 4]) << 16) |
                              (static_cast<uint32_t>(recv_buffer_[frame_size - 3]) << 24);

        if (expected_crc != actual_crc) {
            ++parse_error_count_;
            comm_health_.parse_errors.fetch_add(1, std::memory_order_relaxed);
            if (parse_error_count_ <= 5 || (parse_error_count_ % 100 == 0)) {
                RCLCPP_DEBUG(serialLogger(), "解析失败：CRC 错误(expected=0x%08X actual=0x%08X)，已累计丢弃 %u 次",
                             expected_crc, actual_crc, parse_error_count_);
            }
            std::memmove(recv_buffer_, &recv_buffer_[2], recv_len_ - 2);
            recv_len_ -= 2;
            continue;
        }

        // 调试回调：输出完整接收帧
        DebugCallback debug_cb;
        ReceiveCallback recv_cb;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            debug_cb = debug_callback_;
            recv_cb = recv_callback_;
        }

        if (debug_cb) {
            std::vector<uint8_t> rx_frame(recv_buffer_, recv_buffer_ + frame_size);
            debug_cb(false, rx_frame);
        }

        // SEQ在位置2，RETRY在位置4，CMD在位置5，PAYLOAD从位置6开始
        uint8_t seq = recv_buffer_[2];
        // uint8_t retry = recv_buffer_[4];  // 重发次数字段（当前未使用）
        uint8_t cmd = recv_buffer_[5];
        size_t payload_len = len - 1;
        std::vector<uint8_t> payload(payload_len);
        if (payload_len > 0) {
            std::memcpy(payload.data(), &recv_buffer_[6], payload_len);
        }

        notifyAck(seq, cmd);
        comm_health_.total_frames.fetch_add(1, std::memory_order_relaxed);

        if (recv_cb) {
            tl_in_recv_callback = true;
            try {
                recv_cb(seq, cmd, payload);
            } catch (const std::exception& e) {
                setLastError(std::string("接收回调异常: ") + e.what());
                RCLCPP_ERROR(serialLogger(), "接收回调抛异常：%s", e.what());
            } catch (...) {
                setLastError("接收回调异常: 未知异常");
                RCLCPP_ERROR(serialLogger(), "接收回调抛未知异常");
            }
            tl_in_recv_callback = false;
        }

        std::memmove(recv_buffer_, &recv_buffer_[frame_size], recv_len_ - frame_size);
        recv_len_ -= frame_size;
    }
}

}  // namespace rc26_decision
