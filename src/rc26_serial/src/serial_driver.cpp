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
#include <sys/epoll.h>
#include <termios.h>
#include <unistd.h>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

namespace {
rclcpp::Logger serialLogger() {
    return rclcpp::get_logger("rc26_decision.serial_driver");
}

bool isFatalSerialError(int err) {
    switch (err) {
    case EBADF:
    case EIO:
    case ENODEV:
    case ENXIO:
    case EPIPE:
        return true;
    default:
        return false;
    }
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
    auto_reconnect_enabled_.store(false, std::memory_order_release);
    reconnect_requested_.store(false, std::memory_order_relaxed);
    reconnect_thread_running_ = false;
    reconnect_cv_.notify_all();
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
    closePort();
}

std::string SerialDriver::lastError() const {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    return last_error_;
}

void SerialDriver::setLastError(std::string message) {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    last_error_ = std::move(message);
}

bool SerialDriver::isLinkActive() const {
    return fd_ >= 0 && running_.load(std::memory_order_acquire);
}

void SerialDriver::invokeCallbackSafely(const char* name, const std::function<void()>& callback) {
    if (!callback) {
        return;
    }

    try {
        callback();
    } catch (const std::exception& e) {
        setLastError(std::string(name) + " 回调异常: " + e.what());
        RCLCPP_ERROR(serialLogger(), "%s 回调抛异常：%s", name, e.what());
    } catch (...) {
        setLastError(std::string(name) + " 回调异常: 未知异常");
        RCLCPP_ERROR(serialLogger(), "%s 回调抛未知异常", name);
    }
}

void SerialDriver::invokeDebugCallback(bool is_tx, const std::vector<uint8_t>& data, const DebugCallback& callback) {
    if (!callback) {
        return;
    }

    try {
        callback(is_tx, data);
    } catch (const std::exception& e) {
        setLastError(std::string("Debug") + " 回调异常: " + e.what());
        RCLCPP_ERROR(serialLogger(), "Debug 回调抛异常：%s", e.what());
    } catch (...) {
        setLastError("Debug 回调异常: 未知异常");
        RCLCPP_ERROR(serialLogger(), "Debug 回调抛未知异常");
    }
}

bool SerialDriver::open(const std::string& port, int baudrate) {
    const bool cold_start = port_.empty();

    if (fd_ >= 0) {
        closePort();
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

    auto_reconnect_enabled_.store(true, std::memory_order_release);

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

    ring_parser_.reset();
    if (cold_start) {
        comm_health_.total_frames.store(0, std::memory_order_relaxed);
        comm_health_.parse_errors.store(0, std::memory_order_relaxed);
        comm_health_.ack_timeouts.store(0, std::memory_order_relaxed);
        comm_health_.reconnect_count.store(0, std::memory_order_relaxed);
        comm_health_.heartbeat_failures.store(0, std::memory_order_relaxed);
        comm_health_.resetWindows();
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
    auto_reconnect_enabled_.store(false, std::memory_order_release);
    reconnect_requested_.store(false, std::memory_order_relaxed);
    reconnect_cv_.notify_all();

    closePort();
}

void SerialDriver::closePort() {
    running_ = false;
    link_epoch_.fetch_add(1, std::memory_order_relaxed);
    ack_cv_.notify_all();

    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    if (fd_ >= 0) {
        RCLCPP_DEBUG(serialLogger(), "关闭串口：%s", port_.c_str());
        ::close(fd_);
        fd_ = -1;
    }

    ring_parser_.reset();
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
    invokeCallbackSafely("HeartbeatFailure", cb);
}

void SerialDriver::notifyReconnect() {
    ReconnectCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = reconnect_callback_;
    }
    invokeCallbackSafely("Reconnect", cb);
}

void SerialDriver::notifyReconnectFailed() {
    ReconnectFailedCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = reconnect_failed_callback_;
    }
    invokeCallbackSafely("ReconnectFailed", cb);
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
        if (reconnect_requested_.exchange(false) && auto_reconnect_enabled_.load(std::memory_order_acquire)) {
            reconnect();
        }
    }
}

void SerialDriver::requestReconnect(const char* reason) {
    link_epoch_.fetch_add(1, std::memory_order_relaxed);
    ack_cv_.notify_all();

    if (!auto_reconnect_enabled_.load(std::memory_order_acquire) || !reconnect_thread_running_.load()) {
        return;
    }
    if (reconnecting_.load(std::memory_order_acquire)) {
        RCLCPP_DEBUG(serialLogger(), "重连进行中，忽略重复重连请求: %s", reason);
        return;
    }
    if (reconnect_requested_.exchange(true)) {
        RCLCPP_DEBUG(serialLogger(), "重连请求已排队，忽略重复请求: %s", reason);
        return;
    }

    RCLCPP_WARN(serialLogger(), "请求重连: %s", reason);
    reconnect_cv_.notify_one();
}

bool SerialDriver::waitReconnectInterval() {
    std::unique_lock<std::mutex> lock(reconnect_cv_mutex_);
    const bool interrupted = reconnect_cv_.wait_for(lock, std::chrono::milliseconds(RECONNECT_INTERVAL_MS), [this]() {
        return !reconnect_thread_running_.load() || !auto_reconnect_enabled_.load(std::memory_order_acquire);
    });
    return !interrupted;
}

bool SerialDriver::reconnect() {
    // 防止并发重连
    bool expected = false;
    if (!reconnecting_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(serialLogger(), "重连已在进行中，跳过本次重连请求");
        return false;
    }

    comm_health_.recordReconnect();

    RCLCPP_WARN(serialLogger(), "开始串口重连：%s @ %d", port_.c_str(), baudrate_);

    {
        ReconnectStartCallback cb;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            cb = reconnect_start_callback_;
        }
        invokeCallbackSafely("ReconnectStart", cb);
    }

    for (uint8_t attempt = 1; attempt <= MAX_RECONNECT_ATTEMPTS; ++attempt) {
        if (!reconnect_thread_running_.load() || !auto_reconnect_enabled_.load(std::memory_order_acquire)) {
            reconnecting_ = false;
            return false;
        }

        closePort();
        if (!waitReconnectInterval()) {
            reconnecting_ = false;
            return false;
        }

        if (!reconnect_thread_running_.load() || !auto_reconnect_enabled_.load(std::memory_order_acquire)) {
            reconnecting_ = false;
            return false;
        }

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

        if (written == 0) {
            if (!write_error_active_) {
                write_error_active_ = true;
                setLastError("write() 返回 0：可能设备断开");
                RCLCPP_ERROR(serialLogger(), "串口写入失败：port=%s, size=%zu, write() 返回 0", port_.c_str(), size);
                if (running_.load(std::memory_order_acquire)) {
                    requestReconnect("write_zero");
                }
            }
            return false;
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
            if (running_.load(std::memory_order_acquire)) {
                requestReconnect("write_failure");
            }
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
    waiting_epoch_ = link_epoch_.load(std::memory_order_relaxed);
}

void SerialDriver::endWaitAck() {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    waiting_for_ack_ = false;
    ack_response_received_ = false;
    ack_success_ = false;
}

SerialDriver::AckWaitResult SerialDriver::waitForAck(std::chrono::milliseconds timeout, bool& success) {
    std::unique_lock<std::mutex> lock(ack_mutex_);
    const uint32_t epoch_snapshot = waiting_epoch_;
    bool woken = ack_cv_.wait_for(lock, timeout, [this, epoch_snapshot]() {
        return ack_response_received_ || link_epoch_.load(std::memory_order_acquire) != epoch_snapshot;
    });
    if (!woken) {
        success = false;
        return AckWaitResult::kTimeout;
    }

    if (link_epoch_.load(std::memory_order_acquire) != epoch_snapshot) {
        success = false;
        return AckWaitResult::kLinkDown;
    }

    success = ack_success_;
    return AckWaitResult::kReceived;
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
    uint8_t ignored_seq = 0;
    return sendCommandNoAck(cmd, payload, ignored_seq);
}

bool SerialDriver::sendCommandNoAck(CommandID cmd, const std::vector<uint8_t>& payload) {
    uint8_t ignored_seq = 0;
    return sendCommandNoAck(cmd, payload, ignored_seq);
}

bool SerialDriver::sendCommandNoAck(uint8_t cmd, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    if (!isLinkActive()) {
        setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
        return false;
    }

    out_seq = nextSeq();
    std::vector<uint8_t> frame = buildFrame(out_seq, cmd, payload, 0x00);
    if (frame.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!isLinkActive()) {
        setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
        return false;
    }
    bool ok = writeAll(frame.data(), frame.size());
    if (ok) {
        DebugCallback cb;
        {
            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            cb = debug_callback_;
        }
        invokeDebugCallback(true, frame, cb);
    }
    return ok;
}

bool SerialDriver::sendCommandNoAck(CommandID cmd, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    return sendCommandNoAck(static_cast<uint8_t>(cmd), payload, out_seq);
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

    if (!isLinkActive()) {
        setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
        return false;
    }

    std::lock_guard<std::mutex> cmd_lock(ack_command_mutex_);

    // 重发机制：retry从0x00开始，步长1，每100ms重发一次
    // 每3次为1轮，达到0x09后触发重连
    out_seq = nextSeq();
    for (uint8_t retry = 0x00; retry <= MAX_RETRY_VALUE; ++retry) {
        if (!isLinkActive()) {
            setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
            return false;
        }

        std::vector<uint8_t> frame = buildFrame(out_seq, cmd, payload, retry);
        if (frame.empty()) {
            return false;
        }

        beginWaitAck(out_seq, cmd);

        {
            std::lock_guard<std::mutex> lock(send_mutex_);
            if (!isLinkActive()) {
                endWaitAck();
                setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
                return false;
            }
            if (!writeAll(frame.data(), frame.size())) {
                endWaitAck();
                return false;
            }
            DebugCallback cb;
            {
                std::lock_guard<std::mutex> cb_lock(callback_mutex_);
                cb = debug_callback_;
            }
            invokeDebugCallback(true, frame, cb);
        }

        std::chrono::milliseconds ack_timeout{ACK_TIMEOUT_MS};
        {
            std::lock_guard<std::mutex> lock(timeout_mutex_);
            ack_timeout = adaptive_timeout_.get();
        }

        const auto wait_start = std::chrono::steady_clock::now();
        bool ack_success = false;
        AckWaitResult wait_result = waitForAck(ack_timeout, ack_success);
        const auto measured_ms =
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - wait_start).count();
        if (wait_result == AckWaitResult::kReceived) {
            comm_health_.ack_window.record(true);
        }
        if (wait_result == AckWaitResult::kReceived && ack_success) {
            {
                std::lock_guard<std::mutex> lock(timeout_mutex_);
                adaptive_timeout_.update(measured_ms);
            }
            endWaitAck();
            return true;
        }

        endWaitAck();
        if (wait_result == AckWaitResult::kLinkDown) {
            return false;
        }
        if (wait_result == AckWaitResult::kTimeout) {
            comm_health_.ack_window.record(false);
            comm_health_.ack_timeouts.fetch_add(1, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lock(timeout_mutex_);
                adaptive_timeout_.onTimeout();
            }
        }

        uint8_t round = retry / RETRIES_PER_ROUND + 1;
        if (wait_result == AckWaitResult::kReceived) {
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

bool SerialDriver::sendPose(CommandID cmd, float vx, float vy, float wz, uint8_t& out_seq) {
    std::vector<uint8_t> payload(12);
    std::memcpy(&payload[0], &vx, sizeof(float));
    std::memcpy(&payload[4], &vy, sizeof(float));
    std::memcpy(&payload[8], &wz, sizeof(float));
    return sendCommandNoAck(cmd, payload, out_seq);
}

bool SerialDriver::sendPose(CommandID cmd, float vx, float vy, float wz) {
    uint8_t ignored_seq = 0;
    return sendPose(cmd, vx, vy, wz, ignored_seq);
}

bool SerialDriver::sendStop() {
    return sendCommand(CommandID::STOP);
}

bool SerialDriver::sendHeartbeat() {
    if (!isLinkActive()) {
        setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
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
        if (!isLinkActive()) {
            endWaitAck();
            setLastError(fd_ < 0 ? "串口未打开" : "串口连接未激活");
            return false;
        }
        if (!writeAll(frame.data(), frame.size())) {
            endWaitAck();
            return false;
        }
        DebugCallback cb;
        {
            std::lock_guard<std::mutex> cb_lock(callback_mutex_);
            cb = debug_callback_;
        }
        invokeDebugCallback(true, frame, cb);
    }

    std::chrono::milliseconds ack_timeout{ACK_TIMEOUT_MS};
    {
        std::lock_guard<std::mutex> lock(timeout_mutex_);
        ack_timeout = adaptive_timeout_.get();
    }

    const auto wait_start = std::chrono::steady_clock::now();
    bool ack_success = false;
    AckWaitResult wait_result = waitForAck(ack_timeout, ack_success);
    const auto measured_ms =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - wait_start).count();
    endWaitAck();

    if (wait_result == AckWaitResult::kReceived) {
        comm_health_.ack_window.record(true);
    }

    if (wait_result == AckWaitResult::kReceived && ack_success) {
        {
            std::lock_guard<std::mutex> lock(timeout_mutex_);
            adaptive_timeout_.update(measured_ms);
        }
        heartbeat_failure_count_ = 0;
        RCLCPP_DEBUG(serialLogger(), "心跳成功：seq=%u", seq);
        return true;
    }

    if (wait_result == AckWaitResult::kLinkDown) {
        return false;
    }
    if (wait_result == AckWaitResult::kTimeout) {
        comm_health_.ack_window.record(false);
        comm_health_.ack_timeouts.fetch_add(1, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(timeout_mutex_);
            adaptive_timeout_.onTimeout();
        }
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

void SerialDriver::dispatchFrame(uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
    DebugCallback debug_cb;
    ReceiveCallback recv_cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        debug_cb = debug_callback_;
        recv_cb = recv_callback_;
    }

    if (debug_cb) {
        const uint8_t len = static_cast<uint8_t>(1 + plen);
        const size_t frame_size = 2 + 1 + 1 + 1 + 1 + plen + 4 + 2;
        std::vector<uint8_t> frame(frame_size);
        size_t idx = 0;
        frame[idx++] = FRAME_HEAD_0;
        frame[idx++] = FRAME_HEAD_1;
        frame[idx++] = seq;
        frame[idx++] = len;
        frame[idx++] = 0x00;
        frame[idx++] = cmd;
        for (size_t i = 0; i < plen; ++i) {
            frame[idx++] = payload[i];
        }
        const size_t crc_start = 2;
        const size_t crc_len = 1 + 1 + 1 + static_cast<size_t>(len);
        const uint32_t crc = crc32_mpeg2_calculate(&frame[crc_start], crc_len);
        frame[idx++] = static_cast<uint8_t>(crc & 0xFF);
        frame[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        frame[idx++] = static_cast<uint8_t>((crc >> 16) & 0xFF);
        frame[idx++] = static_cast<uint8_t>((crc >> 24) & 0xFF);
        frame[idx++] = FRAME_TAIL_0;
        frame[idx++] = FRAME_TAIL_1;
        invokeDebugCallback(false, frame, debug_cb);
    }

    notifyAck(seq, cmd);
    comm_health_.parse_window.record(true);
    comm_health_.total_frames.fetch_add(1, std::memory_order_relaxed);

    if (recv_cb) {
        tl_in_recv_callback = true;
        try {
            recv_cb(seq, cmd, std::vector<uint8_t>(payload, payload + plen));
        } catch (const std::exception& e) {
            setLastError(std::string("接收回调异常: ") + e.what());
            RCLCPP_ERROR(serialLogger(), "接收回调抛异常：%s", e.what());
        } catch (...) {
            setLastError("接收回调异常: 未知异常");
            RCLCPP_ERROR(serialLogger(), "接收回调抛未知异常");
        }
        tl_in_recv_callback = false;
    }
}

void SerialDriver::recvThreadFunc() {
    RCLCPP_DEBUG(serialLogger(), "接收线程启动：%s", port_.c_str());

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        const int err = errno;
        setLastError("epoll_create1() 失败: errno=" + errnoText(err) + ", port=" + port_);
        RCLCPP_ERROR(serialLogger(), "串口接收失败：epoll_create1(%s) errno=%d(%s)", port_.c_str(), err,
                     std::strerror(err));
        running_ = false;
        return;
    }

    struct epoll_event ev {};
    ev.events = EPOLLIN | EPOLLERR | EPOLLHUP;
    ev.data.fd = fd_;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd_, &ev) != 0) {
        const int err = errno;
        setLastError("epoll_ctl() 失败: errno=" + errnoText(err) + ", port=" + port_);
        RCLCPP_ERROR(serialLogger(), "串口接收失败：epoll_ctl(%s) errno=%d(%s)", port_.c_str(), err, std::strerror(err));
        ::close(epfd);
        running_ = false;
        return;
    }

    while (running_) {
        struct epoll_event events[1];
        const int ret = epoll_wait(epfd, events, 1, 50);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            const int err = errno;
            if (!recv_error_active_) {
                recv_error_active_ = true;
                setLastError("epoll_wait() 失败: errno=" + errnoText(err) + ", port=" + port_);
                RCLCPP_ERROR(serialLogger(), "串口接收失败：epoll_wait(%s) errno=%d(%s)", port_.c_str(), err,
                             std::strerror(err));
            }

            if (err == EBADF) {
                running_ = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (ret == 0) {
            continue;
        }

        if (events[0].events & (EPOLLERR | EPOLLHUP)) {
            requestReconnect("epoll_err_hup");
            running_ = false;
            break;
        }

        if (!(events[0].events & EPOLLIN)) {
            continue;
        }

        uint8_t tmp_buf[512];
        ssize_t n = ::read(fd_, tmp_buf, sizeof(tmp_buf));
        if (n > 0) {
            if (recv_error_active_) {
                recv_error_active_ = false;
                RCLCPP_DEBUG(serialLogger(), "串口接收已恢复：%s", port_.c_str());
            }

            try {
                ring_parser_.push(tmp_buf, static_cast<size_t>(n));
                ring_parser_.parse([this](uint8_t seq, uint8_t cmd, const uint8_t* payload, size_t plen) {
                    dispatchFrame(seq, cmd, payload, plen);
                });
                const auto delta = ring_parser_.consumeStats();
                const uint32_t parse_error_count = delta.len_invalid + delta.tail_bad + delta.crc_bad + delta.head_drop +
                                                   delta.overflow_drop;
                if (parse_error_count > 0U) {
                    comm_health_.parse_errors.fetch_add(parse_error_count, std::memory_order_relaxed);
                    for (uint32_t i = 0; i < parse_error_count; ++i) {
                        comm_health_.parse_window.record(false);
                    }
                }
            } catch (const std::exception& e) {
                ring_parser_.reset();
                setLastError(std::string("解析接收数据异常: ") + e.what());
                RCLCPP_ERROR(serialLogger(), "解析接收数据异常：%s", e.what());
            } catch (...) {
                ring_parser_.reset();
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
            RCLCPP_ERROR(serialLogger(), "串口读取失败：port=%s, errno=%d(%s)", port_.c_str(), err, std::strerror(err));
        }

        if (err == EAGAIN || err == EWOULDBLOCK) {
            continue;
        }

        if (isFatalSerialError(err)) {
            requestReconnect("read_error");
            running_ = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ::close(epfd);
    RCLCPP_DEBUG(serialLogger(), "接收线程退出：%s", port_.c_str());
}

}  // namespace rc26_decision
