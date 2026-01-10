// 达妙 DM-IMU-L1 六轴IMU驱动实现
#include "rc26_merge_odom/dm_imu_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>

namespace dm_imu
{

namespace
{
rclcpp::Logger dmImuLogger()
{
    return rclcpp::get_logger("dm_imu.driver");
}

bool toTermiosBaudrate(int baudrate, speed_t& baud)
{
    switch (baudrate)
    {
        case 9600:    baud = B9600;    return true;
        case 19200:   baud = B19200;   return true;
        case 38400:   baud = B38400;   return true;
        case 57600:   baud = B57600;   return true;
        case 115200:  baud = B115200;  return true;
        case 230400:  baud = B230400;  return true;
        case 460800:  baud = B460800;  return true;
        case 500000:  baud = B500000;  return true;
        case 576000:  baud = B576000;  return true;
        case 921600:  baud = B921600;  return true;
        case 1000000: baud = B1000000; return true;
        case 1152000: baud = B1152000; return true;
        case 1500000: baud = B1500000; return true;
        case 2000000: baud = B2000000; return true;
        case 2500000: baud = B2500000; return true;
        case 3000000: baud = B3000000; return true;
        case 3500000: baud = B3500000; return true;
        case 4000000: baud = B4000000; return true;
        default:     return false;
    }
}

std::string errnoText(int err)
{
    return std::to_string(err) + " (" + std::string(std::strerror(err)) + ")";
}

double nowSeconds()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}
}  // namespace

DmImuDriver::DmImuDriver() = default;

DmImuDriver::~DmImuDriver()
{
    close();
}

std::string DmImuDriver::lastError() const
{
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    return last_error_;
}

void DmImuDriver::setLastError(std::string message)
{
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    last_error_ = std::move(message);
}

bool DmImuDriver::open(const std::string& port, int baudrate)
{
    if (fd_ >= 0)
    {
        close();
    }

    if (port.empty())
    {
        setLastError("serial_port 不能为空");
        RCLCPP_ERROR(dmImuLogger(), "打开串口失败：serial_port 为空");
        return false;
    }

    speed_t baud;
    if (!toTermiosBaudrate(baudrate, baud))
    {
        setLastError("不支持的 baudrate: " + std::to_string(baudrate));
        RCLCPP_ERROR(dmImuLogger(), "打开串口失败：不支持的 baudrate=%d", baudrate);
        return false;
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
        const int err = errno;
        setLastError("open() 失败: errno=" + errnoText(err) + ", port=" + port);
        RCLCPP_ERROR(dmImuLogger(), "打开串口失败：open(%s) errno=%d(%s)",
            port.c_str(), err, std::strerror(err));
        return false;
    }

    port_ = port;

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0)
    {
        const int err = errno;
        setLastError("tcgetattr() 失败: errno=" + errnoText(err));
        RCLCPP_ERROR(dmImuLogger(), "配置串口失败：tcgetattr errno=%d", err);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0)
    {
        const int err = errno;
        setLastError("cfsetispeed/cfsetospeed 失败");
        RCLCPP_ERROR(dmImuLogger(), "配置串口失败：设置波特率失败 errno=%d", err);
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
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        const int err = errno;
        setLastError("tcsetattr() 失败");
        RCLCPP_ERROR(dmImuLogger(), "配置串口失败：tcsetattr errno=%d", err);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 清空输入缓冲
    tcflush(fd_, TCIFLUSH);

    recv_len_ = 0;
    cnt_ok_ = 0;
    cnt_crc_ = 0;
    cnt_short_ = 0;
    cnt_nohdr_ = 0;
    recv_error_active_ = false;

    // 初始化数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = ImuData{};
    }

    try
    {
        running_ = true;
        recv_thread_ = std::thread(&DmImuDriver::recvThreadFunc, this);
    }
    catch (const std::exception& e)
    {
        running_ = false;
        setLastError(std::string("启动接收线程失败: ") + e.what());
        RCLCPP_ERROR(dmImuLogger(), "启动接收线程失败：%s", e.what());
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    setLastError({});
    RCLCPP_INFO(dmImuLogger(), "达妙IMU串口打开成功：%s @ %d", port_.c_str(), baudrate);

    return true;
}

void DmImuDriver::close()
{
    running_ = false;

    if (recv_thread_.joinable())
    {
        recv_thread_.join();
    }

    if (fd_ >= 0)
    {
        RCLCPP_DEBUG(dmImuLogger(), "关闭串口：%s", port_.c_str());
        ::close(fd_);
        fd_ = -1;
    }

    recv_len_ = 0;
}

ImuData DmImuDriver::getLatestData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}

void DmImuDriver::setDataCallback(DataCallback callback)
{
    data_callback_ = std::move(callback);
}

DmImuDriver::Stats DmImuDriver::getStats() const
{
    return Stats{
        cnt_ok_.load(),
        cnt_crc_.load(),
        cnt_short_.load(),
        cnt_nohdr_.load()
    };
}

void DmImuDriver::recvThreadFunc()
{
    RCLCPP_DEBUG(dmImuLogger(), "接收线程启动：%s", port_.c_str());

    while (running_)
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;  // 10ms

        int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }

            const int err = errno;
            if (!recv_error_active_)
            {
                recv_error_active_ = true;
                setLastError("select() 失败: errno=" + errnoText(err));
                RCLCPP_ERROR(dmImuLogger(), "串口接收失败：select errno=%d", err);
            }

            if (err == EBADF)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        if (ret == 0)
        {
            continue;
        }

        if (FD_ISSET(fd_, &read_fds))
        {
            size_t available = RECV_BUFFER_SIZE - recv_len_;
            if (available > 0)
            {
                ssize_t n = ::read(fd_, &recv_buffer_[recv_len_], available);
                if (n > 0)
                {
                    if (recv_error_active_)
                    {
                        recv_error_active_ = false;
                        RCLCPP_DEBUG(dmImuLogger(), "串口接收已恢复");
                    }
                    recv_len_ += static_cast<size_t>(n);
                    parseReceivedData();
                    continue;
                }

                if (n == 0)
                {
                    if (!recv_error_active_)
                    {
                        recv_error_active_ = true;
                        setLastError("read() 返回 0：可能设备断开");
                        RCLCPP_WARN(dmImuLogger(), "串口读取返回 0：可能设备断开");
                    }
                    continue;
                }

                if (errno == EINTR)
                {
                    continue;
                }

                const int err = errno;
                if (!recv_error_active_)
                {
                    recv_error_active_ = true;
                    setLastError("read() 失败: errno=" + errnoText(err));
                    RCLCPP_ERROR(dmImuLogger(), "串口读取失败：errno=%d", err);
                }
            }
        }
    }

    RCLCPP_DEBUG(dmImuLogger(), "接收线程退出：%s", port_.c_str());
}

void DmImuDriver::parseReceivedData()
{
    while (recv_len_ >= DM_FRAME_LEN)
    {
        // 查找帧头 0x55 0xAA
        size_t head_pos = 0;
        bool found = false;
        for (size_t i = 0; i + 1 < recv_len_; ++i)
        {
            if (recv_buffer_[i] == DM_FRAME_HEAD_0 && 
                recv_buffer_[i + 1] == DM_FRAME_HEAD_1)
            {
                head_pos = i;
                found = true;
                break;
            }
        }

        if (!found)
        {
            if (recv_len_ > 1)
            {
                recv_buffer_[0] = recv_buffer_[recv_len_ - 1];
                recv_len_ = 1;
            }
            ++cnt_nohdr_;
            return;
        }

        // 丢弃帧头之前的数据
        if (head_pos > 0)
        {
            std::memmove(recv_buffer_, &recv_buffer_[head_pos], recv_len_ - head_pos);
            recv_len_ -= head_pos;
        }

        // 检查是否有完整帧
        if (recv_len_ < DM_FRAME_LEN)
        {
            ++cnt_short_;
            return;
        }

        // 检查帧尾
        if (recv_buffer_[DM_FRAME_LEN - 1] != DM_FRAME_TAIL)
        {
            // 帧尾不对，跳过这个帧头继续找
            std::memmove(recv_buffer_, &recv_buffer_[2], recv_len_ - 2);
            recv_len_ -= 2;
            continue;
        }

        // 解析帧
        if (parseFrame(recv_buffer_, DM_FRAME_LEN))
        {
            ++cnt_ok_;
        }

        // 移除已解析的帧
        std::memmove(recv_buffer_, &recv_buffer_[DM_FRAME_LEN], recv_len_ - DM_FRAME_LEN);
        recv_len_ -= DM_FRAME_LEN;
    }
}

bool DmImuDriver::parseFrame(const uint8_t* frame, size_t len)
{
    if (len != DM_FRAME_LEN)
    {
        return false;
    }

    // 帧结构: HEAD(2) + ID(1) + RID(1) + DATA(12) + CRC16(2) + TAIL(1)
    uint8_t rid = frame[3];
    
    // 检查RID有效性
    if (rid < 0x01 || rid > 0x04)
    {
        return false;
    }

    // CRC16校验 (对前16字节计算)
    uint16_t crc_calc = dm_crc16(frame, 16);
    uint16_t crc_wire = static_cast<uint16_t>(frame[16]) | 
                        (static_cast<uint16_t>(frame[17]) << 8);
    
    if (crc_calc != crc_wire)
    {
        ++cnt_crc_;
        return false;
    }

    // 解析3个float数据 (小端序)
    float f1, f2, f3;
    std::memcpy(&f1, &frame[4], sizeof(float));
    std::memcpy(&f2, &frame[8], sizeof(float));
    std::memcpy(&f3, &frame[12], sizeof(float));

    double now = nowSeconds();

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        switch (static_cast<DataType>(rid))
        {
            case DataType::ACCEL:
                latest_data_.accel[0] = f1;
                latest_data_.accel[1] = f2;
                latest_data_.accel[2] = f3;
                latest_data_.accel_valid = true;
                latest_data_.accel_ts = now;
                break;
                
            case DataType::GYRO:
                latest_data_.gyro[0] = f1;
                latest_data_.gyro[1] = f2;
                latest_data_.gyro[2] = f3;
                latest_data_.gyro_valid = true;
                latest_data_.gyro_ts = now;
                break;
                
            case DataType::EULER:
                latest_data_.euler[0] = f1;  // Roll
                latest_data_.euler[1] = f2;  // Pitch
                latest_data_.euler[2] = f3;  // Yaw
                latest_data_.euler_valid = true;
                latest_data_.euler_ts = now;
                break;
                
            case DataType::QUATERNION:
                // 四元数帧格式不同，这里暂不处理
                break;
        }
    }

    // 调用回调
    if (data_callback_)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        data_callback_(latest_data_);
    }

    return true;
}

}  // namespace dm_imu
