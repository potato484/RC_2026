/*
 * @Author: potato potato@potato.com
 * @Date: 2026-01-05 15:06:55
 * @LastEditors: potato potato@potato.com
 * @LastEditTime: 2026-01-05 15:23:47
 * @FilePath: /RC_2026/RC_2026_1/rc26_merge_odom/include/rc26_merge_odom/dm_imu_driver.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 达妙 DM-IMU-L1 六轴IMU驱动
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "rc26_merge_odom/dm_protocol.hpp"

namespace dm_imu
{

class DmImuDriver
{
public:
    using DataCallback = std::function<void(const ImuData&)>;

    DmImuDriver();
    ~DmImuDriver();

    DmImuDriver(const DmImuDriver&) = delete;
    DmImuDriver& operator=(const DmImuDriver&) = delete;

    // ========================================================================
    // 连接管理
    // ========================================================================
    bool open(const std::string& port, int baudrate = DM_DEFAULT_BAUDRATE);
    void close();
    bool isOpen() const { return fd_ >= 0 && running_.load(); }
    std::string lastError() const;

    // ========================================================================
    // 数据获取
    // ========================================================================
    ImuData getLatestData() const;
    void setDataCallback(DataCallback callback);

    // ========================================================================
    // 统计信息
    // ========================================================================
    struct Stats
    {
        uint32_t frames_ok;
        uint32_t frames_crc_error;
        uint32_t frames_short;
        uint32_t frames_no_header;
    };
    Stats getStats() const;

private:
    int fd_ = -1;
    std::string port_;

    std::atomic<bool> running_{false};
    std::thread recv_thread_;

    mutable std::mutex data_mutex_;
    ImuData latest_data_{};

    mutable std::mutex callback_mutex_;
    DataCallback data_callback_;

    mutable std::mutex last_error_mutex_;
    std::string last_error_;

    static constexpr size_t RECV_BUFFER_SIZE = 256;
    uint8_t recv_buffer_[RECV_BUFFER_SIZE];
    size_t recv_len_ = 0;

    std::atomic<uint32_t> cnt_ok_{0};
    std::atomic<uint32_t> cnt_crc_{0};
    std::atomic<uint32_t> cnt_short_{0};
    std::atomic<uint32_t> cnt_nohdr_{0};

    bool recv_error_active_{false};

    void setLastError(std::string message);
    void recvThreadFunc();
    void parseReceivedData();
    bool parseFrame(const uint8_t* frame, size_t len);
};

}  // namespace dm_imu
