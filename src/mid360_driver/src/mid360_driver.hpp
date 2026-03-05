/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#define ASIO_NO_DEPRECATED
#include <atomic>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include <asio.hpp>

namespace mid360_driver {

struct Point {
    float x, y, z;
    float intensity;
    double timestamp;
};

struct FrameMeta {
    double stamp_sec;
    uint8_t frame_cnt;
    uint32_t lost_in_frame;
    uint32_t lost_total;
};

struct FrameState {
    uint8_t current_frame_cnt = 0;
    uint16_t expected_udp_cnt = 0;
    double frame_min_ts = 0.0;
    uint32_t lost_in_frame = 0;
    uint32_t lost_total = 0;
    std::vector<Point> frame_points;
    bool has_data = false;
};

struct TimeSyncState {
    double delta = 0.0;
    bool initialized = false;
};

struct ImuMsg {
    double timestamp;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
};

struct IpAddressHasher {
    std::size_t operator()(const asio::ip::address& addr) const noexcept;
};

class Mid360Driver {
private:
    std::atomic<bool> is_running = true;
    asio::ip::address host_ip;
    asio::ip::udp::socket receive_pointcloud_socket;
    asio::ip::udp::socket receive_imu_socket;
    std::unordered_map<asio::ip::address, FrameState, IpAddressHasher> frame_state_map;
    std::unordered_map<asio::ip::address, TimeSyncState, IpAddressHasher> time_sync_map;
    std::function<void(const asio::ip::address& lidar_ip, const FrameMeta& meta, const std::vector<Point>& points)>
        on_receive_pointcloud;
    std::function<void(const asio::ip::address& lidar_ip, const ImuMsg& imu_msg)> on_receive_imu;

public:
    Mid360Driver(asio::io_context& io_context, const asio::ip::address& host_ip,
                 const std::function<void(const asio::ip::address& lidar_ip, const FrameMeta& meta,
                                          const std::vector<Point>& points)>& on_receive_pointcloud,
                 const std::function<void(const asio::ip::address& lidar_ip, const ImuMsg& imu_msg)>& on_receive_imu);

    ~Mid360Driver();

    void stop();

    asio::awaitable<void> receive_pointcloud();

    asio::awaitable<void> receive_imu();
};

}  // namespace mid360_driver
