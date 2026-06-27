/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 * Maintained by DongXuan Chen <2220362462@qq.com>
 */

#define _USE_MATH_DEFINES
#include "mid360_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace mid360_driver {
namespace {

constexpr std::size_t kUdpPacketBufferSize = 1400;
constexpr int kUdpReceiveBufferBytes = 16 * 1024 * 1024;
constexpr std::size_t kMaxFramePointCount = 200000;
constexpr std::uint16_t kPointcloudReceivePort = 56301;
constexpr std::uint16_t kPointcloudSendPort = 56300;
constexpr std::uint16_t kImuReceivePort = 56401;
constexpr std::uint16_t kImuSendPort = 56400;
constexpr std::uint8_t kSupportedProtocolVersion = 0;

bool has_valid_point_tag(const std::uint8_t tag) noexcept {
    return (tag & 0b00110000) == 0b00000000 && (tag & 0b00001100) == 0b00000000 &&
           (tag & 0b00000011) == 0b00000000;
}

bool should_exit_receive_loop(const asio::error_code& error_code, const bool running) noexcept {
    return !running || error_code == asio::error::operation_aborted || error_code == asio::error::bad_descriptor ||
           error_code == asio::error::not_socket;
}

void reset_frame_state(FrameState& frame_state) {
    frame_state.expected_udp_cnt = 0;
    frame_state.frame_min_ts = 0.0;
    frame_state.lost_in_frame = 0;
    frame_state.frame_points.clear();
    frame_state.drop_current_frame = false;
    frame_state.has_data = false;
}

void start_new_frame(FrameState& frame_state, const std::uint8_t frame_cnt, const std::uint16_t initial_udp_cnt) {
    frame_state.current_frame_cnt = frame_cnt;
    frame_state.expected_udp_cnt = initial_udp_cnt;
    frame_state.frame_min_ts = std::numeric_limits<double>::max();
    frame_state.lost_in_frame = 0;
    frame_state.frame_points.clear();
    frame_state.drop_current_frame = false;
    frame_state.has_data = true;
}

void finalize_frame(
    const asio::ip::address& lidar_ip, FrameState& frame_state,
    const std::function<void(const asio::ip::address& lidar_ip, const FrameMeta& meta, const std::vector<Point>& points)>&
        on_receive_pointcloud) {
    if (!frame_state.has_data) {
        return;
    }

    if (!frame_state.drop_current_frame && !frame_state.frame_points.empty() &&
        frame_state.frame_min_ts != std::numeric_limits<double>::max()) {
        FrameMeta meta{frame_state.frame_min_ts, frame_state.current_frame_cnt, frame_state.lost_in_frame,
                       frame_state.lost_total};
        on_receive_pointcloud(lidar_ip, meta, frame_state.frame_points);
    }

    reset_frame_state(frame_state);
}

}  // namespace

enum DataType : std::uint8_t {
    kLivoxLidarImuData = 0,
    kLivoxLidarCartesianCoordinateHighData = 0x01,
    kLivoxLidarCartesianCoordinateLowData = 0x02,
    kLivoxLidarSphericalCoordinateData = 0x03
};

enum TimestampType : std::uint8_t {
    kTimestampTypeNoSync = 0,     // 没有同步信号
    kTimestampTypeGptpOrPtp = 1,  // gPTP 或 PTP 同步模式
    kTimestampTypeGps = 2         // GPS 同步模式
};

#pragma pack(1)

struct DataHeader {
    uint8_t version;  // 包协议版本：当前为0
    uint16_t length;  // 整个UDP数据段长度
    uint16_t time_interval;  // 帧内点云采样时间(单位0.1us)；这帧点云数据中最后一个点减去第一个点时间
    uint16_t dot_num;         // 当前UDP包的点数目
    uint16_t udp_cnt;         // 点云UDP包计数，每个UDP包依次加1，点云帧起始包清0
    uint8_t frame_cnt;        // 点云帧计数，每帧点云(10Hz/15Hz等)依次加1
    DataType data_type;       // 数据类型
    TimestampType time_type;  // 时间戳类型
    uint8_t reserved[12];     // 保留
    uint32_t crc32;           // timestamp+点云数据校验码，使用CRC-32算法
    uint64_t timestamp;       // 点云时间戳，单位: ns
};

struct Imu {
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
};

struct CartesianHighPoint {
    int32_t x;             // unit:mm
    int32_t y;             // unit:mm
    int32_t z;             // unit:mm
    uint8_t reflectivity;  // 反射率
    uint8_t tag;           // 标签
};

struct CartesianLowPoint {
    int16_t x;             // unit:mm
    int16_t y;             // unit:mm
    int16_t z;             // unit:mm
    uint8_t reflectivity;  // 反射率
    uint8_t tag;           // 标签
};

struct SphericalPoint {
    uint32_t depth;        // 深度
    uint16_t theta;        // 天顶角[0, 18000], Unit: 0.01 度
    uint16_t phi;          // 方位角[0, 36000], Unit: 0.01 度
    uint8_t reflectivity;  // 反射率
    uint8_t tag;           // 标签
};

#pragma pack()

static_assert(sizeof(Point) == 24);

void combine_4_bytes(std::size_t& seed, const unsigned char* bytes) {
    const std::size_t bytes_hash = (static_cast<std::size_t>(bytes[0]) << 24) |
                                   (static_cast<std::size_t>(bytes[1]) << 16) |
                                   (static_cast<std::size_t>(bytes[2]) << 8) | (static_cast<std::size_t>(bytes[3]));
    seed ^= bytes_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

std::size_t IpAddressHasher::operator()(const asio::ip::address& addr) const noexcept {
    if (addr.is_v4()) {
        return std::hash<unsigned int>()(addr.to_v4().to_uint());
    }

    const asio::ip::address_v6::bytes_type bytes = addr.to_v6().to_bytes();
    std::size_t result = static_cast<std::size_t>(addr.to_v6().scope_id());
    combine_4_bytes(result, &bytes[0]);
    combine_4_bytes(result, &bytes[4]);
    combine_4_bytes(result, &bytes[8]);
    combine_4_bytes(result, &bytes[12]);
    return result;
}

double normalize_header_timestamp(const TimestampType time_type, const asio::ip::address& lidar_ip,
                                  double header_timestamp,
                                  std::unordered_map<asio::ip::address, TimeSyncState, IpAddressHasher>&
                                      time_sync_map) {
    if (time_type != TimestampType::kTimestampTypeNoSync) {
        return header_timestamp;
    }

    auto& sync_state = time_sync_map[lidar_ip];
    const auto now = std::chrono::system_clock::now();
    const auto now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
    const double new_delta = now_sec - header_timestamp;
    if (!sync_state.initialized) {
        sync_state.delta = new_delta;
        sync_state.initialized = true;
    } else {
        constexpr double kAlpha = 0.001;
        sync_state.delta += kAlpha * (new_delta - sync_state.delta);
    }
    return header_timestamp + sync_state.delta;
}

Mid360Driver::Mid360Driver(
    asio::io_context& io_context, const asio::ip::address& host_ip, const double lidar_publish_time_interval_sec,
    const std::function<void(const asio::ip::address& lidar_ip, const FrameMeta& meta, const std::vector<Point>& points)>&
        on_receive_pointcloud,
    const std::function<void(const asio::ip::address& lidar_ip, const ImuMsg& imu_msg)>& on_receive_imu)
    : lidar_publish_time_interval_sec(lidar_publish_time_interval_sec > 0.0 ? lidar_publish_time_interval_sec : 0.0),
      host_ip(host_ip), receive_pointcloud_socket(io_context), receive_imu_socket(io_context),
      on_receive_pointcloud(on_receive_pointcloud), on_receive_imu(on_receive_imu) {
    try {
        receive_pointcloud_socket.open(asio::ip::udp::v4());
        receive_pointcloud_socket.bind(asio::ip::udp::endpoint(host_ip, kPointcloudReceivePort));
        receive_imu_socket.open(asio::ip::udp::v4());
        receive_imu_socket.bind(asio::ip::udp::endpoint(host_ip, kImuReceivePort));

        asio::socket_base::receive_buffer_size receive_buffer_size(kUdpReceiveBufferBytes);
        receive_pointcloud_socket.set_option(receive_buffer_size);
        receive_imu_socket.set_option(receive_buffer_size);

        co_spawn(io_context, receive_pointcloud(), asio::detached);
        co_spawn(io_context, receive_imu(), asio::detached);
    } catch (const std::exception& exception) {
        stop();
        throw std::runtime_error(std::string("Failed to initialize Mid360Driver: ") + exception.what());
    }
}

Mid360Driver::~Mid360Driver() {
    stop();
}

void Mid360Driver::stop() {
    if (!is_running.exchange(false, std::memory_order_relaxed)) {
        return;
    }

    asio::error_code error_code;
    receive_pointcloud_socket.close(error_code);
    receive_imu_socket.close(error_code);
}

asio::awaitable<void> Mid360Driver::receive_pointcloud() {
    std::uint8_t buffer[kUdpPacketBufferSize];
    asio::ip::udp::endpoint sender_endpoint;
    std::vector<Point> points;

    while (is_running.load(std::memory_order_relaxed)) {
        asio::error_code error_code;
        const std::size_t bytes_received = co_await receive_pointcloud_socket.async_receive_from(
            asio::buffer(buffer, kUdpPacketBufferSize), sender_endpoint,
            asio::redirect_error(asio::use_awaitable, error_code));
        if (error_code) {
            if (should_exit_receive_loop(error_code, is_running.load(std::memory_order_relaxed))) {
                break;
            }
            continue;
        }
        if (sender_endpoint.port() != kPointcloudSendPort) [[unlikely]] {
            continue;
        }
        if (bytes_received < sizeof(DataHeader)) [[unlikely]] {
            continue;
        }

        DataHeader header;
        std::memcpy(&header, buffer, sizeof(header));
        if (header.version != kSupportedProtocolVersion) [[unlikely]] {
            continue;
        }
        if (header.length > bytes_received) [[unlikely]] {
            continue;
        }

        std::size_t expected_bytes = sizeof(DataHeader);
        if (header.data_type == DataType::kLivoxLidarCartesianCoordinateHighData) {
            expected_bytes += static_cast<std::size_t>(header.dot_num) * sizeof(CartesianHighPoint);
        } else if (header.data_type == DataType::kLivoxLidarCartesianCoordinateLowData) {
            expected_bytes += static_cast<std::size_t>(header.dot_num) * sizeof(CartesianLowPoint);
        } else if (header.data_type == DataType::kLivoxLidarSphericalCoordinateData) {
            expected_bytes += static_cast<std::size_t>(header.dot_num) * sizeof(SphericalPoint);
        } else {
            continue;
        }
        if (header.length < expected_bytes) [[unlikely]] {
            continue;
        }

        double header_timestamp = static_cast<double>(header.timestamp) * 1e-9;
        header_timestamp = normalize_header_timestamp(header.time_type, sender_endpoint.address(), header_timestamp,
                                                      time_sync_map);

        points.clear();
        points.reserve(static_cast<std::size_t>(header.dot_num));
        const double point_time_step =
            (header.dot_num > 1)
                ? static_cast<double>(header.time_interval) * 1e-7 / static_cast<double>(header.dot_num - 1)
                : 0.0;

        if (header.data_type == DataType::kLivoxLidarCartesianCoordinateHighData) {
            const auto* raw_points = reinterpret_cast<const CartesianHighPoint*>(buffer + sizeof(DataHeader));
            for (std::size_t i = 0; i < header.dot_num; ++i) {
                const auto& raw_point = raw_points[i];
                if (!has_valid_point_tag(raw_point.tag)) {
                    continue;
                }
                Point point{};
                point.timestamp = header_timestamp + point_time_step * static_cast<double>(i);
                point.x = static_cast<float>(raw_point.x * 0.001);
                point.y = static_cast<float>(raw_point.y * 0.001);
                point.z = static_cast<float>(raw_point.z * 0.001);
                point.intensity = raw_point.reflectivity;
                points.push_back(point);
            }
        } else if (header.data_type == DataType::kLivoxLidarCartesianCoordinateLowData) {
            const auto* raw_points = reinterpret_cast<const CartesianLowPoint*>(buffer + sizeof(DataHeader));
            for (std::size_t i = 0; i < header.dot_num; ++i) {
                const auto& raw_point = raw_points[i];
                if (!has_valid_point_tag(raw_point.tag)) {
                    continue;
                }
                Point point{};
                point.timestamp = header_timestamp + point_time_step * static_cast<double>(i);
                point.x = static_cast<float>(raw_point.x * 0.001);
                point.y = static_cast<float>(raw_point.y * 0.001);
                point.z = static_cast<float>(raw_point.z * 0.001);
                point.intensity = raw_point.reflectivity;
                points.push_back(point);
            }
        } else {
            const auto* raw_points = reinterpret_cast<const SphericalPoint*>(buffer + sizeof(DataHeader));
            for (std::size_t i = 0; i < header.dot_num; ++i) {
                const auto& raw_point = raw_points[i];
                if (!has_valid_point_tag(raw_point.tag)) {
                    continue;
                }
                Point point{};
                point.timestamp = header_timestamp + point_time_step * static_cast<double>(i);
                const double radius = raw_point.depth / 1000.0;
                const double theta = raw_point.theta / 100.0 / 180.0 * M_PI;
                const double phi = raw_point.phi / 100.0 / 180.0 * M_PI;
                const double sin_theta = std::sin(theta);
                point.x = static_cast<float>(radius * sin_theta * std::cos(phi));
                point.y = static_cast<float>(radius * sin_theta * std::sin(phi));
                point.z = static_cast<float>(radius * std::cos(theta));
                point.intensity = raw_point.reflectivity;
                points.push_back(point);
            }
        }

        auto& frame_state = frame_state_map[sender_endpoint.address()];
        if (!frame_state.has_data) {
            start_new_frame(frame_state, header.frame_cnt, header.udp_cnt);
        } else if (header.frame_cnt != frame_state.current_frame_cnt) {
            finalize_frame(sender_endpoint.address(), frame_state, on_receive_pointcloud);
            start_new_frame(frame_state, header.frame_cnt, header.udp_cnt);
        } else if (lidar_publish_time_interval_sec > 0.0 &&
                   frame_state.frame_min_ts != std::numeric_limits<double>::max() &&
                   header_timestamp - frame_state.frame_min_ts >= lidar_publish_time_interval_sec) {
            finalize_frame(sender_endpoint.address(), frame_state, on_receive_pointcloud);
            start_new_frame(frame_state, header.frame_cnt, header.udp_cnt);
        }

        if (header.udp_cnt < frame_state.expected_udp_cnt) {
            continue;
        }

        const auto gap = static_cast<std::uint16_t>(header.udp_cnt - frame_state.expected_udp_cnt);
        frame_state.lost_in_frame += gap;
        frame_state.lost_total += gap;
        frame_state.expected_udp_cnt = static_cast<std::uint16_t>(header.udp_cnt + 1);

        if (header_timestamp < frame_state.frame_min_ts) {
            frame_state.frame_min_ts = header_timestamp;
        }

        if (!frame_state.drop_current_frame) {
            if (frame_state.frame_points.size() + points.size() > kMaxFramePointCount) {
                frame_state.drop_current_frame = true;
                frame_state.frame_points.clear();
                continue;
            }
            frame_state.frame_points.insert(frame_state.frame_points.end(), points.begin(), points.end());
        }
    }
}

asio::awaitable<void> Mid360Driver::receive_imu() {
    std::uint8_t buffer[kUdpPacketBufferSize];
    asio::ip::udp::endpoint sender_endpoint;

    while (is_running.load(std::memory_order_relaxed)) {
        asio::error_code error_code;
        const std::size_t bytes_received = co_await receive_imu_socket.async_receive_from(
            asio::buffer(buffer, kUdpPacketBufferSize), sender_endpoint,
            asio::redirect_error(asio::use_awaitable, error_code));
        if (error_code) {
            if (should_exit_receive_loop(error_code, is_running.load(std::memory_order_relaxed))) {
                break;
            }
            continue;
        }
        if (sender_endpoint.port() != kImuSendPort) [[unlikely]] {
            continue;
        }
        if (bytes_received < sizeof(DataHeader) + sizeof(Imu)) [[unlikely]] {
            continue;
        }

        DataHeader header;
        std::memcpy(&header, buffer, sizeof(header));
        if (header.version != kSupportedProtocolVersion) [[unlikely]] {
            continue;
        }
        if (header.length > bytes_received || header.length < sizeof(DataHeader) + sizeof(Imu)) [[unlikely]] {
            continue;
        }
        if (header.data_type != DataType::kLivoxLidarImuData) [[unlikely]] {
            continue;
        }

        double header_timestamp = static_cast<double>(header.timestamp) * 1e-9;
        header_timestamp = normalize_header_timestamp(header.time_type, sender_endpoint.address(), header_timestamp,
                                                      time_sync_map);

        const auto& raw_imu = *reinterpret_cast<const Imu*>(buffer + sizeof(DataHeader));
        ImuMsg imu_msg{};
        imu_msg.timestamp = header_timestamp;
        imu_msg.angular_velocity_x = raw_imu.angular_velocity_x;
        imu_msg.angular_velocity_y = raw_imu.angular_velocity_y;
        imu_msg.angular_velocity_z = raw_imu.angular_velocity_z;
        imu_msg.linear_acceleration_x = raw_imu.linear_acceleration_x;
        imu_msg.linear_acceleration_y = raw_imu.linear_acceleration_y;
        imu_msg.linear_acceleration_z = raw_imu.linear_acceleration_z;
        on_receive_imu(sender_endpoint.address(), imu_msg);
    }
}

}  // namespace mid360_driver
