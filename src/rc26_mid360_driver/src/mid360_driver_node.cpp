/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 * Maintained by DongXuan Chen <2220362462@qq.com>
 */

#include "mid360_driver_node.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace mid360_driver {
namespace {

std::string lidar_topic_suffix(const asio::ip::address& lidar_ip) {
    if (lidar_ip.is_v4()) {
        const auto lidar_ip_bytes = lidar_ip.to_v4().to_bytes();
        std::string suffix;
        suffix.reserve(16);
        for (const auto ip_byte : lidar_ip_bytes) {
            suffix.push_back('_');
            suffix.append(std::to_string(static_cast<int>(ip_byte)));
        }
        return suffix;
    }

    const std::string address_text = lidar_ip.to_string();
    std::string suffix;
    suffix.reserve(address_text.size() + 1);
    suffix.push_back('_');
    for (const char character : address_text) {
        const auto unsigned_character = static_cast<unsigned char>(character);
        suffix.push_back(std::isalnum(unsigned_character) ? character : '_');
    }
    return suffix;
}

const std::vector<sensor_msgs::msg::PointField>& point_fields() {
    static const auto fields = [] {
        std::vector<sensor_msgs::msg::PointField> configured_fields(5);
        auto set_field = [](sensor_msgs::msg::PointField& field, const char* name, uint32_t offset, uint8_t datatype) {
            field.name = name;
            field.offset = offset;
            field.datatype = datatype;
            field.count = 1;
        };
        set_field(configured_fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32);
        set_field(configured_fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32);
        set_field(configured_fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32);
        set_field(configured_fields[3], "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
        set_field(configured_fields[4], "timestamp", 16, sensor_msgs::msg::PointField::FLOAT64);
        return configured_fields;
    }();
    return fields;
}

}  // namespace

void LidarPublisher::make_sure_init(rclcpp::Node& node, const std::string& lidar_topic, const std::string& imu_topic) {
    if (!is_init) {
        const auto qos = rclcpp::SensorDataQoS();
        pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, qos);
        imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic, qos);
        is_init = true;
    }
}

void LidarPublisher::make_sure_init(rclcpp::Node& node, const std::string& lidar_topic, const std::string& imu_topic,
                                    const asio::ip::address& lidar_ip) {
    if (!is_init) {
        const std::string suffix = lidar_topic_suffix(lidar_ip);
        const auto qos = rclcpp::SensorDataQoS();
        pointcloud_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic + suffix, qos);
        imu_publisher = node.create_publisher<sensor_msgs::msg::Imu>(imu_topic + suffix, qos);
        is_init = true;
    }
}

void LidarPublisher::on_receive_imu(const ImuMsg& imu_msg) {
    imu_wait_to_publish.push_back(imu_msg);
}

void LidarPublisher::prepare_imu_to_publish() {
    std::swap(imu_wait_to_publish, imu_to_publish);
    imu_wait_to_publish.clear();
}

void LidarPublisher::publish_pointcloud(const std::string& frame_id, const FrameMeta& meta,
                                        const std::vector<Point>& points) const {
    if (!pointcloud_publisher || points.empty()) {
        return;
    }

    constexpr std::size_t kMaxPointCount = 200000;
    if (points.size() > kMaxPointCount) {
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(std::floor(meta.stamp_sec));
    msg.header.stamp.nanosec =
        static_cast<uint32_t>((meta.stamp_sec - static_cast<double>(msg.header.stamp.sec)) * 1e9);
    msg.header.frame_id = frame_id;
    msg.width = static_cast<uint32_t>(points.size());
    msg.height = 1;
    msg.fields = point_fields();
    msg.is_bigendian = false;
    msg.point_step = static_cast<uint32_t>(sizeof(Point));
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(points.size() * sizeof(Point));
    std::memcpy(msg.data.data(), points.data(), points.size() * sizeof(Point));
    msg.is_dense = true;
    pointcloud_publisher->publish(msg);
}

void LidarPublisher::publish_imu(const std::string& frame_id) const {
    if (!imu_publisher) {
        return;
    }

    for (const auto& imu : imu_to_publish) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp.sec = static_cast<int32_t>(std::floor(imu.timestamp));
        msg.header.stamp.nanosec = static_cast<uint32_t>((imu.timestamp - static_cast<double>(msg.header.stamp.sec)) * 1e9);
        msg.header.frame_id = frame_id;
        msg.angular_velocity.x = imu.angular_velocity_x;
        msg.angular_velocity.y = imu.angular_velocity_y;
        msg.angular_velocity.z = imu.angular_velocity_z;
        msg.linear_acceleration.x = imu.linear_acceleration_x;
        msg.linear_acceleration.y = imu.linear_acceleration_y;
        msg.linear_acceleration.z = imu.linear_acceleration_z;
        imu_publisher->publish(msg);
    }
}

Mid360DriverNode::Mid360DriverNode(const rclcpp::NodeOptions& options) : Node("mid360_driver_node", options) {
    const std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
    const std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
    const std::string imu_topic = declare_parameter<std::string>("imu_topic");
    const std::string imu_frame = declare_parameter<std::string>("imu_frame");
    const std::string host_ip = declare_parameter<std::string>("host_ip");
    const double lidar_publish_time_interval = declare_parameter<double>("lidar_publish_time_interval");
    const bool is_topic_name_with_lidar_ip = declare_parameter<bool>("is_topic_name_with_lidar_ip");

    asio::ip::address parsed_host_ip;
    try {
        parsed_host_ip = asio::ip::make_address(host_ip);
    } catch (const std::exception& exception) {
        throw std::invalid_argument("Invalid parameter host_ip='" + host_ip + "': " + exception.what());
    }

    if (!is_topic_name_with_lidar_ip) {
        lidar_publisher.make_sure_init(*this, lidar_topic, imu_topic);
    }

    mid360_driver = std::make_unique<mid360_driver::Mid360Driver>(
        io_context, parsed_host_ip, lidar_publish_time_interval,
        [this, is_topic_name_with_lidar_ip, lidar_topic, imu_topic, lidar_frame](
            const asio::ip::address& lidar_ip, const FrameMeta& meta, const std::vector<Point>& points) {
            try {
                std::lock_guard<std::mutex> lock(mutex);
                if (is_topic_name_with_lidar_ip) {
                    auto [iter, inserted] = muti_lidar_publisher.try_emplace(lidar_ip, nullptr);
                    if (inserted || !iter->second) {
                        iter->second = std::make_shared<LidarPublisher>();
                        iter->second->make_sure_init(*this, lidar_topic, imu_topic, lidar_ip);
                    }
                    if (meta.lost_in_frame > 0) {
                        RCLCPP_WARN_THROTTLE(
                            get_logger(), *get_clock(), 1000,
                            "Lidar %s frame=%u lost_in_frame=%u lost_total=%u", lidar_ip.to_string().c_str(),
                            static_cast<unsigned int>(meta.frame_cnt), meta.lost_in_frame, meta.lost_total);
                    }
                    iter->second->publish_pointcloud(lidar_frame, meta, points);
                } else {
                    if (meta.lost_in_frame > 0) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                             "frame=%u lost_in_frame=%u lost_total=%u",
                                             static_cast<unsigned int>(meta.frame_cnt), meta.lost_in_frame,
                                             meta.lost_total);
                    }
                    lidar_publisher.publish_pointcloud(lidar_frame, meta, points);
                }
            } catch (const std::exception& exception) {
                RCLCPP_ERROR(get_logger(), "Pointcloud callback failed for %s: %s", lidar_ip.to_string().c_str(),
                             exception.what());
            }
        },
        [this, is_topic_name_with_lidar_ip, lidar_topic, imu_topic](const asio::ip::address& lidar_ip,
                                                                     const ImuMsg& imu_msg) {
            try {
                std::lock_guard<std::mutex> lock(mutex);
                if (is_topic_name_with_lidar_ip) {
                    auto [iter, inserted] = muti_lidar_publisher.try_emplace(lidar_ip, nullptr);
                    if (inserted || !iter->second) {
                        iter->second = std::make_shared<LidarPublisher>();
                        iter->second->make_sure_init(*this, lidar_topic, imu_topic, lidar_ip);
                    }
                    iter->second->on_receive_imu(imu_msg);
                } else {
                    lidar_publisher.on_receive_imu(imu_msg);
                }
            } catch (const std::exception& exception) {
                RCLCPP_ERROR(get_logger(), "IMU callback failed for %s: %s", lidar_ip.to_string().c_str(),
                             exception.what());
            }
        });

    if (is_topic_name_with_lidar_ip) {
        publish_imu_timer =
            rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(4), [this, imu_frame]() {
                try {
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        for (auto& [lidar_ip, lidar_publisher] : muti_lidar_publisher) {
                            static_cast<void>(lidar_ip);
                            lidar_publisher->prepare_imu_to_publish();
                        }
                        if (muti_lidar_publisher_temp.size() != muti_lidar_publisher.size()) {
                            muti_lidar_publisher_temp.clear();
                            muti_lidar_publisher_temp.reserve(muti_lidar_publisher.size());
                            for (auto& [lidar_ip, lidar_publisher] : muti_lidar_publisher) {
                                muti_lidar_publisher_temp.emplace_back(lidar_ip, lidar_publisher);
                            }
                        }
                    }
                    for (auto& [lidar_ip, lidar_publisher] : muti_lidar_publisher_temp) {
                        static_cast<void>(lidar_ip);
                        if (lidar_publisher) {
                            lidar_publisher->publish_imu(imu_frame);
                        }
                    }
                } catch (const std::exception& exception) {
                    RCLCPP_ERROR(get_logger(), "Failed to publish multi-lidar IMU data: %s", exception.what());
                }
            });
    } else {
        publish_imu_timer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(4), [this, imu_frame]() {
            try {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    lidar_publisher.prepare_imu_to_publish();
                }
                lidar_publisher.publish_imu(imu_frame);
            } catch (const std::exception& exception) {
                RCLCPP_ERROR(get_logger(), "Failed to publish IMU data: %s", exception.what());
            }
        });
    }

    io_thread = std::thread([this]() {
        try {
            io_context.run();
        } catch (const std::exception& exception) {
            RCLCPP_FATAL(get_logger(), "asio io_context stopped unexpectedly: %s", exception.what());
        }
    });
}

Mid360DriverNode::~Mid360DriverNode() {
    if (mid360_driver) {
        mid360_driver->stop();
    }
    io_context.stop();
    if (io_thread.joinable()) {
        io_thread.join();
    }
}

}  // namespace mid360_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_driver::Mid360DriverNode)
