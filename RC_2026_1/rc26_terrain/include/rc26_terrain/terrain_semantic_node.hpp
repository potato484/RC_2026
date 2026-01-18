#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_terrain {

class TerrainSemanticNode : public rclcpp::Node {
public:
    explicit TerrainSemanticNode(const rclcpp::NodeOptions& options);

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    std::optional<tf2::Transform> getTransform(const std::string& target_frame,
                                               const std::string& source_frame,
                                               const rclcpp::Time& time);
    void initGrid();
    void estimateCellHeights(double stamp_sec);
    void classifyAndUpdate(double stamp_sec);
    void publishOutputs(const rclcpp::Time& stamp, double base_x, double base_y,
                        double base_z, double cos_yaw, double sin_yaw);

    // Parameters
    std::string input_cloud_topic_;
    std::string target_frame_;
    std::string base_frame_;
    std::string output_obstacles_topic_;
    std::string output_drop_topic_;

    double tf_timeout_sec_{0.2};
    double perception_radius_m_{3.2};
    double grid_resolution_m_{0.1};
    double voxel_leaf_size_m_{0.05};

    double min_rel_z_m_{-1.5};
    double max_rel_z_m_{0.5};
    double dis_ratio_z_{0.1};

    int min_points_per_cell_{5};
    double ground_quantile_{0.25};
    double top_quantile_{0.95};
    double ground_ema_alpha_{0.6};

    double h_climb_m_{0.30};
    double h_obstacle_m_{0.33};
    double h_drop_m_{0.15};

    bool enable_hysteresis_{true};
    int score_max_{10};
    int score_inc_{2};
    int score_dec_{1};
    int obstacle_on_score_{6};
    int obstacle_off_score_{3};
    int drop_on_score_{6};
    int drop_off_score_{3};
    double stale_time_sec_{0.7};

    // Grid state
    int half_width_{0};
    int width_{0};
    int num_cells_{0};
    std::vector<uint8_t> cell_in_radius_;
    std::vector<float> ground_z_filtered_;
    std::vector<float> top_z_;
    std::vector<double> last_seen_sec_;
    std::vector<int> obstacle_score_;
    std::vector<int> drop_score_;
    std::vector<uint8_t> obstacle_state_;
    std::vector<uint8_t> drop_state_;

    // Per-frame buffers
    std::vector<std::vector<float>> cell_z_samples_;
    std::vector<int> touched_cells_;

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_drop_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rc26_terrain
