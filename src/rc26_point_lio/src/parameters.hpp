// Maintained by DongXuan Chen <2220362462@qq.com>
// #ifndef PARAM_H
// #define PARAM_H
#pragma once
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/vector3.hpp>
#include <ivox/ivox3d.hpp>
#include <math.h>
#include <omp.h>
#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include "IMU_Processing.hpp"
#include "preprocess.hpp"

// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

struct LioRuntimeState {
    bool is_first_frame = true;
    double lidar_end_time = 0.0;
    double first_lidar_time = 0.0;
    double time_con = 0.0;
    double last_timestamp_lidar = -1.0;
    double last_timestamp_imu = -1.0;
    int pcd_index = 0;
    IVoxType::Options ivox_options_;
    int ivox_nearby_type = 6;
    state_input state_in;
    state_output state_out;
    std::string lid_topic;
    std::string imu_topic;
    bool prop_at_freq_of_imu = true;
    bool check_satu = true;
    bool con_frame = false;
    bool cut_frame = false;
    bool use_imu_as_input = false;
    bool space_down_sample = true;
    bool extrinsic_est_en = true;
    bool publish_odometry_without_downsample = false;
    int init_map_size = 10;
    int point_filter_num = 2;
    int con_frame_num = 1;
    double match_s = 81.0;
    double satu_acc = 0.0;
    double satu_gyro = 0.0;
    double cut_frame_time_interval = 0.1;
    float plane_thr = 0.1f;
    double filter_size_surf_min = 0.5;
    double filter_size_map_min = 0.5;
    double fov_deg = 180.0;
    float DET_RANGE = 450.0f;
    bool imu_en = true;
    double imu_time_inte = 0.005;
    double laser_point_cov = 0.01;
    double acc_norm = 1.0;
    double acc_cov_input = 0.0;
    double gyr_cov_input = 0.0;
    double vel_cov = 0.0;
    double gyr_cov_output = 0.0;
    double acc_cov_output = 0.0;
    double b_gyr_cov = 0.0;
    double b_acc_cov = 0.0;
    double imu_meas_acc_cov = 0.0;
    double imu_meas_omg_cov = 0.0;
    int lidar_type = 0;
    int pcd_save_interval = 0;
    std::vector<double> gravity_init;
    std::vector<double> gravity;
    bool runtime_pos_log = false;
    bool pcd_save_en = false;
    bool path_en = false;
    bool scan_pub_en = false;
    bool scan_body_pub_en = false;
    bool tf_send_en = false;
    bool full_map_publish_en = false;
    std::string full_map_topic = "point_lio/map_cloud";
    double full_map_interval_sec = 2.0;
    double full_map_voxel_size = 0.1;
    int full_map_max_points = 1500000;
    bool filter_car_body = true;
    double body_x_min = -0.4;
    double body_x_max = 0.32;
    double body_y_min = -0.3;
    double body_y_max = 0.3;
    double body_z_min = 0.0;
    double body_z_max = 0.8;
    shared_ptr<Preprocess> p_pre;
    shared_ptr<ImuProcess> p_imu;
    std::vector<double> extrinT = std::vector<double>(3, 0.0);
    std::vector<double> extrinR = std::vector<double>(9, 0.0);
    double lidar_time_inte = 0.1;
    double first_imu_time = 0.0;
    int cut_frame_num = 1;
    int orig_odom_freq = 10;
    double online_refine_time = 20.0;
    bool cut_frame_init = false;
    double time_update_last = 0.0;
    double time_current = 0.0;
    double time_predict_last_const = 0.0;
    double t_last = 0.0;
    std::string odom_frame = "odom";
    std::string body_frame = "body";
    MeasureGroup Measures;
    ofstream fout_out;
    ofstream fout_imu_pbp;
};

LioRuntimeState& runtime();

#define DECLARE_LIO_STATE_REF(member) inline decltype(runtime().member)& member = runtime().member

DECLARE_LIO_STATE_REF(is_first_frame);
DECLARE_LIO_STATE_REF(lidar_end_time);
DECLARE_LIO_STATE_REF(first_lidar_time);
DECLARE_LIO_STATE_REF(time_con);
DECLARE_LIO_STATE_REF(last_timestamp_lidar);
DECLARE_LIO_STATE_REF(last_timestamp_imu);
DECLARE_LIO_STATE_REF(pcd_index);
DECLARE_LIO_STATE_REF(ivox_options_);
DECLARE_LIO_STATE_REF(ivox_nearby_type);
DECLARE_LIO_STATE_REF(state_in);
DECLARE_LIO_STATE_REF(state_out);
DECLARE_LIO_STATE_REF(lid_topic);
DECLARE_LIO_STATE_REF(imu_topic);
DECLARE_LIO_STATE_REF(prop_at_freq_of_imu);
DECLARE_LIO_STATE_REF(check_satu);
DECLARE_LIO_STATE_REF(con_frame);
DECLARE_LIO_STATE_REF(cut_frame);
DECLARE_LIO_STATE_REF(use_imu_as_input);
DECLARE_LIO_STATE_REF(space_down_sample);
DECLARE_LIO_STATE_REF(extrinsic_est_en);
DECLARE_LIO_STATE_REF(publish_odometry_without_downsample);
DECLARE_LIO_STATE_REF(init_map_size);
DECLARE_LIO_STATE_REF(point_filter_num);
DECLARE_LIO_STATE_REF(con_frame_num);
DECLARE_LIO_STATE_REF(match_s);
DECLARE_LIO_STATE_REF(satu_acc);
DECLARE_LIO_STATE_REF(satu_gyro);
DECLARE_LIO_STATE_REF(cut_frame_time_interval);
DECLARE_LIO_STATE_REF(plane_thr);
DECLARE_LIO_STATE_REF(filter_size_surf_min);
DECLARE_LIO_STATE_REF(filter_size_map_min);
DECLARE_LIO_STATE_REF(fov_deg);
DECLARE_LIO_STATE_REF(DET_RANGE);
DECLARE_LIO_STATE_REF(imu_en);
DECLARE_LIO_STATE_REF(imu_time_inte);
DECLARE_LIO_STATE_REF(laser_point_cov);
DECLARE_LIO_STATE_REF(acc_norm);
DECLARE_LIO_STATE_REF(acc_cov_input);
DECLARE_LIO_STATE_REF(gyr_cov_input);
DECLARE_LIO_STATE_REF(vel_cov);
DECLARE_LIO_STATE_REF(gyr_cov_output);
DECLARE_LIO_STATE_REF(acc_cov_output);
DECLARE_LIO_STATE_REF(b_gyr_cov);
DECLARE_LIO_STATE_REF(b_acc_cov);
DECLARE_LIO_STATE_REF(imu_meas_acc_cov);
DECLARE_LIO_STATE_REF(imu_meas_omg_cov);
DECLARE_LIO_STATE_REF(lidar_type);
DECLARE_LIO_STATE_REF(pcd_save_interval);
DECLARE_LIO_STATE_REF(gravity_init);
DECLARE_LIO_STATE_REF(gravity);
DECLARE_LIO_STATE_REF(runtime_pos_log);
DECLARE_LIO_STATE_REF(pcd_save_en);
DECLARE_LIO_STATE_REF(path_en);
DECLARE_LIO_STATE_REF(scan_pub_en);
DECLARE_LIO_STATE_REF(scan_body_pub_en);
DECLARE_LIO_STATE_REF(tf_send_en);
DECLARE_LIO_STATE_REF(full_map_publish_en);
DECLARE_LIO_STATE_REF(full_map_topic);
DECLARE_LIO_STATE_REF(full_map_interval_sec);
DECLARE_LIO_STATE_REF(full_map_voxel_size);
DECLARE_LIO_STATE_REF(full_map_max_points);
DECLARE_LIO_STATE_REF(filter_car_body);
DECLARE_LIO_STATE_REF(body_x_min);
DECLARE_LIO_STATE_REF(body_x_max);
DECLARE_LIO_STATE_REF(body_y_min);
DECLARE_LIO_STATE_REF(body_y_max);
DECLARE_LIO_STATE_REF(body_z_min);
DECLARE_LIO_STATE_REF(body_z_max);
DECLARE_LIO_STATE_REF(p_pre);
DECLARE_LIO_STATE_REF(p_imu);
DECLARE_LIO_STATE_REF(extrinT);
DECLARE_LIO_STATE_REF(extrinR);
DECLARE_LIO_STATE_REF(lidar_time_inte);
DECLARE_LIO_STATE_REF(first_imu_time);
DECLARE_LIO_STATE_REF(cut_frame_num);
DECLARE_LIO_STATE_REF(orig_odom_freq);
DECLARE_LIO_STATE_REF(online_refine_time);
DECLARE_LIO_STATE_REF(cut_frame_init);
DECLARE_LIO_STATE_REF(time_update_last);
DECLARE_LIO_STATE_REF(time_current);
DECLARE_LIO_STATE_REF(time_predict_last_const);
DECLARE_LIO_STATE_REF(t_last);
DECLARE_LIO_STATE_REF(odom_frame);
DECLARE_LIO_STATE_REF(body_frame);
DECLARE_LIO_STATE_REF(Measures);
DECLARE_LIO_STATE_REF(fout_out);
DECLARE_LIO_STATE_REF(fout_imu_pbp);

#undef DECLARE_LIO_STATE_REF
void readParameters(std::shared_ptr<rclcpp::Node>& n);
void open_file();
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3& orient);
void reset_cov(Eigen::Matrix<double, 24, 24>& P_init);
void reset_cov_output(Eigen::Matrix<double, 30, 30>& P_init_output);
