#pragma once

#include <common_lib.hpp>

#include "Estimator.hpp"
#define MAXN (720000)

extern bool data_accum_finished, data_accum_start, online_calib_finish, refine_print;
extern int frame_num_init;
extern double move_start_time, online_calib_starts_time;

extern V3D gravity_lio;
extern mutex mtx_buffer;
extern condition_variable sig_buffer;
extern int scan_count;
extern int frame_ct, wait_num;
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;
extern std::deque<double> time_buffer;
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;
extern std::mutex m_time;
extern bool lidar_pushed, imu_pushed;
extern double imu_first_time;
extern bool lose_lid;
extern sensor_msgs::msg::Imu imu_last, imu_next;
extern PointCloudXYZI::Ptr ptr_con;
extern double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

// extern sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg_in);
bool sync_packages(MeasureGroup& meas);

// #endif