// Maintained by DongXuan Chen <2220362462@qq.com>
#include "parameters.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace {

std::string ensureLogDirectory() {
    const std::filesystem::path log_dir = std::filesystem::path(ROOT_DIR) / "Log";
    std::error_code error_code;
    std::filesystem::create_directories(log_dir, error_code);
    if (error_code) {
        std::cout << "~~~~failed to create log dir: " << log_dir << " (" << error_code.message() << ')' << '\n';
    }
    return log_dir.string();
}

bool hasFiniteEntries(const std::vector<double>& values, const size_t count) {
    if (values.size() < count) {
        return false;
    }
    return std::all_of(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(count),
                       [](const double value) { return std::isfinite(value); });
}

void ensureSizedFiniteVector(std::vector<double>& values,
                             const size_t expected_size,
                             const std::vector<double>& fallback,
                             const char* name,
                             const rclcpp::Logger& logger) {
    if (values.size() < expected_size) {
        RCLCPP_WARN(logger, "%s size %zu < %zu, fallback to defaults", name, values.size(), expected_size);
        values = fallback;
    } else if (values.size() > expected_size) {
        RCLCPP_WARN(logger, "%s size %zu > %zu, extra entries will be ignored", name, values.size(), expected_size);
    }
    values.resize(expected_size);

    if (!hasFiniteEntries(values, expected_size)) {
        RCLCPP_WARN(logger, "%s contains non-finite entries, fallback to defaults", name);
        values = fallback;
        values.resize(expected_size);
    }
}

std::string bodyFilterRangeError(bool enabled,
                                 double x_min,
                                 double x_max,
                                 double y_min,
                                 double y_max,
                                 double z_min,
                                 double z_max) {
    const double values[] = {x_min, x_max, y_min, y_max, z_min, z_max};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return "body ROI bounds must be finite";
        }
    }

    if (x_min > x_max) {
        return "body_x_min must be <= body_x_max";
    }
    if (y_min > y_max) {
        return "body_y_min must be <= body_y_max";
    }
    if (z_min > z_max) {
        return "body_z_min must be <= body_z_max";
    }

    (void)enabled;
    return {};
}

void validateBodyFilterParametersOrThrow(const rclcpp::Logger& logger) {
    const std::string error =
        bodyFilterRangeError(filter_car_body, body_x_min, body_x_max, body_y_min, body_y_max, body_z_min, body_z_max);
    if (!error.empty()) {
        std::ostringstream message;
        message << "Invalid body ROI filter parameters: " << error << " (x=[" << body_x_min << ", " << body_x_max
                << "], y=[" << body_y_min << ", " << body_y_max << "], z=[" << body_z_min << ", " << body_z_max
                << "])";
        RCLCPP_FATAL(logger, "%s", message.str().c_str());
        throw std::runtime_error(message.str());
    }
}

void applyBodyFilterConfigToPreprocess() {
    p_pre->setBodyFilterConfig(filter_car_body, body_x_min, body_x_max, body_y_min, body_y_max, body_z_min, body_z_max);
}

}  // namespace

LioRuntimeState& runtime() {
    static LioRuntimeState state;
    return state;
}

void readParameters(std::shared_ptr<rclcpp::Node>& nh) {
    p_pre.reset(new Preprocess());
    p_imu.reset(new ImuProcess());
    try {
        nh->declare_parameter<bool>("prop_at_freq_of_imu", true);
        nh->get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);

        nh->declare_parameter<bool>("use_imu_as_input", false);
        nh->get_parameter("use_imu_as_input", use_imu_as_input);

        nh->declare_parameter<bool>("check_satu", true);
        nh->get_parameter("check_satu", check_satu);

        nh->declare_parameter<int>("init_map_size", 100);
        nh->get_parameter("init_map_size", init_map_size);

        nh->declare_parameter<bool>("space_down_sample", true);
        nh->get_parameter("space_down_sample", space_down_sample);

        nh->declare_parameter<double>("mapping.satu_acc", 3.0);
        nh->get_parameter("mapping.satu_acc", satu_acc);

        nh->declare_parameter<double>("mapping.satu_gyro", 35.0);
        nh->get_parameter("mapping.satu_gyro", satu_gyro);

        nh->declare_parameter<double>("mapping.acc_norm", 1.0);
        nh->get_parameter("mapping.acc_norm", acc_norm);

        nh->declare_parameter<float>("mapping.plane_thr", 0.05f);
        nh->get_parameter("mapping.plane_thr", plane_thr);

        nh->declare_parameter<int>("point_filter_num", 2);
        nh->get_parameter("point_filter_num", point_filter_num);

        nh->declare_parameter<std::string>("common.lid_topic", ".livox.lidar");
        nh->get_parameter("common.lid_topic", lid_topic);

        nh->declare_parameter<std::string>("common.imu_topic", ".livox.imu");
        nh->get_parameter("common.imu_topic", imu_topic);

        nh->declare_parameter<bool>("common.con_frame", false);
        nh->get_parameter("common.con_frame", con_frame);

        nh->declare_parameter<int>("common.con_frame_num", 1);
        nh->get_parameter("common.con_frame_num", con_frame_num);

        nh->declare_parameter<bool>("common.cut_frame", false);
        nh->get_parameter("common.cut_frame", cut_frame);

        nh->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
        nh->get_parameter("common.cut_frame_time_interval", cut_frame_time_interval);

        nh->declare_parameter<double>("filter_size_surf", 0.5);
        nh->get_parameter("filter_size_surf", filter_size_surf_min);

        nh->declare_parameter<double>("filter_size_map", 0.5);
        nh->get_parameter("filter_size_map", filter_size_map_min);

        nh->declare_parameter<float>("mapping.det_range", 300.f);
        nh->get_parameter("mapping.det_range", DET_RANGE);

        nh->declare_parameter<double>("mapping.fov_degree", 180);
        nh->get_parameter("mapping.fov_degree", fov_deg);

        nh->declare_parameter<bool>("mapping.imu_en", true);
        nh->get_parameter("mapping.imu_en", imu_en);

        nh->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        nh->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);

        nh->declare_parameter<double>("mapping.imu_time_inte", 0.005);
        nh->get_parameter("mapping.imu_time_inte", imu_time_inte);

        nh->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
        nh->get_parameter("mapping.lidar_meas_cov", laser_point_cov);

        nh->declare_parameter<double>("mapping.acc_cov_input", 0.1);
        nh->get_parameter("mapping.acc_cov_input", acc_cov_input);

        nh->declare_parameter<double>("mapping.vel_cov", 20);
        nh->get_parameter("mapping.vel_cov", vel_cov);

        nh->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
        nh->get_parameter("mapping.gyr_cov_input", gyr_cov_input);

        nh->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
        nh->get_parameter("mapping.gyr_cov_output", gyr_cov_output);

        nh->declare_parameter<double>("mapping.acc_cov_output", 0.1);
        nh->get_parameter("mapping.acc_cov_output", acc_cov_output);

        nh->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        nh->get_parameter("mapping.b_gyr_cov", b_gyr_cov);

        nh->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        nh->get_parameter("mapping.b_acc_cov", b_acc_cov);

        nh->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
        nh->get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);

        nh->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
        nh->get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);

        nh->declare_parameter<double>("preprocess.blind", 1.0);
        nh->get_parameter("preprocess.blind", p_pre->blind);

        nh->declare_parameter<double>("preprocess.det_range", 1000.0);
        nh->get_parameter("preprocess.det_range", p_pre->det_range);

        nh->declare_parameter<int>("preprocess.lidar_type", 1);
        nh->get_parameter("preprocess.lidar_type", lidar_type);

        nh->declare_parameter<int>("preprocess.scan_line", 16);
        nh->get_parameter("preprocess.scan_line", p_pre->N_SCANS);

        nh->declare_parameter<int>("preprocess.scan_rate", 10);
        nh->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);

        nh->declare_parameter<int>("preprocess.timestamp_unit", 1);
        nh->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);

        nh->declare_parameter<double>("mapping.match_s", 81);
        nh->get_parameter("mapping.match_s", match_s);

        nh->declare_parameter<std::vector<double>>("mapping.gravity", std::vector<double>());
        nh->get_parameter("mapping.gravity", gravity);

        nh->declare_parameter<std::vector<double>>("mapping.gravity_init", std::vector<double>());
        nh->get_parameter("mapping.gravity_init", gravity_init);

        nh->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
        nh->get_parameter("mapping.extrinsic_T", extrinT);

        nh->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());
        nh->get_parameter("mapping.extrinsic_R", extrinR);

        nh->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
        nh->get_parameter("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);

        nh->declare_parameter<bool>("publish.path_en", true);
        nh->get_parameter("publish.path_en", path_en);

        nh->declare_parameter<bool>("publish.scan_publish_en", true);
        nh->get_parameter("publish.scan_publish_en", scan_pub_en);

        nh->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        nh->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);

        nh->declare_parameter<bool>("publish.tf_send_en", true);
        nh->get_parameter("publish.tf_send_en", tf_send_en);

        nh->declare_parameter<bool>("runtime_pos_log_enable", false);
        nh->get_parameter("runtime_pos_log_enable", runtime_pos_log);

        nh->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        nh->get_parameter("pcd_save.pcd_save_en", pcd_save_en);

        nh->declare_parameter<int>("pcd_save.interval", -1);
        nh->get_parameter("pcd_save.interval", pcd_save_interval);

        // 坐标系名称参数（统一与系统其他部分一致）
        nh->declare_parameter<std::string>("frame.odom_frame", "odom");
        nh->get_parameter("frame.odom_frame", odom_frame);

        nh->declare_parameter<std::string>("frame.body_frame", "body");
        nh->get_parameter("frame.body_frame", body_frame);

        nh->declare_parameter<double>("mapping.lidar_time_inte", 0.1);
        nh->get_parameter("mapping.lidar_time_inte", lidar_time_inte);

        nh->declare_parameter<float>("mapping.ivox_grid_resolution", 0.2);
        nh->get_parameter("mapping.ivox_grid_resolution", ivox_options_.resolution_);

        nh->declare_parameter<int>("ivox_nearby_type", 18);
        nh->get_parameter("ivox_nearby_type", ivox_nearby_type);
    } catch (const rclcpp::ParameterTypeException& e) {
        RCLCPP_ERROR(nh->get_logger(), "Parameter type exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(nh->get_logger(), "Exception: %s", e.what());
    }

    nh->declare_parameter<bool>("filter_car_body", true);
    nh->get_parameter("filter_car_body", filter_car_body);

    nh->declare_parameter<double>("body_x_min", -0.4);
    nh->get_parameter("body_x_min", body_x_min);

    nh->declare_parameter<double>("body_x_max", 0.32);
    nh->get_parameter("body_x_max", body_x_max);

    nh->declare_parameter<double>("body_y_min", -0.3);
    nh->get_parameter("body_y_min", body_y_min);

    nh->declare_parameter<double>("body_y_max", 0.3);
    nh->get_parameter("body_y_max", body_y_max);

    nh->declare_parameter<double>("body_z_min", 0.0);
    nh->get_parameter("body_z_min", body_z_min);

    nh->declare_parameter<double>("body_z_max", 0.8);
    nh->get_parameter("body_z_max", body_z_max);

    validateBodyFilterParametersOrThrow(nh->get_logger());
    applyBodyFilterConfigToPreprocess();

    if (point_filter_num < 1) {
        RCLCPP_WARN(nh->get_logger(), "point_filter_num=%d < 1, clamp to 1", point_filter_num);
        point_filter_num = 1;
    }
    p_pre->point_filter_num = point_filter_num;

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    const std::vector<double> default_gravity = {0.0, 0.0, -9.81};
    const std::vector<double> default_extrin_t = {0.0, 0.0, 0.0};
    const std::vector<double> default_extrin_r = {1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0};

    ensureSizedFiniteVector(gravity, 3, default_gravity, "mapping.gravity", nh->get_logger());
    const double gravity_norm = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
    if (!std::isfinite(gravity_norm) || gravity_norm < 1e-3) {
        RCLCPP_WARN(nh->get_logger(), "mapping.gravity norm invalid (%.6f), fallback to [0,0,-9.81]", gravity_norm);
        gravity = default_gravity;
    }
    ensureSizedFiniteVector(gravity_init, 3, gravity, "mapping.gravity_init", nh->get_logger());
    ensureSizedFiniteVector(extrinT, 3, default_extrin_t, "mapping.extrinsic_T", nh->get_logger());
    ensureSizedFiniteVector(extrinR, 9, default_extrin_r, "mapping.extrinsic_R", nh->get_logger());

    if (!std::isfinite(acc_norm) || std::abs(acc_norm) < 1e-6) {
        RCLCPP_WARN(nh->get_logger(), "mapping.acc_norm=%.6f invalid, fallback to 1.0", acc_norm);
        acc_norm = 1.0;
    }

    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);

    RCLCPP_INFO(nh->get_logger(), "IMU config: topic=%s, acc_norm=%.2f", imu_topic.c_str(), acc_norm);
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3& rot) {
    double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    } else {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file() {
    const std::string log_dir = ensureLogDirectory();
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    if (fout_out && fout_imu_pbp)
        std::cout << "~~~~" << log_dir << " file opened" << '\n';
    else
        std::cout << "~~~~failed to open log files under " << log_dir << '\n';
}

void reset_cov(Eigen::Matrix<double, 24, 24>& P_init) {
    P_init = MD(24, 24)::Identity() * 0.1;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
}

void reset_cov_output(Eigen::Matrix<double, 30, 30>& P_init_output) {
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}
