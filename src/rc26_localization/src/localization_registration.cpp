#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

#include "localization_internal.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pcl/filters/filter.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace rc26_localization {

namespace {
constexpr std::array<int, 6> kPoseCovarianceOrder{3, 4, 5, 0, 1, 2};

Eigen::Matrix<double, 6, 6> reorderCovariance(const Eigen::Matrix<double, 6, 6>& covariance,
                                              const std::array<int, 6>& order) {
    Eigen::Matrix<double, 6, 6> reordered = Eigen::Matrix<double, 6, 6>::Zero();
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            reordered(row, col) = covariance(order[row], order[col]);
        }
    }
    return 0.5 * (reordered + reordered.transpose());
}
}  // namespace

void LocalizationNode::performRegistration() {
    if (!map_loaded_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "地图未加载，跳过配准");
        return;
    }

    if (!prepareTargetMap()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "地图坐标转换未完成");
        return;
    }

    const LocalizationState state = getLocalizationState();
    if (isRelocatingState(state)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "重定位进行中，跳过常规配准");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(reloc_request_mutex_);
        if (reloc_request_pending_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "重定位请求待处理，跳过常规配准");
            return;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_register;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (accumulated_cloud_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "无累积点云数据");
            return;
        }
        cloud_to_register = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::swap(cloud_to_register, accumulated_cloud_);
    }

    pcl::Indices nan_indices;
    pcl::removeNaNFromPointCloud(*cloud_to_register, *cloud_to_register, nan_indices);

    if (acrylic_filter_enable_) {
        applyAcrylicROIFilter(cloud_to_register);
    }

    source_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *cloud_to_register, registered_leaf_size_);

    size_t min_points_for_reg =
        static_cast<size_t>(min_points_for_registration_ > 0 ? min_points_for_registration_ : 1);
    if (!source_ || source_->size() < min_points_for_reg) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "降采样后点云不足: %zu < %zu，跳过配准",
                             source_ ? source_->size() : 0, min_points_for_reg);
        return;
    }

    small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        initial_guess = previous_result_t_;
    }

    if (dynamic_filter_enable_ && state == LocalizationState::TRACKING) {
        const auto static_mask = static_voxel_filter_.computeStaticMask(source_, initial_guess.matrix());
        if (static_mask.size() == source_->size()) {
            auto filtered_source = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
            filtered_source->reserve(source_->size());
            for (size_t i = 0; i < source_->size(); ++i) {
                if (static_mask[i]) {
                    filtered_source->push_back(source_->points[i]);
                }
            }
            if (filtered_source->size() >= min_points_for_reg) {
                source_ = filtered_source;
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "动态过滤后点数不足: %zu < %zu，跳过动态过滤结果", filtered_source->size(), min_points_for_reg);
            }
        }
    }

    DegenAnalysis degen_analysis;
    if (degen_enable_) {
        degen_analysis = analyzeObservability(source_);
    }
    last_degen_ = degen_analysis;

    auto configure_local_registration = [this](auto& reg) {
        reg->reduction.num_threads = num_threads_;
        reg->rejector.max_dist_sq = max_dist_sq_;
        reg->optimizer.max_iterations = gicp_max_iterations_;
        reg->point_factor.robust_kernel.c = robust_enable_ ? huber_c_ : 1e9;
    };
    configure_local_registration(register_lm_);
    configure_local_registration(register_gn_);

    const Eigen::Vector3d t_init = initial_guess.translation();
    const double init_jump_m = (t_init - last_t_init_).norm();
    const bool use_gn =
        (gicp_optimizer_mode_ == "gn") ||
        (gicp_optimizer_mode_ == "gn_auto" && init_jump_m < gn_auto_trans_threshold_m_);

    // T0: 局部配准计时起点
    const auto t_align_start = std::chrono::steady_clock::now();
    small_gicp::RegistrationResult result =
        use_gn ? register_gn_->align(*target_, *source_, *target_tree_, initial_guess)
               : register_lm_->align(*target_, *source_, *target_tree_, initial_guess);
    last_t_init_ = t_init;
    const double dt_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_align_start).count();

    // 新硬退化层：Hessian 最小特征值门控
    double h_min_eig = 0.0;
    if (result.converged && hessian_degen_enable_) {
        const Eigen::Matrix<double, 6, 6> H_sym = 0.5 * (result.H + result.H.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H_sym);
        if (solver.info() == Eigen::Success) {
            h_min_eig = solver.eigenvalues().minCoeff();
        }
        if (h_min_eig < hessian_lambda_hard_) {
            const int hard_consec = consecutive_s2_count_.fetch_add(1) + 1;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "Hard degen: lambda_min=%.2f < %.2f (consec=%d/%d)",
                h_min_eig, hessian_lambda_hard_, hard_consec, s2_max_continuous_frames_);

            Eigen::Matrix<double, 6, 6> hard_cov = Eigen::Matrix<double, 6, 6>::Zero();
            hard_cov.diagonal() << 1e6, 1e6, 1e6, 4.0, 4.0, 4.0;
            {
                std::lock_guard<std::mutex> lk(result_mutex_);
                last_pose_cov_ = hard_cov;
            }

            if (hard_consec >= s2_max_continuous_frames_) {
                consecutive_s2_count_.store(0);
                requestRelocalization(RelocTriggerReason::KIDNAP, cloud_to_register);
            }
            const double hard_norm_error =
                (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                         : std::numeric_limits<double>::max();
            publishDiagnostics(hard_norm_error, result.num_inliers, true, result);
            return;
        }
        consecutive_s2_count_.store(0);
    }

    // legacy S2: 仅在关闭新硬退化门控时启用
    double s2_min_eig = 0.0;
    int s2_consec = 0;
    if (!hessian_degen_enable_ && s2_enable_ && result.converged) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(0.5 * (result.H + result.H.transpose()));
        s2_min_eig = solver.eigenvalues().minCoeff();
        if (s2_min_eig < s2_hessian_min_eigenvalue_) {
            s2_consec = consecutive_s2_count_.fetch_add(1) + 1;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "S2: degenerate axis (min_eig=%.4f < %.4f), update rejected (consec=%d/%d)",
                s2_min_eig, s2_hessian_min_eigenvalue_, s2_consec, s2_max_continuous_frames_);
            if (s2_consec >= s2_max_continuous_frames_) {
                consecutive_s2_count_.store(0);
                RCLCPP_ERROR(get_logger(), "S2: %d frames continuous degeneration, forcing GLOBAL_RECOVERY",
                    s2_max_continuous_frames_);
                requestRelocalization(RelocTriggerReason::KIDNAP, cloud_to_register);
            }
            return;
        }
        consecutive_s2_count_.store(0);
    }

    const double normalized_error =
        (result.num_inliers > 0) ? (result.error / static_cast<double>(result.num_inliers))
                                 : std::numeric_limits<double>::max();
    const auto sigma_obs = buildObsCovariance(result);
    const auto sigma_pose = reorderCovariance(sigma_obs, kPoseCovarianceOrder);
    {
        std::lock_guard<std::mutex> lk(result_mutex_);
        last_pose_cov_ = sigma_obs;
    }

    Eigen::Isometry3d constrained_pose = result.T_target_source;
    if (degen_enable_ && result.converged) {
        constrained_pose = constrainUpdate(result.T_target_source, initial_guess, last_degen_);
    }

    // S1: IMU Spike 门控（种子更新前，同时暂停绑架计数）
    if (s1_enable_ && imu_spike_active_.load()) {
        rclcpp::Time deadline;
        { std::lock_guard<std::mutex> lk(imu_spike_mutex_); deadline = imu_spike_deadline_; }
        if (now() < deadline) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "S1: IMU spike gate active, seed frozen");
            return;
        }
        imu_spike_active_.store(false);
    }
    {
        std::lock_guard<std::mutex> lk(imu_spike_mutex_);
        const rclcpp::Time now_time = now();
        if (imu_spike_last_stamp_.nanoseconds() <= 0 ||
            imu_spike_last_stamp_.get_clock_type() != now_time.get_clock_type()) {
            imu_spike_recent_.store(false);
        } else if ((now_time - imu_spike_last_stamp_).seconds() * 1000.0 > static_cast<double>(l0_max_imu_gap_ms_)) {
            imu_spike_recent_.store(false);
        }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "配准: converged=%d, inliers=%zu, normalized_error=%.4f (阈值=%.4f)", result.converged,
                         result.num_inliers, normalized_error, kidnap_fitness_threshold_);

    if (detectKidnapping(normalized_error, result.num_inliers, last_degen_)) {
        RCLCPP_WARN(this->get_logger(), "检测到绑架，提交重定位请求...");
        requestRelocalization(RelocTriggerReason::KIDNAP, cloud_to_register);
        publishDiagnostics(normalized_error, result.num_inliers, true, result);
        return;
    }

    const bool bad_quality =
        (normalized_error > freeze_update_err_) || (result.num_inliers < static_cast<size_t>(min_inliers_));
    Eigen::Isometry3d slope_corrected_pose = constrained_pose;

    // T8: 坡道法向一致性门控 + 姿态主导权（transform 更新前）
    if (!bad_quality && result.converged && slope_roll_pitch_from_imu_ && imu_attitude_valid_) {
        const Eigen::Matrix3d R = constrained_pose.rotation();
        const double pitch_gicp = std::asin(-R(2, 0));
        const double roll_gicp  = std::atan2(R(2, 1), R(2, 2));
        double imu_r, imu_p;
        {
            std::lock_guard<std::mutex> lk(imu_attitude_mutex_);
            imu_r = imu_roll_;
            imu_p = imu_pitch_;
        }
        const double roll_dev  = roll_gicp - imu_r;
        const double pitch_dev = pitch_gicp - imu_p;
        const double dev_deg = std::sqrt(roll_dev * roll_dev + pitch_dev * pitch_dev) * 180.0 / M_PI;
        if (dev_deg > slope_normal_consistency_deg_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "T8: slope normal inconsistency %.1f deg > %.1f deg, frame rejected",
                dev_deg, slope_normal_consistency_deg_);
            return;
        }

        // roll/pitch 以 IMU 为主，仅允许 GICP 在有限幅度内修正
        const double max_correction_rad = kMaxSlopeRollPitchCorrectionDeg * M_PI / 180.0;
        const double fused_roll = imu_r + std::clamp(roll_dev, -max_correction_rad, max_correction_rad);
        const double fused_pitch = imu_p + std::clamp(pitch_dev, -max_correction_rad, max_correction_rad);
        const double yaw_gicp = std::atan2(R(1, 0), R(0, 0));
        slope_corrected_pose.linear() =
            (Eigen::AngleAxisd(yaw_gicp, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(fused_pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(fused_roll, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();

        // 将 slope_z_weight 用于 z 轴更新抑制，减小坡道场景 z 抖动
        const double prev_z = initial_guess.translation().z();
        const double z_alpha = 1.0 / slope_z_weight_;
        slope_corrected_pose.translation().z() =
            prev_z + (slope_corrected_pose.translation().z() - prev_z) * z_alpha;
    }

    if (!bad_quality) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (result.converged) {
            Eigen::Isometry3d committed_pose = slope_corrected_pose;
            if (esikf_enable_) {
                std::lock_guard<std::mutex> lk(esikf_mutex_);
                esikf_.update(slope_corrected_pose.matrix(), sigma_pose);
                committed_pose = esikf_.getMapToOdom();
            }
            result_t_ = previous_result_t_ = committed_pose;
        } else {
            Eigen::Vector3d delta_translation = constrained_pose.translation() - previous_result_t_.translation();

            Eigen::Quaterniond q_result(constrained_pose.rotation());
            Eigen::Quaterniond q_prev(previous_result_t_.rotation());
            Eigen::Quaterniond q_diff = q_result * q_prev.inverse();
            if (q_diff.w() < 0) {
                q_diff.coeffs() = -q_diff.coeffs();
            }
            double delta_rotation = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.w())));

            if (delta_translation.norm() < max_delta_translation_ && delta_rotation < max_delta_rotation_) {
                previous_result_t_ = constrained_pose;
                RCLCPP_WARN(this->get_logger(), "GICP 配准未收敛，偏移量 %.3fm / %.2f° 可接受，更新初值",
                            delta_translation.norm(), delta_rotation * 180.0 / M_PI);
            } else {
                RCLCPP_WARN(this->get_logger(), "GICP 配准未收敛，偏移量 %.3fm / %.2f° 过大，保持原初值",
                            delta_translation.norm(), delta_rotation * 180.0 / M_PI);
            }
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "配准质量低 (error=%.4f, inliers=%zu < %d), 冻结 TF 更新", normalized_error,
                             result.num_inliers, min_inliers_);
        setLocalizationState(LocalizationState::SUSPECT, "local_quality_bad");
    }

    if (result.converged && !bad_quality) {
        {
            std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
            last_successful_registration_time_ = this->now();
        }
        setLocalizationState(LocalizationState::TRACKING, "local_registration_ok");
    }

    rclcpp::Time last_success_snapshot;
    {
        std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
        last_success_snapshot = last_successful_registration_time_;
    }
    const double time_since_last_success = (this->now() - last_success_snapshot).seconds();
    if (time_since_last_success > registration_timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(), "配准失败超时 %.1f秒，提交重定位请求...", time_since_last_success);
        requestRelocalization(RelocTriggerReason::TIMEOUT, cloud_to_register);
    }

    // T0: PERF_METRIC 可观测性日志
    RCLCPP_INFO(get_logger(),
        "PERF_METRIC phase=LOCAL dt_ms=%.1f inliers=%d norm_err=%.4f state=%s opt=%s init_jump=%.3f "
        "S1=%d hard_hmin=%.2f hard_consec=%d S2=%d(eig=%.2f,consec=%d) degen=(%.2f,%.2f,%.2f)",
        dt_ms, static_cast<int>(result.num_inliers), normalized_error,
        toString(getLocalizationState()),
        use_gn ? "gn" : "lm",
        init_jump_m,
        static_cast<int>(imu_spike_active_.load()),
        h_min_eig,
        consecutive_s2_count_.load(),
        static_cast<int>(!hessian_degen_enable_ && s2_enable_ && result.converged &&
                         s2_min_eig < s2_hessian_min_eigenvalue_),
        s2_min_eig, s2_consec,
        last_degen_.degen_risk.x(), last_degen_.degen_risk.y(), last_degen_.degen_risk.z());

    publishDiagnostics(normalized_error, result.num_inliers, bad_quality, result);
}

LocalizationNode::DegenAnalysis
LocalizationNode::analyzeObservability(const pcl::PointCloud<pcl::PointCovariance>::Ptr& source) const {
    DegenAnalysis analysis;
    if (!source || source->empty()) {
        return analysis;
    }

    Eigen::Matrix3d hessian_2d = Eigen::Matrix3d::Zero();
    int valid_rows = 0;
    for (const auto& pt : source->points) {
        const Eigen::Matrix3d cov = pt.cov.block<3, 3>(0, 0).cast<double>();
        if (!cov.allFinite()) {
            continue;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> cov_solver(cov);
        if (cov_solver.info() != Eigen::Success) {
            continue;
        }
        Eigen::Vector3d normal = cov_solver.eigenvectors().col(0);
        if (!normal.allFinite() || normal.squaredNorm() < kNearZero) {
            continue;
        }
        normal.normalize();

        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::RowVector3d jac;
        jac << normal.x(), normal.y(), (-normal.x() * p.y() + normal.y() * p.x());
        hessian_2d.noalias() += jac.transpose() * jac;
        ++valid_rows;
    }

    if (valid_rows < 10) {
        return analysis;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(hessian_2d);
    if (solver.info() != Eigen::Success) {
        return analysis;
    }

    const Eigen::Vector3d eigvals = solver.eigenvalues();
    const Eigen::Matrix3d eigvecs = solver.eigenvectors();
    const double max_eig = std::max(eigvals.maxCoeff(), kNearZero);

    Eigen::Matrix3d P_obs = Eigen::Matrix3d::Identity();
    int degen_axes = 0;
    for (int i = 0; i < 3; ++i) {
        const double ratio = std::clamp(eigvals(i) / max_eig, 0.0, 1.0);
        const double degen_strength = 1.0 - ratio;
        const Eigen::Vector3d axis = eigvecs.col(i);

        analysis.degen_risk.x() = std::max(analysis.degen_risk.x(), degen_strength * std::abs(axis.x()));
        analysis.degen_risk.y() = std::max(analysis.degen_risk.y(), degen_strength * std::abs(axis.y()));
        analysis.degen_risk.z() = std::max(analysis.degen_risk.z(), degen_strength * std::abs(axis.z()));

        if (ratio < degen_eigenvalue_ratio_threshold_) {
            P_obs.noalias() -= axis * axis.transpose();
            ++degen_axes;
        }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> proj_solver((P_obs + P_obs.transpose()) * 0.5);
    if (proj_solver.info() == Eigen::Success) {
        const Eigen::Vector3d clamped = proj_solver.eigenvalues().cwiseMax(0.0).cwiseMin(1.0);
        analysis.P_obs = proj_solver.eigenvectors() * clamped.asDiagonal() * proj_solver.eigenvectors().transpose();
    } else {
        analysis.P_obs = Eigen::Matrix3d::Identity();
    }
    analysis.degen_risk = analysis.degen_risk.cwiseMax(0.0).cwiseMin(1.0);
    analysis.is_fully_degenerate = (degen_axes >= 2);
    return analysis;
}

Eigen::Isometry3d LocalizationNode::constrainUpdate(const Eigen::Isometry3d& aligned_pose,
                                                    const Eigen::Isometry3d& initial_guess,
                                                    const DegenAnalysis& degen) const {
    if (!degen_enable_) {
        return aligned_pose;
    }

    const Eigen::Isometry3d delta = initial_guess.inverse() * aligned_pose;
    const double dyaw = std::atan2(delta.rotation()(1, 0), delta.rotation()(0, 0));
    const Eigen::Vector3d delta_2d(delta.translation().x(), delta.translation().y(), dyaw);
    const Eigen::Vector3d constrained_2d = degen.P_obs * delta_2d;

    const Eigen::Vector3d euler_zyx = delta.rotation().eulerAngles(2, 1, 0);
    const double pitch = euler_zyx[1];
    const double roll = euler_zyx[2];

    Eigen::Isometry3d constrained_delta = Eigen::Isometry3d::Identity();
    constrained_delta.linear() = (Eigen::AngleAxisd(constrained_2d.z(), Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                                     .toRotationMatrix();
    constrained_delta.translation() << constrained_2d.x(), constrained_2d.y(), delta.translation().z();
    return initial_guess * constrained_delta;
}

void LocalizationNode::computeHessianStats(const Eigen::Matrix<double, 6, 6>& H,
                                           double& min_eig, double& max_eig, double& cond) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
    if (es.info() != Eigen::Success) {
        min_eig = 0.0;
        max_eig = 0.0;
        cond = 1e12;
        return;
    }
    const auto& ev = es.eigenvalues();
    min_eig = ev.minCoeff();
    max_eig = ev.maxCoeff();
    cond = (min_eig > 1e-12) ? max_eig / min_eig : 1e12;
}

Eigen::Matrix<double, 6, 6> LocalizationNode::computeObsCov(
    const small_gicp::RegistrationResult& result) const {
    const Eigen::Matrix<double, 6, 6> H = 0.5 * (result.H + result.H.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
    if (es.info() != Eigen::Success) {
        return Eigen::Matrix<double, 6, 6>::Identity() * kDiagObsNoiseDegenerate;
    }

    Eigen::Matrix<double, 6, 1> eigs_inv;
    const auto& eigs = es.eigenvalues();
    for (int i = 0; i < 6; ++i) {
        eigs_inv(i) = 1.0 / std::max(eigs(i), cov_eig_floor_);
    }

    Eigen::Matrix<double, 6, 6> sigma =
        es.eigenvectors() * eigs_inv.asDiagonal() * es.eigenvectors().transpose();

    if (cov_scale_enable_) {
        const int dof = std::max(static_cast<int>(result.num_inliers) - 6, 1);
        const double raw_s2 = std::isfinite(result.error) ? (2.0 * result.error / static_cast<double>(dof)) : cov_scale_max_;
        const double s2 = std::clamp(raw_s2, cov_scale_min_, cov_scale_max_);
        sigma *= s2;
    }

    sigma = 0.5 * (sigma + sigma.transpose());
    if (!sigma.allFinite()) {
        return Eigen::Matrix<double, 6, 6>::Identity() * kDiagObsNoiseDegenerate;
    }
    return sigma;
}

Eigen::Matrix<double, 6, 6> LocalizationNode::buildObsCovariance(
    const small_gicp::RegistrationResult& result) const {
    if (!cov_from_hessian_enable_) {
        Eigen::Matrix<double, 6, 6> R_obs = Eigen::Matrix<double, 6, 6>::Identity() * kDiagObsNoiseNominal;
        R_obs(2, 2) = 0.05;
        R_obs(3, 3) = 0.05;
        R_obs(4, 4) = 0.05;

        const std::array<int, 3> obs_indices{3, 4, 2};
        for (size_t i = 0; i < obs_indices.size(); ++i) {
            const double risk = last_degen_.degen_risk(static_cast<Eigen::Index>(i));
            R_obs(obs_indices[i], obs_indices[i]) = (risk > 0.5) ? kDiagObsNoiseDegenerate : kDiagObsNoiseNominal;
        }
        return R_obs;
    }
    return computeObsCov(result);
}

void LocalizationNode::publishPoseWithCov(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& cov) const {
    if (!pose_cov_pub_) {
        return;
    }
    const Eigen::Matrix<double, 6, 6> ros_cov = reorderCovariance(cov, kPoseCovarianceOrder);
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = pose.translation().x();
    msg.pose.pose.position.y = pose.translation().y();
    msg.pose.pose.position.z = pose.translation().z();

    const Eigen::Quaterniond q(pose.rotation());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    for (size_t r = 0; r < 6; ++r) {
        for (size_t c = 0; c < 6; ++c) {
            msg.pose.covariance[r * 6 + c] = ros_cov(r, c);
        }
    }
    pose_cov_pub_->publish(msg);
}

void LocalizationNode::publishDiagnostics(double normalized_error, size_t inliers, bool bad_quality,
                                          const small_gicp::RegistrationResult& result) const {
    if (!diag_pub_) {
        return;
    }

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rc26_localization";
    status.hardware_id = "R2";
    status.level = bad_quality ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                               : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = bad_quality ? "localization_suspect" : "localization_ok";

    auto kv = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
    };

    kv("state", toString(getLocalizationState()));
    kv("normalized_error", std::to_string(normalized_error));
    kv("inliers", std::to_string(inliers));
    kv("degen_risk_x", std::to_string(last_degen_.degen_risk.x()));
    kv("degen_risk_y", std::to_string(last_degen_.degen_risk.y()));
    kv("degen_risk_yaw", std::to_string(last_degen_.degen_risk.z()));
    kv("fully_degenerate", last_degen_.is_fully_degenerate ? "1" : "0");
    Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
    {
        std::lock_guard<std::mutex> lk(result_mutex_);
        pose_cov = last_pose_cov_;
    }
    double h_min = 0.0;
    double h_max = 0.0;
    double h_cond = 1e12;
    computeHessianStats(0.5 * (result.H + result.H.transpose()), h_min, h_max, h_cond);
    const double sigma_xy = std::sqrt(std::max(0.0, pose_cov(3, 3) + pose_cov(4, 4)));
    const double sigma_yaw = std::sqrt(std::max(0.0, pose_cov(2, 2)));
    kv("h_min_eig", std::to_string(h_min));
    kv("h_max_eig", std::to_string(h_max));
    kv("h_cond", std::to_string(h_cond));
    kv("sigma_xy", std::to_string(sigma_xy));
    kv("sigma_yaw", std::to_string(sigma_yaw));
    kv("obs_cov_source", cov_from_hessian_enable_ ? "hessian" : "hardcoded");
    kv("hard_degen_consec", std::to_string(consecutive_s2_count_.load()));

    diag.status.push_back(status);
    diag_pub_->publish(diag);
}

void LocalizationNode::publishTransform() {
    if (!map_loaded_) {
        return;
    }

    Eigen::Isometry3d result_snapshot;
    Eigen::Matrix<double, 6, 6> cov_snapshot = Eigen::Matrix<double, 6, 6>::Zero();
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_snapshot = result_t_;
        cov_snapshot = last_pose_cov_;
    }

    if (result_snapshot.matrix().isZero()) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = odom_frame_;

    const Eigen::Vector3d translation = result_snapshot.translation();
    const Eigen::Quaterniond rotation(result_snapshot.rotation());

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);

    publishPoseWithCov(result_snapshot, cov_snapshot);
}

bool LocalizationNode::detectKidnapping(double fitness_score, size_t num_inliers, const DegenAnalysis& degen) {
    if (isRelocatingState(getLocalizationState())) {
        return false;
    }

    if (degen_enable_) {
        const bool kidnap =
            degen.is_fully_degenerate && fitness_score > kidnap_fitness_threshold_ &&
            num_inliers < static_cast<size_t>(std::max(min_inliers_, 0));
        if (kidnap) {
            setLocalizationState(LocalizationState::SUSPECT, "degen_and_error_and_low_inliers");
            RCLCPP_WARN(this->get_logger(),
                        "退化触发绑架: full_degen=1, norm_err=%.4f(阈值=%.4f), inliers=%zu(<%d)", fitness_score,
                        kidnap_fitness_threshold_, num_inliers, min_inliers_);
            return true;
        }
        return false;
    }

    if (fitness_score > kidnap_fitness_threshold_) {
        const int count = consecutive_high_error_count_.fetch_add(1) + 1;
        setLocalizationState(LocalizationState::SUSPECT, "high_error_counting");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "配准误差过高: %.4f (连续 %d/%d 帧)",
                             fitness_score, count, kidnap_threshold_count_);
        if (count >= kidnap_threshold_count_) {
            consecutive_high_error_count_.store(0);
            return true;
        }
    } else {
        consecutive_high_error_count_.store(0);
    }
    return false;
}

std::vector<Eigen::Matrix4f> LocalizationNode::generateCandidateTransforms(const Eigen::Matrix4f& sac_transform) {
    std::vector<Eigen::Matrix4f> candidates;
    candidates.push_back(sac_transform);

    int yaw_hypotheses = num_yaw_hypotheses_;
    if (yaw_hypotheses < 1) {
        yaw_hypotheses = 1;
    }

    if (!use_multi_hypothesis_) {
        return candidates;
    }

    const double yaw_step = 2.0 * M_PI / static_cast<double>(yaw_hypotheses);
    for (int i = 1; i < yaw_hypotheses; ++i) {
        const double yaw_offset = i * yaw_step;
        Eigen::Matrix4f perturbed = sac_transform;
        Eigen::AngleAxisf rot(static_cast<float>(yaw_offset), Eigen::Vector3f::UnitZ());
        perturbed.block<3, 3>(0, 0) = perturbed.block<3, 3>(0, 0) * rot.matrix();
        candidates.push_back(perturbed);
    }

    return candidates;
}

}  // namespace rc26_localization
