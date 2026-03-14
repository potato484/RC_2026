#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <cctype>

#include "localization_internal.hpp"

namespace rc26_localization {

rcl_interfaces::msg::SetParametersResult LocalizationNode::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    bool need_target_rebuild = false;
    bool need_sc_rebuild = false;

    auto reject = [&](const std::string& reason) {
        result.successful = false;
        result.reason = reason;
    };

    auto log_update = [&](const std::string& param, double old_value, double new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%.6f,new=%.6f",
                    param.c_str(), old_value, new_value);
    };

    auto log_update_int = [&](const std::string& param, int old_value, int new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%d,new=%d",
                    param.c_str(), old_value, new_value);
    };

    auto log_update_bool = [&](const std::string& param, bool old_value, bool new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%d,new=%d",
                    param.c_str(), static_cast<int>(old_value), static_cast<int>(new_value));
    };

    auto log_update_str = [&](const std::string& param, const std::string& old_value, const std::string& new_value) {
        RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%s,new=%s",
                    param.c_str(), old_value.c_str(), new_value.c_str());
    };

    for (const auto& p : parameters) {
        const std::string& name = p.get_name();

        if (name == "registered_leaf_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("registered_leaf_size expects double");
                break;
            }
            const double old_v = registered_leaf_size_;
            registered_leaf_size_ = static_cast<float>(std::clamp(p.as_double(), 0.01, 2.0));
            log_update(name, old_v, registered_leaf_size_);
            continue;
        }
        if (name == "max_dist_sq") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("max_dist_sq expects double");
                break;
            }
            const double old_v = max_dist_sq_;
            max_dist_sq_ = static_cast<float>(std::clamp(p.as_double(), 0.01, 25.0));
            log_update(name, old_v, max_dist_sq_);
            continue;
        }
        if (name == "robust_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("robust_enable expects bool");
                break;
            }
            const bool old_v = robust_enable_;
            robust_enable_ = p.as_bool();
            log_update_bool(name, old_v, robust_enable_);
            continue;
        }
        if (name == "huber_c") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("huber_c expects double");
                break;
            }
            const double old_v = huber_c_;
            huber_c_ = std::clamp(p.as_double(), 1e-6, 1e6);
            log_update(name, old_v, huber_c_);
            continue;
        }
        if (name == "cov_eig_floor") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("cov_eig_floor expects double");
                break;
            }
            const double old_v = cov_eig_floor_;
            cov_eig_floor_ = std::clamp(p.as_double(), 1e-6, 1e8);
            log_update(name, old_v, cov_eig_floor_);
            continue;
        }
        if (name == "cov_scale_min") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("cov_scale_min expects double");
                break;
            }
            const double old_v = cov_scale_min_;
            cov_scale_min_ = std::clamp(p.as_double(), 1e-8, 1e3);
            cov_scale_max_ = std::max(cov_scale_max_, cov_scale_min_);
            log_update(name, old_v, cov_scale_min_);
            continue;
        }
        if (name == "cov_scale_max") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("cov_scale_max expects double");
                break;
            }
            const double old_v = cov_scale_max_;
            cov_scale_max_ = std::clamp(p.as_double(), 1e-8, 1e6);
            cov_scale_min_ = std::min(cov_scale_min_, cov_scale_max_);
            log_update(name, old_v, cov_scale_max_);
            continue;
        }
        if (name == "gn_auto_trans_threshold_m") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("gn_auto_trans_threshold_m expects double");
                break;
            }
            const double old_v = gn_auto_trans_threshold_m_;
            gn_auto_trans_threshold_m_ = std::clamp(p.as_double(), 0.0, 10.0);
            log_update(name, old_v, gn_auto_trans_threshold_m_);
            continue;
        }
        if (name == "gicp_max_iterations") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("gicp_max_iterations expects integer");
                break;
            }
            const int old_v = gicp_max_iterations_;
            gicp_max_iterations_ = std::clamp(static_cast<int>(p.as_int()), 1, 500);
            log_update_int(name, old_v, gicp_max_iterations_);
            continue;
        }
        if (name == "min_points_for_registration") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("min_points_for_registration expects integer");
                break;
            }
            const int old_v = min_points_for_registration_;
            min_points_for_registration_ = std::max(1, static_cast<int>(p.as_int()));
            log_update_int(name, old_v, min_points_for_registration_);
            continue;
        }
        if (name == "acceptable_match_streak_to_recover") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("acceptable_match_streak_to_recover expects integer");
                break;
            }
            const int old_v = acceptable_match_streak_to_recover_;
            acceptable_match_streak_to_recover_ = std::clamp(static_cast<int>(p.as_int()), 1, 100);
            log_update_int(name, old_v, acceptable_match_streak_to_recover_);
            continue;
        }
        if (name == "good_match_streak_to_lock") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("good_match_streak_to_lock expects integer");
                break;
            }
            const int old_v = good_match_streak_to_lock_;
            good_match_streak_to_lock_ = std::clamp(static_cast<int>(p.as_int()), 1, 100);
            log_update_int(name, old_v, good_match_streak_to_lock_);
            continue;
        }
        if (name == "bad_match_streak_to_suspect") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("bad_match_streak_to_suspect expects integer");
                break;
            }
            const int old_v = bad_match_streak_to_suspect_;
            bad_match_streak_to_suspect_ = std::clamp(static_cast<int>(p.as_int()), 1, 100);
            log_update_int(name, old_v, bad_match_streak_to_suspect_);
            continue;
        }
        if (name == "low_confidence_streak_to_unlock") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("low_confidence_streak_to_unlock expects integer");
                break;
            }
            const int old_v = low_confidence_streak_to_unlock_;
            low_confidence_streak_to_unlock_ = std::clamp(static_cast<int>(p.as_int()), 1, 100);
            log_update_int(name, old_v, low_confidence_streak_to_unlock_);
            continue;
        }
        if (name == "locked_min_startup_sec") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("locked_min_startup_sec expects double");
                break;
            }
            const double old_v = locked_min_startup_sec_;
            locked_min_startup_sec_ = std::clamp(p.as_double(), 0.0, 60.0);
            log_update(name, old_v, locked_min_startup_sec_);
            continue;
        }
        if (name == "lock_good_normalized_error_max") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("lock_good_normalized_error_max expects double");
                break;
            }
            const double old_v = lock_good_normalized_error_max_;
            lock_good_normalized_error_max_ = std::clamp(p.as_double(), 1e-6, 10.0);
            log_update(name, old_v, lock_good_normalized_error_max_);
            continue;
        }
        if (name == "lock_good_min_inliers") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("lock_good_min_inliers expects integer");
                break;
            }
            const int old_v = lock_good_min_inliers_;
            lock_good_min_inliers_ = std::max(min_inliers_, std::clamp(static_cast<int>(p.as_int()), 0, 100000));
            log_update_int(name, old_v, lock_good_min_inliers_);
            continue;
        }
        if (name == "lock_jump_reject_translation_m") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("lock_jump_reject_translation_m expects double");
                break;
            }
            const double old_v = lock_jump_reject_translation_m_;
            lock_jump_reject_translation_m_ = std::clamp(p.as_double(), 0.01, 10.0);
            log_update(name, old_v, lock_jump_reject_translation_m_);
            continue;
        }
        if (name == "lock_jump_reject_yaw_deg") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("lock_jump_reject_yaw_deg expects double");
                break;
            }
            const double old_v = lock_jump_reject_yaw_deg_;
            lock_jump_reject_yaw_deg_ = std::clamp(p.as_double(), 0.1, 180.0);
            log_update(name, old_v, lock_jump_reject_yaw_deg_);
            continue;
        }
        if (name == "locked_pose_max_stale_sec") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("locked_pose_max_stale_sec expects double");
                break;
            }
            const double old_v = locked_pose_max_stale_sec_;
            locked_pose_max_stale_sec_ = std::clamp(p.as_double(), 0.1, 60.0);
            log_update(name, old_v, locked_pose_max_stale_sec_);
            continue;
        }
        if (name == "global_leaf_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("global_leaf_size expects double");
                break;
            }
            const double old_v = global_leaf_size_;
            global_leaf_size_ = static_cast<float>(std::clamp(p.as_double(), 0.01, 2.0));
            need_target_rebuild = true;
            log_update(name, old_v, global_leaf_size_);
            continue;
        }
        if (name == "num_neighbors") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("num_neighbors expects integer");
                break;
            }
            const int old_v = num_neighbors_;
            num_neighbors_ = std::max(1, static_cast<int>(p.as_int()));
            need_target_rebuild = true;
            log_update_int(name, old_v, num_neighbors_);
            continue;
        }
        if (name == "max_accumulated_points") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("max_accumulated_points expects integer");
                break;
            }
            const size_t old_v = max_accumulated_points_;
            max_accumulated_points_ = static_cast<size_t>(std::max<int64_t>(1000, p.as_int()));
            RCLCPP_INFO(this->get_logger(), "PARAM_UPDATE,node=localization,param=%s,old=%zu,new=%zu",
                        name.c_str(), old_v, max_accumulated_points_);
            continue;
        }
        if (name == "degen_eigenvalue_ratio_threshold") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("degen_eigenvalue_ratio_threshold expects double");
                break;
            }
            const double old_v = degen_eigenvalue_ratio_threshold_;
            degen_eigenvalue_ratio_threshold_ = std::clamp(p.as_double(), 1e-6, 1.0);
            log_update(name, old_v, degen_eigenvalue_ratio_threshold_);
            continue;
        }
        if (name == "degen_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("degen_enable expects bool");
                break;
            }
            const bool old_v = degen_enable_;
            degen_enable_ = p.as_bool();
            log_update_bool(name, old_v, degen_enable_);
            continue;
        }
        if (name == "sc_num_rings") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_num_rings expects integer");
                break;
            }
            const int old_v = sc_num_rings_;
            sc_num_rings_ = std::max(4, static_cast<int>(p.as_int()));
            need_sc_rebuild = true;
            log_update_int(name, old_v, sc_num_rings_);
            continue;
        }
        if (name == "sc_num_sectors") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_num_sectors expects integer");
                break;
            }
            const int old_v = sc_num_sectors_;
            sc_num_sectors_ = std::max(12, static_cast<int>(p.as_int()));
            need_sc_rebuild = true;
            log_update_int(name, old_v, sc_num_sectors_);
            continue;
        }
        if (name == "sc_max_radius") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("sc_max_radius expects double");
                break;
            }
            const double old_v = sc_max_radius_;
            sc_max_radius_ = std::clamp(p.as_double(), 0.5, 100.0);
            need_sc_rebuild = true;
            log_update(name, old_v, sc_max_radius_);
            continue;
        }
        if (name == "sc_submap_radius") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("sc_submap_radius expects double");
                break;
            }
            const double old_v = sc_submap_radius_;
            sc_submap_radius_ = std::clamp(p.as_double(), 0.5, 50.0);
            need_sc_rebuild = true;
            log_update(name, old_v, sc_submap_radius_);
            continue;
        }
        if (name == "sc_grid_resolution") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("sc_grid_resolution expects double");
                break;
            }
            const double old_v = sc_grid_resolution_;
            sc_grid_resolution_ = std::clamp(p.as_double(), 0.2, 10.0);
            need_sc_rebuild = true;
            log_update(name, old_v, sc_grid_resolution_);
            continue;
        }
        if (name == "sc_topk") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_topk expects integer");
                break;
            }
            const int old_v = sc_topk_;
            sc_topk_ = std::max(1, static_cast<int>(p.as_int()));
            log_update_int(name, old_v, sc_topk_);
            continue;
        }
        if (name == "sc_sim_threshold") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("sc_sim_threshold expects double");
                break;
            }
            const double old_v = sc_sim_threshold_;
            sc_sim_threshold_ = std::clamp(p.as_double(), 0.0, 1.0);
            log_update(name, old_v, sc_sim_threshold_);
            continue;
        }
        if (name == "sc_min_points_per_submap") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("sc_min_points_per_submap expects integer");
                break;
            }
            const int old_v = sc_min_points_per_submap_;
            sc_min_points_per_submap_ = std::max(10, static_cast<int>(p.as_int()));
            need_sc_rebuild = true;
            log_update_int(name, old_v, sc_min_points_per_submap_);
            continue;
        }
        if (name == "cov_from_hessian_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("cov_from_hessian_enable expects bool");
                break;
            }
            const bool old_v = cov_from_hessian_enable_;
            cov_from_hessian_enable_ = p.as_bool();
            log_update_bool(name, old_v, cov_from_hessian_enable_);
            continue;
        }
        if (name == "cov_scale_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("cov_scale_enable expects bool");
                break;
            }
            const bool old_v = cov_scale_enable_;
            cov_scale_enable_ = p.as_bool();
            log_update_bool(name, old_v, cov_scale_enable_);
            continue;
        }
        if (name == "hessian_degen_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("hessian_degen_enable expects bool");
                break;
            }
            const bool old_v = hessian_degen_enable_;
            hessian_degen_enable_ = p.as_bool();
            log_update_bool(name, old_v, hessian_degen_enable_);
            continue;
        }
        if (name == "hessian_lambda_hard") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("hessian_lambda_hard expects double");
                break;
            }
            const double old_v = hessian_lambda_hard_;
            hessian_lambda_hard_ = std::clamp(p.as_double(), 1e-6, 1e8);
            log_update(name, old_v, hessian_lambda_hard_);
            continue;
        }
        if (name == "dynamic_filter_voxel_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                reject("dynamic_filter_voxel_size expects double");
                break;
            }
            const double old_v = dynamic_filter_voxel_size_;
            dynamic_filter_voxel_size_ = std::clamp(p.as_double(), 0.05, 2.0);
            StaticVoxelFilter::Config cfg{dynamic_filter_voxel_size_, dynamic_filter_window_size_,
                                          dynamic_filter_stable_threshold_};
            static_voxel_filter_.setConfig(cfg);
            log_update(name, old_v, dynamic_filter_voxel_size_);
            continue;
        }
        if (name == "dynamic_filter_window_size") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("dynamic_filter_window_size expects integer");
                break;
            }
            const int old_v = dynamic_filter_window_size_;
            dynamic_filter_window_size_ = std::clamp(static_cast<int>(p.as_int()), 1, 200);
            StaticVoxelFilter::Config cfg{dynamic_filter_voxel_size_, dynamic_filter_window_size_,
                                          dynamic_filter_stable_threshold_};
            static_voxel_filter_.setConfig(cfg);
            log_update_int(name, old_v, dynamic_filter_window_size_);
            continue;
        }
        if (name == "dynamic_filter_stable_threshold") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("dynamic_filter_stable_threshold expects integer");
                break;
            }
            const int old_v = dynamic_filter_stable_threshold_;
            dynamic_filter_stable_threshold_ = std::clamp(static_cast<int>(p.as_int()), 1, 200);
            StaticVoxelFilter::Config cfg{dynamic_filter_voxel_size_, dynamic_filter_window_size_,
                                          dynamic_filter_stable_threshold_};
            static_voxel_filter_.setConfig(cfg);
            log_update_int(name, old_v, dynamic_filter_stable_threshold_);
            continue;
        }
        if (name == "esikf_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("esikf_enable expects bool");
                break;
            }
            const bool old_v = esikf_enable_;
            esikf_enable_ = p.as_bool();
            log_update_bool(name, old_v, esikf_enable_);
            continue;
        }
        if (name == "l0_enable") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                reject("l0_enable expects bool");
                break;
            }
            const bool old_v = l0_enable_;
            l0_enable_ = p.as_bool();
            log_update_bool(name, old_v, l0_enable_);
            continue;
        }
        if (name == "gicp_omp_threads") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                reject("gicp_omp_threads expects integer");
                break;
            }
            const int old_v = gicp_omp_threads_;
            gicp_omp_threads_ = std::max(1, static_cast<int>(p.as_int()));
            num_threads_ = gicp_omp_threads_;
            configureThreadAffinityQcs8550();
            log_update_int(name, old_v, gicp_omp_threads_);
            continue;
        }
        if (name == "gicp_optimizer_mode") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                reject("gicp_optimizer_mode expects string");
                break;
            }
            const std::string old_v = gicp_optimizer_mode_;
            std::string new_v = p.as_string();
            std::transform(new_v.begin(), new_v.end(), new_v.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (new_v != "gn_auto" && new_v != "gn" && new_v != "lm") {
                reject("gicp_optimizer_mode expects gn_auto|gn|lm");
                break;
            }
            gicp_optimizer_mode_ = new_v;
            log_update_str(name, old_v, gicp_optimizer_mode_);
            continue;
        }

        reject("parameter not in hot-update whitelist: " + name);
        break;
    }

    if (result.successful) {
        if (need_target_rebuild) {
            target_ready_ = false;
        }
        if (need_sc_rebuild) {
            sc_db_ready_ = false;
            std::lock_guard<std::mutex> lk(sc_mutex_);
            sc_database_.clear();
            sc_ring_index_.reset();
        }
    }

    return result;
}

}  // namespace rc26_localization
