#include "rc26_localization/localization.hpp"

#include "localization_internal.hpp"

#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace rc26_localization {

void LocalizationNode::loadGlobalMap(const std::string& file_name) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (file_name.empty()) {
        RCLCPP_ERROR(this->get_logger(), "PCD 文件路径为空，定位将不可用");
        map_loaded_ = false;
        target_ready_ = false;
        return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "无法读取 PCD 文件: %s，定位将不可用", file_name.c_str());
        map_loaded_ = false;
        target_ready_ = false;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "加载先验地图，共 %zu 个点", global_map_->points.size());
    map_loaded_ = true;
    target_ready_ = false;
    map_needs_transform_ = false;
}

bool LocalizationNode::prepareTargetMap() {
    if (!map_loaded_) {
        return false;
    }
    if (target_ready_ && target_tree_) {
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        target_ =
            small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
                *global_map_, global_leaf_size_);
    }

    size_t min_points_for_map =
        static_cast<size_t>(min_points_for_registration_ > 0 ? min_points_for_registration_ : 1);
    if (!target_ || target_->size() < min_points_for_map) {
        target_ready_ = false;
        target_tree_.reset();
        RCLCPP_ERROR(this->get_logger(), "先验地图点数不足: %zu < %zu，无法准备目标地图",
                     target_ ? target_->size() : static_cast<size_t>(0), min_points_for_map);
        return false;
    }
    small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
    target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    target_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "先验地图准备完成，目标点数: %zu", target_->points.size());

    if (enable_scan_context_ && !sc_db_ready_) {
        if (!buildScanContextDatabase()) {
            RCLCPP_WARN(this->get_logger(), "Scan Context 数据库构建失败，L2 将不可用");
        }
    }

    return true;
}

}  // namespace rc26_localization

