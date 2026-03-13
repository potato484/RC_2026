#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <thread>

#include "localization_internal.hpp"

#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/search/kdtree.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace rc26_localization {

namespace {
constexpr int kScanContextYawWindow = 3;
}  // namespace

void LocalizationNode::publishRelocMetrics(const RelocMetrics& metrics) const {
    RCLCPP_INFO(this->get_logger(),
                "RELOC_METRIC,trigger_reason=%s,path_used=%s,t_total_ms=%.2f,t_l0_ms=%.2f,t_l1_ms=%.2f,t_l2_ms=%.2f,"
                "candidate_count=%d,best_fitness=%.6f,best_J=%.6f,winner_channel=%s,cancel_reason=%s,accepted=%d",
                toString(metrics.trigger_reason), metrics.path_used.c_str(), metrics.t_total_ms, metrics.t_l0_ms,
                metrics.t_l1_ms, metrics.t_l2_ms, metrics.candidate_count, metrics.best_fitness, metrics.best_j,
                metrics.winner_channel.c_str(), metrics.cancel_reason.c_str(), metrics.accepted ? 1 : 0);
}

void LocalizationNode::requestRelocalization(RelocTriggerReason reason, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud) {
    if (shutdown_requested_.load()) {
        return;
    }

    const rclcpp::Time request_stamp = this->now();
    Eigen::Isometry3d request_odom_to_base = Eigen::Isometry3d::Identity();
    const bool request_odom_valid = tryLookupOdomToBase(request_stamp, request_odom_to_base);

    {
        std::lock_guard<std::mutex> lock(reloc_request_mutex_);
        reloc_pending_reason_ = reason;
        if (source_cloud && !source_cloud->empty()) {
            reloc_pending_cloud_ = source_cloud;
        } else {
            reloc_pending_cloud_.reset();
        }
        reloc_request_odom_to_base_ = request_odom_to_base;
        reloc_request_odom_valid_ = request_odom_valid;
        reloc_request_stamp_ = request_stamp;
        reloc_request_pending_ = true;
    }

    if (!request_odom_valid) {
        RCLCPP_WARN(this->get_logger(), "重定位请求: 无法获取请求时刻 odom->base TF，成功后将回退到未补偿写回");
    }

    setLocalizationState(LocalizationState::SUSPECT, "reloc_requested");
    reloc_request_cv_.notify_one();
}

double LocalizationNode::computeCandidateCost(double fitness, const Eigen::Matrix4f& seed,
                                              const Eigen::Matrix4f& refined) const {
    const double fitness_th = std::max(global_fitness_threshold_, kNearZero);
    const double xy_th = std::max(retry_zone_max_xy_offset_, kNearZero);
    const double yaw_th = std::max(retry_zone_max_yaw_offset_deg_, kNearZero);

    const Eigen::Vector2d seed_xy(seed(0, 3), seed(1, 3));
    const Eigen::Vector2d refined_xy(refined(0, 3), refined(1, 3));
    const double dxy = (refined_xy - seed_xy).norm();

    const double seed_yaw = std::atan2(static_cast<double>(seed(1, 0)), static_cast<double>(seed(0, 0)));
    const double refined_yaw = std::atan2(static_cast<double>(refined(1, 0)), static_cast<double>(refined(0, 0)));
    const double dyaw = std::atan2(std::sin(refined_yaw - seed_yaw), std::cos(refined_yaw - seed_yaw));
    const double dyaw_deg = std::abs(dyaw) * 180.0 / M_PI;

    return kCostWf * (fitness / fitness_th) + kCostWxy * std::pow(dxy / xy_th, 2.0) +
           kCostWyaw * std::pow(dyaw_deg / yaw_th, 2.0);
}

bool LocalizationNode::tryGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                        Eigen::Isometry3d& best_pose, double& best_fitness, double& best_cost,
                                        int& candidate_count, const std::atomic<bool>* stop_flag) {
    if (bevplace_enable_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "BEVPlace++ 通道尚未启用，回退 Scan Context");
    }
    return tryScanContextGlobalChannel(source_down, target_down, best_pose, best_fitness, best_cost, candidate_count,
                                       stop_flag);
}

bool LocalizationNode::tryL0FastChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                        Eigen::Isometry3d& best_pose, double& best_fitness, double& best_cost,
                                        int& candidate_count, const std::atomic<bool>* stop_flag) {
    (void)target_down;
    if (stop_flag && stop_flag->load()) {
        return false;
    }
    if (!source_down || source_down->empty()) {
        return false;
    }

    auto source_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *source_down, registered_leaf_size_);
    if (!source_cov || source_cov->empty()) {
        return false;
    }
    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors_, num_threads_);

    Eigen::Isometry3d seed = Eigen::Isometry3d::Identity();
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        seed = previous_result_t_;
    }

    bool seed_ready = false;
    if (esikf_enable_) {
        std::lock_guard<std::mutex> lk(esikf_mutex_);
        seed = esikf_.getMapToOdom();
        seed_ready = true;
    } else {
        std::deque<ImuSample> samples;
        {
            std::lock_guard<std::mutex> lk(imu_buffer_mutex_);
            samples = imu_buffer_;
        }
        if (!samples.empty()) {
            const double stale_ms = (now() - samples.back().stamp).seconds() * 1000.0;
            if (stale_ms <= static_cast<double>(l0_max_imu_gap_ms_)) {
                double yaw = std::atan2(seed.rotation()(1, 0), seed.rotation()(0, 0));
                Eigen::Vector2d velocity_xy = Eigen::Vector2d::Zero();
                for (size_t i = 1; i < samples.size(); ++i) {
                    if (stop_flag && stop_flag->load()) {
                        return false;
                    }
                    double dt = (samples[i].stamp - samples[i - 1].stamp).seconds();
                    if (dt <= 0.0 || dt > 0.1) {
                        continue;
                    }
                    yaw += samples[i - 1].gyro.z() * dt;
                    const Eigen::Rotation2Dd rot(yaw);
                    const Eigen::Vector2d accel_xy = rot * samples[i - 1].accel.head<2>();
                    seed.translation().x() += velocity_xy.x() * dt + 0.5 * accel_xy.x() * dt * dt;
                    seed.translation().y() += velocity_xy.y() * dt + 0.5 * accel_xy.y() * dt * dt;
                    velocity_xy += accel_xy * dt;
                }
                const Eigen::Vector3d euler_zyx = seed.rotation().eulerAngles(2, 1, 0);
                seed.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(euler_zyx[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(euler_zyx[2], Eigen::Vector3d::UnitX()))
                                    .toRotationMatrix();
                seed_ready = true;
            }
        }
    }

    if (!seed_ready) {
        return false;
    }

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq =
        4.0 * global_icp_max_correspondence_distance_ * global_icp_max_correspondence_distance_;
    registration.optimizer.max_iterations = std::max(global_icp_max_iterations_, gicp_max_iterations_ * 2);
    auto result = registration.align(*target_, *source_cov, *target_tree_, seed);
    if (!result.converged || result.num_inliers == 0) {
        return false;
    }

    candidate_count = 1;
    best_pose = result.T_target_source;
    best_fitness = result.error / static_cast<double>(result.num_inliers);
    best_cost = best_fitness;

    if (best_fitness < global_fitness_threshold_) {
        std::lock_guard<std::mutex> lk(imu_buffer_mutex_);
        imu_buffer_.clear();
        imu_spike_recent_.store(false);
        return true;
    }
    return false;
}

bool LocalizationNode::buildScanContextDatabase() {
    if (!enable_scan_context_) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(sc_mutex_);
        if (sc_db_ready_) {
            return true;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!global_map_ || global_map_->empty()) {
            return false;
        }
        map_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*global_map_);
    }

    pcl::Indices map_indices;
    pcl::removeNaNFromPointCloud(*map_copy, *map_copy, map_indices);
    if (map_copy->empty()) {
        return false;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& pt : map_copy->points) {
        min_x = std::min(min_x, static_cast<double>(pt.x));
        min_y = std::min(min_y, static_cast<double>(pt.y));
        max_x = std::max(max_x, static_cast<double>(pt.x));
        max_y = std::max(max_y, static_cast<double>(pt.y));
    }

    if (!(min_x < max_x && min_y < max_y)) {
        return false;
    }

    std::vector<ScanContextEntry> database;
    pcl::search::KdTree<pcl::PointXYZ> map_tree;
    map_tree.setInputCloud(map_copy);
    const float radius = static_cast<float>(sc_submap_radius_);

    for (double cx = min_x; cx <= max_x; cx += sc_grid_resolution_) {
        for (double cy = min_y; cy <= max_y; cy += sc_grid_resolution_) {
            pcl::PointXYZ query;
            query.x = static_cast<float>(cx);
            query.y = static_cast<float>(cy);
            query.z = 0.0F;

            std::vector<int> nn_indices;
            std::vector<float> nn_dist_sq;
            map_tree.radiusSearch(query, radius, nn_indices, nn_dist_sq);
            if (nn_indices.size() < static_cast<size_t>(sc_min_points_per_submap_)) {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>());
            submap->points.reserve(nn_indices.size());
            for (int idx : nn_indices) {
                if (idx >= 0 && static_cast<size_t>(idx) < map_copy->points.size()) {
                    submap->points.push_back(map_copy->points[static_cast<size_t>(idx)]);
                }
            }

            if (submap->size() < static_cast<size_t>(sc_min_points_per_submap_)) {
                continue;
            }

            ScanContextEntry entry;
            entry.center_xy = Eigen::Vector2d(cx, cy);
            entry.descriptor = makeScanContextDescriptor(submap, entry.center_xy);
            entry.ring_key = makeRingKey(entry.descriptor);
            entry.sector_key = makeSectorKey(entry.descriptor);
            if (entry.ring_key.size() == 0) {
                continue;
            }
            database.push_back(std::move(entry));
        }
    }

    if (database.empty()) {
        return false;
    }

    auto ring_index = std::make_shared<ScanContextRingKeyIndex>();
    std::vector<Eigen::VectorXf> ring_keys;
    ring_keys.reserve(database.size());
    for (const auto& entry : database) {
        ring_keys.push_back(entry.ring_key);
    }
    if (!ring_index->build(ring_keys)) {
        ring_index.reset();
        RCLCPP_WARN(this->get_logger(), "Scan Context ring_key KD-tree 构建失败，回退线性检索");
    }

    {
        std::lock_guard<std::mutex> lock(sc_mutex_);
        sc_database_ = std::move(database);
        sc_ring_index_ = ring_index;
        sc_db_ready_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Scan Context 数据库构建完成: entries=%zu, ring_index=%s", sc_database_.size(),
                ring_index ? "ready" : "fallback_linear");
    return true;
}

Eigen::MatrixXf LocalizationNode::makeScanContextDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                            const Eigen::Vector2d& center_xy) const {
    Eigen::MatrixXf descriptor = Eigen::MatrixXf::Zero(sc_num_rings_, sc_num_sectors_);
    if (!cloud || cloud->empty()) {
        return descriptor;
    }

    for (const auto& pt : cloud->points) {
        const double dx = static_cast<double>(pt.x) - center_xy.x();
        const double dy = static_cast<double>(pt.y) - center_xy.y();
        const double r = std::hypot(dx, dy);
        if (r < kNearZero || r > sc_max_radius_) {
            continue;
        }

        double theta = std::atan2(dy, dx);
        if (theta < 0.0) {
            theta += 2.0 * M_PI;
        }

        const int ring_idx = std::min(sc_num_rings_ - 1, static_cast<int>((r / sc_max_radius_) * sc_num_rings_));
        const int sector_idx =
            std::min(sc_num_sectors_ - 1, static_cast<int>((theta / (2.0 * M_PI)) * sc_num_sectors_));

        descriptor(ring_idx, sector_idx) = std::max(descriptor(ring_idx, sector_idx), static_cast<float>(pt.z));
    }

    const float max_abs = descriptor.cwiseAbs().maxCoeff();
    if (max_abs > static_cast<float>(kNearZero)) {
        descriptor /= max_abs;
    }

    return descriptor;
}

Eigen::VectorXf LocalizationNode::makeRingKey(const Eigen::MatrixXf& descriptor) const {
    if (descriptor.rows() == 0 || descriptor.cols() == 0) {
        return Eigen::VectorXf();
    }
    return descriptor.rowwise().mean();
}

Eigen::VectorXf LocalizationNode::makeSectorKey(const Eigen::MatrixXf& descriptor) const {
    if (descriptor.rows() == 0 || descriptor.cols() == 0) {
        return Eigen::VectorXf();
    }
    return descriptor.colwise().mean().transpose();
}

int LocalizationNode::fastAlignUsingSectorKey(const Eigen::VectorXf& query_sector,
                                              const Eigen::VectorXf& target_sector) const {
    if (query_sector.size() == 0 || target_sector.size() == 0 || query_sector.size() != target_sector.size()) {
        return 0;
    }

    const int cols = query_sector.size();
    const double query_norm = std::sqrt(std::max(0.0, static_cast<double>(query_sector.squaredNorm())));
    if (query_norm <= kNearZero) {
        return 0;
    }

    int best_shift = 0;
    double best_sim = -1.0;
    for (int shift = 0; shift < cols; ++shift) {
        double dot = 0.0;
        double target_sq = 0.0;
        for (int c = 0; c < cols; ++c) {
            const double q = query_sector(c);
            const double t = target_sector((c + shift) % cols);
            dot += q * t;
            target_sq += t * t;
        }

        if (target_sq <= kNearZero) {
            continue;
        }

        const double sim = dot / (query_norm * std::sqrt(target_sq));
        if (sim > best_sim) {
            best_sim = sim;
            best_shift = shift;
        }
    }
    return best_shift;
}

double LocalizationNode::bestSectorSimilarityWindowed(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                                      int coarse_shift, int window_radius, int& best_shift) const {
    best_shift = 0;
    if (query_desc.rows() != target_desc.rows() || query_desc.cols() != target_desc.cols() || query_desc.size() == 0) {
        return -1.0;
    }

    const int rows = query_desc.rows();
    const int cols = query_desc.cols();

    auto wrap_shift = [cols](int shift) {
        int wrapped = (cols > 0) ? (shift % cols) : 0;
        if (wrapped < 0) {
            wrapped += cols;
        }
        return wrapped;
    };

    double query_sq = 0.0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const double q = query_desc(r, c);
            query_sq += q * q;
        }
    }
    if (query_sq <= kNearZero) {
        return -1.0;
    }

    const int bounded_window = std::clamp(window_radius, 0, cols / 2);
    double best_sim = -1.0;
    for (int delta = -bounded_window; delta <= bounded_window; ++delta) {
        const int shift = wrap_shift(coarse_shift + delta);
        double dot = 0.0;
        double target_sq = 0.0;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                const double q = query_desc(r, c);
                const double t = target_desc(r, (c + shift) % cols);
                dot += q * t;
                target_sq += t * t;
            }
        }

        if (target_sq <= kNearZero) {
            continue;
        }

        const double sim = dot / (std::sqrt(query_sq) * std::sqrt(target_sq));
        if (sim > best_sim) {
            best_sim = sim;
            best_shift = shift;
        }
    }

    return best_sim;
}

double LocalizationNode::bestSectorSimilarity(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                              int& best_shift) const {
    const int full_window = std::max<int>(0, static_cast<int>(query_desc.cols() / 2));
    return bestSectorSimilarityWindowed(query_desc, target_desc, 0, full_window, best_shift);
}

bool LocalizationNode::tryRetryZoneFastChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                               Eigen::Isometry3d& best_pose, double& best_fitness,
                                               double& best_cost, int& candidate_count,
                                               const std::atomic<bool>* stop_flag) {
    if (stop_flag && stop_flag->load()) {
        return false;
    }

    Eigen::Vector2d seed_xy(retry_zone_x_, retry_zone_y_);
    std::vector<double> yaw_candidates = retry_zone_yaw_candidates_deg_;
    bool use_uwb_seed = false;
    if (uwb_enable_) {
        std::lock_guard<std::mutex> lk(uwb_mutex_);
        if (uwb_available_ && (now() - uwb_last_stamp_).seconds() < uwb_max_stale_sec_) {
            seed_xy = uwb_position_;
            use_uwb_seed = true;
        }
    }
    if (use_uwb_seed) {
        yaw_candidates.clear();
        double center_yaw_deg = 0.0;
        {
            std::lock_guard<std::mutex> lk(result_mutex_);
            center_yaw_deg = std::atan2(previous_result_t_.rotation()(1, 0), previous_result_t_.rotation()(0, 0)) * 180.0 / M_PI;
        }
        const std::array<double, 5> offsets{
            -uwb_yaw_spread_deg_, -uwb_yaw_spread_deg_ * 0.5, 0.0, uwb_yaw_spread_deg_ * 0.5, uwb_yaw_spread_deg_};
        for (double off : offsets) {
            yaw_candidates.push_back(center_yaw_deg + off);
        }
    }

    const double accept_threshold = std::min(retry_zone_fast_accept_th_, global_fitness_threshold_);
    RCLCPP_INFO(this->get_logger(), "尝试重试区快速通道: 坐标(%.2f, %.2f), %zu 个朝向候选, uwb_seed=%d", seed_xy.x(),
                seed_xy.y(), yaw_candidates.size(), static_cast<int>(use_uwb_seed));

    auto source_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *source_down, registered_leaf_size_);
    if (!source_cov || source_cov->empty()) {
        return false;
    }
    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors_, num_threads_);

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq =
        global_icp_max_correspondence_distance_ * global_icp_max_correspondence_distance_;
    registration.optimizer.max_iterations = global_icp_max_iterations_;

    bool found = false;
    best_fitness = std::numeric_limits<double>::max();
    best_cost = std::numeric_limits<double>::max();
    candidate_count = 0;

    for (double yaw_deg : yaw_candidates) {
        if (stop_flag && stop_flag->load()) {
            return false;
        }
        const double yaw = yaw_deg * M_PI / 180.0;
        const float cy = static_cast<float>(std::cos(yaw));
        const float sy = static_cast<float>(std::sin(yaw));

        Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
        seed(0, 0) = cy;
        seed(0, 1) = -sy;
        seed(1, 0) = sy;
        seed(1, 1) = cy;
        seed(0, 3) = static_cast<float>(seed_xy.x());
        seed(1, 3) = static_cast<float>(seed_xy.y());

        Eigen::Isometry3d seed_iso = Eigen::Isometry3d::Identity();
        seed_iso.matrix() = seed.cast<double>();
        auto result = registration.align(*target_, *source_cov, *target_tree_, seed_iso);

        if (!result.converged || result.num_inliers == 0) {
            continue;
        }

        ++candidate_count;
        const Eigen::Matrix4f refined = result.T_target_source.matrix().cast<float>();
        double fitness = result.error / static_cast<double>(result.num_inliers);
        double cost = computeCandidateCost(fitness, seed, refined);

        RCLCPP_INFO(this->get_logger(), "RZ候选 yaw=%.1f°: fitness=%.4f, J=%.4f", yaw_deg, fitness, cost);

        if (!found || cost < best_cost || (std::abs(cost - best_cost) < 1e-6 && fitness < best_fitness)) {
            found = true;
            best_fitness = fitness;
            best_cost = cost;
            best_pose = Eigen::Isometry3d::Identity();
            best_pose.matrix() = refined.cast<double>();
        }

        if (found && best_fitness < accept_threshold && best_cost < 1.0) {
            break;
        }
    }

    return found && best_fitness < accept_threshold && best_cost < 1.0;
}

bool LocalizationNode::tryScanContextGlobalChannel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_down,
                                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_down,
                                                   Eigen::Isometry3d& best_pose, double& best_fitness,
                                                   double& best_cost, int& candidate_count,
                                                   const std::atomic<bool>* stop_flag) {
    if (stop_flag && stop_flag->load()) {
        return false;
    }

    if (!enable_scan_context_) {
        return false;
    }

    if (!sc_db_ready_ && !buildScanContextDatabase()) {
        RCLCPP_WARN(this->get_logger(), "Scan Context 数据库不可用");
        return false;
    }

    if (!source_down || source_down->empty()) {
        return false;
    }

    Eigen::Vector2d query_center = Eigen::Vector2d::Zero();
    for (const auto& pt : source_down->points) {
        query_center.x() += static_cast<double>(pt.x);
        query_center.y() += static_cast<double>(pt.y);
    }
    query_center /= static_cast<double>(source_down->size());

    const Eigen::MatrixXf query_desc = makeScanContextDescriptor(source_down, query_center);
    const Eigen::VectorXf query_ring = makeRingKey(query_desc);
    const Eigen::VectorXf query_sector = makeSectorKey(query_desc);
    if (query_ring.size() == 0) {
        return false;
    }

    std::vector<ScanContextEntry> database_snapshot;
    std::shared_ptr<ScanContextRingKeyIndex> ring_index_snapshot;
    {
        std::lock_guard<std::mutex> lock(sc_mutex_);
        database_snapshot = sc_database_;
        ring_index_snapshot = sc_ring_index_;
    }

    if (database_snapshot.empty()) {
        return false;
    }

    std::vector<std::pair<double, size_t>> ranked;
    const size_t topk = std::min<size_t>(static_cast<size_t>(std::max(sc_topk_, 1)),
                                         database_snapshot.size());

    if (ring_index_snapshot && !ring_index_snapshot->empty() && ring_index_snapshot->dimension() == query_ring.size()) {
        const auto neighbors = ring_index_snapshot->knnSearch(query_ring, topk);
        ranked.reserve(neighbors.size());
        for (const auto& neighbor : neighbors) {
            if (neighbor.index >= database_snapshot.size()) {
                continue;
            }
            ranked.emplace_back(neighbor.distance, neighbor.index);
        }
    }

    if (ranked.empty()) {
        ranked.reserve(database_snapshot.size());
        for (size_t i = 0; i < database_snapshot.size(); ++i) {
            if (database_snapshot[i].ring_key.size() != query_ring.size()) {
                continue;
            }
            const double dist = (database_snapshot[i].ring_key - query_ring).norm();
            ranked.emplace_back(dist, i);
        }
    }

    if (ranked.empty()) {
        return false;
    }

    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    if (ranked.size() > topk) {
        ranked.resize(topk);
    }

    auto source_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *source_down, registered_leaf_size_);
    if (!source_cov || source_cov->empty()) {
        return false;
    }
    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors_, num_threads_);

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq =
        global_icp_max_correspondence_distance_ * global_icp_max_correspondence_distance_;
    registration.optimizer.max_iterations = global_icp_max_iterations_;

    bool found = false;
    best_fitness = std::numeric_limits<double>::max();
    best_cost = std::numeric_limits<double>::max();
    candidate_count = 0;
    std::vector<double> all_fitness;  // S3: 收集所有收敛候选的 fitness

    const double yaw_step = 2.0 * M_PI / static_cast<double>(sc_num_sectors_);

    for (const auto& [ring_dist, idx] : ranked) {
        if (stop_flag && stop_flag->load()) {
            return false;
        }
        if (idx >= database_snapshot.size()) {
            continue;
        }
        const ScanContextEntry& entry = database_snapshot[idx];

        const bool use_sector_seed =
            query_sector.size() > 0 && entry.sector_key.size() > 0 && query_sector.size() == entry.sector_key.size();
        const int coarse_shift = use_sector_seed ? fastAlignUsingSectorKey(query_sector, entry.sector_key) : 0;
        const int window_radius =
            use_sector_seed ? kScanContextYawWindow : std::max<int>(0, static_cast<int>(query_desc.cols() / 2));

        int best_shift = coarse_shift;
        const double similarity =
            bestSectorSimilarityWindowed(query_desc, entry.descriptor, coarse_shift, window_radius, best_shift);
        if (similarity < 0.0) {
            continue;
        }

        const double sim_cost = 1.0 - similarity;
        const double yaw = -static_cast<double>(best_shift) * yaw_step;

        const float cy = static_cast<float>(std::cos(yaw));
        const float sy = static_cast<float>(std::sin(yaw));

        Eigen::Matrix4f seed = Eigen::Matrix4f::Identity();
        seed(0, 0) = cy;
        seed(0, 1) = -sy;
        seed(1, 0) = sy;
        seed(1, 1) = cy;
        seed(0, 3) = static_cast<float>(entry.center_xy.x());
        seed(1, 3) = static_cast<float>(entry.center_xy.y());

        Eigen::Isometry3d seed_iso = Eigen::Isometry3d::Identity();
        seed_iso.matrix() = seed.cast<double>();
        auto result = registration.align(*target_, *source_cov, *target_tree_, seed_iso);

        if (!result.converged || result.num_inliers == 0) {
            continue;
        }

        ++candidate_count;
        Eigen::Matrix4f refined = result.T_target_source.matrix().cast<float>();
        double fitness = result.error / static_cast<double>(result.num_inliers);
        all_fitness.push_back(fitness);  // S3: 收集
        double cost = computeCandidateCost(fitness, seed, refined) + std::max(0.0, sim_cost - sc_sim_threshold_);

        RCLCPP_INFO(this->get_logger(),
                    "SC候选 idx=%zu ring=%.4f coarse=%d shift=%d sim=%.3f fitness=%.4f J=%.4f center=(%.2f,%.2f)",
                    idx, ring_dist, coarse_shift, best_shift, similarity, fitness, cost, entry.center_xy.x(),
                    entry.center_xy.y());

        if (!found || cost < best_cost || (std::abs(cost - best_cost) < 1e-6 && fitness < best_fitness)) {
            found = true;
            best_fitness = fitness;
            best_cost = cost;
            best_pose = Eigen::Isometry3d::Identity();
            best_pose.matrix() = refined.cast<double>();
        }
    }

    // S3: SC 对称歧义拒绝（TopK 遍历结束后）
    if (s3_enable_ && all_fitness.size() >= 2) {
        std::sort(all_fitness.begin(), all_fitness.end());
        if (std::abs(all_fitness[0] - all_fitness[1]) < s3_min_score_gap_) {
            RCLCPP_WARN(get_logger(),
                "S3: SC symmetry ambiguity (best=%.4f, 2nd=%.4f), reloc rejected",
                all_fitness[0], all_fitness[1]);
            return false;
        }
    }

    return found && best_fitness < global_fitness_threshold_ && best_cost < 1.0;
}

void LocalizationNode::markRelocalizationSuccess(const Eigen::Isometry3d& map_to_odom,
                                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& anchor_cloud,
                                                 const Eigen::Isometry3d& request_odom_to_base,
                                                 bool request_odom_valid, const rclcpp::Time& request_stamp) {
    const bool graph_mode = enable_graph_backend_ && !legacy_hard_reloc_enable_;
    GraphAnchorAttachResult graph_anchor_result = GraphAnchorAttachResult::VALIDATED_ANCHOR;
    Eigen::Matrix<double, 6, 6> cov_snapshot = Eigen::Matrix<double, 6, 6>::Zero();
    const rclcpp::Time success_stamp = this->now();

    Eigen::Isometry3d compensated_map_to_odom = map_to_odom;
    if (request_odom_valid) {
        Eigen::Isometry3d odom_to_base_now = Eigen::Isometry3d::Identity();
        if (tryLookupOdomToBase(success_stamp, odom_to_base_now)) {
            const Eigen::Isometry3d map_to_base_request = map_to_odom * request_odom_to_base;
            compensated_map_to_odom = map_to_base_request * odom_to_base_now.inverse();
            RCLCPP_INFO(this->get_logger(), "重定位接管: 已应用 odom 运动补偿(request=%ld, success=%ld)",
                        request_stamp.nanoseconds(), success_stamp.nanoseconds());
        } else {
            RCLCPP_WARN(this->get_logger(), "重定位接管: 成功时刻 odom->base 不可用，回退未补偿写回");
        }
    } else if (request_stamp.nanoseconds() > 0) {
        RCLCPP_WARN(this->get_logger(), "重定位接管: 请求时刻 odom->base 无快照，回退未补偿写回");
    }

    if (graph_mode) {
        graph_anchor_result = processGraphBackendAnchor(compensated_map_to_odom, success_stamp, anchor_cloud);
        if (graph_anchor_result == GraphAnchorAttachResult::REJECTED_ANCHOR) {
            RCLCPP_WARN(this->get_logger(), "图后端锚点接入失败，保持当前 map->odom，拒绝硬切");
            setLocalizationState(LocalizationState::SUSPECT, "relocalization_anchor_rejected");
            publishLocalizationHealth("relocalization_anchor_rejected");
            publishBackendStatus();
            publishRouteObservability();
            return;
        }
    }
    if (!graph_mode) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_t_ = previous_result_t_ = compensated_map_to_odom;
        cov_snapshot = last_pose_cov_;
    } else {
        Eigen::Isometry3d smoother_target = compensated_map_to_odom;
        {
            std::lock_guard<std::mutex> graph_lock(graph_mutex_);
            if (graph_backend_initialized_ && map_to_odom_smoother_) {
                if (map_to_odom_smoother_->isInitialized()) {
                    smoother_target = map_to_odom_smoother_->target();
                }
                map_to_odom_smoother_->reset(compensated_map_to_odom, success_stamp);
                map_to_odom_smoother_->setTarget(smoother_target, success_stamp);
            }
        }
        std::lock_guard<std::mutex> lock(result_mutex_);
        result_t_ = previous_result_t_ = compensated_map_to_odom;
        cov_snapshot = last_pose_cov_;
    }
    if (esikf_enable_) {
        std::lock_guard<std::mutex> lk(esikf_mutex_);
        esikf_.reset(compensated_map_to_odom);
    }
    {
        std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
        last_successful_registration_time_ = success_stamp;
        last_local_registration_time_ = last_successful_registration_time_;
    }
    consecutive_high_error_count_.store(0);
    resetConfidenceState();
    locked_pose_fallback_active_.store(false);
    saveLockedPoseSnapshot(compensated_map_to_odom, cov_snapshot, success_stamp);
    const char* success_reason = "relocalization_success";
    if (graph_mode) {
        success_reason = (graph_anchor_result == GraphAnchorAttachResult::TRUSTED_RELOC_ANCHOR)
                             ? "relocalization_graph_anchor_trusted"
                             : "relocalization_graph_anchor_validated";
    }
    setLocalizationState(LocalizationState::TRACKING, success_reason);
    publishLocalizationHealth(success_reason);
    publishBackendStatus();
    publishRouteObservability();
}

void LocalizationNode::performGlobalRelocalization(RelocTriggerReason reason,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr passed_cloud,
                                                   const Eigen::Isometry3d& request_odom_to_base,
                                                   bool request_odom_valid, const rclcpp::Time& request_stamp) {
    RelocMetrics metrics;
    metrics.trigger_reason = reason;

    const auto t0 = std::chrono::steady_clock::now();

    if (shutdown_requested_.load()) {
        return;
    }

    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
        if (passed_cloud && !passed_cloud->empty()) {
            source_cloud = passed_cloud;
        } else {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!accumulated_cloud_->empty()) {
                source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*accumulated_cloud_);
                accumulated_cloud_->clear();
            }
        }

        if (!source_cloud || source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "重定位失败：无点云数据");
            setLocalizationState(LocalizationState::RELOC_FAILED, "no_source_cloud");
            metrics.path_used = "no_cloud";
            metrics.accepted = false;
            metrics.t_total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            publishRelocMetrics(metrics);
            return;
        }

        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);

        if (acrylic_filter_enable_) {
            applyAcrylicROIFilter(source_cloud);
        }
        if (source_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "重定位失败：过滤后无有效点云");
            setLocalizationState(LocalizationState::RELOC_FAILED, "empty_after_filter");
            metrics.path_used = "empty_after_filter";
            metrics.accepted = false;
            metrics.t_total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            publishRelocMetrics(metrics);
            return;
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        const float leaf_size = static_cast<float>(global_downsample_leaf_size_);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.setInputCloud(source_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_down(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*source_down);

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_down(new pcl::PointCloud<pcl::PointXYZ>);
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_copy(new pcl::PointCloud<pcl::PointXYZ>(*global_map_));
            pcl::Indices map_indices;
            pcl::removeNaNFromPointCloud(*map_copy, *map_copy, map_indices);
            vg.setInputCloud(map_copy);
            vg.filter(*target_down);
        }

        const size_t min_points =
            static_cast<size_t>(min_points_for_relocalization_ > 0 ? min_points_for_relocalization_ : 1);
        if (source_down->size() < min_points || target_down->size() < min_points) {
            RCLCPP_WARN(this->get_logger(), "重定位失败：点云数量不足 (source=%zu, target=%zu, min=%zu)",
                        source_down->size(), target_down->size(), min_points);
            setLocalizationState(LocalizationState::RELOC_FAILED, "insufficient_points");
            metrics.path_used = "insufficient_points";
            metrics.accepted = false;
            metrics.t_total_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            publishRelocMetrics(metrics);
            return;
        }

        bool accepted = false;
        Eigen::Isometry3d best_pose = Eigen::Isometry3d::Identity();
        const bool can_try_l0 =
            l0_enable_ && imu_spike_recent_.load() && reason == RelocTriggerReason::KIDNAP;

        struct ChannelResult {
            std::string channel{"none"};
            bool attempted{false};
            bool success{false};
            bool canceled{false};
            double elapsed_ms{0.0};
            int candidate_count{0};
            double best_fitness{std::numeric_limits<double>::max()};
            double best_cost{std::numeric_limits<double>::max()};
            Eigen::Isometry3d best_pose{Eigen::Isometry3d::Identity()};
        };

        auto fold_channel_metrics = [&](const ChannelResult& channel_result) {
            metrics.candidate_count += channel_result.candidate_count;
            metrics.best_fitness = std::min(metrics.best_fitness, channel_result.best_fitness);
            metrics.best_j = std::min(metrics.best_j, channel_result.best_cost);
            if (channel_result.channel == "L0") {
                metrics.t_l0_ms = channel_result.elapsed_ms;
            } else if (channel_result.channel == "L1") {
                metrics.t_l1_ms = channel_result.elapsed_ms;
            } else if (channel_result.channel == "L2") {
                metrics.t_l2_ms = channel_result.elapsed_ms;
            }
        };

        auto run_l0 = [&](const std::atomic<bool>* stop_flag) -> ChannelResult {
            ChannelResult ch;
            ch.channel = "L0";
            if (!can_try_l0) {
                ch.canceled = true;
                return ch;
            }
            const auto t_ch = std::chrono::steady_clock::now();
            ch.attempted = true;
            ch.success = tryL0FastChannel(source_down, target_down, ch.best_pose, ch.best_fitness, ch.best_cost,
                                          ch.candidate_count, stop_flag);
            ch.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_ch).count();
            return ch;
        };

        auto run_l1 = [&](const std::atomic<bool>* stop_flag) -> ChannelResult {
            ChannelResult ch;
            ch.channel = "L1";
            if (!retry_zone_enable_ || retry_zone_yaw_candidates_deg_.empty()) {
                ch.canceled = true;
                return ch;
            }
            const auto t_ch = std::chrono::steady_clock::now();
            ch.attempted = true;
            ch.success = tryRetryZoneFastChannel(source_down, target_down, ch.best_pose, ch.best_fitness, ch.best_cost,
                                                 ch.candidate_count, stop_flag);
            ch.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_ch).count();
            return ch;
        };

        auto run_l2 = [&](const std::atomic<bool>* stop_flag) -> ChannelResult {
            ChannelResult ch;
            ch.channel = "L2";
            const auto t_ch = std::chrono::steady_clock::now();
            ch.attempted = true;
            ch.success = tryGlobalChannel(source_down, target_down, ch.best_pose, ch.best_fitness, ch.best_cost,
                                          ch.candidate_count, stop_flag);
            ch.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_ch).count();
            return ch;
        };

        if (parallel_reloc_enable_) {
            setLocalizationState(LocalizationState::GLOBAL_RECOVERY, "run_parallel_reloc");
            std::atomic<bool> reloc_done{false};

            auto fut_l0 = std::async(std::launch::async, run_l0, &reloc_done);
            auto fut_l1 = std::async(std::launch::async, run_l1, &reloc_done);
            auto fut_l2 = std::async(std::launch::async, run_l2, &reloc_done);

            ChannelResult res_l0, res_l1, res_l2;
            bool got_l0 = false;
            bool got_l1 = false;
            bool got_l2 = false;
            std::string winner{"none"};

            auto pull_if_ready = [&](std::future<ChannelResult>& fut, ChannelResult& out, bool& done) {
                if (done) {
                    return;
                }
                if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    out = fut.get();
                    done = true;
                    fold_channel_metrics(out);
                    if (!accepted && out.success) {
                        accepted = true;
                        best_pose = out.best_pose;
                        winner = out.channel;
                        reloc_done.store(true);
                    }
                }
            };

            while (!(got_l0 && got_l1 && got_l2)) {
                pull_if_ready(fut_l0, res_l0, got_l0);
                pull_if_ready(fut_l1, res_l1, got_l1);
                pull_if_ready(fut_l2, res_l2, got_l2);
                if (!accepted && got_l0 && got_l1 && got_l2) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            reloc_done.store(true);
            if (!got_l0) {
                res_l0 = fut_l0.get();
                got_l0 = true;
                fold_channel_metrics(res_l0);
            }
            if (!got_l1) {
                res_l1 = fut_l1.get();
                got_l1 = true;
                fold_channel_metrics(res_l1);
            }
            if (!got_l2) {
                res_l2 = fut_l2.get();
                got_l2 = true;
                fold_channel_metrics(res_l2);
            }

            RCLCPP_INFO(
                get_logger(),
                "PERF_METRIC phase=L0 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                res_l0.elapsed_ms, res_l0.candidate_count, res_l0.best_fitness, res_l0.best_cost,
                static_cast<int>(res_l0.success), toString(getLocalizationState()));
            RCLCPP_INFO(
                get_logger(),
                "PERF_METRIC phase=L1 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                res_l1.elapsed_ms, res_l1.candidate_count, res_l1.best_fitness, res_l1.best_cost,
                static_cast<int>(res_l1.success), toString(getLocalizationState()));
            RCLCPP_INFO(
                get_logger(),
                "PERF_METRIC phase=L2 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                res_l2.elapsed_ms, res_l2.candidate_count, res_l2.best_fitness, res_l2.best_cost,
                static_cast<int>(res_l2.success), toString(getLocalizationState()));

            metrics.winner_channel = winner;
            metrics.cancel_reason = accepted ? "winner_selected" : "none";
            if (accepted) {
                metrics.path_used = winner;
                metrics.accepted = true;
                markRelocalizationSuccess(best_pose, source_cloud, request_odom_to_base, request_odom_valid,
                                          request_stamp);
            } else {
                metrics.path_used = "parallel_failed";
            }
        } else {
            if (can_try_l0) {
                setLocalizationState(LocalizationState::FAST_RECOVERY, "run_l0_fast_channel");
                ChannelResult l0 = run_l0(nullptr);
                fold_channel_metrics(l0);
                accepted = l0.success;
                if (accepted) {
                    best_pose = l0.best_pose;
                    metrics.path_used = "L0";
                    metrics.winner_channel = "L0";
                    metrics.accepted = true;
                    markRelocalizationSuccess(best_pose, source_cloud, request_odom_to_base, request_odom_valid,
                                              request_stamp);
                }
                RCLCPP_INFO(
                    get_logger(),
                    "PERF_METRIC phase=L0 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                    l0.elapsed_ms, l0.candidate_count, l0.best_fitness, l0.best_cost, static_cast<int>(l0.success),
                    toString(getLocalizationState()));
            }

            if (!accepted && retry_zone_enable_ && !retry_zone_yaw_candidates_deg_.empty()) {
                setLocalizationState(LocalizationState::FAST_RECOVERY, "run_l1_retry_zone");
                ChannelResult l1 = run_l1(nullptr);
                fold_channel_metrics(l1);
                accepted = l1.success;
                if (accepted) {
                    best_pose = l1.best_pose;
                    metrics.path_used = "L1";
                    metrics.winner_channel = "L1";
                    metrics.accepted = true;
                    markRelocalizationSuccess(best_pose, source_cloud, request_odom_to_base, request_odom_valid,
                                              request_stamp);
                }
                RCLCPP_INFO(
                    get_logger(),
                    "PERF_METRIC phase=L1 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                    l1.elapsed_ms, l1.candidate_count, l1.best_fitness, l1.best_cost, static_cast<int>(l1.success),
                    toString(getLocalizationState()));
            }

            if (!accepted) {
                setLocalizationState(LocalizationState::GLOBAL_RECOVERY, "run_l2_scan_context");
                ChannelResult l2 = run_l2(nullptr);
                fold_channel_metrics(l2);
                accepted = l2.success;
                if (accepted) {
                    best_pose = l2.best_pose;
                    metrics.path_used = "L2";
                    metrics.winner_channel = "L2";
                    metrics.accepted = true;
                    markRelocalizationSuccess(best_pose, source_cloud, request_odom_to_base, request_odom_valid,
                                              request_stamp);
                } else {
                    metrics.path_used = "L2_failed";
                }
                RCLCPP_INFO(
                    get_logger(),
                    "PERF_METRIC phase=L2 dt_ms=%.1f candidates=%d best_fit=%.4f best_j=%.4f accepted=%d state=%s",
                    l2.elapsed_ms, l2.candidate_count, l2.best_fitness, l2.best_cost, static_cast<int>(l2.success),
                    toString(getLocalizationState()));
            }
        }

        if (!accepted) {
            setLocalizationState(LocalizationState::RELOC_FAILED, "all_recovery_failed");
            metrics.accepted = false;
        }

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "重定位异常: %s", ex.what());
        setLocalizationState(LocalizationState::RELOC_FAILED, "exception");
        metrics.path_used = "exception";
        metrics.accepted = false;
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "重定位异常: unknown");
        setLocalizationState(LocalizationState::RELOC_FAILED, "unknown_exception");
        metrics.path_used = "unknown_exception";
        metrics.accepted = false;
    }

    metrics.t_total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    publishRelocMetrics(metrics);
}

bool LocalizationNode::tryGetReliableMapToOdom(Eigen::Isometry3d& map_to_odom) {
    const LocalizationState state = getLocalizationState();
    const bool state_reliable = isMapToOdomReliableState(state);

    if (state_reliable) {
        rclcpp::Time last_success_snapshot;
        {
            std::lock_guard<std::mutex> time_lock(registration_time_mutex_);
            last_success_snapshot = last_successful_registration_time_;
        }
        const double stale_sec = (this->now() - last_success_snapshot).seconds();
        if (stale_sec <= acrylic_filter_max_stale_sec_) {
            std::lock_guard<std::mutex> lock(result_mutex_);
            map_to_odom = result_t_;
            return true;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "亚克力过滤使用当前 map->odom 失败: 过期 %.2fs > %.2fs", stale_sec, acrylic_filter_max_stale_sec_);
    }

    if (tryGetLockedPoseSnapshot(map_to_odom)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tryGetReliableMapToOdom 使用 last_locked_map_to_odom 回退");
        return true;
    }

    setLocalizationState(LocalizationState::SUSPECT, "map_to_odom_unreliable");
    return false;
}

void LocalizationNode::applyAcrylicROIFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (acrylic_roi_boxes_.size() < 6) {
        return;
    }

    Eigen::Isometry3d map_to_odom;
    if (!tryGetReliableMapToOdom(map_to_odom)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "亚克力过滤跳过: map->odom 当前不可靠");
        return;
    }

    const size_t n_boxes = acrylic_roi_boxes_.size() / 6;
    pcl::PointCloud<pcl::PointXYZ> filtered;
    filtered.reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        const Eigen::Vector3d p_map = map_to_odom * Eigen::Vector3d(pt.x, pt.y, pt.z);
        bool in_box = false;
        for (size_t i = 0; i < n_boxes && !in_box; ++i) {
            const size_t b = i * 6;
            if (p_map.x() >= acrylic_roi_boxes_[b] && p_map.x() <= acrylic_roi_boxes_[b + 3] &&
                p_map.y() >= acrylic_roi_boxes_[b + 1] && p_map.y() <= acrylic_roi_boxes_[b + 4] &&
                p_map.z() >= acrylic_roi_boxes_[b + 2] && p_map.z() <= acrylic_roi_boxes_[b + 5]) {
                in_box = true;
            }
        }
        if (!in_box) {
            filtered.push_back(pt);
        }
    }

    cloud->points = std::move(filtered.points);
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
}

}  // namespace rc26_localization
