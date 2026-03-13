#include "rc26_localization/localization.hpp"

#include <algorithm>
#include <cmath>

#include "pcl/filters/voxel_grid.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace rc26_localization {

namespace {
double normalizeAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double yawOf(const Eigen::Isometry3d& pose) {
    return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

constexpr double kTrustedRelocAnchorNoiseScale = 2.0;
}  // namespace

void LocalizationNode::initializeGraphBackend() {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    graph_backend_initialized_ = false;
    keyframe_manager_.reset();
    online_sc_db_.reset();
    constraint_validator_.reset();
    pose_graph_backend_.reset();
    map_to_odom_smoother_.reset();
    graph_status_cache_ = PoseGraphStatus{};

    if (!enable_graph_backend_) {
        backend_candidate_conflict_count_.store(0U);
        backend_map_to_odom_jump_suppressed_.store(false);
        return;
    }

    KeyframeManager::Config keyframe_cfg;
    keyframe_cfg.translation_thresh_m = graph_keyframe_translation_thresh_m_;
    keyframe_cfg.yaw_thresh_deg = graph_keyframe_yaw_thresh_deg_;
    keyframe_cfg.time_thresh_sec = graph_keyframe_time_thresh_sec_;
    keyframe_cfg.trigger_on_control_degraded_rising = graph_keyframe_trigger_on_degraded_rising_;
    keyframe_cfg.trigger_on_hessian_drop = graph_keyframe_trigger_on_hessian_drop_;
    keyframe_cfg.trigger_on_sigma_cross = graph_keyframe_trigger_on_sigma_cross_;
    keyframe_cfg.hessian_alert_eig_min = lhi_yellow_h_min_eig_max_;
    keyframe_cfg.sigma_xy_alert_min = lhi_yellow_sigma_xy_min_;

    ConstraintValidator::Config validator_cfg;
    validator_cfg.accept_fitness_threshold = graph_validator_accept_fitness_threshold_;
    validator_cfg.conflict_fitness_threshold = graph_validator_conflict_fitness_threshold_;
    validator_cfg.max_correspondence_distance_m = graph_validator_max_corr_dist_m_;
    validator_cfg.max_iterations = graph_validator_max_iterations_;
    validator_cfg.num_neighbors = num_neighbors_;
    validator_cfg.num_threads = num_threads_;
    validator_cfg.voxel_leaf_size = std::max(0.05F, registered_leaf_size_);

    PoseGraphBackend::Config backend_cfg;
    backend_cfg.prior_sigma_xy_m = std::max(1e-4, graph_odom_sigma_translation_m_);
    backend_cfg.prior_sigma_yaw_rad = std::max(1e-6, graph_odom_sigma_yaw_deg_ * M_PI / 180.0);
    backend_cfg.graph_health_conflict_penalty = 0.05;

    MapToOdomSmoother::Config smoother_cfg;
    smoother_cfg.max_translation_speed_mps = graph_smoother_max_translation_speed_mps_;
    smoother_cfg.max_yaw_speed_radps = graph_smoother_max_yaw_speed_degps_ * M_PI / 180.0;

    keyframe_manager_ = std::make_unique<KeyframeManager>(keyframe_cfg);
    online_sc_db_ = std::make_unique<OnlineScanContextDB>();
    constraint_validator_ = std::make_unique<ConstraintValidator>(validator_cfg);
    pose_graph_backend_ = std::make_unique<PoseGraphBackend>(backend_cfg);
    map_to_odom_smoother_ = std::make_unique<MapToOdomSmoother>(smoother_cfg);

    Eigen::Isometry3d initial_map_to_odom = Eigen::Isometry3d::Identity();
    {
        std::lock_guard<std::mutex> result_lock(result_mutex_);
        initial_map_to_odom = result_t_;
    }
    map_to_odom_smoother_->reset(initial_map_to_odom, this->now());

    graph_status_cache_.optimizer_state = "graph_backend_enabled";
    graph_backend_initialized_ = true;
    backend_candidate_conflict_count_.store(0U);
    backend_map_to_odom_jump_suppressed_.store(false);

    RCLCPP_INFO(this->get_logger(),
                "P1图后端启用: keyframe(%.2fm, %.1fdeg, %.2fs), loop_topk=%d, smoother=(%.2fm/s, %.1fdeg/s)",
                graph_keyframe_translation_thresh_m_, graph_keyframe_yaw_thresh_deg_, graph_keyframe_time_thresh_sec_,
                graph_loop_topk_, graph_smoother_max_translation_speed_mps_, graph_smoother_max_yaw_speed_degps_);
}

bool LocalizationNode::tryLookupOdomToBase(const rclcpp::Time& stamp, Eigen::Isometry3d& odom_to_base) const {
    auto lookup = [this, &odom_to_base](const rclcpp::Time& query_time) -> bool {
        try {
            const auto tf = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, query_time,
                                                        rclcpp::Duration::from_seconds(tf_timeout_sec_));
            odom_to_base = tf2::transformToEigen(tf.transform);
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    };

    if (stamp.nanoseconds() > 0 && lookup(stamp)) {
        return true;
    }
    return lookup(rclcpp::Time(0));
}

std::vector<LocalizationNode::ExternalCandidate> LocalizationNode::consumeExternalCandidates(const rclcpp::Time& now,
                                                                                              size_t max_count) {
    std::vector<ExternalCandidate> out;
    if (!p4_candidate_enable_ || max_count == 0) {
        return out;
    }

    std::lock_guard<std::mutex> lock(external_candidates_mutex_);
    while (!pending_external_candidates_.empty()) {
        const auto& front = pending_external_candidates_.front();
        if ((now - front.stamp).seconds() <= p4_candidate_max_stale_sec_) {
            break;
        }
        pending_external_candidates_.pop_front();
    }

    const size_t n = std::min(max_count, pending_external_candidates_.size());
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        out.push_back(std::move(pending_external_candidates_.front()));
        pending_external_candidates_.pop_front();
    }
    return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalizationNode::buildMapPatchAround(const Eigen::Vector2d& center_map,
                                                                           double radius_m) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>());
    const double radius_sq = radius_m * radius_m;
    std::lock_guard<std::mutex> map_lock(map_mutex_);

    auto push_if_near = [&](double x, double y, double z) {
        const double dx = x - center_map.x();
        const double dy = y - center_map.y();
        if (dx * dx + dy * dy <= radius_sq) {
            patch->push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
        }
    };

    if (target_ && !target_->empty()) {
        patch->reserve(target_->size() / 4);
        for (const auto& pt : target_->points) {
            push_if_near(pt.x, pt.y, pt.z);
        }
    } else if (global_map_ && !global_map_->empty()) {
        patch->reserve(global_map_->size() / 4);
        for (const auto& pt : global_map_->points) {
            push_if_near(pt.x, pt.y, pt.z);
        }
    }

    if (patch->empty() && global_map_ && !global_map_->empty()) {
        // 回退到全图，避免候选因为局部补丁为空而无法验证。
        patch = global_map_;
    }
    return patch;
}

void LocalizationNode::applyExternalAnchorCandidates(const KeyframeData& inserted, bool& graph_changed,
                                                     const rclcpp::Time& stamp) {
    if (!p4_candidate_enable_ || !constraint_validator_ || !pose_graph_backend_) {
        return;
    }

    const auto candidates = consumeExternalCandidates(stamp, static_cast<size_t>(p4_candidate_max_per_cycle_));
    if (candidates.empty()) {
        return;
    }

    for (const auto& candidate : candidates) {
        bool accepted = false;
        bool conflict = false;
        std::string reject_reason = "not_run";

        if (inserted.cloud && !inserted.cloud->empty()) {
            const Eigen::Vector2d center(candidate.pose_map.translation().x(), candidate.pose_map.translation().y());
            const auto patch_cloud = buildMapPatchAround(center, p4_candidate_patch_radius_m_);
            if (patch_cloud && !patch_cloud->empty()) {
                ConstraintValidationInput validation_input;
                validation_input.type = ConstraintType::UWB_SOFT_ANCHOR;
                validation_input.from_id = inserted.id;
                validation_input.to_id = inserted.id;
                validation_input.initial_relative_pose = candidate.pose_map * inserted.pose_odom.inverse();
                validation_input.source_cloud = inserted.cloud;
                validation_input.target_cloud = patch_cloud;
                validation_input.imu_spike = imu_spike_active_.load() || imu_spike_recent_.load();

                const auto validation_result = constraint_validator_->validate(validation_input);
                accepted = validation_result.accepted;
                conflict = validation_result.conflict;
                reject_reason = validation_result.reason;
                if (accepted) {
                    const Eigen::Isometry3d observed_pose_map = validation_result.relative_pose * inserted.pose_odom;
                    graph_changed = pose_graph_backend_
                                        ->addAnchorPrior(inserted.id, observed_pose_map, p4_candidate_sigma_translation_m_,
                                                         p4_candidate_sigma_yaw_deg_ * M_PI / 180.0, true) ||
                                    graph_changed;
                }
            } else {
                reject_reason = "empty_map_patch";
            }
        } else {
            reject_reason = "empty_keyframe_cloud";
        }

        pose_graph_backend_->markAnchorCandidate(accepted, conflict, candidate.stamp);
        if (accepted) {
            RCLCPP_INFO(this->get_logger(), "P4候选入图成功: source=%s keyframe=%u", candidate.source.c_str(), inserted.id);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "P4候选拒绝: source=%s keyframe=%u reason=%s",
                         candidate.source.c_str(), inserted.id, reject_reason.c_str());
        }
    }
}

bool LocalizationNode::processGraphBackendOnLocalRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                              const Eigen::Isometry3d& map_to_odom,
                                                              const rclcpp::Time& stamp) {
    if (!enable_graph_backend_ || !cloud || cloud->empty()) {
        return false;
    }

    Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
    if (!tryLookupOdomToBase(stamp, odom_to_base)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "图后端: odom->base TF 不可用，跳过关键帧");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const float leaf = std::max(0.05F, registered_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*keyframe_cloud);
    if (keyframe_cloud->empty()) {
        keyframe_cloud = cloud;
    }

    KeyframeData candidate;
    candidate.stamp = stamp;
    candidate.pose_odom = odom_to_base;
    candidate.pose_map_guess = map_to_odom * odom_to_base;
    candidate.cloud = keyframe_cloud;
    candidate.control_degraded = control_degraded_.load();
    {
        std::lock_guard<std::mutex> result_lock(result_mutex_);
        candidate.h_min_eig = last_h_min_eig_;
        candidate.h_cond = last_h_cond_;
        const double sigma_xy = std::sqrt(std::max(0.0, last_pose_cov_(3, 3) + last_pose_cov_(4, 4)));
        const double sigma_yaw_rad = std::sqrt(std::max(0.0, last_pose_cov_(2, 2)));
        candidate.sigma_xy = sigma_xy;
        candidate.sigma_yaw_deg = sigma_yaw_rad * 180.0 / M_PI;
    }

    const Eigen::Vector2d keyframe_center(candidate.pose_odom.translation().x(), candidate.pose_odom.translation().y());
    candidate.descriptor = makeScanContextDescriptor(candidate.cloud, keyframe_center);
    candidate.ring_key = makeRingKey(candidate.descriptor);
    candidate.sector_key = makeSectorKey(candidate.descriptor);

    std::lock_guard<std::mutex> graph_lock(graph_mutex_);
    if (!graph_backend_initialized_ || !keyframe_manager_ || !online_sc_db_ || !constraint_validator_ ||
        !pose_graph_backend_ || !map_to_odom_smoother_) {
        return false;
    }

    std::string trigger_reason;
    if (!keyframe_manager_->shouldCreate(candidate, trigger_reason)) {
        return false;
    }
    candidate.trigger_reason = trigger_reason;
    KeyframeData inserted = keyframe_manager_->push(std::move(candidate));

    if (inserted.ring_key.size() > 0 && inserted.descriptor.size() > 0) {
        OnlineScanContextRecord record;
        record.keyframe_id = inserted.id;
        record.stamp = inserted.stamp;
        record.center_xy = keyframe_center;
        record.descriptor = inserted.descriptor;
        record.ring_key = inserted.ring_key;
        record.sector_key = inserted.sector_key;
        online_sc_db_->addRecord(record);
    }

    bool graph_changed = pose_graph_backend_->addKeyframeNode(inserted.id, inserted.pose_map_guess);

    const auto previous = keyframe_manager_->previous();
    if (previous.has_value()) {
        GraphConstraint odom_constraint;
        odom_constraint.kind = GraphConstraintKind::ODOM;
        odom_constraint.from_id = previous->id;
        odom_constraint.to_id = inserted.id;
        odom_constraint.relative_pose = previous->pose_odom.inverse() * inserted.pose_odom;
        odom_constraint.sigma_translation_m = graph_odom_sigma_translation_m_;
        odom_constraint.sigma_yaw_rad = graph_odom_sigma_yaw_deg_ * M_PI / 180.0;
        odom_constraint.robust = false;
        graph_changed = pose_graph_backend_->addConstraint(odom_constraint) || graph_changed;
    }

    if (inserted.ring_key.size() > 0 && inserted.descriptor.size() > 0 && graph_loop_topk_ > 0) {
        const auto loop_candidates = online_sc_db_->query(inserted.ring_key, inserted.descriptor, graph_loop_topk_ + 1);
        for (const auto& candidate_match : loop_candidates) {
            if (candidate_match.keyframe_id == inserted.id) {
                continue;
            }
            if (inserted.id > candidate_match.keyframe_id &&
                (inserted.id - candidate_match.keyframe_id) < static_cast<uint32_t>(graph_loop_min_keyframe_gap_)) {
                continue;
            }
            if (candidate_match.similarity < graph_loop_similarity_min_) {
                continue;
            }

            const auto target_keyframe = keyframe_manager_->getById(candidate_match.keyframe_id);
            if (!target_keyframe.has_value() || !target_keyframe->cloud || target_keyframe->cloud->empty()) {
                continue;
            }

            ConstraintValidationInput validation_input;
            validation_input.type = ConstraintType::LOOP;
            validation_input.from_id = target_keyframe->id;
            validation_input.to_id = inserted.id;
            validation_input.initial_relative_pose = target_keyframe->pose_map_guess.inverse() * inserted.pose_map_guess;
            validation_input.source_cloud = inserted.cloud;
            validation_input.target_cloud = target_keyframe->cloud;
            validation_input.imu_spike = imu_spike_active_.load() || imu_spike_recent_.load();

            const auto validation_result = constraint_validator_->validate(validation_input);
            pose_graph_backend_->markLoopCandidate(validation_result.accepted, validation_result.conflict, stamp);
            if (!validation_result.accepted) {
                continue;
            }

            GraphConstraint loop_constraint;
            loop_constraint.kind = GraphConstraintKind::LOOP;
            loop_constraint.from_id = target_keyframe->id;
            loop_constraint.to_id = inserted.id;
            loop_constraint.relative_pose = validation_result.relative_pose;
            loop_constraint.sigma_translation_m = graph_loop_sigma_translation_m_;
            loop_constraint.sigma_yaw_rad = graph_loop_sigma_yaw_deg_ * M_PI / 180.0;
            loop_constraint.robust = true;
            graph_changed = pose_graph_backend_->addConstraint(loop_constraint) || graph_changed;
            break;
        }
    }

    applyExternalAnchorCandidates(inserted, graph_changed, stamp);

    bool updated = false;
    if (graph_changed) {
        updated = pose_graph_backend_->update(stamp);
    }
    if (updated) {
        Eigen::Isometry3d optimized_pose_map = Eigen::Isometry3d::Identity();
        if (pose_graph_backend_->queryPoseMap(inserted.id, optimized_pose_map)) {
            const Eigen::Isometry3d target_map_to_odom = optimized_pose_map * inserted.pose_odom.inverse();
            if (!map_to_odom_smoother_->isInitialized()) {
                map_to_odom_smoother_->reset(map_to_odom, stamp);
            }
            const Eigen::Isometry3d current_map_to_odom = map_to_odom_smoother_->current();
            const double delta_xy = (target_map_to_odom.translation().head<2>() -
                                     current_map_to_odom.translation().head<2>())
                                        .norm();
            const double delta_yaw_deg = std::abs(normalizeAngle(yawOf(target_map_to_odom) - yawOf(current_map_to_odom))) *
                                         180.0 / M_PI;
            const bool jump_suppressed =
                delta_xy > graph_jump_detect_translation_m_ || delta_yaw_deg > graph_jump_detect_yaw_deg_;
            pose_graph_backend_->setMapToOdomJumpSuppressed(jump_suppressed);
            map_to_odom_smoother_->setTarget(target_map_to_odom, stamp);
        }
    }

    graph_status_cache_ = pose_graph_backend_->statusSnapshot(this->now());
    backend_candidate_conflict_count_.store(graph_status_cache_.candidate_conflict_count);
    backend_map_to_odom_jump_suppressed_.store(graph_status_cache_.map_to_odom_jump_suppressed);
    return updated;
}

LocalizationNode::GraphAnchorAttachResult LocalizationNode::processGraphBackendAnchor(
    const Eigen::Isometry3d& map_to_odom, const rclcpp::Time& stamp,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& anchor_cloud) {
    if (!enable_graph_backend_) {
        return GraphAnchorAttachResult::REJECTED_ANCHOR;
    }

    Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
    if (!tryLookupOdomToBase(stamp, odom_to_base)) {
        RCLCPP_WARN(this->get_logger(), "图后端锚点: 无法获取 odom->base TF");
        return GraphAnchorAttachResult::REJECTED_ANCHOR;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_anchor_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (anchor_cloud && !anchor_cloud->empty()) {
        *raw_anchor_cloud = *anchor_cloud;
    } else {
        std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
        if (accumulated_cloud_ && !accumulated_cloud_->empty()) {
            *raw_anchor_cloud = *accumulated_cloud_;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (raw_anchor_cloud && !raw_anchor_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        const float leaf = std::max(0.05F, registered_leaf_size_);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(raw_anchor_cloud);
        voxel.filter(*keyframe_cloud);
        if (keyframe_cloud->empty()) {
            keyframe_cloud = raw_anchor_cloud;
        }
    }

    KeyframeData anchor_keyframe;
    anchor_keyframe.stamp = stamp;
    anchor_keyframe.pose_odom = odom_to_base;
    anchor_keyframe.pose_map_guess = map_to_odom * odom_to_base;
    anchor_keyframe.cloud = keyframe_cloud;
    anchor_keyframe.control_degraded = control_degraded_.load();
    anchor_keyframe.trigger_reason = "relocalization_anchor";
    if (!keyframe_cloud->empty()) {
        const Eigen::Vector2d center(anchor_keyframe.pose_odom.translation().x(), anchor_keyframe.pose_odom.translation().y());
        anchor_keyframe.descriptor = makeScanContextDescriptor(keyframe_cloud, center);
        anchor_keyframe.ring_key = makeRingKey(anchor_keyframe.descriptor);
        anchor_keyframe.sector_key = makeSectorKey(anchor_keyframe.descriptor);
    }
    {
        std::lock_guard<std::mutex> result_lock(result_mutex_);
        anchor_keyframe.h_min_eig = last_h_min_eig_;
        anchor_keyframe.h_cond = last_h_cond_;
        const double sigma_xy = std::sqrt(std::max(0.0, last_pose_cov_(3, 3) + last_pose_cov_(4, 4)));
        const double sigma_yaw_rad = std::sqrt(std::max(0.0, last_pose_cov_(2, 2)));
        anchor_keyframe.sigma_xy = sigma_xy;
        anchor_keyframe.sigma_yaw_deg = sigma_yaw_rad * 180.0 / M_PI;
    }

    std::lock_guard<std::mutex> graph_lock(graph_mutex_);
    if (!graph_backend_initialized_ || !keyframe_manager_ || !online_sc_db_ || !constraint_validator_ ||
        !pose_graph_backend_ || !map_to_odom_smoother_) {
        return GraphAnchorAttachResult::REJECTED_ANCHOR;
    }

    const auto previous = keyframe_manager_->latest();

    bool anchor_accepted = !previous.has_value();
    bool trusted_reloc_anchor = false;
    bool anchor_conflict = false;
    std::string anchor_reason = anchor_accepted ? "bootstrap_anchor" : "validation_not_run";
    if (previous.has_value() && anchor_keyframe.cloud && !anchor_keyframe.cloud->empty() &&
        previous->cloud && !previous->cloud->empty()) {
        ConstraintValidationInput validation_input;
        validation_input.type = ConstraintType::ANCHOR;
        validation_input.from_id = previous->id;
        validation_input.to_id = previous->id + 1U;
        validation_input.initial_relative_pose = previous->pose_map_guess.inverse() * anchor_keyframe.pose_map_guess;
        validation_input.source_cloud = anchor_keyframe.cloud;
        validation_input.target_cloud = previous->cloud;
        validation_input.imu_spike = imu_spike_active_.load() || imu_spike_recent_.load();
        const auto validation_result = constraint_validator_->validate(validation_input);
        anchor_accepted = validation_result.accepted;
        anchor_conflict = validation_result.conflict;
        anchor_reason = validation_result.reason;
        if (!anchor_accepted && anchor_conflict) {
            anchor_reason = "validation_conflict";
        }
    } else if (previous.has_value()) {
        anchor_reason = (!anchor_keyframe.cloud || anchor_keyframe.cloud->empty()) ? "empty_anchor_cloud"
                                                                                   : "empty_previous_cloud";
        anchor_accepted = false;
    }

    if (!anchor_accepted && !anchor_conflict && previous.has_value()) {
        if (anchor_reason == "empty_anchor_cloud") {
            trusted_reloc_anchor = true;
            RCLCPP_WARN(this->get_logger(), "图后端锚点: trusted_reloc_empty_anchor");
        } else if (anchor_reason == "empty_previous_cloud") {
            trusted_reloc_anchor = true;
            RCLCPP_WARN(this->get_logger(), "图后端锚点: trusted_reloc_empty_previous");
        }
    }

    const bool anchor_applied = anchor_accepted || trusted_reloc_anchor;
    pose_graph_backend_->markAnchorCandidate(anchor_applied, anchor_conflict, stamp);
    if (!anchor_applied) {
        if (anchor_reason == "empty_anchor_cloud") {
            RCLCPP_WARN(this->get_logger(), "图后端锚点验证失败: reason=empty_anchor_cloud");
        } else if (anchor_reason == "empty_previous_cloud") {
            RCLCPP_WARN(this->get_logger(), "图后端锚点验证失败: reason=empty_previous_cloud");
        } else if (anchor_reason == "validation_conflict") {
            RCLCPP_WARN(this->get_logger(), "图后端锚点验证失败: reason=validation_conflict");
        }
        RCLCPP_WARN(this->get_logger(), "图后端锚点验证失败: reason=%s", anchor_reason.c_str());
        graph_status_cache_ = pose_graph_backend_->statusSnapshot(this->now());
        backend_candidate_conflict_count_.store(graph_status_cache_.candidate_conflict_count);
        backend_map_to_odom_jump_suppressed_.store(graph_status_cache_.map_to_odom_jump_suppressed);
        return GraphAnchorAttachResult::REJECTED_ANCHOR;
    }

    KeyframeData inserted = keyframe_manager_->push(std::move(anchor_keyframe));
    if (inserted.ring_key.size() > 0 && inserted.descriptor.size() > 0) {
        OnlineScanContextRecord record;
        record.keyframe_id = inserted.id;
        record.stamp = inserted.stamp;
        record.center_xy = Eigen::Vector2d(inserted.pose_odom.translation().x(), inserted.pose_odom.translation().y());
        record.descriptor = inserted.descriptor;
        record.ring_key = inserted.ring_key;
        record.sector_key = inserted.sector_key;
        online_sc_db_->addRecord(record);
    }

    bool graph_changed = pose_graph_backend_->addKeyframeNode(inserted.id, inserted.pose_map_guess);
    if (!graph_changed) {
        RCLCPP_ERROR(this->get_logger(), "图后端锚点: addKeyframeNode 失败，keyframe=%u", inserted.id);
    }

    if (previous.has_value()) {
        GraphConstraint odom_constraint;
        odom_constraint.kind = GraphConstraintKind::ODOM;
        odom_constraint.from_id = previous->id;
        odom_constraint.to_id = inserted.id;
        odom_constraint.relative_pose = previous->pose_odom.inverse() * inserted.pose_odom;
        odom_constraint.sigma_translation_m = graph_odom_sigma_translation_m_;
        odom_constraint.sigma_yaw_rad = graph_odom_sigma_yaw_deg_ * M_PI / 180.0;
        odom_constraint.robust = false;
        graph_changed = pose_graph_backend_->addConstraint(odom_constraint) || graph_changed;
    }

    const double noise_scale = trusted_reloc_anchor ? kTrustedRelocAnchorNoiseScale : 1.0;
    const Eigen::Isometry3d observed_pose_map = map_to_odom * inserted.pose_odom;
    const bool anchor_prior_added =
        pose_graph_backend_->addAnchorPrior(inserted.id, observed_pose_map, graph_anchor_sigma_translation_m_ * noise_scale,
                                            graph_anchor_sigma_yaw_deg_ * noise_scale * M_PI / 180.0, true);
    if (!anchor_prior_added) {
        RCLCPP_ERROR(this->get_logger(), "图后端锚点: addAnchorPrior 失败，keyframe=%u", inserted.id);
    }
    graph_changed = anchor_prior_added || graph_changed;

    const GraphAnchorAttachResult attach_result =
        anchor_accepted ? GraphAnchorAttachResult::VALIDATED_ANCHOR : GraphAnchorAttachResult::TRUSTED_RELOC_ANCHOR;

    bool updated = false;
    if (graph_changed) {
        updated = pose_graph_backend_->update(stamp);
    }
    if (updated) {
        Eigen::Isometry3d optimized_pose_map = Eigen::Isometry3d::Identity();
        if (pose_graph_backend_->queryPoseMap(inserted.id, optimized_pose_map)) {
            const Eigen::Isometry3d target_map_to_odom = optimized_pose_map * inserted.pose_odom.inverse();
            if (!map_to_odom_smoother_->isInitialized()) {
                map_to_odom_smoother_->reset(map_to_odom, stamp);
            }
            const Eigen::Isometry3d current_map_to_odom = map_to_odom_smoother_->current();
            const double delta_xy = (target_map_to_odom.translation().head<2>() -
                                     current_map_to_odom.translation().head<2>())
                                        .norm();
            const double delta_yaw_deg = std::abs(normalizeAngle(yawOf(target_map_to_odom) - yawOf(current_map_to_odom))) *
                                         180.0 / M_PI;
            const bool jump_suppressed =
                delta_xy > graph_jump_detect_translation_m_ || delta_yaw_deg > graph_jump_detect_yaw_deg_;
            pose_graph_backend_->setMapToOdomJumpSuppressed(jump_suppressed);
            map_to_odom_smoother_->setTarget(target_map_to_odom, stamp);
        }
    }

    graph_status_cache_ = pose_graph_backend_->statusSnapshot(this->now());
    backend_candidate_conflict_count_.store(graph_status_cache_.candidate_conflict_count);
    backend_map_to_odom_jump_suppressed_.store(graph_status_cache_.map_to_odom_jump_suppressed);
    (void)updated;
    return attach_result;
}

}  // namespace rc26_localization
