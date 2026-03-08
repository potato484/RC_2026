#include "rc26_localization/constraint_validator.hpp"

#include <algorithm>
#include <cmath>

#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace rc26_localization {

ConstraintValidator::ConstraintValidator() : ConstraintValidator(Config{}) {}

ConstraintValidator::ConstraintValidator(const Config& cfg) : config_(cfg) {}

void ConstraintValidator::setConfig(const Config& cfg) {
    config_ = cfg;
}

ConstraintValidator::Config ConstraintValidator::getConfig() const {
    return config_;
}

ConstraintValidationResult ConstraintValidator::validate(const ConstraintValidationInput& input) const {
    ConstraintValidationResult output;
    output.relative_pose = input.initial_relative_pose;

    if (input.imu_spike) {
        output.reason = "imu_spike_active";
        return output;
    }
    if (!input.source_cloud || !input.target_cloud || input.source_cloud->empty() || input.target_cloud->empty()) {
        output.reason = "empty_cloud";
        return output;
    }

    auto source_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *input.source_cloud, config_.voxel_leaf_size);
    auto target_cov =
        small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
            *input.target_cloud, config_.voxel_leaf_size);
    if (!source_cov || !target_cov || source_cov->empty() || target_cov->empty()) {
        output.reason = "empty_downsampled_cloud";
        return output;
    }

    small_gicp::estimate_covariances_omp(*source_cov, config_.num_neighbors, config_.num_threads);
    small_gicp::estimate_covariances_omp(*target_cov, config_.num_neighbors, config_.num_threads);
    auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(target_cov);

    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = config_.num_threads;
    registration.rejector.max_dist_sq = config_.max_correspondence_distance_m * config_.max_correspondence_distance_m;
    registration.optimizer.max_iterations = config_.max_iterations;

    const auto result = registration.align(*target_cov, *source_cov, *target_tree, input.initial_relative_pose);
    if (!result.converged || result.num_inliers == 0) {
        output.reason = "gicp_not_converged";
        return output;
    }

    output.relative_pose = result.T_target_source;
    output.fitness = result.error / static_cast<double>(result.num_inliers);
    output.score = std::exp(-std::max(0.0, output.fitness) * 10.0);
    output.conflict = output.fitness > config_.conflict_fitness_threshold;
    output.accepted = output.fitness < config_.accept_fitness_threshold;
    output.reason = output.accepted ? "accepted" : "fitness_too_high";
    return output;
}

}  // namespace rc26_localization
