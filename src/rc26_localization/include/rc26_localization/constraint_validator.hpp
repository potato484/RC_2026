// Copyright 2025 RC2026

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rc26_localization {

enum class ConstraintType : uint8_t {
    LOOP = 0U,
    ANCHOR = 1U,
    UWB_SOFT_ANCHOR = 2U,
};

struct ConstraintValidationInput {
    ConstraintType type{ConstraintType::LOOP};
    uint32_t from_id{0U};
    uint32_t to_id{0U};
    Eigen::Isometry3d initial_relative_pose{Eigen::Isometry3d::Identity()};
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> source_cloud;
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> target_cloud;
    bool imu_spike{false};
};

struct ConstraintValidationResult {
    bool accepted{false};
    bool conflict{false};
    Eigen::Isometry3d relative_pose{Eigen::Isometry3d::Identity()};
    double fitness{1e9};
    double score{0.0};
    std::string reason{"not_run"};
};

class ConstraintValidator {
public:
    struct Config {
        double accept_fitness_threshold{0.1};
        double conflict_fitness_threshold{0.25};
        double max_correspondence_distance_m{1.0};
        int max_iterations{50};
        int num_neighbors{20};
        int num_threads{4};
        float voxel_leaf_size{0.25F};
    };

    ConstraintValidator();
    explicit ConstraintValidator(const Config& cfg);

    void setConfig(const Config& cfg);
    Config getConfig() const;
    ConstraintValidationResult validate(const ConstraintValidationInput& input) const;

private:
    Config config_;
};

}  // namespace rc26_localization
