// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/traits.hpp>

namespace small_gicp {

/// @brief Point-to-plane per-point error factor.
struct PointToPlaneICPFactor {
  struct Setting {};

  PointToPlaneICPFactor(const Setting& setting = Setting()) : target_index(std::numeric_limits<size_t>::max()), source_index(std::numeric_limits<size_t>::max()) {}

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree, typename CorrespondenceRejector>
  bool linearize(
    const TargetPointCloud& target,
    const SourcePointCloud& source,
    const TargetTree& target_tree,
    const Eigen::Isometry3d& T,
    size_t source_index,
    const CorrespondenceRejector& rejector,
    Eigen::Matrix<double, 6, 6>* H,
    Eigen::Matrix<double, 6, 1>* b,
    double* e) {
    //
    this->source_index = source_index;
    this->target_index = std::numeric_limits<size_t>::max();

    const Eigen::Vector4d transed_source_pt = T * traits::point(source, source_index);

    size_t k_index;
    double k_sq_dist;
    if (!traits::nearest_neighbor_search(target_tree, transed_source_pt, &k_index, &k_sq_dist) || rejector(target, source, T, k_index, source_index, k_sq_dist)) {
      return false;
    }

    const Eigen::Vector3d target_normal = traits::normal(target, k_index).template head<3>();
    if (target_normal.squaredNorm() < 1e-12) {
      return false;
    }

    target_index = k_index;
    const Eigen::Vector3d residual = (traits::point(target, target_index) - transed_source_pt).template head<3>();
    const double err = target_normal.dot(residual);

    Eigen::Matrix<double, 1, 6> J;
    J.block<1, 3>(0, 0) = target_normal.transpose() * T.linear() * skew(traits::point(source, source_index).template head<3>());
    J.block<1, 3>(0, 3) = -target_normal.transpose() * T.linear();

    *H = J.transpose() * J;
    *b = J.transpose() * err;
    *e = 0.5 * err * err;

    return true;
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  double error(const TargetPointCloud& target, const SourcePointCloud& source, const Eigen::Isometry3d& T) const {
    if (target_index == std::numeric_limits<size_t>::max()) {
      return 0.0;
    }

    const Eigen::Vector4d transed_source_pt = T * traits::point(source, source_index);
    const Eigen::Vector3d normal = traits::normal(target, target_index).template head<3>();
    if (normal.squaredNorm() < 1e-12) {
      return 0.0;
    }
    const Eigen::Vector3d residual = (traits::point(target, target_index) - transed_source_pt).template head<3>();
    const double err = normal.dot(residual);
    return 0.5 * err * err;
  }

  bool inlier() const { return target_index != std::numeric_limits<size_t>::max(); }

  size_t target_index;
  size_t source_index;
};
}  // namespace small_gicp
