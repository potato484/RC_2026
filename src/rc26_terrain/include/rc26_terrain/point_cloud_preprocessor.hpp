#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rc26_terrain {

struct PreprocessorConfig {
    double voxel_leaf_size_m{0.05};
};

class PointCloudPreprocessor {
public:
    explicit PointCloudPreprocessor(const PreprocessorConfig& cfg) : cfg_(cfg) {}

    pcl::PointCloud<pcl::PointXYZ>::Ptr process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input) const;

private:
    PreprocessorConfig cfg_;
};

}  // namespace rc26_terrain
