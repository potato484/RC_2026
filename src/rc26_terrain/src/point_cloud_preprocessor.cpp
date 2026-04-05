#include "rc26_terrain/point_cloud_preprocessor.hpp"

#include "pcl/filters/voxel_grid.h"

namespace rc26_terrain {

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPreprocessor::process(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input) const {
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (!input) {
        return output;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(cfg_.voxel_leaf_size_m),
                      static_cast<float>(cfg_.voxel_leaf_size_m),
                      static_cast<float>(cfg_.voxel_leaf_size_m));
    voxel.setInputCloud(input);
    voxel.filter(*output);
    return output;
}

}  // namespace rc26_terrain
