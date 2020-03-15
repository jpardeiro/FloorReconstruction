#include "floor_reconstruction/filter/voxel_grid.hpp"

VoxelGrid::VoxelGrid(const float &leaf_size_x, const float &leaf_size_y,
                     const float &leaf_size_z) {
  filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

void VoxelGrid::perform_filtering(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
  // Set the input cloud
  filter_.setInputCloud(cloud);

  // Perform the filtering
  filter_.filter(*cloud);
}
