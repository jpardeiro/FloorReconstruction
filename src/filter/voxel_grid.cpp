#include <filter/voxel_grid.hpp>

VoxelGrid::VoxelGrid(const float &leaf_size_x,
                     const float &leaf_size_y,
                     const float &leaf_size_z) :
    _filter(std::make_shared<pcl::VoxelGrid<pcl::PointXYZRGBA>>()) {
	_filter->setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelGrid::~VoxelGrid() {}

void VoxelGrid::filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
	// Set the input cloud
	_filter->setInputCloud(cloud);

	// Perform the filtering
	_filter->filter(*cloud);
}
