#include <floor_reconstruction/executor/executor.hpp>
#include <floor_reconstruction/filter/voxel_grid.hpp>
#include <floor_reconstruction/segment/sac_segmentation.hpp>
#include <floor_reconstruction/cluster/kmeans.hpp>

#include <pcl/console/time.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Read PointCloud from PCD file
	pcl::PCDReader reader;
	reader.read (argv[1], *cloud);

	// Instantiate the executor
	auto executor = std::shared_ptr<Executor>(std::make_shared<Executor>());

	// Enable filter
	auto filter = std::make_shared<VoxelGrid>();
	executor->set_filter(filter);

	// Enable segment
	auto segment = std::make_shared<SacSegmentation>();
	executor->set_segment(segment);

	auto cluster = std::make_shared<Kmeans>();
	executor->set_cluster(cluster);

	pcl::console::TicToc timer;
	timer.tic();

	// Execute the filter
	executor->execute(cloud);

	timer.toc_print();

	return 0;
}
