#include <executor/executor.hpp>
#include <filter/voxel_grid.hpp>

#include <pcl/console/time.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Read PointCloud from PCD file
	pcl::PCDReader reader;
	reader.read (argv[1], *cloud);

	// Instantiate the executor
	auto executor = std::shared_ptr<Executor>(std::make_shared<Executor>());

	// Instantiate the filter
	auto filter = std::shared_ptr<Filter>(std::make_shared<VoxelGrid>());

	// Set the filter
	executor->set_filter(filter);

	pcl::console::TicToc timer;
	timer.tic();

	// Execute the filter
	executor->execute(cloud);

	timer.toc_print();

	return 0;
}
