#include <executor/executor.hpp>

Executor::Executor() {}

Executor::~Executor() {}

Result Executor::execute(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
	// If no filter defined return an error
	if (!_filter)
		return Result::NO_FILTER;

	// Execute filtering process
	_filter->filter(cloud);

	return Result::SUCCESS;
}

void Executor::set_filter(const std::shared_ptr<Filter> &filter) {
	_filter = filter;
}
