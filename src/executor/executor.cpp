#include "floor_reconstruction/executor/executor.hpp"

Executor::Executor() {}

Executor::~Executor() {}

Result Executor::execute(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
  // If no filter defined return an error
  if (!_filter) return Result::NO_FILTER;

  if (!_segment) return Result::NO_SEGMENT;

  // Execute filtering process
  _filter->filter(cloud);

  // Execute the segmentation process
  std::vector<Surface::Ptr> surfaces;
  _segment->segment(cloud, surfaces);
  std::cout << "surfaces size: " << surfaces.size() << std::endl;

  Centroids centroids;
  _cluster->cluster(surfaces[0]->cloud, 3, centroids);

  return Result::SUCCESS;
}

void Executor::set_filter(const std::shared_ptr<Filter> &filter) {
  _filter = filter;
}

void Executor::set_segment(const std::shared_ptr<Segment> &segment) {
  _segment = segment;
}

void Executor::set_cluster(const std::shared_ptr<Cluster> &cluster) {
  _cluster = cluster;
}
