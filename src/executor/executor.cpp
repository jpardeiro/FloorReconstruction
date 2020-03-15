#include "floor_reconstruction/executor/executor.hpp"

Result Executor::execute(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
  // If no filter defined return an error
  if (!filter_) return Result::NO_FILTER;

  if (!segment_) return Result::NO_SEGMENT;

  // Execute filtering process
  filter_->perform_filtering(cloud);

  // Execute the segmentation process
  std::vector<Surface::Ptr> surfaces;
  segment_->perform_segmentation(cloud, surfaces);
  std::cout << "surfaces size: " << surfaces.size() << std::endl;

  std::vector<Centroid> centroids;
  cluster_->perform_clustering(surfaces[0]->cloud, 3, centroids);

  return Result::SUCCESS;
}

void Executor::set_filter(const std::shared_ptr<Filter> &filter) {
  filter_ = filter;
}

void Executor::set_segment(const std::shared_ptr<Segment> &segment) {
  segment_ = segment;
}

void Executor::set_cluster(const std::shared_ptr<Cluster> &cluster) {
  cluster_ = cluster;
}
