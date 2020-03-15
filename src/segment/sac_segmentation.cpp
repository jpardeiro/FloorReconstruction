#include "floor_reconstruction/segment/sac_segmentation.hpp"

SacSegmentation::SacSegmentation() : remaining_points_coefficient_(0.3) {
  //@todo use a config file
  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setModelType(pcl::SACMODEL_PLANE);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
  segmentation_.setMaxIterations(1000);
  segmentation_.setDistanceThreshold(0.02);
}

void SacSegmentation::perform_segmentation(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
    std::vector<Surface::Ptr>& surfaces) {
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

  const uint32_t n_points = cloud->points.size();
  while (cloud->points.size() > remaining_points_coefficient_ * n_points) {
    // Segment the largest planar component from the remaining cloud
    segmentation_.setInputCloud(cloud);
    segmentation_.segment(*inliers, coefficients);

    Surface::Ptr surf = std::make_shared<Surface>();

    if (inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset."
                << std::endl;
      // @todo return error code
      break;
    }
    // Extract the inliers
    extract_.setInputCloud(cloud);
    extract_.setIndices(inliers);
    extract_.setNegative(false);
    extract_.filter(*surf->cloud);

    surf->a = coefficients.values[0];
    surf->b = coefficients.values[1];
    surf->c = coefficients.values[2];
    surf->d = coefficients.values[3];

    surfaces.push_back(surf);

    // Create the filtering object
    extract_.setNegative(true);
    extract_.filter(*cloud);
  }
}