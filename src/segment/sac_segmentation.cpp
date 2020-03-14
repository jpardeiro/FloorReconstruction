#include "floor_reconstruction/segment/sac_segmentation.hpp"

SacSegmentation::SacSegmentation() : _remaining_points_coefficient(0.3) {
  //@todo use a config file
  _segmentation.setOptimizeCoefficients(true);
  _segmentation.setModelType(pcl::SACMODEL_PLANE);
  _segmentation.setMethodType(pcl::SAC_RANSAC);
  _segmentation.setMaxIterations(1000);
  _segmentation.setDistanceThreshold(0.02);
}

SacSegmentation::~SacSegmentation() {}

void SacSegmentation::segment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                              std::vector<Surface::Ptr>& surfaces) {
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

  const uint32_t n_points = cloud->points.size();
  while (cloud->points.size() > _remaining_points_coefficient * n_points) {
    // Segment the largest planar component from the remaining cloud
    _segmentation.setInputCloud(cloud);
    _segmentation.segment(*inliers, coefficients);

    Surface::Ptr surf = std::make_shared<Surface>();

    if (inliers->indices.size() == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset."
                << std::endl;
      // @todo return error code
      break;
    }
    // Extract the inliers
    _extract.setInputCloud(cloud);
    _extract.setIndices(inliers);
    _extract.setNegative(false);
    _extract.filter(*surf->cloud);

    surf->a = coefficients.values[0];
    surf->b = coefficients.values[1];
    surf->c = coefficients.values[2];
    surf->d = coefficients.values[3];

    surfaces.push_back(surf);

    // Create the filtering object
    _extract.setNegative(true);
    _extract.filter(*cloud);
  }
}