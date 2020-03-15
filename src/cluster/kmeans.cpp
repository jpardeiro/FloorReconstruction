#include "floor_reconstruction/cluster/kmeans.hpp"

void Kmeans::perform_clustering(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                                const uint32_t k,
                                std::vector<Centroid>& centroids) {
  pcl::Kmeans kmeans(cloud->points.size(), 3);
  kmeans.setClusterSize(k);

  for (const auto& cloud_point : cloud->points) {
    pcl::Kmeans::Point point{cloud_point.x, cloud_point.y, cloud_point.z};
    kmeans.addDataPoint(point);
  }
  kmeans.kMeans();
  pcl_to_centroids_converter(kmeans.get_centroids(), centroids);
}

void Kmeans::pcl_to_centroids_converter(const pcl::Kmeans::Centroids& pcl_centroids,
                                        std::vector<Centroid>& centroids)
{
  const size_t n_centroids = pcl_centroids.size();
  if (centroids.size() != n_centroids) {
    centroids.resize(n_centroids);
  }

  for (size_t i = 0; i < n_centroids; ++i) {
    auto& centroid = pcl_centroids[i];
    centroids[i] = Centroid(centroid[0], centroid[1], centroid[2]);
  }
}