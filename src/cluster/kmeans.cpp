#include "floor_reconstruction/cluster/kmeans.hpp"

#include <pcl/ml/kmeans.h>

Kmeans::Kmeans() {

}

Kmeans::~Kmeans() {

}

void Kmeans::cluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, const uint32_t k,
        Centroids& centroids) {

    if(centroids.size() == 0) {
        centroids.reserve(k);
    } else if(centroids.size() != k) {
        std::cout << "The vector has " << centroids.size() << " elements but " <<
            k << " sre required" << std::endl;
        
        return;
    }

    pcl::Kmeans kmeans(cloud->points.size(), 3);
    kmeans.setClusterSize(k);

    for(const auto& cloud_point: cloud->points) {
        auto point = pcl::Kmeans::Point{cloud_point.x, cloud_point.y, cloud_point.z};
        kmeans.addDataPoint(point);
    }
    kmeans.kMeans();
    pcl::Kmeans::Centroids pcl_centroids = kmeans.get_centroids();

    assert(centroids.size() != pcl_centroids.size());

    for(const auto& centroid: pcl_centroids) {
        centroids.emplace_back(Centroid(centroid[0], centroid[1], centroid[2]));
    }
}