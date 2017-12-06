#include <cluster/kmeans.hpp>

#include <pcl/ml/kmeans.h>

Kmeans::Kmeans() {

}

Kmeans::~Kmeans() {

}   

void Kmeans::cluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, const uint32_t k) {
    // pcl::Kmeans kmeans(cloud->points.size(), 3);
    // kmeans.setClusterSize(k);

    // const uint32_t n_points = cloud->points.size();
    // std::vector<pcl::Kmeans::Point> points(n_points);

    // for(uint32_t i = 0; i < n_points; ++i) {
    //     points[i] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    // }
    // kmeans.setInputData(points);
    // kmeans.computeCentroids();
    // pcl::Kmeans::Centroids centroids = kmeans.get_centroids();
    // std::cout << "n centroids: " << centroids.size() << std::endl;
    // std::cout << "size: " << centroids[0].size() << std::endl;
}