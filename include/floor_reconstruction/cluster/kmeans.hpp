/** @file kmeans.hpp

    Copyright (C) 2016 Jose Pardeiro <jose.pardeiro@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CLUSTER__KMEANS_HPP_
#define CLUSTER__KMEANS_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>

#include <floor_reconstruction/cluster/cluster.hpp>

/**
 *  Kmeans cluster implementation.
 *
 *  This class contains the implementation of the Kmeans cluster.
 *
 *  Detailed information about the Kmeans clustering can be found
 * [here](https://en.wikipedia.org/wiki/K-means_clustering)
 */

class Kmeans : public Cluster {
 public:
  /**
   *  @brief  Constructor of the cluster.
   */
  Kmeans() = default;

  /**
   *  @brief  Destructor of the cluster.
   */
  virtual ~Kmeans() = default;

  /**
   *  @brief  Kmeans specific implementation of the cluster function.
   */
  void perform_clustering(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                          const uint32_t k,
                          std::vector<Centroid>& centroids) override;

 private:
   /**
   *  @brief  Converter function from the specific return type from the PCL
   *          library to the general Centroids struct used by the code.
   *
   *  @param  pcl_centroids Centroids in the pcl::Kmeans library format
   *  @param  centroids Array of centroids to be fullfilled with the containt of
   *                    pcl_centroids.
   */
  void pcl_to_centroids_converter(const pcl::Kmeans::Centroids& pcl_centroids,
                                  std::vector<Centroid>& centroids);
};
#endif /* CLUSTER__CLUSTER_HPP_ */