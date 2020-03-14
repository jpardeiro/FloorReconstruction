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
  Kmeans();

  /**
   *  @brief  Destructor of the cluster.
   */
  virtual ~Kmeans();

  /**
   *  @brief  Kmeans specific implementation of the cluster function.
   */
  void cluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, const uint32_t k,
               Centroids& centroids);
};
#endif /* CLUSTER__CLUSTER_HPP_ */