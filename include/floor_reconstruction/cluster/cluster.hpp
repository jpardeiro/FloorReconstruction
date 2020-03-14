/** @file cluster.hpp

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

#ifndef CLUSTER__CLUSTER_HPP_
#define CLUSTER__CLUSTER_HPP_

#include <pcl/io/pcd_io.h>
#include <floor_reconstruction/base/structs.hpp>

/**
 *  Interface class for the cluster.
 *
 *  This class is purely virtual and includes the basic interface for the
 *  clustering process.
 *
 *  The purpose of the clustering consists on split the input data into multiple
 *  groups which share certain relevant characteristics.
 */

class Cluster {
public:
    /**
     *  @brief  Constructor of the cluster.
     */
    Cluster() {}

	/**
     *  @brief  Destructor of the cluster.
     */
	virtual ~Cluster() {}

    /**
     *  @brief  Cluster function definition. The function is meant to be
     *          implemented by the cluster specific classes.
     *
     *  @param  cloud Pointer to cloud containing the data to run the cluster.
     *  @param  k Number of clusters in which the cloud is going to be split.
     *            This number is crucial for both performance and robustness.
     *            In performance because the number of clusters have a
     *            significant impact on the execution time of the function.
     *            In robustness, because a large number of clusters can split
     *            very similar data into multiple clusters due to very small
     *            differences.
     *  @param  centroids Resulting vector of Centroid containing the center of
     *                    each cluster.
     */
	virtual void cluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, const uint32_t k,
        Centroids& centroids) = 0;
};
#endif /* CLUSTER__CLUSTER_HPP_ */
