/** @executor.hpp

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

#ifndef EXECUTOR_HPP_
#define EXECUTOR_HPP_

#include <floor_reconstruction/cluster/cluster.hpp>
#include <floor_reconstruction/filter/filter.hpp>
#include <floor_reconstruction/segment/segment.hpp>

/** @brief Result codification struct */
enum Result {
    NO_SEGMENT = -2,
	NO_FILTER = -1,
	SUCCESS = 0
};

/**
 *  Basic executor implementation.
 *
 *  This class contains the calls to all the steps which compose the algorithm.
 *  The specific implementation of each step has to be define after instantiate
 *  the class using the set functions.
 *
 * @todo Once all the steps are implemented the algorithm should work in two
 *       threads: one for the point cloud steps and the second one for the
 *       image analysis.
 *
 * @todo Once all the steps are implemented, remove the setters and pass them in
 * 		 the constructor.
 */

class Executor {
public:
    /**
     *  @brief  Constructor of the executor.
     */
	Executor();

	/**
     *  @brief  Destructor of the executor.
     */
	virtual ~Executor();

    /**
     *  @brief  Execute all the algorithim steps over a given pointcloud.
     *
     *  @param  cloud Pointer to pointcloud to be processed.
     *  @return result of the process, using the Result enum.
     */
	Result execute(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

	/**
     *  @brief  Set filter implementation.
     */
	void set_filter(const std::shared_ptr<Filter> &filter);

    /**
     *  @brief  Set segment implementation.
     */
	void set_segment(const std::shared_ptr<Segment> &segment);

    /**
     *  @brief  Set cluster implementation.
     */
	void set_cluster(const std::shared_ptr<Cluster> &cluster);

private:
	std::shared_ptr<Cluster> _cluster; // Cluster defined.
	std::shared_ptr<Filter> _filter; // Filter defined.
	std::shared_ptr<Segment> _segment; // Segment defined.
};
#endif /* EXECUTOR_HPP_ */
