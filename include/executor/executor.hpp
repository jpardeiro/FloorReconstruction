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

#include <cluster/cluster.hpp>
#include <filter/filter.hpp>
#include <segment/segment.hpp>

/** @brief Result codification struct */
enum Result {
    NO_SEGMENT = -2,
	NO_FILTER = -1,
	SUCCESS = 0
};

/** @brief Basic executor implementation.

    This class contains the calls to all the steps which compose the algorithm.
    The specific implementation of each step has to be define after instantiate
    the class using the set functions.

	@todo Once all the steps are implemented the algorithm should
	work in two threads: one for the point cloud steps and the
	second one for the image analysis.

    @author Jose Pardeiro
 */

class Executor {
public:
	/** @brief Constructor. */
	Executor();

	/** @brief Destructor. */
	virtual ~Executor();

	/** @brief Execute the algorithm over an specific point cloud.

	 	@return Result code */
	Result execute(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

	/** @brief Set filter function. */
	void set_filter(const std::shared_ptr<Filter> &filter);

    /** @brief Set segment function. */
	void set_segment(const std::shared_ptr<Segment> &segment);

    /** @brief Set cluster function. */
	void set_cluster(const std::shared_ptr<Cluster> &cluster);

private:
	std::shared_ptr<Cluster> _cluster; ///< Cluster defined.
	std::shared_ptr<Filter> _filter; ///< Filter defined.
	std::shared_ptr<Segment> _segment; ///< Segment defined.
};
#endif /* EXECUTOR_HPP_ */
