/** @file voxel_grid.hpp

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

#ifndef VOXEL_GRID_HPP_
#define VOXEL_GRID_HPP_

#include <filter/filter.hpp>
#include <pcl/filters/voxel_grid.h>

/** @brief VoxelGrid filter implementation.

    This class contains the implementation of the VoxelGrid filter.

    @author Jose Pardeiro
 */

class VoxelGrid : public Filter {
public:
	/** @brief Constructor. Define the default parameters */
	VoxelGrid(const float &leaf_size_x = 0.03f,
			  const float &leaf_size_y = 0.03f,
			  const float &leaf_size_z = 0.03f);

	/** @brief Destructor. */
	virtual ~VoxelGrid();

	/** @brief VoxelGrid specific implementation of the filter function */
	virtual void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

private:
	std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZRGBA>> _filter; ///< Filter definition.
};
#endif /* VOXEL_GRID_HPP_ */
