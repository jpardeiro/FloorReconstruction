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

#ifndef FILTER__VOXEL_GRID_HPP_
#define FILTER__VOXEL_GRID_HPP_

#include <pcl/filters/voxel_grid.h>

#include <floor_reconstruction/filter/filter.hpp>

/**
 *  VoxelGrid filter implementation.
 *
 *  This class contains the implementation of the voxel grid filter.
 *
 *  The voxel grid filter down-samples the data by taking an average of the
 *  points in the cloud in terms of spatial distance. The filter divides the
 *  cloud in a set of cubes, called voxels, and the set of points which are
 *  located inside a voxel are combined into one output point. The size of the
 *  voxel can be configured in every axis.
 */

class VoxelGrid : public Filter {
 public:
  /**
   *  @brief  Constructor of the filter.
   *  @param  leaf_size_x Size of the voxel in the X axis, in meters.
   *  @param  leaf_size_y Size of the voxel in the Y axis, in meters.
   *  @param  leaf_size_z Size of the voxel in the Z axis, in meters.
   */
  VoxelGrid(const float &leaf_size_x = 0.03f, const float &leaf_size_y = 0.03f,
            const float &leaf_size_z = 0.03f);

  /**
   *  @brief  Destructor of the filter.
   */
  virtual ~VoxelGrid();

  /**
   *  @brief  VoxelGrid specific implementation of the filter function.
   */
  void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

 private:
  std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZRGBA>>
      _filter;  // Filter implementation.
};

#endif /* FILTER__VOXEL_GRID_HPP_ */
