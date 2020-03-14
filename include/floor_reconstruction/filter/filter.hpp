/** @file filter.hpp

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

#ifndef FILTER__FILTER_HPP_
#define FILTER__FILTER_HPP_

#include <pcl/io/pcd_io.h>

/**
 *  Interface class for the filter.
 *
 *  This class is purely virtual and includes the basic interface for the
 *  filtering process.
 *
 *  The purpose of the filtering consists on make the original pointcloud more
 *  lightweight in order to reduce complexity during the processing steps
 *  executed afterwards. The filtered pointcloud should be more lightweight but
 *  still contain all the key components of the original pointcloud.
 */

class Filter {
 public:
  /**
   *  @brief  Constructor of the filter.
   */
  Filter() {}

  /**
   *  @brief  Destructor of the filter.
   */
  virtual ~Filter() {}

  /**
   *  @brief  Filter function definition. The function is meant to be
   *          implemented by the filter specific classes.
   *
   *  @param  cloud Pointer to cloud to be filtered. At the end of the
   *                function, the pointer will point to the resulted filtered
   *                cloud.
   */
  virtual void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) = 0;
};

#endif /* FILTER__FILTER_HPP_ */
