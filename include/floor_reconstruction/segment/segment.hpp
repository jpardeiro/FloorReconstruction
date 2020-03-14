/** @file segment.hpp

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

#ifndef SEGMENT__SEGMENT_HPP_
#define SEGMENT__SEGMENT_HPP_

#include <pcl/io/pcd_io.h>

#include <floor_reconstruction/base/structs.hpp>

/**
 *  Interface class for the segmentation.
 *
 *  This class is purely virtual and includes the basic interface for the
 *  segmentation process.
 *
 *  The purpose of the segmentation consists on take an input pointcloud and
 *  split it into multiple planes which contain a significant amount of data.
 *  As an example, if the point cloud contains a floor and two walls as
 *  significant areas, the resulting planes will contain the data required to
 *  define them.
 */

class Segment {
 public:
  /**
   *  @brief  Constructor of the segmentation object.
   */
  Segment() {}

  /**
   *  @brief  Destructor of the segmentation object.
   */
  virtual ~Segment() {}

  /**
   *  @brief  Segment function definition. The function is meant to be
   *          implemented by the segment specific classes.
   *
   *  @param  cloud Pointer to cloud to be segmented.
   *  @param  surfaces Vector of pointers to the Surface struct. This vector
   *                   will contain the resulting segmented planes.
   */

  virtual void segment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                       std::vector<Surface::Ptr>& surfaces) = 0;
};

#endif /* SEGMENT__SEGMENT_HPP_ */
