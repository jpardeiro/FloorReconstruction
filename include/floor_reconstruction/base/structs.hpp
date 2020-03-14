/** @file structs.hpp

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

#ifndef BASE__STRUCTS_HPP_
#define BASE__STRUCTS_HPP_

#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>

/**
 *  Set of structures implemented to run the algorithm.
 */

/**
 *  Surface definition. It contains the plane parameters and the cloud contained
 *  there.
 */
struct Surface {
  Surface() : cloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>()) {}
  double a, b, c, d;  // plane parameters
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      cloud;  // cloud on PointXYZRGBA format

  typedef typename std::shared_ptr<Surface> Ptr;
};

/**
 *  Point in 3D defined.
 */
struct Point {
  Point() : x(0), y(0), z(0) {}

  Point(const float& x_in, const float& y_in, const float& z_in)
      : x(x_in), y(y_in), z(z_in) {}

  float x;
  float y;
  float z;
};

typedef Point Centroid;

typedef std::vector<Centroid> Centroids;

#endif /* BASE__STRUCTS_HPP_ */