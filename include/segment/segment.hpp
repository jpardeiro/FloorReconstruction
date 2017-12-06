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
#include <base/structs.hpp>

/** @brief Basic interface for segment implementation.

    This class contains the basic interface class with the basic function to execute the
    segmentation process. This class is going to be inherited by the segment implementations where
   	the functions behavior is going to be defined.

    @author Jose Pardeiro
 */

class Segment {
public:
	/** @brief Constructor. */
	Segment() {}

	/** @brief Destructor. */
	virtual ~Segment() {}

	/** @brief Segment function definition. The function is implemented in
	 	the segment specific classes. */
	virtual void segment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
        std::vector<Surface::Ptr>& surfaces) = 0;
};
#endif /* SEGMENT__SEGMENT_HPP_ */