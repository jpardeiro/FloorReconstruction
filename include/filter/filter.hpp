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

#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <pcl/io/pcd_io.h>

/** @brief Basic interface for filter implementation.

    This class contains the basic interface class with the basic function to execute the
    filtering process. This class is going to be inherited by the filter implementations where
   	the functions behavior is going to be defined.

    @author Jose Pardeiro
 */

class Filter {
public:
	/** @brief Constructor. */
	Filter() {};

	/** @brief Destructor. */
	virtual ~Filter() {};

	/** @brief Filter function definition. The function is implemented in
	 	the filter specific classes. */
	virtual void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) = 0;
};
#endif /* FILTER_HPP_ */
