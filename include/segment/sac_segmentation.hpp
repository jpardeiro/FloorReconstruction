/** @file extract_indices.hpp

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

#ifndef SEGMENT__SAC_SEGMENTATION_HPP_
#define SEGMENT__SAC_SEGMENTATION_HPP_

#include <segment/segment.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

/** @brief Basic interface for segment implementation.

    This class contains the basic interface class with the basic function to execute the
    segmentation process. This class is going to be inherited by the segment implementations where
   	the functions behavior is going to be defined.

    @author Jose Pardeiro
 */

class SacSegmentation : public Segment {
public:
	/** @brief Constructor. */
    //@todo pass the configuration in the constructor
	SacSegmentation();

	/** @brief Destructor. */
	virtual ~SacSegmentation();

	/** @brief Segment function implementation. It uses the ExtractIndices algorithm to segment the
     * point cloud and 
     */
	void segment(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
        std::vector<Surface::Ptr>& surfaces);

private:
    float _remaining_points_coefficient;
    pcl::SACSegmentation<pcl::PointXYZRGBA> _segmentation;
    pcl::ExtractIndices<pcl::PointXYZRGBA> _extract;
};
#endif /* SEGMENT__SAC_SEGMENTATION_HPP_ */
