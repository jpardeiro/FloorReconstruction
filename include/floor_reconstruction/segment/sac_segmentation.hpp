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

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <floor_reconstruction/segment/segment.hpp>

/**
 *  Random Sample Consensus segmentation implementation.
 *
 *  This class contains the implementation of the Random Sample Consensus
 *  segmentation. Detailed information about this type of segmentation
 *  algorithms can be found
 * [here](https://en.wikipedia.org/wiki/Random_sample_consensus)
 */

class SacSegmentation : public Segment {
 public:
  /**
   *  @brief  Constructor of the sac segmentation object.
   */
  //@todo pass the configuration in the constructor
  SacSegmentation();

  /**
   *  @brief  Destructor of the sac segmentation object.
   */
  virtual ~SacSegmentation() = default;

  /**
   *  @brief  Sac segmentation specific implementation of the segment function.
   */
  void perform_segmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,
                            std::vector<Surface::Ptr>& surfaces) override;

 private:
  float remaining_points_coefficient_;  // As the segmentation is iterative,
                                        // this variable specifies minimum the
                                        // size of the unsegmented cloud to
                                        // stop the process
  pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation_;  // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract_;        // Filtering object
};
#endif /* SEGMENT__SAC_SEGMENTATION_HPP_ */
