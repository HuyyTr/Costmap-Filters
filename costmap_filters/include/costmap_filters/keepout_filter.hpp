/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#ifndef __COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_
#define __COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_

#include <string>
#include <memory>

#include "costmap_filters/costmap_filter.hpp"

#include "costmap_filters/CostmapFilterInfo.h"

namespace costmap_2d
{

/**
 * @class KeepoutFilter
 * @brief Reads in a keepout mask and marks keepout regions in the map
 * to prevent planning or control in restricted areas
 */
class KeepoutFilter : public CostmapFilter
{
public:
  /**
   * @brief A constructor
   */
  KeepoutFilter();

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(
    const std::string & filter_info_topic);

  /**
   * @brief Process the keepout layer at the current pose / bounds / grid
   */
  void process(
    costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::Pose2D & pose);

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter();

  /**
   * @brief If this filter is active
   */
  bool isActive();

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const costmap_filters::CostmapFilterInfo::ConstPtr &msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  boost::shared_ptr<ros::Subscriber> filter_info_sub_;
  boost::shared_ptr<ros::Subscriber> mask_sub_;

  boost::shared_ptr<const nav_msgs::OccupancyGrid> filter_mask_;

  std::string global_frame_;  // Frame of currnet layer (master_grid)
};

}  // namespace costmap_2d

#endif  // __COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_
