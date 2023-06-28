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

#include <string>
#include <memory>
#include <algorithm>
#include "tf2/convert.h"
#include "tf/tf.h"

#include "costmap_filters/keepout_filter.hpp"
#include "costmap_filters/filter_values.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::KeepoutFilter, costmap_2d::Layer)

namespace costmap_2d
{

KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr), mask_sub_(nullptr), filter_mask_(nullptr),
  global_frame_("")
{
}

void KeepoutFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  ROS_INFO(
    "KeepoutFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());
  
  filter_info_sub_ = boost::make_shared<ros::Subscriber>(node_->subscribe(filter_info_topic_, 1, &KeepoutFilter::filterInfoCallback, this));

  global_frame_ = layered_costmap_->getGlobalFrameID();
}

void KeepoutFilter::filterInfoCallback(
  const costmap_filters::CostmapFilterInfo::ConstPtr &msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_sub_) {
    ROS_INFO(
      "KeepoutFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    ROS_WARN(
      "KeepoutFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    ROS_ERROR(
      "KeepoutFilter: For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  mask_topic_ = msg->filter_mask_topic;

  // Setting new filter mask subscriber
  ROS_INFO(
    "KeepoutFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = boost::make_shared<ros::Subscriber>(node_->subscribe(mask_topic_, 1, &KeepoutFilter::maskCallback, this));
}

void KeepoutFilter::maskCallback(
  const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    ROS_INFO(
      "KeepoutFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    ROS_WARN(
      "KeepoutFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  // Store filter_mask_
  filter_mask_ = msg;
}

void KeepoutFilter::process(
  costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::Pose2D & /*pose*/)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    // Show warning message every 2 seconds to not litter an output
    ROS_WARN_THROTTLE(2000,
      "KeepoutFilter: Filter mask was not received");
    return;
  }

  tf::Transform tf_transform;
  tf_transform.setIdentity();  // initialize by identical transform
  int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left window corner
  int mg_max_x, mg_max_y;  // masger_grid indexes of top-right window corner

  const std::string mask_frame = filter_mask_->header.frame_id;

  if (mask_frame != global_frame_) {
    // Filter mask and current layer are in different frames:
    // prepare frame transformation if mask_frame != global_frame_
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame, global_frame_, ros::Time(0),
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      ROS_ERROR(
        "KeepoutFilter: Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame_.c_str(), mask_frame.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf_transform);

    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {
    // Filter mask and current layer are in the same frame:
    // apply the following optimization - iterate only in overlapped
    // (min_i, min_j)..(max_i, max_j) & filter_mask_ area.
    //
    //           filter_mask_
    //       *----------------------------*
    //       |                            |
    //       |                            |
    //       |      (2)                   |
    // *-----+-------*                    |
    // |     |///////|<- overlapped area  |
    // |     |///////|   to iterate in    |
    // |     *-------+--------------------*
    // |    (1)      |
    // |             |
    // *-------------*
    //  master_grid (min_i, min_j)..(max_i, max_j) window
    //
    // ToDo: after costmap rotation will be added, this should be re-worked.

    double wx, wy;  // world coordinates

    // Calculating bounds corresponding to bottom-left overlapping (1) corner
    // filter_mask_ -> master_grid indexes conversion
    const double half_cell_size = 0.5 * filter_mask_->info.resolution;
    wx = filter_mask_->info.origin.position.x + half_cell_size;
    wy = filter_mask_->info.origin.position.y + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    // Calculation of (1) corner bounds
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    // Calculating bounds corresponding to top-right window (2) corner
    // filter_mask_ -> master_grid intexes conversion
    wx = filter_mask_->info.origin.position.x +
      filter_mask_->info.width * filter_mask_->info.resolution + half_cell_size;
    wy = filter_mask_->info.origin.position.y +
      filter_mask_->info.height * filter_mask_->info.resolution + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    // Calculation of (2) corner bounds
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }

  // unsigned<-signed conversions.
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;  // master_grid iterators
  unsigned int index;  // corresponding index of master_grid
  double gl_wx, gl_wy;  // world coordinates in a global_frame_
  double msk_wx, msk_wy;  // world coordinates in a mask_frame
  unsigned int mx, my;  // filter_mask_ coordinates
  unsigned char data, old_data;  // master_grid element data

  // Main master_grid updating loop
  // Iterate in costmap window by master_grid indexes
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at filter_mask_:
      // Get world coordinates in global_frame_
      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame != global_frame_) {
        // Transform (i, j) point from global_frame_ to mask_frame
        tf::Vector3 point(gl_wx, gl_wy, 0);
        point = tf_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {
        // In this case master_grid and filter-mask are in the same frame
        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }
      // Get mask coordinates corresponding to (i, j) point at filter_mask_
      if (worldToMask(filter_mask_, msk_wx, msk_wy, mx, my)) {
        data = getMaskCost(filter_mask_, mx, my);
        // Update if mask_ data is valid and greater than existing master_grid's one
        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
      }
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

bool KeepoutFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (filter_mask_) {
    return true;
  }
  return false;
}

}  // namespace nav2_costmap_2d
