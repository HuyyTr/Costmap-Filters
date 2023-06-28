/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *********************************************************************/

#include "costmap_filters/costmap_filter.hpp"

#include <exception>

#include "geometry_msgs/PointStamped.h"

#include "costmap_2d/cost_values.h"
#include "costmap_filters/occ_grid_values.hpp"

namespace costmap_2d
{

CostmapFilter::CostmapFilter()
: filter_info_topic_(""), mask_topic_("")
{
  access_ = new mutex_t();
  node_ = boost::make_shared<ros::NodeHandle>("~/" + name_);
}

CostmapFilter::~CostmapFilter()
{
  delete access_;
}

void CostmapFilter::onInitialize()
{
    // Get parameters
    node_->param<bool>("enabled", enabled_, true);
    node_->param<std::string>("filter_info_topic", filter_info_topic_, "");
    double transform_tolerance {};
    node_->param<double>("transform_tolerance", transform_tolerance, 0.1);

    transform_tolerance_ = ros::Duration(transform_tolerance);

    // Costmap Filter enabling service
    enable_service_ = boost::make_shared<ros::ServiceServer>(node_->advertiseService("toggle_filter", &CostmapFilter::enableCallback, this));
}

void CostmapFilter::activate()
{
  initializeFilter(filter_info_topic_);
}

void CostmapFilter::deactivate()
{
  resetFilter();
}

void CostmapFilter::reset()
{
  resetFilter();
  initializeFilter(filter_info_topic_);
  current_ = false;
}

void CostmapFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  if (!enabled_) {
    return;
  }

  latest_pose_.x = robot_x;
  latest_pose_.y = robot_y;
  latest_pose_.theta = robot_yaw;
}

void CostmapFilter::updateCosts(
  costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
  current_ = true;
}

bool CostmapFilter::enableCallback(
  std_srvs::SetBool::Request &request,
  std_srvs::SetBool::Response &response)
{
  enabled_ = request.data;
  response.success = true;
  if (enabled_) {
    response.message = "Enabled";
  } else {
    response.message = "Disabled";
  }
}

bool CostmapFilter::transformPose(
  const std::string global_frame,
  const geometry_msgs::Pose2D & global_pose,
  const std::string mask_frame,
  geometry_msgs::Pose2D & mask_pose) const
{
  if (mask_frame != global_frame) {
    // Filter mask and current layer are in different frames:
    // Transform (global_pose.x, global_pose.y) point from current layer frame (global_frame)
    // to mask_pose point in mask_frame
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PointStamped in, out;
    in.header.stamp = ros::Time::now();
    in.header.frame_id = global_frame;
    in.point.x = global_pose.x;
    in.point.y = global_pose.y;
    in.point.z = 0;

    try {
      tf_->transform(in, out, mask_frame, transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      ROS_ERROR(
        "CostmapFilter: failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        global_frame.c_str(), mask_frame.c_str(), ex.what());
      return false;
    }
    mask_pose.x = out.point.x;
    mask_pose.y = out.point.y;
  } else {
    // Filter mask and current layer are in the same frame:
    // Just use global_pose coordinates
    mask_pose = global_pose;
  }

  return true;
}

bool CostmapFilter::worldToMask(
  nav_msgs::OccupancyGrid::ConstPtr filter_mask,
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  const double origin_x = filter_mask->info.origin.position.x;
  const double origin_y = filter_mask->info.origin.position.y;
  const double resolution = filter_mask->info.resolution;
  const unsigned int size_x = filter_mask->info.width;
  const unsigned int size_y = filter_mask->info.height;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);
  if (mx >= size_x || my >= size_y) {
    return false;
  }

  return true;
}

unsigned char CostmapFilter::getMaskCost(
  nav_msgs::OccupancyGrid::ConstPtr filter_mask,
  const unsigned int mx, const unsigned int & my) const
{
  const unsigned int index = my * filter_mask->info.width + mx;

  const char data = filter_mask->data[index];
  if (data == costmap_2d::OCC_GRID_UNKNOWN) {
    return NO_INFORMATION;
  } else {
    // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
    // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
    return std::round(
      static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
      (costmap_2d::OCC_GRID_OCCUPIED - costmap_2d::OCC_GRID_FREE));
  }
}

}  // namespace costmap_2d
