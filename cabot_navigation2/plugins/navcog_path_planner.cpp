// Copyright (c) 2020  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <algorithm>

#include <cabot_navigation2/navcog_path_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

using nav2_util::declare_parameter_if_not_declared;

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::NavCogPathPlanner, nav2_core::GlobalPlanner)

namespace cabot_navigation2
{
NavCogPathPlanner::NavCogPathPlanner()
{
}

// nav2_core::GlobalPlanner interface

NavCogPathPlanner::~NavCogPathPlanner()
{
}

void NavCogPathPlanner::cleanup()
{
}

void NavCogPathPlanner::activate()
{
}

void NavCogPathPlanner::deactivate()
{
}

void NavCogPathPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();

  auto node = parent_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring NavCog Path Planner: %s", name_.c_str());

  declare_parameter_if_not_declared(node, name + ".path_width", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".path_width", options_.path_width);

  declare_parameter_if_not_declared(node, name + ".path_min_width", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".path_min_width", options_.path_min_width);

  declare_parameter_if_not_declared(node, name + ".path_adjusted_center", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".path_adjusted_center", options_.path_adjusted_center);

  declare_parameter_if_not_declared(node, name + ".path_adjusted_minimum_path_width", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".path_adjusted_minimum_path_width", options_.path_adjusted_minimum_path_width);

  declare_parameter_if_not_declared(node, name + ".safe_margin", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".safe_margin", options_.safe_margin);

  declare_parameter_if_not_declared(node, name + ".robot_radius", rclcpp::ParameterValue(0.45));
  node->get_parameter(name + ".robot_radius", options_.robot_radius);

  declare_parameter_if_not_declared(node, name + ".path_length_to_width_factor", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".path_length_to_width_factor", options_.path_length_to_width_factor);

  declare_parameter_if_not_declared(node, name + ".path_topic", rclcpp::ParameterValue("/path"));
  node->get_parameter(name + ".path_topic", path_topic_);

  declare_parameter_if_not_declared(node, name + ".cost_threshold", rclcpp::ParameterValue(254));
  node->get_parameter(name + ".cost_threshold", cost_threshold_);

  callback_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NavCogPathPlanner::param_set_callback, this, std::placeholders::_1));

  rclcpp::QoS path_qos(10);
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    path_topic_, path_qos,
    std::bind(&NavCogPathPlanner::pathCallBack, this, std::placeholders::_1));
}

nav_msgs::msg::Path
NavCogPathPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  if (navcog_path_ == nullptr) {
    RCLCPP_INFO(logger_, "navcog path is null");
    return nav_msgs::msg::Path();
  }

  nav_msgs::msg::Path path = normalizedPath(*navcog_path_);

  int i = 0;
  RCLCPP_INFO(logger_, "navcog path planner ---- start");
  for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++, i++) {
    RCLCPP_INFO(logger_, "[%d] (%.2f %.2f) %.2f", i, (*it).pose.position.x, (*it).pose.position.y, distance(*it, *(it + 1)));
  }

  if (path.poses.empty()) {
    return nav_msgs::msg::Path();
  }

  estimatePathWidthAndAdjust(path, costmap_, options_);

  path = adjustedPathByStart(path, start);

  // cabot_planner will take account the collisition
  //
  // RCLCPP_INFO(logger_, "navcog path planner ---- filtering by collision: poses: %ld", path.poses.size());
  // path = adjustedPathByCollision(path, costmap_, cost_threshold_);
  // RCLCPP_INFO(logger_, "navcog path planner ---- filtered by collision: poses: %ld", path.poses.size());

  RCLCPP_INFO(logger_, "navcog path planner ---- end");
  return path;
}

// prepare navcog path by topic
void NavCogPathPlanner::pathCallBack(nav_msgs::msg::Path::SharedPtr path)
{
  navcog_path_ = path;
  RCLCPP_INFO(logger_, "received navcog path");
}

// private functions
rcl_interfaces::msg::SetParametersResult
NavCogPathPlanner::param_set_callback(const std::vector<rclcpp::Parameter> params)
{
  auto node = parent_.lock();

  RCLCPP_INFO(logger_, "NavCogPathPlanner::param_set_callback");
  auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

  for (auto && param : params) {
    RCLCPP_INFO(logger_, "change param %s", param.get_name().c_str());
    if (!node->has_parameter(param.get_name())) {
      continue;
    }

    if (param.get_name() == name_ + ".path_adjusted_center") {
      options_.path_adjusted_center = param.as_double();
    }

    if (param.get_name() == name_ + ".path_adjusted_minimum_path_width_") {
      options_.path_adjusted_minimum_path_width = param.as_double();
    }
    if (param.get_name() == name_ + ".cost_threshold") {
      cost_threshold_ = param.as_int();
    }
  }
  results->successful = true;
  results->reason = "";
  return *results;
}
}  // namespace cabot_navigation2
