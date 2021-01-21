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

#include <memory>
#include <string>
#include <cabot_navigation2/cabot_simple_goal_checker.h>
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#include "cabot_navigation2/util.hpp"
#pragma GCC diagnostic pop

/* 
  CabotSimpleGoalChecker
  Check if the robot cross the goal line
 */

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CabotSimpleGoalChecker, nav2_core::GoalChecker)

namespace cabot_navigation2
{

  CabotSimpleGoalChecker::CabotSimpleGoalChecker()
      : xy_goal_tolerance_(0.25),
        xy_goal_tolerance_sq_(0.0625)
  {
  }

  void CabotSimpleGoalChecker::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                          const std::string &plugin_name)
  {
    parent_ = parent;
    auto nh = parent.lock();

    nav2_util::declare_parameter_if_not_declared(nh,
                                                 plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));

    nh->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);

    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

    RCLCPP_INFO(nh->get_logger(), "CabotSimpleGoalChecker, parameter initialze, xy_goal_tolerance = %.2f", xy_goal_tolerance_);

    plugin_name_ = plugin_name;
    param_change_callback_handle_ =
        nh->add_on_set_parameters_callback(std::bind(&CabotSimpleGoalChecker::param_set_callback, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult CabotSimpleGoalChecker::param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (auto param : params)
    {
      if (param.get_name() == this->plugin_name_ + ".xy_goal_tolerance")
      {
        this->xy_goal_tolerance_ = param.as_double();
        this->xy_goal_tolerance_sq_ = this->xy_goal_tolerance_ * this->xy_goal_tolerance_;
        RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "CabotSimpleGoalChecker, parameter updated, xy_goal_tolerance = %.2f", this->xy_goal_tolerance_);
      }
    }
    return result;
  }

  void CabotSimpleGoalChecker::reset()
  {
  }

  bool CabotSimpleGoalChecker::isGoalReached(const geometry_msgs::msg::Pose &query_pose, const geometry_msgs::msg::Pose &goal_pose,
                                             const geometry_msgs::msg::Twist &)
  {

    Safety::Point p1(query_pose.position.x, query_pose.position.y);
    Safety::Point p2(goal_pose.position.x, goal_pose.position.y);
    Safety::Line l1(p1, p2);
    Safety::Line l2(p1, tf2::Quaternion(query_pose.orientation.x, query_pose.orientation.y, query_pose.orientation.z, query_pose.orientation.w));

    double dot = l1.dot(l2);
    double dx = query_pose.position.x - goal_pose.position.x,
           dy = query_pose.position.y - goal_pose.position.y;
    double dist = dx * dx + dy * dy;

    auto nh = parent_.lock();
    RCLCPP_INFO(nh->get_logger(), "dot=%.2f, xy=%.2f", dot, dist);

    if ((dist < 0.5 && l1.dot(l2) < 0) || dist < xy_goal_tolerance_sq_)
    {
      return true;
    }
    return false;
  }

} // namespace cabot_navigation2
