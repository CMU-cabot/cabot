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

#ifndef CABOT_NAVIGATION2__CABOT_SIMPLE_GOAL_CHECKER_HPP_
#define CABOT_NAVIGATION2__CABOT_SIMPLE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <nav2_core/goal_checker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*
  This class extends navigation2 SimpleGoalChecker to support dynamically changed xy_goal_tolerance, yaw_goal_tolerance parameters.

  https://github.com/ros-planning/navigation2/blob/master/nav2_controller/include/nav2_controller/plugins/simple_goal_checker.hpp
*/

namespace cabot_navigation2
{
class CabotSimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  CabotSimpleGoalChecker();
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  rcl_interfaces::msg::SetParametersResult
  param_set_callback(const std::vector<rclcpp::Parameter> params);

  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  double xy_goal_tolerance_sq_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  std::string plugin_name_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_change_callback_handle_;
};

}  // namespace cabot_navigation2
#endif  // CABOT_NAVIGATION2__CABOT_SIMPLE_GOAL_CHECKER_HPP_
