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

#include <angles/angles.h>
#include <tf2/utils.h>

#include <string>
#include <vector>

#include <dwb_core/exceptions.hpp>
#include <dwb_core/trajectory_utils.hpp>
#include <dwb_critics/base_obstacle.hpp>
#include <nav_2d_utils/parameters.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dwb_critics
{

class VelocityObstacleCritic : public dwb_critics::BaseObstacleCritic
{
public:
  void onInit()
  {
    BaseObstacleCritic::onInit();
    auto node = node_.lock();
    clock_ = node->get_clock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + name_ + ".low_speed_threshold",
      rclcpp::ParameterValue(0.3));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".low_speed_threshold", low_speed_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + name_ + ".low_cost_threshold",
      rclcpp::ParameterValue(16.0));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".low_cost_threshold", low_cost_threshold_);

    nav2_util::declare_parameter_if_not_declared(
      node, dwb_plugin_name_ + "." + name_ + ".cost_threshold",
      rclcpp::ParameterValue(64.0));
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".cost_threshold", cost_threshold_);

    low_speed_threshold_ = std::min(0.9, std::max(0.1, low_speed_threshold_));
    low_cost_threshold_ = std::min(64.0, std::max(1.0, low_cost_threshold_));
    cost_threshold_ = std::min(127.0, std::max(1.0, cost_threshold_));

    factor_ = std::log(low_cost_threshold_ / cost_threshold_) / std::log(1.0 - low_speed_threshold_);
  }

  double func2(double score, double vel)
  {
    return score;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
  {
    double pose_score = scorePose(traj.poses.back());
    if (pose_score < 0) {
      return pose_score;
    }
    double vel = traj.velocity.x;
    if (vel < low_speed_threshold_) {
      return 0;
    }
    if (pose_score < std::pow(1 - vel, factor_) * cost_threshold_) {
      return 0;
    }
    return pose_score;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("VelocityObstacle")};
  rclcpp::Clock::SharedPtr clock_;
  double low_speed_threshold_;
  double low_cost_threshold_;
  double cost_threshold_;
  double factor_;
};

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::VelocityObstacleCritic, dwb_core::TrajectoryCritic)
