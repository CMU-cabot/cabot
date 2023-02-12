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

#include <cmath>
#include <vector>
#include <string>

#include <dwb_core/exceptions.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <dwb_core/trajectory_utils.hpp>
#include <nav_2d_utils/parameters.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dwb_critics
{

class PathForwardPreferCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit()
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    nav2_util::declare_parameter_if_not_declared(
      node,
      dwb_plugin_name_ + "." + name_ + ".num_of_poses",
      rclcpp::ParameterValue(100));

    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".num_of_poses", num_of_poses_);
  }

  void reset()
  {
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &,
    const nav_2d_msgs::msg::Path2D & global_plan)
  {
    global_plan_ = global_plan;
    return true;
  }

  uint64_t nearest_pose_index(const geometry_msgs::msg::Pose2D target)
  {
    double min = std::numeric_limits<double>::max();
    uint64_t mini = global_plan_.poses.size();
    for (uint64_t i = 0; i < global_plan_.poses.size(); i++) {
      auto pose = global_plan_.poses[i];

      double dx = target.x - pose.x;
      double dy = target.y - pose.y;
      double dist = dx * dx + dy * dy;
      if (dist < min) {
        min = dist;
        mini = i;
      }
    }
    return mini;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
  {
    auto first_pose = traj.poses.front();
    auto last_pose = traj.poses.back();

    uint64_t i = nearest_pose_index(first_pose);
    uint64_t j = nearest_pose_index(last_pose);
    int diff = j - i;

    auto node = node_.lock();
    if (diff > num_of_poses_) {
      return 0;
    }
    return fmax(num_of_poses_ - diff, 0) / static_cast<double>(num_of_poses_);
  }

private:
  nav_2d_msgs::msg::Path2D global_plan_;
  int num_of_poses_;
};

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathForwardPreferCritic, dwb_core::TrajectoryCritic)
