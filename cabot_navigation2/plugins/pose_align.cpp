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

#include <string>
#include <vector>
#include <cmath>
#include "dwb_core/trajectory_critic.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace dwb_critics
{

  class PoseAlignCritic : public dwb_core::TrajectoryCritic
  {
  public:
    void onInit()
    {

      auto node = node_.lock();
      if (!node)
      {
        throw std::runtime_error{"Failed to lock node"};
      }

      nav2_util::declare_parameter_if_not_declared(
          node,
          dwb_plugin_name_ + "." + name_ + ".minimum_angle",
          rclcpp::ParameterValue(45.0));

      node->get_parameter(dwb_plugin_name_ + "." + name_ + ".minimum_angle", minimum_angle_);
      RCLCPP_INFO(node_.lock()->get_logger(), "minimum_angle=%.2f", minimum_angle_);
    }
    void reset()
    {
    }
    bool prepare(
        const geometry_msgs::msg::Pose2D, const nav_2d_msgs::msg::Twist2D,
        const geometry_msgs::msg::Pose2D, const nav_2d_msgs::msg::Path2D &global_plan)
    {

      global_plan_ = global_plan;
      return true;
    }

    double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj)
    {

      auto lastPose = traj.poses[traj.poses.size() - 1];

      double min = 10000;
      unsigned long mini = 0;
      for (unsigned long i = 0; i < global_plan_.poses.size(); i++)
      {
        auto pose = global_plan_.poses[i];
        auto dx = pose.x - lastPose.x;
        auto dy = pose.y - lastPose.y;
        auto dist = sqrt(dx * dx + dy * dy);
        if (dist < min)
        {
          min = dist;
          mini = i;
        }
      }
      double c = 0;
      double s = 0;
      long unsigned int start = mini;
      if (start > global_plan_.poses.size() - 10)
      {
        return 0;
      }
      else if (start < 5)
      {
        start = 0;
      }
      else
      {
        start = start - 5;
      }
      for (unsigned long i = start; i < std::min(global_plan_.poses.size() - 2, mini + 5); i++)
      {
        auto ps1 = global_plan_.poses[i];
        auto ps2 = global_plan_.poses[i + 1];
        auto dx = ps2.x - ps1.x;
        auto dy = ps2.y - ps1.y;
        auto yaw = atan2(dy, dx);
        c += cos(yaw);
        s += sin(yaw);
        //RCLCPP_INFO(node_.lock()->get_logger(), "i=%ld, yaw=%.2f (%.2f, %.2f)", i, yaw, c, s);
      }
      auto yaw = atan2(s, c);
      auto dt = lastPose.theta - yaw;
      if (dt < -M_PI)
        dt += M_PI * 2;
      if (dt > +M_PI)
        dt -= M_PI * 2;

      //RCLCPP_INFO(node_.lock()->get_logger(), "mini=%d, pose.yaw=%.2f path.yaw=%.2f(%.2f,%.2f) dt=%.2f", mini, lastPose.theta, yaw, c, s, dt);

      auto angle = abs(dt) / M_PI * 180;
      if (angle < minimum_angle_)
      {
        return std::max(0.0, 1.0 - traj.velocity.x) / 100.0;
      }
      return angle;
    }

  private:
    nav_2d_msgs::msg::Path2D global_plan_;
    double minimum_angle_;
  };

} // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PoseAlignCritic, dwb_core::TrajectoryCritic)
