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

#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_utils.hpp"
#include "dwb_critics/base_obstacle.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace dwb_critics {

class VelocityObstacleCritic : public dwb_critics::BaseObstacleCritic {
 public:
  void onInit() {
    BaseObstacleCritic::onInit();
    auto node = node_.lock();
    clock_ = node->get_clock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".low_speed_threshold",
                                                 rclcpp::ParameterValue(0.2));

    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".low_speed_threshold", low_speed_threshold_);
  }


  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) {
    double score = 0.0;

    auto vel = traj.velocity.x;
    auto factor = std::max(0.0, vel - low_speed_threshold_) / (1.0 - low_speed_threshold_);

    for (unsigned int i = 0; i < traj.poses.size(); ++i) {
      double pose_score = scorePose(traj.poses[i]);
      // Optimized/branchless version of if (sum_scores_) score += pose_score
      // else score = pose_score;
      if (pose_score < 64) {
        score = static_cast<double>(sum_scores_) * score + pose_score * factor;
      } else {
        score = static_cast<double>(sum_scores_) * score + pose_score;
      }
    }
    return score;
  }

 private:
  rclcpp::Logger logger_{rclcpp::get_logger("VelocityObstacle")};
  rclcpp::Clock::SharedPtr clock_;
  double low_speed_threshold_;
};

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::VelocityObstacleCritic, dwb_core::TrajectoryCritic)
