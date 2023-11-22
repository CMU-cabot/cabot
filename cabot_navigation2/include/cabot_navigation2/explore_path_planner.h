// Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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

#ifndef _EXPLORE_PATH_PLANNER_H_
#define _EXPLORE_PATH_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>

namespace cabot_navigation2
{
  class ExplorePathPlanner : public nav2_core::GlobalPlanner
  {
  public:
    ExplorePathPlanner();

    ~ExplorePathPlanner() override;

    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
      
    void activate() override;

    void deactivate() override;

    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;

  private:
    rcl_interfaces::msg::SetParametersResult
    param_set_callback(const std::vector<rclcpp::Parameter> params);

    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("ExplorePathPlanner")};
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string name_;

    nav_msgs::msg::Path::SharedPtr explore_path_;

    double interval_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  };
} // namespace cabot_navigation2
#endif
