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

#ifndef _NAVCOG_PATH_PLANNER_H_
#define _NAVCOG_PATH_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <deque>

namespace cabot_navigation2
{
  struct PathWidth
  {
    double left;
    double right;
    double length;
  };

  class NavCogPathPlanner : public nav2_core::GlobalPlanner
  {
  public:
    NavCogPathPlanner();

    ~NavCogPathPlanner() override;

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
      
    void pathCallBack(const nav_msgs::msg::Path::SharedPtr path);
    
  private:
    rcl_interfaces::msg::SetParametersResult
    param_set_callback(const std::vector<rclcpp::Parameter> params);

    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("NavCogPathPlanner")};
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string global_frame_, name_;

    nav_msgs::msg::Path::SharedPtr navcog_path_;

    nav_msgs::msg::Path
    createPlanWithPath(nav_msgs::msg::Path path,
		       const geometry_msgs::msg::PoseStamped & start,
		       const geometry_msgs::msg::PoseStamped & goal);
    

    nav_msgs::msg::Path normalizedPath(const nav_msgs::msg::Path & path);

    void updateWithPath(nav_msgs::msg::Path &path);

    void traversePath(nav_msgs::msg::Path &path);

    std::vector<PathWidth> estimatePathWidthAndAdjust(nav_msgs::msg::Path &path);
    void removeOutlier(std::vector<PathWidth> &estimate, nav_msgs::msg::Path &path);
    PathWidth estimateWidthAt(double x, double y, double yaw);

    double max_cost_;
    // 1 = left, 0 = center, -1 = right
    double path_adjusted_center_;
    double path_adjusted_minimum_path_width_;
    double path_min_width_;
    double path_width_;
    double robot_radius_;
    double safe_margin_;

    nav_msgs::msg::Path path_;
    std::string path_topic_, path_out_topic_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> path_out_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

    double previous_path_width_ = path_width_ / 2;
  };
} // namespace cabot_navigation2
#endif
