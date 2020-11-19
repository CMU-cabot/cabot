// Copyright (c) 2020 Carnegie Mellon University
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

#ifndef _NAVCOG_PATH_LAYER_H_
#define _NAVCOG_PATH_LAYER_H_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <deque>

namespace cabot_navigation2
{
  struct PathWidth
  {
    double left;
    double right;
    double length;
  };

  enum PathMode
  {
    EXACT = 0,
    ADJUST
  };

  class NavCogPathLayer : public nav2_costmap_2d::CostmapLayer
  {
  public:
    NavCogPathLayer();

    ~NavCogPathLayer() override;

    void onInitialize() override;

    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double *min_x, double *min_y,
                      double *max_x, double *max_y) override;

    void updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                     int min_i, int min_j, int max_i, int max_j) override;

    void activate() override;

    void deactivate() override;

    void reset() override;

    void pathCallBack(const nav_msgs::msg::Path::SharedPtr path);

  private:
    rcl_interfaces::msg::SetParametersResult
    param_set_callback(const std::vector<rclcpp::Parameter> params);

    nav_msgs::msg::Path normalizePath(nav_msgs::msg::Path &path);

    void updateWithPath(nav_msgs::msg::Path &path);

    void traversePath(nav_msgs::msg::Path &path);

    std::vector<PathWidth> estimatePathWidth(nav_msgs::msg::Path &path);
    void removeOutlier(std::vector<PathWidth> &estimate, nav_msgs::msg::Path &path);
    PathWidth estimateWidthAt(double x, double y, double yaw);

    void drawPath(geometry_msgs::msg::Pose &p1, PathWidth w1,
                  geometry_msgs::msg::Pose &p2, PathWidth w2,
                  geometry_msgs::msg::Pose &p3, PathWidth w3);

    void drawPath(geometry_msgs::msg::Pose &p1,
                  geometry_msgs::msg::Pose &p2,
                  geometry_msgs::msg::Pose &p3);

    void drawPath(double wx1, double wy1, double wx2, double wy2, int cost);

    double max_cost_;
    // 1 = left, 0 = center, -1 = right
    double path_adjusted_center_;
    double path_adjusted_minimum_path_width_;
    double path_min_width_;
    double path_width_;
    PathMode path_mode_;
    bool path_width_detect_;
    bool dirty_;
    bool need_update_;
    std::vector<double> walk_weight_;
    double weight_grid_;
    double robot_radius_;
    double safe_margin_;

    nav_msgs::msg::Path path_;
    std::string path_topic_;
    std::string goal_topic_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

    double previous_path_width_ = path_width_ / 2;
  };
} // namespace cabot_navigation2
#endif
