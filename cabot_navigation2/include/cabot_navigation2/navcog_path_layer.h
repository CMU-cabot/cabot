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

#ifndef _NAVCOG_PATH_LAYER_H_
#define _NAVCOG_PATH_LAYER_H_

#include <rclcpp/rclcpp.hpp>
#include "cabot_navigation2/navcog_path_util.hpp"

namespace cabot_navigation2
{
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

    bool isClearable() override;

    void pathCallBack(const nav_msgs::msg::Path::SharedPtr path);

  private:
    rcl_interfaces::msg::SetParametersResult
    param_set_callback(const std::vector<rclcpp::Parameter> params);

    void updateWithPath(nav_msgs::msg::Path &path);

    void traversePath(nav_msgs::msg::Path &path);

    void drawPath(geometry_msgs::msg::Pose &p1, PathWidth w1,
                  geometry_msgs::msg::Pose &p2, PathWidth w2,
                  geometry_msgs::msg::Pose &p3, PathWidth w3);

    void drawPath(geometry_msgs::msg::Pose &p1,
                  geometry_msgs::msg::Pose &p2,
                  geometry_msgs::msg::Pose &p3);

    void drawPath(double wx1, double wy1, double wx2, double wy2, int cost);

    PathEstimateOptions options_;

    double max_cost_;
    bool dirty_;
    bool need_update_;
    std::vector<double> walk_weight_;
    double weight_grid_;
    std::string path_topic_;

    nav_msgs::msg::Path path_;
    nav_msgs::msg::Path::SharedPtr navcog_path_;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  };
} // namespace cabot_navigation2
#endif
