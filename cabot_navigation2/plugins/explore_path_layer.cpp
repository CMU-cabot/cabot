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

#include <cabot_navigation2/explore_path_layer.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "angles/angles.h"
#include <nav2_util/node_utils.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::ExplorePathLayer, nav2_costmap_2d::Layer)

using nav2_util::declare_parameter_if_not_declared;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation2
{
  ExplorePathLayer::ExplorePathLayer() : max_cost_(127.0),
                                       need_update_(false),
                                       path_width_(2.0),
                                       weight_grid_(0.25),
                                       path_topic_("/path")
  {
    costmap_ = nullptr;
  }

  ExplorePathLayer::~ExplorePathLayer()
  {
  }

  void ExplorePathLayer::activate()
  {
    enabled_ = true;
  }

  void ExplorePathLayer::deactivate()
  {
    enabled_ = false;
  }

  bool ExplorePathLayer::isClearable()
  {
    return false;
  }

  void ExplorePathLayer::reset()
  {
    deactivate();
    current_ = true;
    need_update_ = true;
    activate();
  }

  void ExplorePathLayer::onInitialize()
  {
    auto node = node_.lock();
    
    matchSize();
    current_ = true;

    declareParameter("path_width", rclcpp::ParameterValue(path_width_));
    declareParameter("path_topic", rclcpp::ParameterValue(path_topic_));
    declareParameter("weight_grid", rclcpp::ParameterValue(weight_grid_));
    declareParameter("max_cost", rclcpp::ParameterValue(max_cost_));

    node->get_parameter(name_ + "." + "path_width", path_width_);
    node->get_parameter(name_ + "." + "path_topic", path_topic_);
    node->get_parameter(name_ + "." + "weight_grid", weight_grid_);
    node->get_parameter(name_ + "." + "max_cost", max_cost_);

    callback_handler_ = node->add_on_set_parameters_callback(
        std::bind(&ExplorePathLayer::param_set_callback, this, std::placeholders::_1));

    rclcpp::QoS path_qos(10);
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&ExplorePathLayer::pathCallBack, this, std::placeholders::_1));

    memset(costmap_, 255, size_x_ * size_y_ * sizeof(unsigned char));

    RCLCPP_INFO(node->get_logger(), "ExplorePathLayer::onInitialize x=%d y=%d", size_x_, size_y_);
  }

  rcl_interfaces::msg::SetParametersResult
  ExplorePathLayer::param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto node = node_.lock();

    RCLCPP_INFO(node->get_logger(), "ExplorePathLayer::param_set_callback");
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

    for (auto &&param : params)
    {
      RCLCPP_INFO(node->get_logger(), "change param %s", param.get_name().c_str());
      if (!node->has_parameter(param.get_name()))
      {
        continue;
      }

      if (param.get_name() == name_ + ".path_width")
      {
        path_width_ = param.as_double();
        need_update_ = true;
      }
    }
    results->successful = true;
    results->reason = "";
    return *results;
  }

  void ExplorePathLayer::updateBounds(double, double, double, double *min_x,
                                     double *min_y, double *max_x, double *max_y)
  {
    using namespace std::chrono;
    milliseconds s = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    if (need_update_)
    {
      for (auto pose : path_.poses)
      {
        auto x = pose.pose.position.x;
        auto y = pose.pose.position.y;
        addExtraBounds(x - path_width_, y - path_width_, x + path_width_, y + path_width_);
      }
    }

    useExtraBounds(min_x, min_y, max_x, max_y);
    milliseconds e = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    //RCLCPP_INFO(node->get_logger(), "ExplorePathLayer::updateBounds %d ms", e - s);
  }

  void ExplorePathLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    using namespace std::chrono;
    milliseconds s = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    if (need_update_)
    {
      need_update_ = false;
      updateWithPath(master_grid, path_);
    }

    // copy buffered data
    unsigned char *master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    for (int j = min_j; j < max_j; j++)
    {
      unsigned int it = span * j + min_i;
      for (int i = min_i; i < max_i; i++)
      {
        if (master[it] < costmap_[it] && costmap_[it] < max_cost_)
        {
          master[it] = costmap_[it];
        }
        it++;
      }
    }
    milliseconds e = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    //RCLCPP_INFO(node->get_logger(), "ExplorePathLayer::updateCosts %d ms", e - s);
  }

  void ExplorePathLayer::pathCallBack(nav_msgs::msg::Path::SharedPtr path)
  {
    path_ = *path;
    need_update_ = true;
  }

  void ExplorePathLayer::updateWithPath(nav2_costmap_2d::Costmap2D &master_grid, nav_msgs::msg::Path &path)
  {
    // auto node = node_.lock();
    // RCLCPP_INFO(logger_, "explore path layer ---- start");
    // RCLCPP_INFO(node->get_logger(), "path.poses.size() = %lu", path.poses.size());
    // RCLCPP_INFO(node->get_logger(), "path_.header.stamp.sec = %d", path_.header.stamp.sec);
    // RCLCPP_INFO(node->get_logger(), "path.header.stamp.sec = %d", path.header.stamp.sec);

    memset(costmap_, 255, size_x_ * size_y_ * sizeof(unsigned char));

    for (long unsigned int i = 0; i < path.poses.size()-1; i++) {
      auto p1 = path.poses[i].pose;
      auto p2 = path.poses[i+1].pose;
      if (isFreePath(master_grid, p1, p2)) {
        drawPath(p1, p2);
        // RCLCPP_INFO(node->get_logger(), "draw explore path");
      }
    }
    // RCLCPP_INFO(logger_, "explore path layer ---- end");
  }

  bool ExplorePathLayer::isFreePath(nav2_costmap_2d::Costmap2D &master_grid, geometry_msgs::msg::Pose &p1, geometry_msgs::msg::Pose &p2)
  {
    // auto node = node_.lock();
    double wx1 = p1.position.x;
    double wy1 = p1.position.y;
    double wx2 = p2.position.x;
    double wy2 = p2.position.y;

    unsigned int mx1, my1;
    unsigned int mx2, my2;
    nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    if (!costmap->worldToMap(wx1, wy1, mx1, my1) || !costmap->worldToMap(wx2, wy2, mx2, my2)) {
      return false;
    }

    double dx = mx2 - mx1;
    double dy = my2 - my2;
    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
    double yaw = atan2(dy, dx);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    unsigned char *charmap = costmap->getCharMap();
    unsigned int charmap_width = costmap->getSizeInCellsX();

    bool is_free_path = true;
    double d = 0.0;
    unsigned int x = mx1;
    unsigned int y = my1;
    while (d<distance) {
      if (charmap[charmap_width * y + x]>0) {
        is_free_path = false;
        // RCLCPP_INFO(node->get_logger(), "found non free cost = %u", charmap[charmap_width * y + x]);
        break;
      }
      x += cos_yaw;
      y += sin_yaw;
      d += 1.0;
    }
    return is_free_path;
  }

  void ExplorePathLayer::drawPath(geometry_msgs::msg::Pose &p1, geometry_msgs::msg::Pose &p2)
  {
    const double half_path_width = path_width_ / 2;
    const double resolution = layered_costmap_->getCostmap()->getResolution();
    double stride_length = 0.5 * resolution;
    const int strides_count = static_cast<int>(ceil(half_path_width / stride_length));
    const int windows = static_cast<int>(ceil(half_path_width / weight_grid_));
    const int strides_per_window = static_cast<int>(ceil(weight_grid_ / stride_length));

    double dx = p2.position.x - p1.position.x;
    double dy = p2.position.y - p1.position.y;
    double yaw = atan2(dy, dx);
    double yaw_left = yaw + M_PI / 2;
    double yaw_right = yaw - M_PI / 2;

    // Draw the center line first
    drawPath(p1.position.x, p1.position.y, p2.position.x, p2.position.y, 0);
    for (auto i = 0; i < strides_count; i++)
    {
      double step_cost_ratio = (1.0 / (windows - 1)) * (i / strides_per_window);
      double curr_length = stride_length * (i + 1) * half_path_width;

      // Left lines
      drawPath(p1.position.x + curr_length * cos(yaw_left),
               p1.position.y + curr_length * sin(yaw_left),
               p2.position.x + curr_length * cos(yaw_left),
               p2.position.y + curr_length * sin(yaw_left),
               max_cost_ * step_cost_ratio);

      // Right lines
      drawPath(p1.position.x + curr_length * cos(yaw_right),
               p1.position.y + curr_length * sin(yaw_right),
               p2.position.x + curr_length * cos(yaw_right),
               p2.position.y + curr_length * sin(yaw_right),
               max_cost_ * step_cost_ratio);
    }
  }

  void ExplorePathLayer::drawPath(double wx1, double wy1, double wx2, double wy2, int cost = 63)
  {
    unsigned int mx1, my1;
    unsigned int mx2, my2;
    addExtraBounds(std::min(wx1, wx2), std::min(wy1, wy2), std::max(wx1, wx2), std::max(wy1, wy2));

    nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    if (costmap->worldToMap(wx1, wy1, mx1, my1) &&
        costmap->worldToMap(wx2, wy2, mx2, my2))
    {

      Costmap2D::MarkCell marker(costmap_, cost);
      raytraceLine(marker, mx1, my1, mx2, my2);
    }
  }

} // namespace cabot_navigation2
