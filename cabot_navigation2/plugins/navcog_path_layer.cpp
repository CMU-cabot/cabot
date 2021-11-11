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

#include <cabot_navigation2/navcog_path_layer.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "angles/angles.h"
#include <nav2_util/node_utils.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::NavCogPathLayer, nav2_costmap_2d::Layer)

using nav2_util::declare_parameter_if_not_declared;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation2
{
  NavCogPathLayer::NavCogPathLayer() : max_cost_(127.0),
                                       dirty_(false),
                                       need_update_(false),
                                       walk_weight_({1.0, 0.8, 0.2}),
                                       weight_grid_(0.25),
                                       path_topic_("/path")
  {
    costmap_ = nullptr;
  }

  NavCogPathLayer::~NavCogPathLayer()
  {
  }

  void NavCogPathLayer::activate()
  {
    enabled_ = true;
  }

  void NavCogPathLayer::deactivate()
  {
    enabled_ = false;
  }

  bool NavCogPathLayer::isClearable()
  {
    return false;
  }

  void NavCogPathLayer::reset()
  {
    deactivate();
    current_ = true;
    //need_update_ = true;
    activate();
  }

  void NavCogPathLayer::onInitialize()
  {
    auto node = node_.lock();
    
    matchSize();
    current_ = true;

    declare_parameter_if_not_declared(node, name_ + ".path_width", rclcpp::ParameterValue(2.0));
    node->get_parameter(name_ + ".path_width", options_.path_width);

    declare_parameter_if_not_declared(node, name_ + ".path_min_width", rclcpp::ParameterValue(0.5));
    node->get_parameter(name_ + ".path_min_width", options_.path_min_width);    

    declare_parameter_if_not_declared(node, name_ + ".path_adjusted_center", rclcpp::ParameterValue(0.0));
    node->get_parameter(name_ + ".path_adjusted_center", options_.path_adjusted_center);

    declare_parameter_if_not_declared(node, name_ + ".path_adjusted_minimum_path_width", rclcpp::ParameterValue(1.0));
    node->get_parameter(name_ + ".path_adjusted_minimum_path_width", options_.path_adjusted_minimum_path_width);

    declare_parameter_if_not_declared(node, name_ + ".safe_margin", rclcpp::ParameterValue(0.25));
    node->get_parameter(name_ + ".safe_margin", options_.safe_margin);

    // get robot_radius from parent node
    node->get_parameter("robot_radius", options_.robot_radius);

    
    declareParameter("path_topic", rclcpp::ParameterValue(path_topic_));
    declareParameter("walk_weight", rclcpp::ParameterValue(walk_weight_));
    declareParameter("weight_grid", rclcpp::ParameterValue(weight_grid_));
    declareParameter("max_cost", rclcpp::ParameterValue(max_cost_));


    node->get_parameter(name_ + "." + "path_topic", path_topic_);
    node->get_parameter(name_ + "." + "walk_weight", walk_weight_);
    node->get_parameter(name_ + "." + "weight_grid", weight_grid_);
    node->get_parameter(name_ + "." + "max_cost", max_cost_);

    callback_handler_ = node->add_on_set_parameters_callback(
        std::bind(&NavCogPathLayer::param_set_callback, this, std::placeholders::_1));

    rclcpp::QoS path_qos(10);
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&NavCogPathLayer::pathCallBack, this, std::placeholders::_1));

    dirty_ = true;
    memset(costmap_, 255, size_x_ * size_y_ * sizeof(char));

    RCLCPP_INFO(node->get_logger(), "NavCogPathLayer::onInitialize x=%d y=%d RR=%.2f", size_x_, size_y_, options_.robot_radius);
  }

  rcl_interfaces::msg::SetParametersResult
  NavCogPathLayer::param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto node = node_.lock();

    RCLCPP_INFO(node->get_logger(), "NavCogPathLayer::param_set_callback");
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

    for (auto &&param : params)
    {
      RCLCPP_INFO(node->get_logger(), "change param %s", param.get_name().c_str());
      if (!node->has_parameter(param.get_name()))
      {
        continue;
      }

      if (param.get_name() == name_ + ".walk_weight")
      {
        walk_weight_ = param.as_double_array();
        need_update_ = true;
      }

      if (param.get_name() == name_ + ".path_adjusted_center")
      {
        options_.path_adjusted_center = param.as_double();
        need_update_ = true;
      }

      if (param.get_name() == name_ + ".path_adjusted_minimum_path_width_")
      {
        options_.path_adjusted_minimum_path_width = param.as_double();
        need_update_ = true;
      }
    }
    results->successful = true;
    results->reason = "";
    return *results;
  }

  void NavCogPathLayer::updateBounds(double, double, double, double *min_x,
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
        addExtraBounds(x - options_.path_width, y - options_.path_width, x + options_.path_width, y + options_.path_width);
      }
    }

    useExtraBounds(min_x, min_y, max_x, max_y);
    milliseconds e = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    //RCLCPP_INFO(node->get_logger(), "NavCogPathLayer::updateBounds %d ms", e - s);
  }

  void NavCogPathLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    using namespace std::chrono;
    milliseconds s = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    if (need_update_)
    {
      need_update_ = false;
      updateWithPath(path_);
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
    //RCLCPP_INFO(node->get_logger(), "NavCogPathLayer::updateCosts %d ms", e - s);
  }

  void NavCogPathLayer::pathCallBack(nav_msgs::msg::Path::SharedPtr path)
  {
    updateWithPath(*path);
  }

  void NavCogPathLayer::updateWithPath(nav_msgs::msg::Path &path)
  {
    auto node = node_.lock();

    RCLCPP_INFO(logger_, "navcog path layer ---- start");
    RCLCPP_INFO(node->get_logger(), "path.poses.size() = %lu", path.poses.size());
    RCLCPP_INFO(node->get_logger(), "path_.header.stamp.sec = %d", path_.header.stamp.sec);
    RCLCPP_INFO(node->get_logger(), "path.header.stamp.sec = %d", path.header.stamp.sec);
    if (path_ != path)
    {
      path_ = normalizedPath(path);
    }
    traversePath(path_);
    dirty_ = true;
    RCLCPP_INFO(logger_, "navcog path layer ---- end");
  }

  void NavCogPathLayer::traversePath(nav_msgs::msg::Path &path)
  {
    if (path.poses.empty())
      return;

    // fill the entire costmap with no information
    memset(costmap_, 255, size_x_ * size_y_ * sizeof(char));

    auto path_width_array = estimatePathWidthAndAdjust(path, layered_costmap_->getCostmap(), options_);
    
    auto prev_pose = path.poses[0];
    auto prev_width = path_width_array[0];
    for (long unsigned int i = 0; i < path.poses.size() - 1; i++)
    {
      auto curr_pose = path.poses[i];
      auto curr_width = path_width_array[i];
      auto next_pose = path.poses[i + 1];
      auto next_width = path_width_array[i + 1];
      drawPath(prev_pose.pose, prev_width,
	       curr_pose.pose, curr_width,
	       next_pose.pose, next_width);
      
      prev_pose = curr_pose;
      prev_width = curr_width;
    }
  }

  /* @brief draw path with custom width
   */
  void NavCogPathLayer::drawPath(geometry_msgs::msg::Pose &prev, PathWidth prev_width,
                                 geometry_msgs::msg::Pose &curr, PathWidth curr_width,
                                 geometry_msgs::msg::Pose &next, PathWidth next_width)
  {
    double cost_left = walk_weight_[0];
    double cost_center = walk_weight_[1];
    double cost_right = walk_weight_[2];

    double prev_yaw = tf2::getYaw(prev.orientation);
    double curr_yaw = tf2::getYaw(curr.orientation);
    double next_yaw = tf2::getYaw(next.orientation);
    double curr_diff = normalized_diff(curr_yaw, prev_yaw);
    double next_diff = normalized_diff(next_yaw, curr_yaw);

    // if path is turing, connect differently
    // huristic: inverse of tangent gonna be quiet larger if angle dirrential is small
    double MIN_DIFF = M_PI / 6.0;
    double MAX_DIFF = M_PI * 5.0 / 6.0;
    bool curr_turn = (MIN_DIFF < std::abs(curr_diff) && std::abs(curr_diff) < MAX_DIFF);
    bool next_turn = (MIN_DIFF < std::abs(next_diff) && std::abs(next_diff) < MAX_DIFF);

    double curr_bisector = (M_PI - normalized_diff(curr_yaw, prev_yaw)) / 2;
    double next_bisector = (M_PI - normalized_diff(next_yaw, curr_yaw)) / 2;

    // use bisector if it is almost straight
    double curr_yaw_left = curr_yaw + curr_bisector;
    double curr_yaw_right = curr_yaw + curr_bisector + M_PI;
    double curr_setback_left = 0;
    double curr_setback_right = 0;
    if (curr_turn)
    {
      // use setback if it is turning
      curr_setback_left = prev_width.left / sin(curr_diff) - curr_width.left / tan(curr_diff);
      curr_setback_right = -prev_width.right / sin(curr_diff) + curr_width.right / tan(curr_diff);
      curr_yaw_left = curr_yaw + M_PI / 2;
      curr_yaw_right = curr_yaw - M_PI / 2;
    }

    double next_yaw_left = next_yaw + next_bisector;
    double next_yaw_right = next_yaw + next_bisector + M_PI;
    double next_setback_left = 0;
    double next_setback_right = 0;
    if (next_turn)
    {
      next_setback_left = curr_width.left / sin(next_diff) - next_width.left / tan(next_diff);
      next_setback_right = -curr_width.right / sin(next_diff) + next_width.right / tan(next_diff);
      next_yaw_left = next_yaw + M_PI / 2;
      next_yaw_right = next_yaw - M_PI / 2;
    }

    // calculate start and end points
    double ccx = curr.position.x;
    double ccy = curr.position.y;
    double ncx = next.position.x;
    double ncy = next.position.y;

    double clx = ccx + curr_setback_left * cos(curr_yaw) + curr_width.left * cos(curr_yaw_left);
    double cly = ccy + curr_setback_left * sin(curr_yaw) + curr_width.left * sin(curr_yaw_left);
    double crx = ccx + curr_setback_right * cos(curr_yaw) + curr_width.right * cos(curr_yaw_right);
    double cry = ccy + curr_setback_right * sin(curr_yaw) + curr_width.right * sin(curr_yaw_right);

    double nlx = ncx + next_setback_left * cos(next_yaw) + next_width.left * cos(next_yaw_left);
    double nly = ncy + next_setback_left * sin(next_yaw) + next_width.left * sin(next_yaw_left);
    double nrx = ncx + next_setback_right * cos(next_yaw) + next_width.right * cos(next_yaw_right);
    double nry = ncy + next_setback_right * sin(next_yaw) + next_width.right * sin(next_yaw_right);

    // draw center
    drawPath(ccx, ccy, ncx, ncy, max_cost_ * cost_center);

    const double resolution = layered_costmap_->getCostmap()->getResolution() / 3.0;

    double exponent = 1;

    // Left lines
    double dclx = ccx - clx;
    double dcly = ccy - cly;
    double distcl = sqrt(dclx * dclx + dcly * dcly);
    double dnlx = ncx - nlx;
    double dnly = ncy - nly;
    double distnl = sqrt(dnlx * dnlx + dnly * dnly);
    double NL = ceil(std::max(distcl, distnl) / resolution);
    for (int i = 1; i <= NL; i++)
    {
      double sx = (clx * i + ccx * (NL - i)) / NL;
      double sy = (cly * i + ccy * (NL - i)) / NL;
      double ex = (nlx * i + ncx * (NL - i)) / NL;
      double ey = (nly * i + ncy * (NL - i)) / NL;
      int cost = std::max(0.0, max_cost_ * (cost_left * pow(i, exponent) + cost_center * pow((NL - i), exponent)) / pow(NL, exponent));
      drawPath(sx, sy, ex, ey, cost);
    }

    // Right lines
    double dcrx = ccx - crx;
    double dcry = ccy - cry;
    double distcr = sqrt(dcrx * dcrx + dcry * dcry);
    double dnrx = ncx - nrx;
    double dnry = ncy - nry;
    double distnr = sqrt(dnrx * dnrx + dnry * dnry);
    double NR = ceil(std::max(distcr, distnr) / resolution);
    for (int i = 1; i <= NR; i++)
    {
      double sx = (crx * i + ccx * (NR - i)) / NR;
      double sy = (cry * i + ccy * (NR - i)) / NR;
      double ex = (nrx * i + ncx * (NR - i)) / NR;
      double ey = (nry * i + ncy * (NR - i)) / NR;
      int cost = std::max(0.0, max_cost_ * (cost_right * pow(i, exponent) + cost_center * pow((NR - i), exponent)) / pow(NR, exponent));
      drawPath(sx, sy, ex, ey, cost);
    }

    /* debug
      ROS_INFO("curr_turn %d next_turn %d", curr_turn, next_turn);
      ROS_INFO("curr_diff %.2f next_diff %.2f", curr_diff, next_diff);
      ROS_INFO("width (%.2f, %.2f) (%.2f, %.2f) (%.2f, %.2f)",
      prev_width.left, prev_width.right,
	       curr_width.left, curr_width.right,
	       next_width.left, next_width.right);
	       ROS_INFO("curr_setback_right %.2f next_setback_right %.2f", curr_setback_right, next_setback_right);
      ROS_INFO("crx %.2f cry %.2f nrx %.2f nry %.2f", crx, cry, nrx, nry);
      ROS_INFO("");
    */
  }

  /* @brief draw path with fixed width
   */
  void NavCogPathLayer::drawPath(geometry_msgs::msg::Pose &prev,
                                 geometry_msgs::msg::Pose &curr,
                                 geometry_msgs::msg::Pose &next)
  {
    const double half_path_width = options_.path_width / 2;
    const double resolution = layered_costmap_->getCostmap()->getResolution();
    double stride_length = 0.5 * resolution;
    const int strides_count = static_cast<int>(ceil(half_path_width / stride_length));
    const int windows = static_cast<int>(ceil(half_path_width / weight_grid_));
    const int strides_per_window = static_cast<int>(ceil(weight_grid_ / stride_length));
    double left = walk_weight_[0];
    double center = walk_weight_[1];
    double right = walk_weight_[2];

    auto prev_yaw = tf2::getYaw(prev.orientation);
    auto curr_yaw = tf2::getYaw(curr.orientation);
    auto next_yaw = tf2::getYaw(next.orientation);

    auto curr_bisector = (M_PI - (curr_yaw - prev_yaw)) / 2;
    auto next_bisector = (M_PI - (next_yaw - curr_yaw)) / 2;

    auto curr_yaw_left = curr_yaw + curr_bisector;
    auto curr_yaw_right = curr_yaw + curr_bisector + M_PI;
    auto curr_width = half_path_width / std::sin(curr_bisector);

    auto next_yaw_left = next_yaw + next_bisector;
    auto next_yaw_right = next_yaw + next_bisector + M_PI;
    auto next_width = half_path_width / std::sin(next_bisector);

    // Draw the center line first
    drawPath(curr.position.x, curr.position.y, next.position.x, next.position.y, max_cost_ * center);
    for (auto i = 0; i < strides_count; i++)
    {
      double step_cost_ratio = (1.0 / (windows - 1)) * (i / strides_per_window);
      double curr_length = stride_length * (i + 1) * curr_width;
      double next_length = stride_length * (i + 1) * next_width;

      // Left lines
      drawPath(curr.position.x + curr_length * cos(curr_yaw_left),
               curr.position.y + curr_length * sin(curr_yaw_left),
               next.position.x + next_length * cos(next_yaw_left),
               next.position.y + next_length * sin(next_yaw_left),
               max_cost_ * (center * (1.0 - step_cost_ratio) + left * step_cost_ratio));

      // Right lines
      drawPath(curr.position.x + curr_length * cos(curr_yaw_right),
               curr.position.y + curr_length * sin(curr_yaw_right),
               next.position.x + next_length * cos(next_yaw_right),
               next.position.y + next_length * sin(next_yaw_right),
               max_cost_ * (center * (1.0 - step_cost_ratio) + right * step_cost_ratio));
    }
  }

  void NavCogPathLayer::drawPath(double wx1, double wy1, double wx2, double wy2, int cost = 63)
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
