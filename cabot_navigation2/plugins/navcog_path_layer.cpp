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

#include <cabot_navigation2/navcog_path_layer.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "angles/angles.h"
#include <tf2/utils.h>
#include <chrono>
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::NavCogPathLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace cabot_navigation2
{
  NavCogPathLayer::NavCogPathLayer() : max_cost_(127.0),
                                       path_min_width_(0.5),
                                       path_width_(2.0),
                                       path_mode_(PathMode::EXACT),
                                       path_adjusted_center_(-0.5),
                                       path_adjusted_minimum_path_width_(1.0),
                                       path_width_detect_(false),
                                       path_topic_("/path"),
                                       goal_topic_("/updated_goal"),
                                       dirty_(false),
                                       need_update_(false),
                                       walk_weight_({1.0, 0.8, 0.2}),
                                       weight_grid_(0.25),
                                       robot_radius_(0.45),
                                       safe_margin_(0.25)
  {
    costmap_ = nullptr;
  }

  NavCogPathLayer::~NavCogPathLayer()
  {
  }

  void NavCogPathLayer::activate()
  {
    enabled_ = true;
    goal_pub_->on_activate();
  }

  void NavCogPathLayer::deactivate()
  {
    enabled_ = false;
    goal_pub_->on_deactivate();
  }

  void NavCogPathLayer::reset()
  {
    deactivate();
    current_ = true;
    need_update_ = true;
    activate();
  }

  void NavCogPathLayer::onInitialize()
  {
    matchSize();
    current_ = true;

    declareParameter("path_width", rclcpp::ParameterValue(path_width_));
    declareParameter("path_min_width", rclcpp::ParameterValue(path_min_width_));
    declareParameter("path_mode", rclcpp::ParameterValue(path_mode_));
    declareParameter("path_adjusted_center", rclcpp::ParameterValue(path_adjusted_center_));
    declareParameter("path_adjusted_minimum_path_width", rclcpp::ParameterValue(path_adjusted_minimum_path_width_));
    declareParameter("path_width_detect", rclcpp::ParameterValue(path_width_detect_));
    declareParameter("path_topic", rclcpp::ParameterValue(path_topic_));
    declareParameter("updated_goal_topic", rclcpp::ParameterValue(goal_topic_));
    declareParameter("walk_weight", rclcpp::ParameterValue(walk_weight_));
    declareParameter("weight_grid", rclcpp::ParameterValue(weight_grid_));
    declareParameter("safe_margin", rclcpp::ParameterValue(safe_margin_));
    declareParameter("max_cost", rclcpp::ParameterValue(max_cost_));

    auto node = node_.lock();

    node->get_parameter(name_ + "." + "path_width", path_width_);
    node->get_parameter(name_ + "." + "path_min_width", path_min_width_);
    int pm;
    node->get_parameter(name_ + "." + "path_mode", pm);
    path_mode_ = PathMode(pm);
    node->get_parameter(name_ + "." + "path_adjusted_center", path_adjusted_center_);
    node->get_parameter(name_ + "." + "path_adjusted_minimum_path_width", path_adjusted_minimum_path_width_);
    node->get_parameter(name_ + "." + "path_width_detect", path_width_detect_);
    node->get_parameter(name_ + "." + "path_topic", path_topic_);
    node->get_parameter(name_ + "." + "walk_weight", walk_weight_);
    node->get_parameter(name_ + "." + "weight_grid", weight_grid_);
    // get robot_radius from parent node
    node->get_parameter("robot_radius", robot_radius_);
    node->get_parameter(name_ + "." + "safe_margin", safe_margin_);
    node->get_parameter(name_ + "." + "max_cost", max_cost_);

    callback_handler_ = node->add_on_set_parameters_callback(
        std::bind(&NavCogPathLayer::param_set_callback, this, std::placeholders::_1));

    rclcpp::QoS goal_qos(1);
    goal_qos.transient_local();
    goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, goal_qos);

    rclcpp::QoS path_qos(10);
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&NavCogPathLayer::pathCallBack, this, std::placeholders::_1));

    //dirty_ = true;
    //memset(costmap_, 0, size_x_ * size_y_ * sizeof(char));

    RCLCPP_INFO(node->get_logger(), "NavCogPathLayer::onInitialize x=%d y=%d RR=%.2f", size_x_, size_y_, robot_radius_);
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

      if (param.get_name() == name_ + ".path_mode")
      {
        path_mode_ = PathMode(param.as_int());
        need_update_ = true;
      }

      if (param.get_name() == name_ + ".path_adjusted_center")
      {
        path_adjusted_center_ = param.as_double();
        need_update_ = true;
      }

      if (param.get_name() == name_ + ".path_adjusted_minimum_path_width_")
      {
        path_adjusted_minimum_path_width_ = param.as_double();
        need_update_ = true;
      }
    }
    results->successful = true;
    results->reason = "";
    return *results;
  }

  void NavCogPathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
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
        if (master[it] <= max_cost_)
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

    RCLCPP_INFO(node->get_logger(), "path.poses.size() = %lu", path.poses.size());
    RCLCPP_INFO(node->get_logger(), "path_.header.stamp.sec = %d", path_.header.stamp.sec);
    RCLCPP_INFO(node->get_logger(), "path.header.stamp.sec = %d", path.header.stamp.sec);
    if (path_ != path)
    {
      path_ = normalizePath(path);
    }
    traversePath(path_);
    dirty_ = true;
  }

  nav_msgs::msg::Path NavCogPathLayer::normalizePath(nav_msgs::msg::Path &path)
  {
    nav_msgs::msg::Path normalized;
    normalized.header = path.header;
    auto temp = path.poses.begin();
    for (auto it = temp + 1; it < path.poses.end(); it++)
    {
      auto prev = tf2::getYaw(temp->pose.orientation);
      auto curr = tf2::getYaw(it->pose.orientation);

      if (fabs(angles::shortest_angular_distance(prev, curr)) < M_PI / 10)
      {
        continue;
      }

      normalized.poses.push_back(*temp);
      temp = it;
    }
    if (normalized.poses.size() == 0 || normalized.poses.back() != *temp)
    {
      normalized.poses.push_back(*temp);
    }
    normalized.poses.push_back(path.poses.back());
    auto node = node_.lock();
    RCLCPP_INFO(node->get_logger(), "normalized path length %lu -> %lu", path.poses.size(), normalized.poses.size());
    return normalized;
  }

  void NavCogPathLayer::traversePath(nav_msgs::msg::Path &path)
  {
    if (path.poses.empty())
      return;

    // fill the entire costmap with max cost
    memset(costmap_, max_cost_, size_x_ * size_y_ * sizeof(char));

    if (path_width_detect_)
    {
      auto path_width_array = estimatePathWidth(path);

      auto prev_pose = path.poses[0];
      auto prev_width = path_width_array[0];
      for (int i = 0; i < path.poses.size() - 1; i++)
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
    else
    {
      auto previous = *path.poses.begin();
      for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++)
      {
        auto current = *(it);
        auto next = *(it + 1);
        drawPath(previous.pose, current.pose, next.pose);
        previous = current;
      }
    }
  }

  /* 
   *  @brief estimate path witdh for all poses in the path
   */
  std::vector<PathWidth> NavCogPathLayer::estimatePathWidth(nav_msgs::msg::Path &path)
  {
    auto node = node_.lock();

    std::vector<PathWidth> result;
    nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    double r = costmap->getResolution();

    PathWidth estimate{0, 0, 0};
    for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++)
    {
      auto p1 = it;
      auto p2 = it + 1;

      double wx1 = p1->pose.position.x;
      double wy1 = p1->pose.position.y;
      double wx2 = p2->pose.position.x;
      double wy2 = p2->pose.position.y;
      unsigned int mx1, my1, mx2, my2;

      estimate.left = 1000;
      estimate.right = 1000;
      if (costmap->worldToMap(wx1, wy1, mx1, my1) &&
          costmap->worldToMap(wx2, wy2, mx2, my2))
      {
        double dx = wx2 - wx1;
        double dy = wy2 - wy1;
        double yaw = atan2(dy, dx);
        double dist = sqrt(dx * dx + dy * dy);
        int N = std::ceil(dist / r);
        for (int i = 0; i < N; i++)
        {
          double x = wx1 + dx * i / N;
          double y = wy1 + dy * i / N;
          auto pw = estimateWidthAt(x, y, yaw);
          if (pw.left < estimate.left)
          {
            estimate.left = std::min(pw.left, std::max(path_min_width_, dist / 5.0));
          }
          if (pw.right < estimate.right)
          {
            estimate.right = std::min(pw.right, std::max(path_min_width_, dist / 5.0));
          }
        }
      }

      RCLCPP_INFO(node->get_logger(), "estimate with left = %.2f, right = %.2f (min %.2f)", estimate.left, estimate.right, path_min_width_);

      if (path_mode_ == PathMode::EXACT)
      {
        RCLCPP_INFO(node->get_logger(), "before width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
                    estimate.left, estimate.right, p1->pose.position.x, p1->pose.position.y, p2->pose.position.x, p2->pose.position.y);

        if (estimate.left + estimate.right > path_adjusted_minimum_path_width_)
        {
          auto adjusted_left = estimate.left;
          auto adjusted_right = estimate.right;

          if (path_adjusted_center_ > 0)
          {
            adjusted_left = estimate.left * (1 - path_adjusted_center_);
            adjusted_right = estimate.right + estimate.left * path_adjusted_center_;
          }
          else
          {
            adjusted_right = estimate.right * (1 + path_adjusted_center_);
            adjusted_left = estimate.left - estimate.right * path_adjusted_center_;
          }

          double curr_yaw = tf2::getYaw(it->pose.orientation) + M_PI_2;

          auto diff_left = adjusted_left - estimate.left;
          auto diff_right = adjusted_right - estimate.right;

          estimate.left = adjusted_left;
          estimate.right = adjusted_right;

          auto curr_diff = diff_right;

          p1->pose.position.x += curr_diff * cos(curr_yaw);
          p1->pose.position.y += curr_diff * sin(curr_yaw);
          p2->pose.position.x += curr_diff * cos(curr_yaw);
          p2->pose.position.y += curr_diff * sin(curr_yaw);

          // if last pose (goal) is updated, publish as updated goal
          if (p2 == path.poses.end() - 1)
          {
            goal_pub_->publish(*p2);
            RCLCPP_INFO(node->get_logger(), "update goal position");
          }

          RCLCPP_INFO(node->get_logger(), "after width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
                      estimate.left, estimate.right, p1->pose.position.x, p1->pose.position.y, p2->pose.position.x, p2->pose.position.y);
        }
      }
      if (path_mode_ == PathMode::ADJUST)
      {
        RCLCPP_INFO(node->get_logger(), "before width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
                    estimate.left, estimate.right, p1->pose.position.x, p1->pose.position.y, p2->pose.position.x, p2->pose.position.y);
        auto curr_total_width = estimate.left + estimate.right;
        auto adjusted_left = (curr_total_width * (1 - path_adjusted_center_)) / 2.0;
        auto adjusted_right = (curr_total_width * (1 + path_adjusted_center_)) / 2.0;

        double curr_yaw = tf2::getYaw(it->pose.orientation) + M_PI_2;

        auto diff_left = adjusted_left - estimate.left;
        auto diff_right = adjusted_right - estimate.right;

        estimate.left = adjusted_left;
        estimate.right = adjusted_right;

        auto curr_diff = diff_right;

        p1->pose.position.x += curr_diff * cos(curr_yaw);
        p1->pose.position.y += curr_diff * sin(curr_yaw);
        p2->pose.position.x += curr_diff * cos(curr_yaw);
        p2->pose.position.y += curr_diff * sin(curr_yaw);

        RCLCPP_INFO(node->get_logger(), "after width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
                    estimate.left, estimate.right, p1->pose.position.x, p1->pose.position.y, p2->pose.position.x, p2->pose.position.y);
      }

      estimate.left = std::max(estimate.left, path_min_width_);
      estimate.right = std::max(estimate.right, path_min_width_);

      estimate.length = sqrt(pow(p2->pose.position.x - p1->pose.position.x, 2) +
                             pow(p2->pose.position.y - p1->pose.position.y, 2));

      result.push_back(estimate);
    }
    // dummy one for the last link
    result.push_back(estimate);

    return result;
  }

  void removeOutlier(std::vector<PathWidth> &estimate)
  {
    PathWidth prev{1.0, 1.0, 1};
    double max_rate = 0.25;
    for (int i = 0; i < estimate.size() - 1; i++)
    {
      auto current = estimate[i];
      auto next = estimate[i + 1];
      auto left_rate = (std::abs(current.left * 2 - prev.left - next.left)) / current.length;
      if (max_rate < left_rate)
      {
        if (current.left * 2 < prev.left + next.left)
        {
          current.left = (prev.left + next.right - max_rate * current.length) / 2;
        }
        else
        {
          current.left = (prev.left + next.right + max_rate * current.length) / 2;
        }
      }
      auto right_rate = (std::abs(current.right * 2 - prev.right - next.right)) / current.length;
      if (max_rate < right_rate)
      {
        if (current.right * 2 < prev.right + next.right)
        {
          current.right = (prev.right + next.right - max_rate * current.length) / 2;
        }
        else
        {
          current.right = (prev.right + next.right + max_rate * current.length) / 2;
        }
      }
    }
  }

  /*
   * @brief estimate path width at (x, y) facing to (yaw) direction. 
   *        raytrace to right and left while it hits to the wall (254)
   */
  PathWidth NavCogPathLayer::estimateWidthAt(double x, double y, double yaw)
  {
    double yawl = yaw + M_PI_2;
    double yawr = yaw - M_PI_2;

    nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    double r = costmap->getResolution();
    unsigned char *master = costmap->getCharMap();

    PathWidth pw;

    double robot_size = robot_radius_ + safe_margin_;
    double dist = path_width_; // limit by path width
    int N = std::ceil(dist / r);
    double minr = dist;
    // check right side
    for (int i = 0; i <= N; i++)
    {
      unsigned int mx, my;
      costmap->worldToMap(x + dist * i / N * cos(yawr), y + dist * i / N * sin(yawr), mx, my);
      unsigned int index = getIndex(mx, my);
      if (master[index] == 254)
      {
        minr = dist * i / N;
        break;
      }
    }
    pw.right = std::max(0.0, std::min(minr, minr - robot_size));

    // check left side
    double minl = dist;
    for (int i = 0; i <= N; i++)
    {
      unsigned int mx, my;
      costmap->worldToMap(x + dist * i / N * cos(yawl), y + dist * i / N * sin(yawl), mx, my);
      unsigned int index = getIndex(mx, my);
      if (master[index] == 254)
      {
        minl = dist * i / N;
        break;
      }
    }
    pw.left = std::max(0.0, std::min(minl, minl - robot_size));

    return pw;
  }

  double normalized_diff(double a, double b)
  {
    double c = a - b;
    if (M_PI < c)
      c -= 2 * M_PI;
    if (c < -M_PI)
      c += 2 * M_PI;
    return c;
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
    const double half_path_width = path_width_ / 2;
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
