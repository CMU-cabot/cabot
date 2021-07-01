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

#include <chrono>
#include <cmath>
#include <algorithm>
#include <angles/angles.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

#include <cabot_navigation2/navcog_path_planner.h>

using nav2_util::declare_parameter_if_not_declared;

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::NavCogPathPlanner, nav2_core::GlobalPlanner)

namespace cabot_navigation2
{
  NavCogPathPlanner::NavCogPathPlanner()
  {
  }

  // nav2_core::GlobalPlanner interface

  NavCogPathPlanner::~NavCogPathPlanner()
  {
  }

  void NavCogPathPlanner::cleanup()
  {
  }
  
  void NavCogPathPlanner::activate()
  {
    path_out_pub_->on_activate();
  }

  void NavCogPathPlanner::deactivate()
  {
    path_out_pub_->on_deactivate();
  }

  void NavCogPathPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    parent_ = parent;
    tf_ = tf;
    name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    
    auto node = parent_.lock();
    clock_ = node->get_clock();
    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring NavCog Path Planner: %s", name_.c_str());

    declare_parameter_if_not_declared(node, name + ".path_width", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".path_width", path_width_);

    declare_parameter_if_not_declared(node, name + ".path_min_width", rclcpp::ParameterValue(0.5));
    node->get_parameter(name + ".path_min_width", path_min_width_);    

    declare_parameter_if_not_declared(node, name + ".path_adjusted_center", rclcpp::ParameterValue(0.0));
    node->get_parameter(name + ".path_adjusted_center", path_adjusted_center_);

    declare_parameter_if_not_declared(node, name + ".path_adjusted_minimum_path_width", rclcpp::ParameterValue(1.0));
    node->get_parameter(name + ".path_adjusted_minimum_path_width", path_adjusted_minimum_path_width_);

    declare_parameter_if_not_declared(node, name + ".path_topic", rclcpp::ParameterValue("/path"));
    node->get_parameter(name + ".path_topic", path_topic_);

    declare_parameter_if_not_declared(node, name + ".path_out_topic", rclcpp::ParameterValue("/path_out"));
    node->get_parameter(name + ".path_out_topic", path_out_topic_);

    declare_parameter_if_not_declared(node, name + ".safe_margin", rclcpp::ParameterValue(0.25));
    node->get_parameter(name + ".safe_margin", safe_margin_);

    declare_parameter_if_not_declared(node, name + ".robot_radius", rclcpp::ParameterValue(0.45));
    node->get_parameter(name + ".robot_radius", robot_radius_);

    callback_handler_ = node->add_on_set_parameters_callback(
        std::bind(&NavCogPathPlanner::param_set_callback, this, std::placeholders::_1));

    rclcpp::QoS path_qos(10);
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&NavCogPathPlanner::pathCallBack, this, std::placeholders::_1));

    rclcpp::QoS path_out_qos(1);
    path_out_qos.transient_local();
    path_out_pub_ = node->create_publisher<nav_msgs::msg::Path>(path_out_topic_, path_out_qos);
  }
  
  nav_msgs::msg::Path
  NavCogPathPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
  {
    if (navcog_path_ == nullptr) {
      return nav_msgs::msg::Path();
    }

    auto path = createPlanWithPath(*(navcog_path_), start, goal);
    //path_out_pub_->publish(path);
    //navcog_path_ = nullptr;
    return path;
  }

  // prepare navcog path by topic 
  void NavCogPathPlanner::pathCallBack(nav_msgs::msg::Path::SharedPtr path)
  {
    navcog_path_ = path;
  }

  // private functions
  rcl_interfaces::msg::SetParametersResult
  NavCogPathPlanner::param_set_callback(const std::vector<rclcpp::Parameter> params)
  {
    auto node = parent_.lock();

    RCLCPP_INFO(logger_, "NavCogPathPlanner::param_set_callback");
    auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

    for (auto &&param : params)
    {
      RCLCPP_INFO(logger_, "change param %s", param.get_name().c_str());
      if (!node->has_parameter(param.get_name()))
      {
        continue;
      }

      if (param.get_name() == name_ + ".path_adjusted_center")
      {
        path_adjusted_center_ = param.as_double();
      }

      if (param.get_name() == name_ + ".path_adjusted_minimum_path_width_")
      {
        path_adjusted_minimum_path_width_ = param.as_double();
      }
    }
    results->successful = true;
    results->reason = "";
    return *results;
  }

  nav_msgs::msg::Path
  NavCogPathPlanner::createPlanWithPath(nav_msgs::msg::Path path,
					const geometry_msgs::msg::PoseStamped & start,
					const geometry_msgs::msg::PoseStamped & goal)
  {
    path = normalizedPath(path);
      
    RCLCPP_INFO(logger_, "path.poses.size() = %lu", path.poses.size());
    RCLCPP_INFO(logger_, "path_.header.stamp.sec = %d", path_.header.stamp.sec);
    RCLCPP_INFO(logger_, "path.header.stamp.sec = %d", path.header.stamp.sec);

    if (path.poses.empty()) {
      nav_msgs::msg::Path result;
      return result;
    }

    estimatePathWidthAndAdjust(path);
    return path;
  }

  nav_msgs::msg::Path NavCogPathPlanner::normalizedPath(const nav_msgs::msg::Path & path)
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
    RCLCPP_INFO(logger_, "normalized path length %lu -> %lu", path.poses.size(), normalized.poses.size());
    return normalized;
  }


  /* 
   *  @brief estimate path witdh for all poses in the path
   */
  std::vector<PathWidth> NavCogPathPlanner::estimatePathWidthAndAdjust(nav_msgs::msg::Path &path)
  {

    std::vector<PathWidth> result;

    double r = costmap_->getResolution();

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
      if (costmap_->worldToMap(wx1, wy1, mx1, my1) &&
          costmap_->worldToMap(wx2, wy2, mx2, my2))
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

      RCLCPP_INFO(logger_, "estimate with left = %.2f, right = %.2f (min %.2f)", estimate.left, estimate.right, path_min_width_);

      RCLCPP_INFO(logger_, "before width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
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

	//auto diff_left = adjusted_left - estimate.left;
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
	  RCLCPP_INFO(logger_, "update goal position");
	}

	RCLCPP_INFO(logger_, "after width.left = %.2f right = %.2f, pos1 (%.2f %.2f) pos2 (%.2f %.2f)",
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
    for (long unsigned int i = 0; i < estimate.size() - 1; i++)
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
  PathWidth NavCogPathPlanner::estimateWidthAt(double x, double y, double yaw)
  {
    double yawl = yaw + M_PI_2;
    double yawr = yaw - M_PI_2;

    double r = costmap_->getResolution();
    unsigned char *master = costmap_->getCharMap();

    PathWidth pw;

    double robot_size = robot_radius_ + safe_margin_;
    double dist = path_width_; // limit by path width
    int N = std::ceil(dist / r);
    double minr = dist;
    // check right side
    for (int i = 0; i <= N; i++)
    {
      unsigned int mx, my;
      double wx, wy;
      wx = x + dist * i / N * cos(yawr);
      wy = y + dist * i / N * sin(yawr);
      costmap_->worldToMap(wx, wy, mx, my);
      unsigned int index = costmap_->getIndex(mx, my);

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
      costmap_->worldToMap(x + dist * i / N * cos(yawl), y + dist * i / N * sin(yawl), mx, my);
      unsigned int index = costmap_->getIndex(mx, my);
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

} // namespace cabot_navigation2
