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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include "cabot_navigation2/navcog_path_util.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace cabot_navigation2
{
  class FindCollisionPathPlanner : public nav2_core::GlobalPlanner
  {
  private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("FindCollisionPathPlanner")};
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string name_;

    int cost_lethal_;

    nav_msgs::msg::Path::SharedPtr navcog_path_;

    std::string path_topic_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

    rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params)
    {
      auto node = parent_.lock();

      RCLCPP_INFO(logger_, "FindCollisionPathPlanner::param_set_callback");
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

      for (auto &&param : params)
      {
        RCLCPP_INFO(logger_, "change param %s", param.get_name().c_str());
        if (!node->has_parameter(param.get_name()))
        {
          continue;
        }

        if (param.get_name() == name_ + ".cost_lethal")
        {
          cost_lethal_ = param.as_int();
        }
      }
      results->successful = true;
      results->reason = "";
      return *results;
    };

  public:
    FindCollisionPathPlanner(){

    };

    ~FindCollisionPathPlanner() override{

    };

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
      parent_ = parent;
      name_ = name;
      costmap_ = costmap_ros->getCostmap();

      auto node = parent_.lock();
      clock_ = node->get_clock();
      logger_ = node->get_logger();

      RCLCPP_INFO(logger_, "Configuring FindCollision Path Planner: %s", name_.c_str());

      declare_parameter_if_not_declared(node, name + ".cost_lethal", rclcpp::ParameterValue(254));
      node->get_parameter(name + ".cost_lethal", cost_lethal_);

      callback_handler_ = node->add_on_set_parameters_callback(
          std::bind(&FindCollisionPathPlanner::param_set_callback, this, std::placeholders::_1));

      rclcpp::QoS path_qos(10);
      path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
          path_topic_, path_qos,
          std::bind(&FindCollisionPathPlanner::pathCallBack, this, std::placeholders::_1));
    };

    void cleanup() override
    {

    };
      
    void activate() override
    {

    };

    void deactivate() override
    {

    };

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override
    {
      if (navcog_path_ == nullptr)
      {
        RCLCPP_INFO(logger_, "navcog path is null");
        return nav_msgs::msg::Path();
      }

      nav_msgs::msg::Path path = normalizedPath(*navcog_path_);
      path = adjustedPathByStart(path, start);

      int i = 0;
      for (auto it = path.poses.begin(); it < path.poses.end() - 1; it++, i++)
      {
        RCLCPP_INFO(logger_, "[%d] (%.2f %.2f) %.2f", i, (*it).pose.position.x, (*it).pose.position.y, distance(*it, *(it + 1)));
      }

      if (path.poses.empty())
      {
        return nav_msgs::msg::Path();
      }

      //estimatePathWidthAndAdjust(path, costmap_, options_);TODO put the guts here.
      return path;
    };

    // prepare navcog path by topic 
    void pathCallBack(const nav_msgs::msg::Path::SharedPtr path)
    {
      navcog_path_ = path;
      RCLCPP_INFO(logger_, "received navcog path");
    };
  };
} // namespace cabot_navigation2

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::FindCollisionPathPlanner, nav2_core::GlobalPlanner)
