// Copyright (c) 2018, 2021  Carnegie Mellon University, IBM Corporation, and others
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
  class CheckCollisionPathPlanner : public nav2_core::GlobalPlanner
  {
  private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("CheckCollisionPathPlanner")};
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string name_;

    int cost_threshold_;
    double length_stride_;

    std::string path_topic_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;

    rcl_interfaces::msg::SetParametersResult param_set_callback(const std::vector<rclcpp::Parameter> params)
    {
      auto node = parent_.lock();

      RCLCPP_INFO(logger_, "CheckCollisionPathPlanner::param_set_callback");
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();

      for (auto &&param : params)
      {
        RCLCPP_INFO(logger_, "change param %s", param.get_name().c_str());
        if (!node->has_parameter(param.get_name()))
        {
          continue;
        }

        if (param.get_name() == name_ + ".cost_threshold")
        {
          cost_threshold_ = param.as_int();
        }
        if (param.get_name() == name_ + ".length_stride")
        {
          length_stride_ = param.as_double();
        }
      }
      results->successful = true;
      results->reason = "";
      return *results;
    };
    bool _has_collision(PoseStamped p){
      unsigned int mx, my;
      costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my);
      int cost = (int)costmap_->getCost(mx,my);//TODO*? need to take some radius to check the maximum of the cost??
      return (cost_threshold_ <= cost && 255 != cost );
    };

  public:
    CheckCollisionPathPlanner(){

    };

    ~CheckCollisionPathPlanner() override{

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

      RCLCPP_INFO(logger_, "Configuring CheckCollision Path Planner: %s", name_.c_str());

      declare_parameter_if_not_declared(node, name + ".cost_threshold", rclcpp::ParameterValue(254));
      node->get_parameter(name + ".cost_threshold", cost_threshold_);

      declare_parameter_if_not_declared(node, name + ".length_stride", rclcpp::ParameterValue(0.0));
      node->get_parameter(name + ".length_stride", length_stride_);

      callback_handler_ = node->add_on_set_parameters_callback(
          std::bind(&CheckCollisionPathPlanner::param_set_callback, this, std::placeholders::_1));
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
      nav_msgs::msg::Path ret;
      double ds = distance(goal, start);
      double rt = ds;
      double stride = (0 < length_stride_ ? length_stride_ : rt);
      PoseStamped test;
      test.header = start.header;
      while(rt > 0){
        test.pose.position.x = (goal.pose.position.x * rt + start.pose.position.x * (ds - rt)) / ds;
        test.pose.position.y = (goal.pose.position.y * rt + start.pose.position.y * (ds - rt)) / ds;
        //test.pose.position.z = (goal.pose.position.z * rt + start.pose.position.z + (ds - rt)) / ds;
        //test.pose.orientation.x = (goal.pose.orientation.x * rt + start.pose.orientation.x * (ds - rt)) / ds;
        //test.pose.orientation.y = (goal.pose.orientation.y * rt + start.pose.orientation.y * (ds - rt)) / ds;
        //test.pose.orientation.z = (goal.pose.orientation.z * rt + start.pose.orientation.z * (ds - rt)) / ds;
        //test.pose.orientation.w = (goal.pose.orientation.w + rt + start.pose.orientation.w * (ds - rt)) / ds;
        if(_has_collision(test)){
          RCLCPP_INFO(logger_, "collision: (%.2f %.2f)", test.pose.position.x, test.pose.position.y);
          return ret;
        };
        rt -= stride;
      }
      if(_has_collision(goal)){
        RCLCPP_INFO(logger_, "collision: (%.2f %.2f)", goal.pose.position.x, goal.pose.position.y);
        return ret;
      }
      RCLCPP_INFO(logger_, "no collision: (%.2f %.2f) - (%.2f %.2f)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
      ret.poses.push_back(start);
      ret.poses.push_back(goal);
      return ret;
    };
  };
} // namespace cabot_navigation2

PLUGINLIB_EXPORT_CLASS(cabot_navigation2::CheckCollisionPathPlanner, nav2_core::GlobalPlanner)
