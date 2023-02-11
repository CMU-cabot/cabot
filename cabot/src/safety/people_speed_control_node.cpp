// Copyright (c) 2020, 2022  Carnegie Mellon University
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

// People speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <limits>
#include <sstream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <people_msgs/msg/people.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "cabot/util.hpp"

namespace CaBotSafety
{
// utility struct and functions
// this could be moved to somewhere else
class PeopleSpeedControlNode : public rclcpp::Node
{
 public:
  std::string people_topic_;
  std::string vis_topic_;
  std::string limit_topic_;
  std::string odom_topic_;
  std::string plan_topic_;
  std::string event_topic_;
  std::string set_social_distance_topic_;
  std::string get_social_distance_topic_;

  std::string map_frame_;
  std::string robot_base_frame_;

  double max_speed_;
  double min_speed_;
  double max_acc_;
  double delay_;
  double social_distance_x_;
  double social_distance_y_;

  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_social_distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr get_social_distance_pub_;
  std::mutex thread_sync_;
  tf2_ros::TransformListener *tfListener;
  tf2_ros::Buffer *tfBuffer;
  nav_msgs::msg::Odometry last_odom_;
  nav_msgs::msg::Path last_plan_;

  PeopleSpeedControlNode(const rclcpp::NodeOptions & options)
      : rclcpp::Node("people_speed_control_node", options),
        people_topic_("/people"),
        vis_topic_("/visualize"),
        limit_topic_("/people_limit"),
        odom_topic_("/odom"),
        plan_topic_("/plan"),
        event_topic_("/event"),
        set_social_distance_topic_("/set_social_distance"),
        get_social_distance_topic_("/get_social_distance"),
        map_frame_("map"),
        robot_base_frame_("base_footprint"),
        max_speed_(1.0),
        min_speed_(0.1),
        max_acc_(0.5),
        delay_(0.5),
        social_distance_x_(2.0),
        social_distance_y_(1.0)
  {
    RCLCPP_INFO(get_logger(), "PeopleSpeedControlNodeClass Constructor");
    tfBuffer = new tf2_ros::Buffer(get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer, this);

    RCLCPP_INFO(get_logger(), "People speed control - %s", __FUNCTION__);

    people_topic_ = declare_parameter("people_topic", people_topic_);
    people_sub_ = create_subscription<people_msgs::msg::People>(people_topic_, 10, std::bind(&PeopleSpeedControlNode::peopleCallback, this, std::placeholders::_1));

    odom_topic_ = declare_parameter("odom_topic", odom_topic_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, std::bind(&PeopleSpeedControlNode::odomCallback, this, std::placeholders::_1));

    plan_topic_= declare_parameter("plan_topic", plan_topic_);
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(plan_topic_, 10, std::bind(&PeopleSpeedControlNode::planCallback, this, std::placeholders::_1));

    vis_topic_= declare_parameter("visualize_topic", vis_topic_);
    vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(vis_topic_, 100);

    limit_topic_ = declare_parameter("limit_topic", limit_topic_);
    limit_pub_ = create_publisher<std_msgs::msg::Float32>(limit_topic_, 100);

    event_topic_ = declare_parameter("event_topic", event_topic_);
    event_pub_ = create_publisher<std_msgs::msg::String>(event_topic_, 100);

    max_speed_ = declare_parameter("max_speed_", max_speed_);
    min_speed_ = declare_parameter("min_speed_", min_speed_);
    max_acc_ = declare_parameter("max_acc_", max_acc_);
    social_distance_x_ = declare_parameter("social_distance_x", social_distance_x_);
    social_distance_y_ = declare_parameter("social_distance_y", social_distance_y_);

    RCLCPP_INFO(get_logger(), "PeopleSpeedControl with max_speed=%.2f, social_distance=(%.2f, %.2f)",
             max_speed_, social_distance_x_, social_distance_y_);

    set_social_distance_topic_ = declare_parameter("set_social_distance_topic", set_social_distance_topic_);
    set_social_distance_sub_ = create_subscription<geometry_msgs::msg::Point>(set_social_distance_topic_, 10,
                                                    std::bind(&PeopleSpeedControlNode::setSocialDistanceCallback, this, std::placeholders::_1));

    get_social_distance_topic_ = declare_parameter("get_social_distance_topic", get_social_distance_topic_);
    rclcpp::QoS qos(100);
    qos.transient_local();
    get_social_distance_pub_ = create_publisher<geometry_msgs::msg::Point>(get_social_distance_topic_, qos);
    geometry_msgs::msg::Point msg;
    msg.x = social_distance_x_;
    msg.y = social_distance_y_;
    get_social_distance_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "PeopleSpeedControl with max_speed=%.2f, social_distance=(%.2f, %.2f)",
                 max_speed_, social_distance_x_, social_distance_y_);
  }

  ~PeopleSpeedControlNode()
  {
    RCLCPP_INFO(get_logger(), "PeopleSpeedControlNodeClass Destructor");
  }

 private:

  void peopleCallback(const people_msgs::msg::People::SharedPtr input)
  {
    if (last_plan_.poses.size() == 0)
    {
      RCLCPP_INFO(get_logger(), "PeopleSpeedControl no plan");
      return;
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    try
    {
      transform_msg = tfBuffer->lookupTransform("base_footprint", "map",
                                               rclcpp::Time(0), rclcpp::Duration(std::chrono::duration<double>(1.0)));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
    tf2::Stamped<tf2::Transform> transform_tf2;
    tf2::fromMsg(transform_msg, transform_tf2);
    auto rotation_tf2 = transform_tf2;
    rotation_tf2.setOrigin(tf2::Vector3(0, 0, 0));

    double speed_limit = max_speed_;

    std::lock_guard<std::mutex> lock(thread_sync_);
    for (auto it = input->people.begin(); it != input->people.end(); it++)
    {
      tf2::Vector3 p_frame(it->position.x, it->position.y, 0);
      tf2::Vector3 v_frame(it->velocity.x, it->velocity.y, 0);

      auto p_local = transform_tf2 * p_frame;
      auto v_local = rotation_tf2 * v_frame;

      double x = p_local.x();
      double y = p_local.y();
      double vx = v_local.x();
      double vy = v_local.y();
      double dist = sqrt(x * x + y * y);

      double pt = atan2(y, x);
      double sdx = abs(social_distance_x_ * cos(pt));
      double sdy = abs(social_distance_y_ * sin(pt));
      double min_path_dist = 100;

      if (abs(pt) > M_PI_2)
      {
        continue;
      }
      auto max_v = [](double D, double A, double d)
                   {
                     return (-2 * A * d + sqrt(4 * A * A * d * d + 8 * A * D)) / 2;
                   };

      for (auto pose : last_plan_.poses)
      {
        tf2::Vector3 p_frame(pose.pose.position.x, pose.pose.position.y, 0);
        auto p_local = transform_tf2 * p_frame;
        auto dx = p_local.x() - x;
        auto dy = p_local.y() - y;
        auto dist = sqrt(dx * dx + dy * dy);
        if (dist < min_path_dist)
        {
          min_path_dist = dist;
        }
      }
      if (min_path_dist > social_distance_y_)
      {
        speed_limit = std::min(speed_limit, max_v(std::max(0.0, dist - social_distance_y_), max_acc_, delay_));
      }
      else
      {
        speed_limit = std::min(speed_limit, max_v(std::max(0.0, dist - social_distance_x_), max_acc_, delay_));
      }

      RCLCPP_INFO(get_logger(), "PeopleSpeedControl people_limit %s dist from path=%.2f x=%.2f y=%.2f vx=%.2f"
               " vy=%.2f pt=%.2f sdx=%.2f sdy=%.2f dist=%.2f limit=%.2f",
               it->name.c_str(), min_path_dist, x, y, vx, vy, pt, sdx, sdy, dist, speed_limit);

      if (speed_limit < max_speed_)
      {
        std_msgs::msg::String msg;

        if (fabs(speed_limit) < 0.01)
        {
          msg.data = "navigation;event;people_speed_stopped";
        }
        else if (vx > 0.25 && speed_limit < max_speed_ * 0.75)
        {
          msg.data = "navigation;event;people_speed_following";
        }

        if (!msg.data.empty())
        {
          RCLCPP_INFO(get_logger(), "PeopleSpeedControl %s", msg.data.c_str());
          event_pub_->publish(msg);
        }
      }
    }

    std_msgs::msg::Float32 msg;
    msg.data = speed_limit;
    // RCLCPP_INFO(get_logger(), "limit = %.2f", speed_limit);
    limit_pub_->publish(msg);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    last_odom_ = *input;
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr input)
  {
    RCLCPP_INFO(get_logger(), "PeopleSpeedControl got plan");
    last_plan_ = *input;
  }

  void setSocialDistanceCallback(const geometry_msgs::msg::Point::SharedPtr input)
  {
    std::lock_guard<std::mutex> lock(thread_sync_);

    social_distance_x_ = input->x;
    social_distance_y_ = input->y;

    geometry_msgs::msg::Point msg;
    msg.x = social_distance_x_;
    msg.y = social_distance_y_;
    get_social_distance_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "PeopleSpeedControl setSocialDistanceCallback social_distance=(%.2f, %.2f)",
                 social_distance_x_, social_distance_y_);
  }
};  // class PeopleSpeedControlNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::PeopleSpeedControlNode)
