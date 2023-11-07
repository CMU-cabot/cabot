// Copyright (c) 2023  Carnegie Mellon University
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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cabot_msgs/msg/stop_reason.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

#include "stop_reasoner.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cabot_ui;

#define ODOM_TOPIC "/cabot/odom_raw"
#define EVENT_TOPIC "/cabot/event"
#define CMD_VEL_TOPIC "/cmd_vel"
#define PEOPLE_SPEED_TOPIC "/cabot/people_speed"
#define TF_SPEED_TOPIC "/cabot/tf_speed"
#define TOUCH_SPEED_TOPIC "/cabot/touch_speed_switched"
#define RECEIVED_GLOBAL_PLAN "/received_global_plan"
#define LOCAL_PREFIX "/local"
#define REPLAN_REASON_TOPIC "/replan_reason"
#define CURRENT_FRAME_TOPIC "/current_frame"

namespace CaBotUI
{
class StopReasonsNode : public rclcpp::Node
{
public:
  explicit StopReasonsNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("stop_reasoner_node", options),
    reasoner_(nullptr),
    stop_reason_filter_(nullptr),
    prev_code_(StopReason::NONE)
  {
    stop_reason_pub_ = this->create_publisher<cabot_msgs::msg::StopReason>("/stop_reason", 10);
    event_pub_ = this->create_publisher<std_msgs::msg::String>("/cabot/event", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10, std::bind(&StopReasonsNode::odom_callback, this, _1));
    event_sub_ = this->create_subscription<std_msgs::msg::String>(EVENT_TOPIC, 10, std::bind(&StopReasonsNode::event_callback, this, _1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, 10, std::bind(&StopReasonsNode::cmd_vel_callback, this, _1));
    people_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(PEOPLE_SPEED_TOPIC, 10, std::bind(&StopReasonsNode::people_speed_callback, this, _1));
    touch_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(TOUCH_SPEED_TOPIC, 10, std::bind(&StopReasonsNode::touch_speed_callback, this, _1));
    global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(RECEIVED_GLOBAL_PLAN, 10, std::bind(&StopReasonsNode::global_plan_callback, this, _1));
    local_global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(LOCAL_PREFIX RECEIVED_GLOBAL_PLAN, 10, std::bind(&StopReasonsNode::global_plan_callback, this, _1));
    replan_reason_sub_ = this->create_subscription<people_msgs::msg::Person>(REPLAN_REASON_TOPIC, 10, std::bind(&StopReasonsNode::replan_reason_callback, this, _1));
    current_frame_sub_ = this->create_subscription<std_msgs::msg::String>(CURRENT_FRAME_TOPIC, 10, std::bind(&StopReasonsNode::current_frame_callback, this, _1));
    timer_ = this->create_wall_timer(0.1s, std::bind(&StopReasonsNode::timer_callback, this));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    reasoner_->input_odom(*msg);
  }

  void event_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    reasoner_->input_event(*msg);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    reasoner_->input_cmd_vel(*msg);
  }

  void people_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    reasoner_->input_people_speed(*msg);
  }

  void touch_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    reasoner_->input_touch_speed(*msg);
  }

  void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    reasoner_->input_global_plan(*msg);
  }

  void replan_reason_callback(const people_msgs::msg::Person::SharedPtr msg)
  {
    reasoner_->input_replan_reason(*msg);
  }

  void current_frame_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    reasoner_->input_current_frame(*msg);
  }

  void timer_callback()
  {
    if (reasoner_ == nullptr) {
      reasoner_ = std::make_shared<StopReasoner>(this->shared_from_this());
      auto announce_no_touch = this->declare_parameter("announce_no_touch", rclcpp::ParameterValue(false)).get<bool>();

      if (announce_no_touch) {
        stop_reason_filter_ =
          std::make_shared<StopReasonFilter>(
          std::vector<StopReason>(
          {
            StopReason::NO_NAVIGATION,
            StopReason::NOT_STOPPED,
            StopReason::STOPPED_BUT_UNDER_THRESHOLD
          }));
      } else {
        stop_reason_filter_ =
          std::make_shared<StopReasonFilter>(
          std::vector<StopReason>(
          {
            StopReason::NO_NAVIGATION,
            StopReason::NOT_STOPPED,
            StopReason::NO_TOUCH,
            StopReason::STOPPED_BUT_UNDER_THRESHOLD
          }));
      }
    }

    auto [duration, code] = reasoner_->update();
    stop_reason_filter_->update(duration, code);
    std::tie(duration, code) = stop_reason_filter_->event();

    if (code != StopReason::NONE) {
      cabot_msgs::msg::StopReason msg;
      msg.header.stamp = this->get_clock()->now();
      msg.reason = StopReasonUtil::toStr(code);
      msg.duration = duration;
      stop_reason_pub_->publish(msg);
    }
    std::tie(duration, code) = stop_reason_filter_->summary();
    if (code != StopReason::NONE) {
      std_msgs::msg::String msg;
      msg.data = "navigation;stop-reason;" + StopReasonUtil::toStr(code);
      event_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "%.2f, %s, %.2f", this->get_clock()->now().nanoseconds() / 1e9f, StopReasonUtil::toStr(code).c_str(), duration);
    }
    stop_reason_filter_->conclude();
  }

private:
  std::shared_ptr<StopReasoner> reasoner_;
  std::shared_ptr<StopReasonFilter> stop_reason_filter_;
  StopReason prev_code_;
  rclcpp::Publisher<cabot_msgs::msg::StopReason>::SharedPtr stop_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr people_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr touch_speed_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_global_plan_sub_;
  rclcpp::Subscription<people_msgs::msg::Person>::SharedPtr replan_reason_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_frame_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};  // class StopReasonsNode
}  // namespace CaBotUI

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotUI::StopReasonsNode);
