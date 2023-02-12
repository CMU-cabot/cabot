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

#include <behaviortree_cpp_v3/action_node.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>

using namespace std::chrono_literals;

// implementation of convertFromString for int8_t
namespace BT
{
template<>
std_msgs::msg::Int8 convertFromString<std_msgs::msg::Int8>(StringView str)
{
  std_msgs::msg::Int8 msg;
  msg.data = std::stoi(str.data());
  return msg;
}
}  // namespace BT

namespace cabot_bt
{
class PublishTopicActionBase : public BT::ActionNodeBase
{
public:
  PublishTopicActionBase(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!getInput("topic", topic_name_)) {
      RCLCPP_INFO(node_->get_logger(), "No topic name is specified");
      return;
    }
  }

  PublishTopicActionBase() = delete;
  ~PublishTopicActionBase()
  {
  }

  void halt() override
  {
  }

  void logStuck(const std::string & msg) const
  {
    static std::string prev_msg;

    if (msg == prev_msg) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    prev_msg = msg;
  }

  std::string topic_name_;
  rclcpp::Node::SharedPtr node_;
};

// for primitive messages (msg.data)
template<typename IN, typename OUT>
class PublishTopicAction : public PublishTopicActionBase
{
public:
  PublishTopicAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : PublishTopicActionBase(xml_tag_name, conf)
  {
    pub_ = node_->create_publisher<OUT>(topic_name_, 10);
    RCLCPP_DEBUG(node_->get_logger(), "Setup down PublishTopicAction BT node %s", typeid(OUT).name());
  }

  PublishTopicAction() = delete;
  ~PublishTopicAction()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down PublishTopicAction BT node %s", typeid(OUT).name());
  }

  BT::NodeStatus tick() override
  {
    if (pub_ == nullptr) {
      RCLCPP_INFO(node_->get_logger(), "No publisher");
      return BT::NodeStatus::FAILURE;
    }

    OUT msg;
    if (!getInput("value", msg.data)) {
      RCLCPP_INFO(node_->get_logger(), "Could not get value");
      return BT::NodeStatus::FAILURE;
    }

    pub_->publish(msg);
    rclcpp::spin_some(node_);
    RCLCPP_INFO(node_->get_logger(), "publish topic to %s\n%s", topic_name_.c_str(), std_msgs::msg::to_yaml(msg).c_str());

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<std::string>("topic", "topic name"),
      BT::InputPort<IN>("value", "value of the message")};
  }

  typename rclcpp::Publisher<OUT>::SharedPtr pub_;
};

// for standard message
template<typename IN, typename OUT>
class PublishTopicMsgAction : public PublishTopicActionBase
{
public:
  PublishTopicMsgAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : PublishTopicActionBase(xml_tag_name, conf)
  {
    pub_ = node_->create_publisher<OUT>(topic_name_, 10);
    RCLCPP_DEBUG(node_->get_logger(), "Setup down PublishTopicMsgAction BT node %s", typeid(OUT).name());
  }

  PublishTopicMsgAction() = delete;
  ~PublishTopicMsgAction()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down PublishTopicMsgAction BT node %s", typeid(OUT).name());
  }

  BT::NodeStatus tick() override
  {
    if (pub_ == nullptr) {
      RCLCPP_INFO(node_->get_logger(), "No publisher");
      return BT::NodeStatus::FAILURE;
    }

    OUT msg;
    if (!getInput("value", msg)) {
      RCLCPP_INFO(node_->get_logger(), "Could not get value");
      return BT::NodeStatus::FAILURE;
    }

    pub_->publish(msg);
    rclcpp::spin_some(node_);
    // RCLCPP_INFO(node_->get_logger(), "publish topic to %s\n%s", topic_name_.c_str(), rosidl_generator_traits::to_yaml(msg).c_str());

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<std::string>("topic", "topic name"),
      BT::InputPort<IN>("value", "value of the message")};
  }

  typename rclcpp::Publisher<OUT>::SharedPtr pub_;
};
}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    cabot_bt::PublishTopicAction<
      std::string,
      std_msgs::msg::String
    >
  >("PublishTopic");

  factory.registerNodeType<
    cabot_bt::PublishTopicAction<
      std::string,
      std_msgs::msg::Int8
    >
  >("PublishTopicInt8");

  factory.registerNodeType<
    cabot_bt::PublishTopicMsgAction<
      nav_msgs::msg::Path,
      nav_msgs::msg::Path
    >
  >("PublishTopicPath");

  factory.registerNodeType<
    cabot_bt::PublishTopicMsgAction<
      geometry_msgs::msg::PoseStamped,
      geometry_msgs::msg::PoseStamped
    >
  >("PublishTopicPoseStamped");
}
