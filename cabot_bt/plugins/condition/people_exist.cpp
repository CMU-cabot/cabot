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

#include <behaviortree_cpp_v3/condition_node.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class PeopleExistCondition : public BT::ConditionNode
{
public:
  PeopleExistCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    people_exists_(false),
    people_history_size_(3),
    people_count_(0),
    last_msg_()
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string people_topic;
    rclcpp::QoS people_qos(10);

    if (!getInput("people_topic", people_topic)) {
      RCLCPP_WARN(node_->get_logger(), "no people_topic");
      people_topic = "people";
    }

    people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      people_topic, people_qos,
      std::bind(&PeopleExistCondition::onPeopleReceived, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Initialized an PeopleExistCondition");
  }

  PeopleExistCondition() = delete;

  ~PeopleExistCondition()
  {
    RCLCPP_INFO(node_->get_logger(), "Shutting down PeopleExistCondition BT node");
  }

  void onPeopleReceived(const typename people_msgs::msg::People::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Got people %.2f", rclcpp::Time(msg->header.stamp).seconds());
    last_msg_ = msg;
  }

  void updateStates()
  {
    RCLCPP_INFO(node_->get_logger(), "updateStates");

    bool people_exists = (last_msg_ != nullptr && last_msg_->people.size() > 0);

    if (people_exists == people_exists_) {
      people_count_ = people_history_size_;
    } else {
      people_count_--;
      if (people_count_ <= 0) {
        people_exists_ = people_exists;
        people_count_ = people_history_size_;
      }
    }
  }

  BT::NodeStatus tick() override
  {
    if (last_msg_ == nullptr) {
      // need to receive message
      rclcpp::spin_some(node_);
      return BT::NodeStatus::FAILURE;
    }

    updateStates();
    if (people_exists_) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "People exists! header=%.2f, now=%.2f", rclcpp::Time(last_msg_->header.stamp).seconds(), node_->now().seconds());
      setOutput("people_out", *last_msg_);

      last_msg_ = nullptr;
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "People not exists! header=%.2f, now=%.2f", rclcpp::Time(last_msg_->header.stamp).seconds(), node_->now().seconds());

      last_msg_ = nullptr;
      return BT::NodeStatus::FAILURE;
    }
  }

  void logStuck(const std::string & msg) const
  {
    static std::string prev_msg;

    if (msg == prev_msg) {
      return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "%s", msg.c_str());
    prev_msg = msg;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<std::string>("people_topic", "people topic name"),
      BT::OutputPort<people_msgs::msg::People>("people_out", "people found")};
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  // rclcpp::CallbackGroup::SharedPtr callback_group_;
  // rclcpp::executors::SingleThreadedExecutor callback_group_executor_;


  std::atomic<bool> people_exists_;

  int people_history_size_;
  int people_count_;

  // Listen to odometry
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  people_msgs::msg::People::SharedPtr last_msg_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::PeopleExistCondition>("PeopleExist");
}
