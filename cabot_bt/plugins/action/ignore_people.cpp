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

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class IgnorePeopleAction : public BT::StatefulActionNode
{
public:
  IgnorePeopleAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::StatefulActionNode(action_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string ignore_topic;
    if (!getInput("ignore_topic", ignore_topic)) {
      ignore_topic = "ignore";
    }
    RCLCPP_INFO(node_->get_logger(), "prepareing publisher %s", ignore_topic.c_str());
    ignore_pub_ = node_->create_publisher<people_msgs::msg::People>(ignore_topic, 10);

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an IgnorePeople");
  }

  IgnorePeopleAction() = delete;

  ~IgnorePeopleAction()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down IgnorePeopleAction BT node");
  }

  BT::NodeStatus onStart() override
  {
    people_msgs::msg::People people;
    if (!getInput("people", people)) {
      RCLCPP_ERROR(node_->get_logger(), "people missing");
    }
    ignore_pub_->publish(people);
    RCLCPP_INFO(node_->get_logger(), "publish ignore");

    start_ = clock_.now();
    RCLCPP_INFO(node_->get_logger(), "%.5f", start_.seconds());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    double wait_duration;
    if (!getInput("wait_duration", wait_duration)) {
      RCLCPP_ERROR(node_->get_logger(), "wait_duration missing (default = 1.0 sec)");
      wait_duration = 1.0;
    }
    std::chrono::duration<double, std::ratio<1>> duration(wait_duration);

    if (clock_.now() - start_ < duration) {
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
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

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<people_msgs::msg::People>("people", "people to be ignored"),
      BT::InputPort<double>("ignore_topic", "topic to publish ignoreing people"),
      BT::InputPort<double>("wait_duration", "duration for waiting global costmap update in seconds"),
    };
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  rclcpp::Publisher<people_msgs::msg::People>::SharedPtr ignore_pub_;
  rclcpp::Clock clock_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::IgnorePeopleAction>("IgnorePeople");
}
