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
#include <people_msgs/msg/person_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class SomeoneOnPathCondition : public BT::ConditionNode
{
public:
  SomeoneOnPathCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    is_someone_on_path_(false),
    person_distance_(0.0)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an SomeoneOnPath");
  }

  SomeoneOnPathCondition() = delete;

  ~SomeoneOnPathCondition()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down SomeoneOnPathCondition BT node");
  }

  void updateStates()
  {
    RCLCPP_DEBUG(node_->get_logger(), "updateStates");
    nav_msgs::msg::Path path;
    people_msgs::msg::People people;
    double range = 0.25;

    if (!getInput("people", people)) {
      RCLCPP_ERROR(node_->get_logger(), "people missing");
      is_someone_on_path_ = false;
      return;
    }
    if (!getInput("path", path)) {
      RCLCPP_ERROR(node_->get_logger(), "path is missing");
      is_someone_on_path_ = false;
      return;
    }
    if (!getInput("range", range)) {
      RCLCPP_ERROR(node_->get_logger(), "range is missing");
      is_someone_on_path_ = false;
      return;
    }

    // ToDo: consider surrounding people not on the path

    people_msgs::msg::Person * person_on_the_path = nullptr;

    person_distance_ = 100;
    for (auto person = people.people.begin(); person != people.people.end(); person++) {
      for (auto pose = path.poses.begin(); pose != path.poses.end(); pose++) {
        double dx = pose->pose.position.x - person->position.x;
        double dy = pose->pose.position.y - person->position.y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < range) {
          person_on_the_path = &(*person);
        }
        if (dist < person_distance_) {
          person_distance_ = dist;
        }
      }

      if (person_on_the_path) {
        break;
      }
    }
    if (person_on_the_path) {
      is_someone_on_path_ = true;
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000, "person(%s) is on the path! (%.2f)",
        person_on_the_path->name.c_str(), person_distance_);

      people_msgs::msg::PersonStamped person_stamped;
      person_stamped.person = *person_on_the_path;
      person_stamped.header = people.header;
      setOutput("person_out", person_stamped);
    } else {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000, "person is not on the path! (%.2f)",
        person_distance_);
    }
  }

  BT::NodeStatus tick() override
  {
    is_someone_on_path_ = false;
    updateStates();
    if (is_someone_on_path_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
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
      BT::InputPort<people_msgs::msg::People>("people", "People created by PeopleExists"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
      BT::InputPort<double>("range", "Range to detect person from the path"),
      BT::OutputPort<people_msgs::msg::PersonStamped>("person_out", "Person on the path")};
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  std::atomic<bool> is_someone_on_path_;
  double person_distance_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::SomeoneOnPathCondition>("SomeoneOnPath");
}
