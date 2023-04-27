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

#include <nav2_util/robot_utils.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class NeedToReplanCondition : public BT::ConditionNode
{
public:
  NeedToReplanCondition(const std::string & condition_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    need_to_replan_(false),
    count_(0),
    last_people_(nullptr),
    last_obstacles_(nullptr)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "people", rclcpp::SystemDefaultsQoS(),
      std::bind(&NeedToReplanCondition::peopleCallback, this, std::placeholders::_1));

    obstacles_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "obstacles", rclcpp::SystemDefaultsQoS(),
      std::bind(&NeedToReplanCondition::obstaclesCallback, this, std::placeholders::_1));

    rclcpp::QoS replan_reason_qos(10);
    replan_reason_pub_ = node_->create_publisher<people_msgs::msg::Person>(
      "replan_reason", replan_reason_qos);

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an NeedToReplanCondition");
  }

  NeedToReplanCondition() = delete;

  ~NeedToReplanCondition() {RCLCPP_DEBUG(node_->get_logger(), "Shutting down NeedToReplanCondition BT node");}

  void peopleCallback(const typename people_msgs::msg::People::SharedPtr msg)
  {
    RCLCPP_DEBUG(node_->get_logger(), "NeedToReplan: got people");

    last_people_ = msg;
  }

  void obstaclesCallback(const typename people_msgs::msg::People::SharedPtr msg)
  {
    RCLCPP_DEBUG(node_->get_logger(), "NeedToReplan: got obstacles");

    last_obstacles_ = msg;
  }

  void updateStates()
  {
    nav_msgs::msg::Path path;

    need_to_replan_ = false;
    if (!getInput("path", path)) {
      RCLCPP_WARN(node_->get_logger(), "NeedToReplan: path is missing");
      return;
    }

    double range = 0.70;
    if (last_people_) {
      for (auto person = last_people_->people.begin(); person != last_people_->people.end(); person++) {
        if (std::find(person->tags.begin(), person->tags.end(), "stationary") == person->tags.end()) {
          continue;
        }

        for (auto pose = path.poses.begin(); pose != path.poses.end(); pose++) {
          double dx = pose->pose.position.x - person->position.x;
          double dy = pose->pose.position.y - person->position.y;
          double dist = std::hypot(dx, dy);
          if (dist < range) {
            need_to_replan_ = true;
            person->tagnames.push_back("avoiding person");
            RCLCPP_INFO(node_->get_logger(), "avoiding person (%.2f, %.2f)", person->position.x, person->position.y);
            replan_reason_pub_->publish(*person);
            break;
          }
        }
        if (need_to_replan_) {break;}
      }
    }
    if (last_obstacles_) {
      for (auto obstacle = last_obstacles_->people.begin(); obstacle != last_obstacles_->people.end(); obstacle++) {
        if (std::find(obstacle->tags.begin(), obstacle->tags.end(), "stationary") == obstacle->tags.end()) {
          continue;
        }

        bool flag_person = false;
        if (last_people_) {
          for (auto person = last_people_->people.begin(); person != last_people_->people.end(); person++) {
            auto dx = obstacle->position.x - person->position.x;
            auto dy = obstacle->position.y - person->position.y;
            auto dist = sqrt(dx * dx + dy * dy);
            if (dist < 1.0) {
              flag_person = true;
              break;
            }
          }
        }
        if (flag_person) {
          continue;
        }

        for (auto pose = path.poses.begin(); pose != path.poses.end(); pose++) {
          double dx = pose->pose.position.x - obstacle->position.x;
          double dy = pose->pose.position.y - obstacle->position.y;
          double dist = std::hypot(dx, dy);
          if (dist < range) {
            need_to_replan_ = true;
            obstacle->tagnames.push_back("avoiding obstacle");
            RCLCPP_INFO(node_->get_logger(), "avoiding obstacle (%.2f, %.2f)", obstacle->position.x, obstacle->position.y);
            replan_reason_pub_->publish(*obstacle);
            break;
          }
        }
        if (need_to_replan_) {break;}
      }
    }
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);
    updateStates();
    if (need_to_replan_) {
      return BT::NodeStatus::SUCCESS;
    }
    if (count_ > 0) {
      count_ = 0;
      return BT::NodeStatus::FAILURE;
    }
    count_++;
    return BT::NodeStatus::RUNNING;
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
    return BT::PortsList{BT::InputPort<nav_msgs::msg::Path>("path", "path to be checked")};
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::atomic<bool> need_to_replan_;

  int count_;

  // Listen to odometry
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr obstacles_sub_;  // using People message for obstacle

  rclcpp::Publisher<people_msgs::msg::Person>::SharedPtr replan_reason_pub_;

  people_msgs::msg::People::SharedPtr last_people_;
  people_msgs::msg::People::SharedPtr last_obstacles_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<cabot_bt::NeedToReplanCondition>("NeedToReplan");
}
