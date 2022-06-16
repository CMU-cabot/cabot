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

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace cabot_bt {

class NeedToReplanCondition : public BT::ConditionNode {
 public:
  NeedToReplanCondition(const std::string &condition_name, const BT::NodeConfiguration &conf)
      : BT::ConditionNode(condition_name, conf), 
        need_to_replan_(false),
        count_(0),
        last_people_(nullptr),
        last_obstacles_(nullptr) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    people_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "people", rclcpp::SystemDefaultsQoS(), 
      std::bind(&NeedToReplanCondition::peopleCallback, this, std::placeholders::_1));

    obstacles_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "obstacles", rclcpp::SystemDefaultsQoS(), 
      std::bind(&NeedToReplanCondition::obstaclesCallback, this, std::placeholders::_1));

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an NeedToReplanCondition");
  }

  NeedToReplanCondition() = delete;

  ~NeedToReplanCondition() { RCLCPP_DEBUG(node_->get_logger(), "Shutting down NeedToReplanCondition BT node"); }

  void peopleCallback(const typename people_msgs::msg::People::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "NeedToReplan: got people");
    
    last_people_ = msg;
  }

  void obstaclesCallback(const typename people_msgs::msg::People::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "NeedToReplan: got obstacles");

    last_obstacles_ = msg;
  }

  void updateStates() {
    nav_msgs::msg::Path path;

    need_to_replan_ = false;
    if (!last_people_) { 
      RCLCPP_WARN(node_->get_logger(), "NeedToReplan: people is missing");
      return; 
    }
    if (!last_obstacles_) { 
      RCLCPP_WARN(node_->get_logger(), "NeedToReplan: obstacle is missing");
      return; 
    }
    if (!getInput("path", path)) {
      RCLCPP_WARN(node_->get_logger(), "NeedToReplan: path is missing");
      return;
    }

    people_msgs::msg::Person *person_on_the_path = nullptr;

    double range = 0.5;
    for (auto person = last_people_->people.begin(); person != last_people_->people.end(); person++)
    {
      if (std::find(person->tags.begin(), person->tags.end(), "stationary") == person->tags.end()) {
        continue;
      }

      for (auto pose = path.poses.begin(); pose != path.poses.end(); pose++)
      {
        double dx = pose->pose.position.x - person->position.x;
        double dy = pose->pose.position.y - person->position.y;
        double dist = std::hypot(dx, dy);
        if (dist < range) {
          need_to_replan_ = true;
          break;
        }
      }
    }
    for (auto obstacle = last_obstacles_->people.begin(); obstacle != last_obstacles_->people.end(); obstacle++)
    {
      if (std::find(obstacle->tags.begin(), obstacle->tags.end(), "stationary") == obstacle->tags.end()) {
        continue;
      }

      for (auto pose = path.poses.begin(); pose != path.poses.end(); pose++)
      {
        double dx = pose->pose.position.x - obstacle->position.x;
        double dy = pose->pose.position.y - obstacle->position.y;
        double dist = std::hypot(dx, dy);
        if (dist < range) {
          need_to_replan_ = true;
          break;
        }
      }
    }
  }

  BT::NodeStatus tick() override {
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

  void logStuck(const std::string &msg) const {
    static std::string prev_msg;

    if (msg == prev_msg) {
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    prev_msg = msg;
  }

  static BT::PortsList providedPorts() {
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
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr obstacles_sub_; // using People message for obstacle

  people_msgs::msg::People::SharedPtr last_people_;
  people_msgs::msg::People::SharedPtr last_obstacles_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<cabot_bt::NeedToReplanCondition>("NeedToReplan"); }
