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
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_util/robot_utils.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class NeedToAvoidCondition : public BT::ConditionNode
{
public:
  NeedToAvoidCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    need_to_avoid_(false),
    last_avoid_num_(0)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    /*
    std::string odom_topic;
    if (!getInput("odom_topic", odom_topic)) {
      RCLCPP_ERROR(node_->get_logger(), "odom_topic missing");
      throw std::runtime_error("odom_topic missing");
      return;
    }
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&NeedToAvoidCondition::onOdomReceived, this, std::placeholders::_1));
    */

    RCLCPP_DEBUG(node_->get_logger(), "Initialized an NeedToAvoid");
  }

  NeedToAvoidCondition() = delete;

  ~NeedToAvoidCondition()
  {
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down NeedToAvoidCondition BT node");
  }

  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry");
    last_odom_ = msg;
  }

  void updateStates()
  {
    std::string robot_frame;
    people_msgs::msg::People people;

    need_to_avoid_ = false;
    if (!getInput("robot_frame", robot_frame)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "robot_frame missing");
      return;
    }
    if (!getInput("people", people)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "people is missing");
      return;
    }
    if (people.people.size() == 0) {
      return;
    }
    /*
    double max_acc;
    double delay;
    double min_threshold;
    double max_threshold;
    double V = 0;

    if (!last_odom_) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "odometry is missing, assumes the robot stops");
    } else {
      V = last_odom_->twist.twist.linear.x;
    }
    if (!getInput("max_acc", max_acc)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "max_acc is missing, assumes it is 0.5");
      max_acc = 0.5;
    }
    if (!getInput("delay", delay)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "delay is missing, assumes it is 0.2");
      delay = 0.2;
    }
    if (!getInput("min_threshold", min_threshold)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "min_threshold is missing, assumes it is 2.0");
      min_threshold = 2.0;
    }
    if (!getInput("max_threshold", max_threshold)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "max_threshold is missing, assumes it is 4.0");
      max_threshold = 4.0;
    }
    if (!getInput("global_frame", global_frame)) {
      RCLCPP_WARN_ONCE(node_->get_logger(), "global_frame missing");
      are_far_enough_ = false;
      return;
      }*/

    // transform person pose to global

    double min_walking_vel = 0.25;

    geometry_msgs::msg::TransformStamped transform_person;
    try {
      RCLCPP_DEBUG(node_->get_logger(), "lookupTransform %s->%s", robot_frame.c_str(), people.header.frame_id.c_str());
      transform_person = tf_buffer_->lookupTransform(robot_frame, people.header.frame_id, rclcpp::Time(0));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      return;
    }

    tf2::Transform tf2_transform_person;
    tf2::convert(transform_person.transform, tf2_transform_person);
    tf2::Transform tf2_rotation = tf2_transform_person;
    tf2_rotation.setOrigin(tf2::Vector3(0, 0, 0));

    bool avoid_approaching_person = false;
    getInput("avoid_approaching_person", avoid_approaching_person);

    bool avoid_static_person = true;
    getInput("avoid_approaching_person", avoid_approaching_person);

    people_msgs::msg::People avoid;
    for (auto person : people.people) {
      tf2::Vector3 pos(person.position.x, person.position.y, 0);
      tf2::Vector3 vel(person.velocity.x, person.velocity.y, 0);
      tf2::Vector3 lpos;
      tf2::Vector3 lvel;
      lpos = tf2_transform_person * pos;
      lvel = tf2_rotation * vel;
      RCLCPP_INFO(
        node_->get_logger(), "person[%s] pose (%.2f %.2f), vel (%.2f, %.2f)",
        person.name.c_str(), lpos.x(), lpos.y(), lvel.x(), lvel.y());

      if (avoid_approaching_person) {
        if (lvel.x() < -min_walking_vel) {
          RCLCPP_INFO(
            node_->get_logger(), "person[%s] pose (%.2f %.2f), vel (%.2f, %.2f) -> avoid (approaching)",
            person.name.c_str(), lpos.x(), lpos.y(), lvel.x(), lvel.y());
          avoid.people.push_back(person);
        }
      }

      if (avoid_static_person) {
        auto v = sqrt(lvel.x() * lvel.x() + lvel.y() * lvel.y());
        if (v < 0.2) {
          RCLCPP_INFO(
            node_->get_logger(), "person[%s] pose (%.2f %.2f), vel (%.2f, %.2f) -> avoid (static)",
            person.name.c_str(), lpos.x(), lpos.y(), lvel.x(), lvel.y());
          avoid.people.push_back(person);
        }
      }
    }

    if (avoid.people.size() > 0 ||
      (avoid.people.size() == 0 && last_avoid_num_ > 0))
    {
      need_to_avoid_ = true;
      last_avoid_num_ = avoid.people.size();
      setOutput("avoid", avoid);
      setOutput("avoid_count", last_avoid_num_);
    }

    /*
    // transform robot pose to global
    RCLCPP_DEBUG(node_->get_logger(), "getCurrentPose %s %s", global_frame.c_str(), robot_frame.c_str());
    geometry_msgs::msg::PoseStamped pose;
    nav2_util::getCurrentPose(pose, *tf_buffer_, global_frame, robot_frame, 0.1);

    // check if person is far enough
    RCLCPP_DEBUG(node_->get_logger(), "current pose %.2f %.2f", pose.pose.position.x, pose.pose.position.y);
    RCLCPP_DEBUG(node_->get_logger(), "person pose %.2f %.2f", person.person.position.x, person.person.position.y);
    double dx = pose.pose.position.x - person_position_global.x();
    double dy = pose.pose.position.y - person_position_global.y();
    double dist = sqrt(dx*dx + dy*dy);
    double BD = (V*V + 2*max_acc*V*delay) / (2*max_acc);

    are_far_enough_ = (dist > min_threshold + BD) && (dist < max_threshold + BD);
    RCLCPP_INFO(node_->get_logger(), "distance = %.2f, BD = %.2f, min-max = %.2f-%.2f",
          dist, BD, min_threshold, max_threshold);
    */
  }

  BT::NodeStatus tick() override
  {
    updateStates();
    if (need_to_avoid_) {
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
      BT::InputPort<std::string>("robot_frame", "Robot frame"),
      BT::InputPort<people_msgs::msg::People>("people", "people to be checked"),
      BT::InputPort<nav_msgs::msg::Path>("path", "path to be checked"),
      BT::InputPort<bool>("avoid_approaching_person", "robot avoid approaching person"),
      BT::InputPort<bool>("avoid_static_person", "robot avoid static person"),
      BT::OutputPort<people_msgs::msg::People>("avoid", "people needs to avoid"),
      BT::OutputPort<int>("avoid_count", "number of people needs to avoid")};
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::atomic<bool> need_to_avoid_;
  int last_avoid_num_;

  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::NeedToAvoidCondition>("NeedToAvoid");
}
