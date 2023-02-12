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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_util/robot_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

namespace cabot_bt
{

class CurrentPoseAction : public BT::ActionNodeBase
{
public:
  CurrentPoseAction(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(condition_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialize CurrentPose");
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);
  }

  CurrentPoseAction() = delete;

  ~CurrentPoseAction()
  {
    RCLCPP_INFO(node_->get_logger(), "Shutting down CurrentPoseAction BT node");
  }


  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, 0.5)) {
      return BT::NodeStatus::FAILURE;
    }

    setOutput("pose", current_pose);
    RCLCPP_INFO(
      node_->get_logger(), "CurrentPose pose.position = (%.2f, %.2f)",
      current_pose.pose.position.x, current_pose.pose.position.y);
    return BT::NodeStatus::SUCCESS;
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

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<std::string>("global_frame", "global frame name"),
      BT::InputPort<std::string>("robot_base_frame", "robot base frame name"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "The current pose"),
    };
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_base_frame_, global_frame_;
};

}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::CurrentPoseAction>("CurrentPose");
}
