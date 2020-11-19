// Copyright (c) 2020 Carnegie Mellon University
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class WaitFloatAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>
  {
  public:
    WaitFloatAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
    {
      double duration = 0.1;
      getInput("wait_duration_float", duration);
      if (duration <= 0)
      {
        RCLCPP_WARN(
            node_->get_logger(), "Wait duration is negative or zero "
                                 "(%i). Setting to positive.",
            duration);
        duration *= -1;
      }

      goal_.time = rclcpp::Duration::from_seconds(duration);
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      RCLCPP_INFO(node_->get_logger(), "Setup WaitFloatAction BT node duration=%d.%d", goal_.time.sec, goal_.time.nanosec);
    }

    WaitFloatAction() = delete;

    ~WaitFloatAction()
    {
      RCLCPP_DEBUG(node_->get_logger(), "Shutting down WaitFloatAction BT node");
    }

    void on_tick()
    {
      increment_recovery_count();
    }

    void logStuck(const std::string &msg) const
    {
      static std::string prev_msg;

      if (msg == prev_msg)
      {
        return;
      }

      RCLCPP_INFO(node_->get_logger(), msg);
      prev_msg = msg;
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(BT::PortsList{
          BT::InputPort<double>("wait_duration_float", "wait duration in seconds"),
      });
    }
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<cabot_bt::WaitFloatAction>(name, "wait", config);
      };

  factory.registerBuilder<cabot_bt::WaitFloatAction>("WaitFloat", builder);
}
