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

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class ClearCostmapAroundRobotAction : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>
  {
  public:
    ClearCostmapAroundRobotAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BtServiceNode(action_name, conf)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      RCLCPP_DEBUG(node_->get_logger(), "Setup down ClearCostmapAroundRobotAction BT node");
    }

    ClearCostmapAroundRobotAction() = delete;

    ~ClearCostmapAroundRobotAction()
    {
      RCLCPP_DEBUG(node_->get_logger(), "Shutting down ClearCostmapAroundRobotAction BT node");
    }

    void on_tick()
    {
      getInput("window_size_x", request_->window_size_x);
      getInput("window_size_y", request_->window_size_y);
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
          BT::InputPort<float>("window_size_x", "window size x"),
          BT::InputPort<float>("window_size_y", "window size y")});
    }
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::ClearCostmapAroundRobotAction>("ClearCostmapAroundRobot");
}
