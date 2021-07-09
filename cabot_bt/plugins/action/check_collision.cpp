// Copyright (c) 2018, 2021  Carnegie Mellon University, IBM Corporation, and others
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
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class CheckCollisionAction : public BT::ActionNodeBase
  {
  public:
    CheckCollisionAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ActionNodeBase(condition_name, conf)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      RCLCPP_INFO(node_->get_logger(), "Initialize CheckCollision");
    }

    CheckCollisionAction() = delete;

    ~CheckCollisionAction()
    {
      RCLCPP_INFO(node_->get_logger(), "Shutting down CheckCollision BT node");
    }

    BT::NodeStatus tick() override
    {
      nav_msgs::msg::Path path;
      int cost_lethal = 254;
      getInput("path", path);
      //TODO call getInput to override cost_lethal
      RCLCPP_INFO(node_->get_logger(), "CheckCollision poses.size = %ld", path.poses.size());
      //TODO we might need to consider the current pose to avoid the possibitliy of checking with the debris in the costmap in the "passed" path.
      if (has_collision(path, cost_lethal))
      {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }

    void halt() override
    {
    }

    void logStuck(const std::string &msg) const
    {
      static std::string prev_msg;

      if (msg == prev_msg)
      {
        return;
      }

      RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
      prev_msg = msg;
    }

    static BT::PortsList providedPorts()
    {
      return BT::PortsList{
	      BT::InputPort<nav_msgs::msg::Path>("path", "The path to be converted")
        //,BT::InputPort<int>("lethal cost of collision") //TODO add this to override lethal cost.
      };
    }

    rclcpp::Node::SharedPtr node_;
  private:
    bool has_collision(const nav_msgs::msg::Path path, const int cost_lethal){
      //TODO put the guts here.
      return true;//false;
    }
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::CheckCollisionAction>("CheckCollision");
}
