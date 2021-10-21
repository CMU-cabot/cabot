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
#include "navcog_path_util.hpp"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class ConcatPathAction : public BT::ActionNodeBase
  {
  public:
    ConcatPathAction(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ActionNodeBase(condition_name, conf)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      RCLCPP_INFO(node_->get_logger(), "Initialize ConcatPath");
    }

    ConcatPathAction() = delete;

    ~ConcatPathAction()
    {
      RCLCPP_INFO(node_->get_logger(), "Shutting down ConcatPathAction BT node");
    }

    nav_msgs::msg::Path concatPath(nav_msgs::msg::Path path1, nav_msgs::msg::Path path2)
    {
      nav_msgs::msg::Path ret;
      ret.header = path1.header;
      auto frame_id = path1.header.frame_id;
      for(auto it = path1.poses.begin(); it < path1.poses.end(); it++)
      {
        if (it->header.frame_id == "")
        {
          it->header.frame_id = frame_id;
        }
        ret.poses.push_back(*it);
      }
      for(auto it = path2.poses.begin(); it < path2.poses.end(); it++)
      {
        if (it->header.frame_id == "")
        {
          it->header.frame_id = frame_id;
        }
        ret.poses.push_back(*it);
      }
      return ret;
    }

    BT::NodeStatus tick() override
    {
      nav_msgs::msg::Path path1, path2;
      getInput("path1", path1);
      getInput("path2", path2);
      RCLCPP_INFO(node_->get_logger(), "ConcatPath 1.poses.size = %ld", path1.poses.size());
      RCLCPP_INFO(node_->get_logger(), "ConcatPath 2.poses.size = %ld", path2.poses.size());
      nav_msgs::msg::Path ret = concatPath(path1, path2);

      setOutput("result", ret);
      return BT::NodeStatus::SUCCESS;
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
          BT::InputPort<nav_msgs::msg::Path>("path1", "First path to be concatinated"),
          BT::InputPort<nav_msgs::msg::Path>("path2", "Second path to be concatinated"),
          BT::OutputPort<nav_msgs::msg::Path>("result", "Result path")
        };
    }

    rclcpp::Node::SharedPtr node_;
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::ConcatPathAction>("ConcatPath");
}
