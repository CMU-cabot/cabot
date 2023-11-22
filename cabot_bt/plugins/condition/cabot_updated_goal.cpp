// Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2/utils.h"


namespace cabot_bt
{

  // check if goal in the blackboard is changed, replace this node with GloballyUpdatedGoal after ROS2 Humble
  class CabotUpdatedGoalCondition : public BT::ConditionNode
  {
  public:
    CabotUpdatedGoalCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          first_time_(true)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    CabotUpdatedGoalCondition() = delete;

    ~CabotUpdatedGoalCondition()
    {
      RCLCPP_INFO(node_->get_logger(), "Shutting down CabotGoalUpdatedCondition BT node");
    }

    BT::NodeStatus tick() override
    {
      if (first_time_) {
        first_time_ = false;
        config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
        config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
        return BT::NodeStatus::FAILURE;
      }

      std::vector<geometry_msgs::msg::PoseStamped> current_goals;
      config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
      geometry_msgs::msg::PoseStamped current_goal;
      config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

      if (goal_ != current_goal || goals_ != current_goals) {
        goal_ = current_goal;
        goals_ = current_goals;
        return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts()
    {
      return {};
    }

  private:
    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;

    bool first_time_;
    geometry_msgs::msg::PoseStamped goal_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::CabotUpdatedGoalCondition>("CabotGoalUpdated");
}