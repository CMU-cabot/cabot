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
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "people_msgs/msg/person_stamped.hpp"
#include "people_msgs/msg/people.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class CanPassPersonCondition : public BT::ConditionNode
  {
  public:
    CanPassPersonCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          can_pass_(false)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      std::string action_name = "/dummy/compute_path_to_pose";
      action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node_, action_name);
      RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
      action_client_->wait_for_action_server();

      std::string plan_topic;
      if (!getInput("plan_topic", plan_topic))
      {
        plan_topic = "plan";
      }
      RCLCPP_INFO(node_->get_logger(), "prepareing plan publisher %s", plan_topic.c_str());
      plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(plan_topic, 10);

      RCLCPP_DEBUG(node_->get_logger(), "Initialized an CanPassPerson");
    }

    CanPassPersonCondition() = delete;

    ~CanPassPersonCondition()
    {
      RCLCPP_DEBUG(node_->get_logger(), "Shutting down CanPassPersonCondition BT node");
    }

    bool set_goal()
    {
      getInput("goal", goal_.pose);
      return true;
    }

    void send_goal()
    {
      goal_result_available_ = false;
      error_ = false;
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
      send_goal_options.result_callback =
          [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult &result) {
            if (result.code != rclcpp_action::ResultCode::ABORTED)
            {
              goal_result_available_ = true;
              result_ = result;
            }
          };

      auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

      if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::seconds(1)) !=
          rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        goal_result_available_ = true;
        error_ = true;
        return;
      }

      goal_handle_ = future_goal_handle.get();
      if (goal_handle_ == nullptr)
      {
        goal_result_available_ = true;
        error_ = true;
        return;
      }
    }

    void check_result()
    {
      can_pass_ = false;

      nav_msgs::msg::Path path;
      if (!getInput("path", path))
      {
        RCLCPP_ERROR(node_->get_logger(), "path is missing");
        return;
      }
      if (error_)
      {
        return;
      }

      int n = path.poses.size() + 1;
      int m = result_.result->path.poses.size() + 1;

      double *cost = (double *)malloc(sizeof(double) * n * m);

      for (int i = 0; i < n; i++)
      {
        for (int j = 0; j < m; j++)
        {
          cost[i * m + j] = std::numeric_limits<double>::max();
        }
      }
      cost[0] = 0;
      for (int i = 1; i < n; i++)
      {
        for (int j = 1; j < m; j++)
        {
          auto p1 = path.poses[i - 1];
          auto p2 = result_.result->path.poses[j - 1];

          double dist = sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) +
                             pow(p1.pose.position.y - p2.pose.position.y, 2));
          cost[i * m + j] = dist + std::min(std::min(cost[(i - 1) * m + j], cost[i * m + j - 1]), cost[(i - 1) * m + j - 1]);
        }
      }
      int count = 0;
      for (int i = 0, j = 0; i < n - 1 && j < m - 1;)
      {
        auto current = cost[i * m + j];
        auto min = std::min(std::min(cost[(i + 1) * m + j], cost[i * m + j + 1]), cost[(i + 1) * m + j + 1]);
        if (min - current > 0.1)
        {
          count++;
        }
        if (min == cost[(i + 1) * m + j + 1])
        {
          i++;
          j++;
        }
        else if (min == cost[(i + 1) * m + j])
        {
          i++;
        }
        else if (min == cost[i * m + j + 1])
        {
          j++;
        }
      }

      double total_dist = cost[(n - 1) * +m - 1];
      double ave_dist = total_dist / count;

      RCLCPP_INFO(node_->get_logger(), "cost %.2f, %d, %.2f, %d", total_dist, count, ave_dist, ave_dist < 2.0);
      can_pass_ = ave_dist < 2.0;

      free(cost);
    }

    BT::NodeStatus tick() override
    {
      if (status() == BT::NodeStatus::IDLE)
      {
        setStatus(BT::NodeStatus::RUNNING);
        if (!set_goal())
        {
          return BT::NodeStatus::FAILURE;
        }
        send_goal();
      }

      if (rclcpp::ok() && !goal_result_available_)
      {
        rclcpp::spin_some(node_);

        if (!goal_result_available_)
        {
          return BT::NodeStatus::RUNNING;
        }
      }

      if (result_.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(node_->get_logger(), "failed to get a path");
        return BT::NodeStatus::FAILURE;
      }

      check_result();

      if (can_pass_)
      {
        RCLCPP_INFO(node_->get_logger(), "can pass");
        setOutput("path_out", result_.result->path);
        plan_pub_->publish(result_.result->path);

        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_INFO(node_->get_logger(), "can not pass");
      return BT::NodeStatus::FAILURE;
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
      return BT::PortsList{
          BT::InputPort<nav_msgs::msg::Path>("path", "The path"),
          BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "The goal"),
          BT::OutputPort<nav_msgs::msg::Path>("path_out", "The passing path"),
          BT::InputPort<std::string>("plan_topic", "topic where path published"),
      };
    }

  private:
    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;

    std::atomic<bool> can_pass_;

    nav2_msgs::action::ComputePathToPose::Goal goal_;
    int goal_index_;
    std::atomic<bool> goal_result_available_;
    std::atomic<bool> goal_updated_;
    std::atomic<bool> error_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr goal_handle_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult result_;
    std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> action_client_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::CanPassPersonCondition>("CanPassPerson");
}
