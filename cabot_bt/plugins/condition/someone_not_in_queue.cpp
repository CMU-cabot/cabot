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
#include <chrono>
#include <cmath>
#include <atomic>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "queue_msgs/msg/queue.hpp"
#include "people_msgs/msg/people.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class SomeoneNotInQueueCondition : public BT::ConditionNode
  {
  public:
    SomeoneNotInQueueCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          is_someone_not_in_queue_(false),
          last_msg_()
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      std::string queue_topic;
      rclcpp::QoS queue_qos(10);

      if (!getInput("queue_topic", queue_topic))
      {
        RCLCPP_WARN(node_->get_logger(), "no queue_topic");
        queue_topic = "queue";
      }

      queue_sub_ = node_->create_subscription<queue_msgs::msg::Queue>(
          queue_topic, queue_qos,
          std::bind(&SomeoneNotInQueueCondition::onQueueReceived, this, std::placeholders::_1));

      RCLCPP_DEBUG(node_->get_logger(), "Initialized an SomeoneNotInQueueCondition");
    }

    SomeoneNotInQueueCondition() = delete;

    ~SomeoneNotInQueueCondition()
    {
      RCLCPP_DEBUG(node_->get_logger(), "Shutting down SomeoneNotInQueueCondition BT node");
    }

    void onQueueReceived(const typename queue_msgs::msg::Queue::SharedPtr msg)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Got queue");
      last_msg_ = msg;
    }

    void updateStates()
    {
      RCLCPP_DEBUG(node_->get_logger(), "updateStates");
      people_msgs::msg::People people;

      if (!getInput("people", people))
      {
        RCLCPP_ERROR(node_->get_logger(), "people missing");
        is_someone_not_in_queue_ = false;
        return;
      }

      people_msgs::msg::People people_out_queue;
      people_msgs::msg::People people_out_not_queue;

      if (last_msg_ != nullptr && last_msg_->people_names.size() > 0)
      {

        for (auto person = people.people.begin(); person != people.people.end(); person++)
        {
          bool person_in_queue = false;

          for (int i = 0; i < last_msg_->people_names.size(); i++)
          {
            if (person->name == last_msg_->people_names[i])
            {
              person_in_queue = true;
              break;
            }
          }

          if (person_in_queue)
          {
            people_out_queue.people.push_back(*person);
          }
          else
          {
            people_out_not_queue.people.push_back(*person);
            is_someone_not_in_queue_ = true;
          }
        }
      }

      setOutput("people_out_queue", people_out_queue);
      setOutput("people_out_not_queue", people_out_not_queue);

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "number of people in queue=(%d), not in queue=(%d)", people_out_queue.people.size(), people_out_not_queue.people.size());
    }

    BT::NodeStatus tick() override
    {
      is_someone_not_in_queue_ = false;
      updateStates();
      if (is_someone_not_in_queue_)
      {
        return BT::NodeStatus::SUCCESS;
      }
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
          BT::InputPort<std::string>("queue_topic", "queue topic name"),
          BT::InputPort<people_msgs::msg::People>("people", "People created by PeopleExists"),
          BT::OutputPort<people_msgs::msg::People>("people_out_queue", "People in queue"),
          BT::OutputPort<people_msgs::msg::People>("people_out_not_queue", "People not in queue")};
    }

  private:
    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;

    std::atomic<bool> is_someone_not_in_queue_;

    rclcpp::Subscription<queue_msgs::msg::Queue>::SharedPtr queue_sub_;
    queue_msgs::msg::Queue::SharedPtr last_msg_;
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::SomeoneNotInQueueCondition>("SomeoneNotInQueue");
}
