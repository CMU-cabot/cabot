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
#include "nav_msgs/msg/path.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person_stamped.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class PersonStopsCondition : public BT::ConditionNode
  {
  public:
    PersonStopsCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          person_stops_(BT::NodeStatus::SUCCESS)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      RCLCPP_DEBUG(node_->get_logger(), "Initialized an PersonStops");
    }

    PersonStopsCondition() = delete;

    ~PersonStopsCondition()
    {
      RCLCPP_DEBUG(node_->get_logger(), "Shutting down PersonStopsCondition BT node");
    }

    void updateStates()
    {
      person_stops_ = BT::NodeStatus::FAILURE;

      RCLCPP_DEBUG(node_->get_logger(), "updateStates");
      people_msgs::msg::People people;

      double threshold = 0.1;
      double duration = 2.0;

      if (!getInput("people", people))
      {
        RCLCPP_ERROR(node_->get_logger(), "people missing");
        return;
      }
      if (!getInput("threshold", threshold))
      {
        RCLCPP_ERROR(node_->get_logger(), "threshold is missing");
        return;
      }
      if (!getInput("duration", duration))
      {
        RCLCPP_ERROR(node_->get_logger(), "duration is missing");
        return;
      }

      std::unordered_map<std::string, people_msgs::msg::PersonStamped> update_map_;
      auto now = rclcpp::Time(people.header.stamp);

      for (auto person : people.people)
      {
        auto v = person.velocity;
        double vm = sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
        //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "person velocity is %.2f", vm);

        if (vm > threshold)
        {
          continue;
        }

        auto it = person_map_.find(person.name);
        if (it != person_map_.end())
        {
          double diff = (now - rclcpp::Time(it->second.header.stamp)).seconds();
          RCLCPP_INFO(node_->get_logger(), "checking person[%s] stops (%.2f) for %.2f/%.2f seconds",
                      person.name.c_str(), vm, diff, duration);
          if (diff > duration)
          {
            person_stops_ = BT::NodeStatus::SUCCESS;
          }
          update_map_.insert(*it);
        }
        else
        {
          people_msgs::msg::PersonStamped person_stamped;
          person_stamped.person = person;
          person_stamped.header = people.header;
          update_map_.insert({person.name, person_stamped});
        }
      }
      person_map_ = update_map_;
    }

    BT::NodeStatus tick() override
    {
      updateStates();
      return person_stops_;
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
          BT::InputPort<people_msgs::msg::People>("people", "people to be checked"),
          BT::InputPort<double>("threshold", "Velocity threshold"),
          BT::InputPort<double>("duration", "Duration to see if the person actually stops"),
      };
    }

  private:
    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;

    BT::NodeStatus person_stops_;

    std::unordered_map<std::string, people_msgs::msg::PersonStamped> person_map_;
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::PersonStopsCondition>("PeopleStops");
}
