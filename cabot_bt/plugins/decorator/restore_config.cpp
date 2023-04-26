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

#include <behaviortree_cpp_v3/decorator_node.h>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cabot_bt
{

class RestoreConfig : public BT::DecoratorNode
{
public:
  RestoreConfig(
    const std::string & name,
    const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    if (getInput("store", store_name_)) {
      store_ = std::make_shared<std::map<std::string, std::vector<rclcpp::Parameter>>>();
      config().blackboard->set<std::shared_ptr<std::map<std::string, std::vector<rclcpp::Parameter>>>>(store_name_, store_);
    }
  }

  BT::NodeStatus executeTick()
  {
    BT::NodeStatus status = tick();
    setStatus(status);
    return status;
  }

  BT::NodeStatus tick()
  {
    const BT::NodeStatus child_state = child_node_->executeTick();
    if (child_state == BT::NodeStatus::SUCCESS ||
      child_state == BT::NodeStatus::FAILURE)
    {
      while (store_->size() > 0) {
        auto name = store_->begin()->first;
        auto client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, name);
        RCLCPP_INFO(node_->get_logger(), "Restored parameters (normal) %s", store_->begin()->first.c_str());
        client_->wait_for_service();
        restore_future_ = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
            client_->set_parameters(store_->begin()->second));
        while (restore_future_->wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
          rclcpp::spin_some(node_);
        }
        store_->erase(store_->begin());
      }
      return child_state;
    }
    return BT::NodeStatus::RUNNING;
  }

  void halt()
  {
    // if parameters are not stored yet (canceled), call set_parameters
    while (store_->size() > 0) {
      auto name = store_->begin()->first;
      auto client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, name);
      RCLCPP_INFO(node_->get_logger(), "Restored parameters (normal) %s", store_->begin()->first.c_str());
      client_->wait_for_service();
      restore_future_ = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
          client_->set_parameters(store_->begin()->second));
      while (restore_future_->wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
        rclcpp::spin_some(node_);
      }
      store_->erase(store_->begin());
    }

    restore_future_ = nullptr;

    child_node_->halt();
    setStatus(BT::NodeStatus::IDLE);
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
      BT::InputPort<std::string>("store", "store name")
      };
  }

private:
  std::string store_name_;
  std::shared_ptr<std::map<std::string, std::vector<rclcpp::Parameter>>> store_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>> restore_future_;
};
}  // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::RestoreConfig>("RestoreConfig");
}
