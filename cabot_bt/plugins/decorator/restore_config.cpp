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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <future>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cabot_bt
{

  class RestoreConfig : public BT::DecoratorNode
  {
  public:
    RestoreConfig(
        const std::string &name,
        const BT::NodeConfiguration &conf)
        : BT::DecoratorNode(name, conf),
          stored_params_(nullptr),
          last_child_state_(BT::NodeStatus::FAILURE)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      if (getInput("node_name", remote_node_name_))
      {
        parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, remote_node_name_);
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
      // param is not stored yet
      if (stored_params_ == nullptr)
      {

        // list parameters
        if (list_future_ == nullptr)
        {
          list_future_ = std::make_shared<std::shared_future<rcl_interfaces::msg::ListParametersResult>>(
              parameters_client_->list_parameters({}, 100));
          RCLCPP_INFO(node_->get_logger(), "Request parameter list %s", name().c_str());
          return BT::NodeStatus::RUNNING;
        }

        // get parameters
        if (get_future_ == nullptr)
        {
          rclcpp::spin_some(node_);
          if (list_future_->wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
          {
            get_future_ = std::make_shared<std::shared_future<std::vector<rclcpp::Parameter>>>(
                parameters_client_->get_parameters(list_future_->get().names));
            RCLCPP_INFO(node_->get_logger(), "Request parameters %s", name().c_str());
          }
          return BT::NodeStatus::RUNNING;
        }

        // saved parameters
        rclcpp::spin_some(node_);
        if (get_future_->wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
        {
          stored_params_ = std::make_shared<std::vector<rclcpp::Parameter>>(get_future_->get());
          RCLCPP_INFO(node_->get_logger(), "Saving parameters %s", name().c_str());
        }
        return BT::NodeStatus::RUNNING;
      }

      // restore parameters
      if (restore_future_ != nullptr)
      {
        rclcpp::spin_some(node_);
        if (restore_future_->wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
        {
          RCLCPP_INFO(node_->get_logger(), "Restored parameters (normal) %s", name().c_str());
          stored_params_ = nullptr;
          return last_child_state_;
        }
        return BT::NodeStatus::RUNNING;
      }

      // once stored parameters
      const BT::NodeStatus child_state = child_node_->executeTick();

      // when child finish running, call set_parameter to restore saved parameters
      if (child_state == BT::NodeStatus::SUCCESS ||
          child_state == BT::NodeStatus::FAILURE)
      {
        last_child_state_ = child_state;
        restore_future_ = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
            parameters_client_->set_parameters(*stored_params_));
      }
      return BT::NodeStatus::RUNNING;
    }

    void halt()
    {
      // if parameters are not stored yet (canceled), call set_parameters
      if (stored_params_ != nullptr)
      {
        restore_future_ = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
            parameters_client_->set_parameters(*stored_params_));

        while (restore_future_->wait_for(std::chrono::milliseconds(1)) != std::future_status::ready)
        {
          rclcpp::spin_some(node_);
        }
      }

      list_future_ = nullptr;
      get_future_ = nullptr;
      restore_future_ = nullptr;
      stored_params_ = nullptr;

      child_node_->halt();
      setStatus(BT::NodeStatus::IDLE);
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
          BT::InputPort<std::string>("node_name", "node name")};
    }

  private:
    std::shared_ptr<std::shared_future<rcl_interfaces::msg::ListParametersResult>> list_future_;
    std::shared_ptr<std::shared_future<std::vector<rclcpp::Parameter>>> get_future_;
    std::shared_ptr<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>> restore_future_;
    std::shared_ptr<std::vector<rclcpp::Parameter>> stored_params_;

    rclcpp::Node::SharedPtr node_;
    std::string remote_node_name_;
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    BT::NodeStatus last_child_state_;
  };
} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::RestoreConfig>("RestoreConfig");
}
