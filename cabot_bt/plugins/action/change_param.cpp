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

#include "rcutils/allocator.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rcl_yaml_param_parser/parser.h"
#include "rcl_yaml_param_parser/types.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "behaviortree_cpp_v3/action_node.h"

using namespace std::chrono_literals;

namespace cabot_bt
{

  class ChangeParamAction : public BT::StatefulActionNode
  {
  public:
    ChangeParamAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::StatefulActionNode(action_name, conf)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      if (getInput("node_name", remote_node_name_))
      {
        parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, remote_node_name_);
      }

      int count = 30;
      RCLCPP_INFO(node_->get_logger(), "Waiting %s parameter server", remote_node_name_.c_str());
      while (!parameters_client_->wait_for_service(1s))
      {
        if (count < 0)
        {
          parameters_client_ = nullptr;
          break;
        }
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
    }

    ChangeParamAction() = delete;

    ~ChangeParamAction()
    {
      RCLCPP_DEBUG(node_->get_logger(), "Shutting down ChangeParamAction BT node");
    }

    BT::NodeStatus onStart() override
    {
      if (!parameters_client_)
      {
        RCLCPP_INFO(node_->get_logger(), "Check node_name");
        return BT::NodeStatus::FAILURE;
      }

      std::string param_name;
      if (!getInput("param_name", param_name))
      {
        RCLCPP_INFO(node_->get_logger(), "param_name is missing");
        return BT::NodeStatus::FAILURE;
      }

      std::string param_value;
      if (!getInput("param_value", param_value))
      {
        RCLCPP_INFO(node_->get_logger(), "param_value is missing");
        return BT::NodeStatus::FAILURE;
      }

      rcutils_allocator_t allocator = rcutils_get_default_allocator();
      rcl_params_t *params_hdl = rcl_yaml_node_struct_init(allocator);
      bool res = rcl_parse_yaml_value(remote_node_name_.c_str(),
                                      param_name.c_str(),
                                      param_value.c_str(), params_hdl);
      if (!res)
      {
        RCLCPP_INFO(node_->get_logger(), "Coule not parse param_value %s", param_value.c_str());
        return BT::NodeStatus::FAILURE;
      }
      rclcpp::ParameterMap map = rclcpp::parameter_map_from(params_hdl);

      RCLCPP_INFO(node_->get_logger(), "Set parameters");
      for (auto p : map[remote_node_name_])
      {
        RCLCPP_INFO(node_->get_logger(), "Param %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
      }
      future_ = parameters_client_->set_parameters(map[remote_node_name_]);
      running_ = true;
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      if (running_)
      {
        rclcpp::spin_some(node_);
        if (future_.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
        {
          RCLCPP_INFO(node_->get_logger(), "Success to set parameters");
          running_ = false;
          return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting set parameters");
        return BT::NodeStatus::RUNNING;
      }
      return BT::NodeStatus::FAILURE;
    }

    void onHalted() override
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
          BT::InputPort<std::string>("node_name", "node name"),
          BT::InputPort<std::string>("param_name", "param name"),
          BT::InputPort<std::string>("param_value", "param value"),
      };
    }

  private:
    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    std::string remote_node_name_;
    bool running_;
    BT::NodeStatus result_;
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future_;
  };

} // namespace cabot_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cabot_bt::ChangeParamAction>("ChangeParam");
}
