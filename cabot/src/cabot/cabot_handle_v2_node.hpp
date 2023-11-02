#ifndef CABOT__CABOT_HANDLE_V2_NODE_HPP_
#define CABOT__CABOT_HANDLE_V2_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include "event.hpp"
#include "handle_v2.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>

class CaBotHandleV2Node : public rclcpp::Node{
public:
  CaBotHandleV2Node(const rclcpp::NodeOptions & options);
  std::shared_ptr<Handle> handle_ = nullptr;
  std::vector<std::string> button_keys_ = {};
  void eventListener(const std::string& msg);
  void notificationCallback(const std_msgs::msg::Int8::SharedPtr msg);
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  int main();
};

#endif // CABOT_HANDLE_V2_NODE_HPP_
