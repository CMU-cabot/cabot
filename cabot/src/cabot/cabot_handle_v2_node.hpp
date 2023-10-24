#ifndef CABOT__CABOT_HANDLE_V2_NODE_HPP_
#define CABOT__CABOT_HANDLE_V2_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include "event.hpp"
#include "handle_v2.hpp"

class CaBotHandleV2Node : public rclcpp::Node{
public:
  CaBotHandleV2Node();
  std::shared_ptr<Handle> handle_ = nullptr;
  void eventListener(const std::string& msg);
private:
  std::vector<std::string> button_keys_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr notification_sub_;
  void notificationCallback(const std_msgs::msg::Int8::SharedPtr msg);
};

#endif // CABOT_HANDLE_V2_NODE_HPP_
