/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#ifndef CABOT__CABOT_HANDLE_V2_NODE_HPP_
#define CABOT__CABOT_HANDLE_V2_NODE_HPP_

#include <unistd.h>
#include <execinfo.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <stdexcept>
#include <iostream>
#include "event.hpp"
#include "handle_v2.hpp"

class Handle;

class CaBotHandleV2Node : public rclcpp::Node
{
public:
  explicit CaBotHandleV2Node(const rclcpp::NodeOptions & options);
  std::shared_ptr<Handle> handle_ = nullptr;
  std::vector<std::string> button_keys_ = {};
  void eventListener(const std::map<std::string, std::string> & msg);
  void notificationCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void printStackTrace();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;

private:
  int main();
};

#endif  // CABOT__CABOT_HANDLE_V2_NODE_HPP_
