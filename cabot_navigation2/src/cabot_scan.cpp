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

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace cabot_navigation2
{

  class CabotScan : public rclcpp::Node
  {
  public:
    CabotScan() : Node("cabot_scan"), scan_out_frame_("scan_out")
    {
      this->declare_parameter("scan_topic", rclcpp::ParameterValue(std::string("scan")));
      std::string scan_topic = "scan";
      if (this->get_parameter<std::string>("scan_topic", scan_topic))
      {
        RCLCPP_INFO(this->get_logger(), "scan_topic=%s", scan_topic.c_str());
      }

      this->declare_parameter("scan_out_topic", rclcpp::ParameterValue(std::string("scan_out")));
      std::string scan_out_topic = "scan_out";
      if (this->get_parameter<std::string>("scan_out_topic", scan_out_topic))
      {
        RCLCPP_INFO(this->get_logger(), "scan_out_topic=%s", scan_out_topic.c_str());
      }

      this->declare_parameter("scan_out_frame", rclcpp::ParameterValue(std::string("scan_out")));
      if (this->get_parameter<std::string>("scan_out_frame", scan_out_frame_))
      {
        RCLCPP_INFO(this->get_logger(), "scan_out_frame=%s", scan_out_frame_.c_str());
      }

      sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          scan_topic, 10, std::bind(&CabotScan::scan_callback, this, std::placeholders::_1));
      pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_out_topic, 10);
    }

  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      sensor_msgs::msg::LaserScan copied = *msg;
      copied.header.frame_id = scan_out_frame_;
      pub_->publish(copied);
    }

    std::string scan_out_frame_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  };

} // namespace cabot_navigation2

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cabot_navigation2::CabotScan>());
  rclcpp::shutdown();
  return 0;
}
