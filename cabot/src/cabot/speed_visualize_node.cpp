// Copyright (c) 2020, 2022  Carnegie Mellon University
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
//
// Speed Visualize Node
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

namespace CaBot
{
class SpeedVisualizeNode : public rclcpp::Node
{
public:
  explicit SpeedVisualizeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("speed_visualize_node", options),
    cmdVelInput_("/cmd_vel"),
    visOutput_("/speed_vis")
  {
    RCLCPP_INFO(get_logger(), "NodeClass Constructor");
    RCLCPP_INFO(get_logger(), "Speed Visualize Node - %s", __FUNCTION__);


    cmdVelInput_ = declare_parameter("cmd_vel_topic", cmdVelInput_);
    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
      cmdVelInput_, 10, std::bind(&SpeedVisualizeNode::cmdVelCallback, this, _1));

    visOutput_ = declare_parameter("visualize_topic", visOutput_);
    visPub = create_publisher<visualization_msgs::msg::MarkerArray>(visOutput_, 1);
  }

  ~SpeedVisualizeNode()
  {
    RCLCPP_INFO(get_logger(), "NodeClass Destructor");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint";
    // marker.header.stamp = get_clock()->now();
    marker.ns = "speed";
    marker.id = 99999;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f m/s", input->linear.x);
    marker.text = buff;

    array.markers.push_back(marker);
    visPub->publish(array);
  }

  std::string cmdVelInput_;
  std::string visOutput_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visPub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
};  // class SpeedVisualizeNode

}  // namespace CaBot
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBot::SpeedVisualizeNode)
