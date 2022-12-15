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
// Clutch Control Node
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

namespace CaBotSafety
{
class ClearingTFNode : public rclcpp::Node
{
 public:
  ClearingTFNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("clearing_tf_node", options),
      targetRate_(20),
      angle_(0.25),
      sourceFrame_("lidar_link"),
      targetFrame_("hokuyo_link")
  {
    RCLCPP_INFO(get_logger(), "NodeClass Constructor");

    RCLCPP_INFO(get_logger(), "Clearing TF Node - %s", __FUNCTION__);

    targetRate_ = declare_parameter("target_rate", targetRate_);
    angle_ = declare_parameter("angle", angle_);
    sourceFrame_ = declare_parameter("source_frame", sourceFrame_);
    targetFrame_ = declare_parameter("target_frame", targetFrame_);

    thread_ = std::make_shared<std::thread>(&ClearingTFNode::tfFakeLoop, this, targetRate_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  ~ClearingTFNode()
  {
    RCLCPP_INFO(get_logger(), "NodeClass Destructor");
  }

 private:

  void tfFakeLoop(int publishRate)
  {
    rclcpp::Rate loopRate(publishRate);

    while (rclcpp::ok())
    {
      double diff = angle_ / 180.0 * M_PI;
      double yaw = std::rand() * diff / RAND_MAX - diff / 2;
      // double yaw = 0.10 / 180.0 * M_PI;

      geometry_msgs::msg::TransformStamped transformStamped;

      transformStamped.header.stamp = get_clock()->now();
      // needs to be configualable
      transformStamped.header.frame_id = sourceFrame_;
      transformStamped.child_frame_id = targetFrame_;
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;

      tf2::Quaternion q_;
      q_.setRPY(0, 0, yaw);

      transformStamped.transform.rotation.x = q_.x();
      transformStamped.transform.rotation.y = q_.y();
      transformStamped.transform.rotation.z = q_.z();
      transformStamped.transform.rotation.w = q_.w();

      broadcaster_->sendTransform(transformStamped);

      loopRate.sleep();
    }
  }

  int targetRate_;
  float angle_;
  std::string sourceFrame_;
  std::string targetFrame_;
  std::shared_ptr<std::thread> thread_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};  // class ClearingTFNode


}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::ClearingTFNode)
