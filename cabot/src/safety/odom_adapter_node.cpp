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
// Odom Adapter
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace CaBotSafety
{
class OdomAdapterNode : public rclcpp::Node
{
public:
  explicit OdomAdapterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("odom_adapter_node", options),
    odomInput_("odom_raw"),
    odomOutput_("odom_adapter"),
    odomFrame_("odom"),
    baseFrame_("base_footprint"),
    offsetFrame_("base_control_shift"),
    cmdVelInput_("cmd_vel_raw"),
    cmdVelOutput_("cmd_vel_adapter"),
    lastCmdVel_(0, 0, get_clock()->get_clock_type()),
    offset_(0),
    targetRate_(20),
    publish_tf_(true),
    max_speed_(1.0),
    x_(0),
    y_(0),
    q_(0, 0, 0, 1)
  {
    RCLCPP_INFO(get_logger(), "OdomAdapterNodeClass Constructor");
    tfBuffer = new tf2_ros::Buffer(get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer, this);

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(get_logger(), "Cabot OdomAdapterNode - %s", __FUNCTION__);

    odomFrame_ = declare_parameter("odom_frame", odomFrame_);
    baseFrame_ = declare_parameter("base_frame", baseFrame_);
    offsetFrame_ = declare_parameter("offset_frame", offsetFrame_);

    odomOutput_ = declare_parameter("odom_output", odomOutput_);
    odomPub = create_publisher<nav_msgs::msg::Odometry>(odomOutput_, 10);

    odomInput_ = declare_parameter("odom_input", odomInput_);
    odomSub = create_subscription<nav_msgs::msg::Odometry>(
      odomInput_, 10,
      std::bind(&OdomAdapterNode::odomCallback, this, std::placeholders::_1));

    cmdVelOutput_ = declare_parameter("cmd_vel_output", cmdVelOutput_);
    cmdVelPub = create_publisher<geometry_msgs::msg::Twist>(cmdVelOutput_, 10);

    cmdVelInput_ = declare_parameter("cmd_vel_input", cmdVelInput_);
    cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
      cmdVelInput_, 10,
      std::bind(&OdomAdapterNode::cmdVelCallback, this, std::placeholders::_1));

    publish_tf_ = declare_parameter("publish_tf", publish_tf_);
    targetRate_ = declare_parameter("target_rate", targetRate_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / targetRate_),
      std::bind(&OdomAdapterNode::tfLoop, this));

    max_speed_ = declare_parameter("max_speed", max_speed_);
  }

  ~OdomAdapterNode()
  {
    RCLCPP_INFO(get_logger(), "OdomAdapterNodeClass Destructor");
  }

private:
  void tfLoop()
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = get_clock()->now();
    transformStamped.header.frame_id = odomFrame_;
    transformStamped.child_frame_id = baseFrame_;
    transformStamped.transform.translation.x = x_;
    transformStamped.transform.translation.y = y_;
    transformStamped.transform.translation.z = 0.0;

    {
      std::lock_guard<std::mutex> lock(thread_sync_);
      transformStamped.transform.rotation.x = q_.x();
      transformStamped.transform.rotation.y = q_.y();
      transformStamped.transform.rotation.z = q_.z();
      transformStamped.transform.rotation.w = q_.w();
    }
    if (publish_tf_) {
      broadcaster_->sendTransform(transformStamped);
    }

    // update offset
    tf2::Transform offset_tf(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
    geometry_msgs::msg::TransformStamped offset_tf_msg;
    try {
      offset_tf_msg = tfBuffer->lookupTransform(
        baseFrame_, offsetFrame_, rclcpp::Time(0),
        rclcpp::Duration(std::chrono::duration<double>(1.0)));
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "%s", ex.what());
      return;
    }
    offset_ = offset_tf_msg.transform.translation.y;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = input->header.frame_id;
    odom.child_frame_id = input->child_frame_id;

    {
      std::lock_guard<std::mutex> lock(thread_sync_);
      tf2::convert(input->pose.pose.orientation, q_);
    }

    tf2::Matrix3x3 m(q_);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double c = cos(yaw);
    double s = sin(yaw);

    tf2::Transform t(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(offset_ * s, -offset_ * c, 0));

    geometry_msgs::msg::Transform gt;
    tf2::convert(t, gt);
    geometry_msgs::msg::TransformStamped gts;
    gts.transform = gt;

    odom.pose = input->pose;
    tf2::doTransform(input->pose.pose, odom.pose.pose, gts);

    if (std::isnan(odom.pose.pose.position.x) || std::isnan(odom.pose.pose.position.y)) {
      // reject the value
      return;
    }

    x_ = odom.pose.pose.position.x;
    y_ = odom.pose.pose.position.y;

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[21] = 0.2;
    odom.pose.covariance[28] = 0.2;
    odom.pose.covariance[35] = 0.2;

    odom.twist = input->twist;
    double l = input->twist.twist.linear.x;
    double w = input->twist.twist.angular.z;
    double l2 = l + w * offset_;
    odom.twist.twist.linear.x = l2;

    odomPub->publish(odom);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input)
  {
    rclcpp::Time now = get_clock()->now();
    if (lastCmdVel_ > rclcpp::Time(0, 0, get_clock()->get_clock_type()) && now - lastCmdVel_ < rclcpp::Duration(std::chrono::duration<double>(0.2))) {
      // return;
    }
    lastCmdVel_ = now;
    double l = input->linear.x;
    double w = input->angular.z;
    double l2 = l - w * offset_;

    if (l2 > max_speed_) {
      w = w / l2 * max_speed_;
      l2 = max_speed_;
    }

    RCLCPP_INFO(get_logger(), "offset=%.2f, (%.2f, %.2f) (%.2f, %.2f)", offset_, input->linear.x, input->angular.z, l2, w);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = l2;
    cmd_vel.angular.z = w;
    cmdVelPub->publish(cmd_vel);
  }

  std::mutex thread_sync_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string odomInput_;
  std::string odomOutput_;
  std::string odomFrame_;
  std::string baseFrame_;
  std::string offsetFrame_;
  std::string cmdVelInput_;
  std::string cmdVelOutput_;

  rclcpp::Time lastCmdVel_;
  double offset_;
  int targetRate_;
  bool publish_tf_;
  double max_speed_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;

  // internal state
  double x_;
  double y_;
  tf2::Quaternion q_;

  tf2_ros::TransformListener * tfListener;
  tf2_ros::Buffer * tfBuffer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};  // class OdomAdapterNode

}  // namespace CaBotSafety
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSafety::OdomAdapterNode)
