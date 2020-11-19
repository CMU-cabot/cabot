// Copyright (c) 2020 Carnegie Mellon University
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

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <stdio.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/thread/thread.hpp>

namespace Safety
{
class OdomAdapterNodelet : public nodelet::Nodelet
{
public:
  OdomAdapterNodelet()
    : odomInput_("odom_raw"),
      odomOutput_("odom_adapter"),
      odomFrame_("odom"),
      baseFrame_("base_footprint"),
      cmdVelInput_("cmd_vel_raw"),
      cmdVelOutput_("cmd_vel_adapter"),

      lastCmdVel_(0),
      offset_(0),
      bias_(0),
      targetRate_(20),
      x_(0),
      y_(0),
      q_(0, 0, 0, 1)
  {
    ROS_INFO("NodeletClass Constructor");
  }

  ~OdomAdapterNodelet()
  {
    ROS_INFO("NodeletClass Destructor");
  }

private:

  void onInit()
  {
    NODELET_INFO("Cabot OdomAdapterNodelet - %s", __FUNCTION__);
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("offset", offset_);
    private_nh.getParam("bias", bias_);
    private_nh.getParam("odom_frame", odomFrame_);
    private_nh.getParam("base_frame", baseFrame_);

    private_nh.getParam("odom_output", odomOutput_);
    odomPub = private_nh.advertise<nav_msgs::Odometry>(odomOutput_, 10);

    private_nh.getParam("odom_input", odomInput_);
    odomSub = private_nh.subscribe(odomInput_, 10,
                                   &OdomAdapterNodelet::odomCallback, this);

    private_nh.getParam("cmd_vel_output", cmdVelOutput_);
    cmdVelPub = private_nh.advertise<geometry_msgs::Twist>(cmdVelOutput_, 10);

    private_nh.getParam("cmd_vel_input", cmdVelInput_);
    cmdVelSub = private_nh.subscribe(cmdVelInput_, 10,
                                     &OdomAdapterNodelet::cmdVelCallback, this);

    private_nh.getParam("target_rate", targetRate_);
    boost::thread thread(&OdomAdapterNodelet::tfLoop, this, targetRate_);
  }

  void tfLoop(int publishRate)
  {
    ros::Rate loopRate(publishRate);
    static tf2_ros::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = odomFrame_;
      transformStamped.child_frame_id = baseFrame_;
      transformStamped.transform.translation.x = x_;
      transformStamped.transform.translation.y = y_;
      transformStamped.transform.translation.z = 0.0;

      boost::mutex::scoped_lock lock(thread_sync_);
      transformStamped.transform.rotation.x = q_.x();
      transformStamped.transform.rotation.y = q_.y();
      transformStamped.transform.rotation.z = q_.z();
      transformStamped.transform.rotation.w = q_.w();
      lock.unlock();
      broadcaster.sendTransform(transformStamped);

      loopRate.sleep();
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& input)
  {
    nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

    odom->header.stamp = ros::Time::now();
    odom->header.frame_id = input->header.frame_id;
    odom->child_frame_id = input->child_frame_id;

    boost::mutex::scoped_lock lock(thread_sync_);
    tf2::convert(input->pose.pose.orientation, q_);
    lock.unlock();

    tf2::Matrix3x3 m(q_);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double c = cos(yaw);
    double s = sin(yaw);

    tf2::Transform t(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(offset_ * s, -offset_ * c, 0));

    geometry_msgs::Transform gt;
    tf2::convert(t, gt);
    geometry_msgs::TransformStamped gts;
    gts.transform = gt;

    odom->pose = input->pose;
    tf2::doTransform(input->pose.pose, odom->pose.pose, gts);

    if (std::isnan(odom->pose.pose.position.x) || std::isnan(odom->pose.pose.position.y))
    {
      //reject the value
      return;
    }

    x_ = odom->pose.pose.position.x;
    y_ = odom->pose.pose.position.y;

    odom->pose.covariance[0] = 0.1;
    odom->pose.covariance[7] = 0.1;
    odom->pose.covariance[14] = 0.1;
    odom->pose.covariance[21] = 0.2;
    odom->pose.covariance[28] = 0.2;
    odom->pose.covariance[35] = 0.2;

    odom->twist = input->twist;
    double l = input->twist.twist.linear.x;
    double w = input->twist.twist.angular.z;
    double l2 = l + w * offset_;
    odom->twist.twist.linear.x = l2;

    odomPub.publish(odom);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& input)
  {
    double now = ros::Time::now().toSec();
    if (lastCmdVel_ > 0 && now - lastCmdVel_ < 0.2)
    {
      //return;
    }
    lastCmdVel_ = now;
    double l = input->linear.x;
    double w = input->angular.z;
    double l2 = l - w * offset_;

    geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
    cmd_vel->linear.x = l2;
    cmd_vel->angular.z = input->angular.z;
    cmdVelPub.publish(cmd_vel);
  }

  boost::mutex thread_sync_;

  std::string odomInput_;
  std::string odomOutput_;
  std::string odomFrame_;
  std::string baseFrame_;
  std::string cmdVelInput_;
  std::string cmdVelOutput_;

  double lastCmdVel_;
  double offset_;
  double bias_;
  int targetRate_;

  ros::Publisher odomPub;
  ros::Publisher cmdVelPub;

  ros::Subscriber odomSub;
  ros::Subscriber cmdVelSub;

  // internal state
  double x_;
  double y_;
  tf2::Quaternion q_;


}; // class OdomAdapterNodelet

PLUGINLIB_EXPORT_CLASS(Safety::OdomAdapterNodelet, nodelet::Nodelet)
} // namespace Safety
