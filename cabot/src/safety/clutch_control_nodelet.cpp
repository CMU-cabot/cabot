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
//
// Clutch Control Nodelet
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

namespace Safety
{
class ClutchControlNodelet : public nodelet::Nodelet
{
 public:
  ClutchControlNodelet()
      : cmdVelInput_("/cmd_vel"),
        cmdVelOutput_("/cmd_vel_clutch"),
        odomInput_("/odom"),
        odomOutput_("/odom_clutch"),
        clutchInput_("/clutch"),
        clutchState_(false)
  {
    ROS_INFO("NodeletClass Constructor");
  }

  ~ClutchControlNodelet()
  {
    ROS_INFO("NodeletClass Destructor");
  }

 private:
  void onInit()
  {
    NODELET_INFO("Safety Clutch Adapter Nodelet - %s", __FUNCTION__);
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.getParam("odom_input", odomInput_);
    odomSub = private_nh.subscribe(odomInput_, 10,
                                   &ClutchControlNodelet::odomCallback, this);

    private_nh.getParam("odom_output", odomOutput_);
    odomPub = private_nh.advertise<nav_msgs::Odometry>(odomOutput_, 10);

    private_nh.getParam("cmd_vel_input", cmdVelInput_);
    cmdVelSub = private_nh.subscribe(cmdVelInput_, 10,
                                     &ClutchControlNodelet::cmdVelCallback, this);

    private_nh.getParam("cmd_vel_output", cmdVelOutput_);
    cmdVelPub = private_nh.advertise<geometry_msgs::Twist>(cmdVelOutput_, 10);

    private_nh.getParam("clutch_topic", clutchInput_);
    clutchSub = private_nh.subscribe(clutchInput_, 10,
                                     &ClutchControlNodelet::clutchCallback, this);
  }

  void clutchCallback(const std_msgs::Bool::ConstPtr &input)
  {
    clutchState_ = input->data;
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &input)
  {
    if (clutchState_)
    {
      cmdVelPub.publish(input);
    }
    else
    {
      // geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
      // cmd_vel->linear.x = 0;
      // cmd_vel->angular.z = 0;
      // cmdVelPub.publish(cmd_vel);
      cmdVelPub.publish(input);
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &input)
  {
    if (clutchState_)
    {
      odomPub.publish(input);
    }
    else
    {
      nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
      odomPub.publish(odom);
    }
  }

  std::string cmdVelInput_;
  std::string cmdVelOutput_;
  std::string odomInput_;
  std::string odomOutput_;
  std::string clutchInput_;

  bool clutchState_;

  ros::Publisher odomPub;
  ros::Publisher cmdVelPub;

  ros::Subscriber odomSub;
  ros::Subscriber cmdVelSub;
  ros::Subscriber clutchSub;
};  // class ClutchControlNodelet

PLUGINLIB_EXPORT_CLASS(Safety::ClutchControlNodelet, nodelet::Nodelet)
}  // namespace Safety
