// Copyright 2020 Carnegie Mellon University
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
// Clutch Adapter Nodelet
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

namespace Safety
{
class SpeedControlNodelet : public nodelet::Nodelet
{
public:
  SpeedControlNodelet()
    : cmdVelInput_("/cmd_vel"),
      cmdVelOutput_("/cmd_vel_limit"),
      userSpeedInput_("/user_speed"),
      mapSpeedInput_("/map_speed"),
      userSpeedLimit_(2.0),
      mapSpeedLimit_(2.0)
  {
    ROS_INFO("NodeletClass Constructor");
  }

  ~SpeedControlNodelet()
  {
    ROS_INFO("NodeletClass Destructor");
  }


private:

  void onInit()
  {
    NODELET_INFO("Speed Control Adapter Nodelet - %s", __FUNCTION__);
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("cmd_vel_input", cmdVelInput_);
    cmdVelSub = private_nh.subscribe(cmdVelInput_, 10,
                                     &SpeedControlNodelet::cmdVelCallback, this);

    private_nh.getParam("cmd_vel_output", cmdVelOutput_);
    cmdVelPub = private_nh.advertise<geometry_msgs::Twist>(cmdVelOutput_, 10);

    private_nh.getParam("user_speed_input", userSpeedInput_);
    userSpeedSub = private_nh.subscribe(userSpeedInput_, 10,
                                        &SpeedControlNodelet::userSpeedCallback, this);

    private_nh.getParam("map_speed_input", mapSpeedInput_);
    mapSpeedSub = private_nh.subscribe(mapSpeedInput_, 10,
                                       &SpeedControlNodelet::mapSpeedCallback, this);
  }

  void userSpeedCallback(const std_msgs::Float32::ConstPtr& input)
  {
    userSpeedLimit_ = input->data;
  }

  void mapSpeedCallback(const std_msgs::Float32::ConstPtr& input)
  {
    mapSpeedLimit_ = input->data;
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& input)
  {
    double l = input->linear.x;
    // limit the input speeed
    if (userSpeedLimit_ < l)
    {
      l = userSpeedLimit_;
    }
    if (l < -userSpeedLimit_)
    {
      l = -userSpeedLimit_;
    }

    if (mapSpeedLimit_ < l)
    {
      l = mapSpeedLimit_;
    }
    if (l < -mapSpeedLimit_)
    {
      l = -mapSpeedLimit_;
    }

    double w = input->angular.z;

    geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
    cmd_vel->linear.x = l;
    cmd_vel->angular.z = w;
    cmdVelPub.publish(cmd_vel);
  }


  std::string cmdVelInput_;
  std::string cmdVelOutput_;
  std::string userSpeedInput_;
  std::string mapSpeedInput_;

  bool clutchState_;
  double userSpeedLimit_;
  double mapSpeedLimit_;

  ros::Publisher cmdVelPub;

  ros::Subscriber cmdVelSub;
  ros::Subscriber userSpeedSub;
  ros::Subscriber mapSpeedSub;

}; // class SpeedControlNodelet

PLUGINLIB_EXPORT_CLASS(Safety::SpeedControlNodelet, nodelet::Nodelet)
} // namespace Safety
