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
// Speed Visualize Nodelet
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <cabot_msgs/ArduinoSensors.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <odriver_msgs/MotorStatus.h>
#include <odriver_msgs/MotorTarget.h>

#include <stdio.h>
#include <math.h>
#include <boost/thread/thread.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace CaBot
{
class SpeedVisualizeNodelet : public nodelet::Nodelet
{
 public:
  SpeedVisualizeNodelet()
      : cmdVelInput_("/cmd_vel"),
        visOutput_("/speed_vis")
  {
    ROS_INFO("NodeletClass Constructor");
  }

  ~SpeedVisualizeNodelet()
  {
    ROS_INFO("NodeletClass Destructor");
  }

 private:
  void onInit()
  {
    NODELET_INFO("Speed Visualize Nodelet - %s", __FUNCTION__);
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.getParam("cmd_vel_topic", cmdVelInput_);
    cmdVelSub = private_nh.subscribe(cmdVelInput_, 10,
                                     &SpeedVisualizeNodelet::cmdVelCallback, this);

    private_nh.getParam("visualize_topic", visOutput_);
    visPub = private_nh.advertise<visualization_msgs::MarkerArray>(visOutput_, 1);
  }

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &input)
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    // marker.header.stamp = ros::Time::now();
    marker.ns = "speed";
    marker.id = 99999;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
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
    visPub.publish(array);
  }

  std::string cmdVelInput_;
  std::string visOutput_;

  ros::Publisher visPub;
  ros::Subscriber cmdVelSub;
};  // class SpeedVisualizeNodelet

PLUGINLIB_EXPORT_CLASS(CaBot::SpeedVisualizeNodelet, nodelet::Nodelet)
}  // namespace CaBot
