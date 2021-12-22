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

// tf speed control
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <boost/thread/thread.hpp>

namespace Safety
{
class TFSpeedControlNodelet : public nodelet::Nodelet
{
 public:
  std::string limit_topic_;

  int check_rate_;
  std::string map_frame_;
  std::string robot_base_frame_;

  ros::Publisher limit_pub_;
  tf2_ros::TransformListener *tfListener;
  tf2_ros::Buffer tfBuffer;

  TFSpeedControlNodelet()
      : limit_topic_("tf_limit"),
        check_rate_(1),
        map_frame_("map"),
        robot_base_frame_("base_footprint")
  {
    NODELET_INFO("TFSpeedControlNodeletClass Constructor");
    tfListener = new tf2_ros::TransformListener(tfBuffer);
  }

  ~TFSpeedControlNodelet()
  {
    NODELET_INFO("TFSpeedControlNodeletClass Destructor");
  }

 private:
  void onInit()
  {
    NODELET_INFO("tf speed control - %s", __FUNCTION__);
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.getParam("limit_topic", limit_topic_);
    limit_pub_ = private_nh.advertise<std_msgs::Float32>(limit_topic_, 100);

    boost::thread thread2(&TFSpeedControlNodelet::tfCheckLoop, this, check_rate_);
  }

  void tfCheckLoop(int checkRate)
  {
    ros::Rate loopRate(checkRate);

    while (ros::ok())
    {
      double speed_limit = 1.0;

      try
      {
        geometry_msgs::TransformStamped transform_msg = tfBuffer.lookupTransform(robot_base_frame_, map_frame_,
                                                                                 ros::Time(0), ros::Duration(1.0));
        ROS_INFO("TFSpeedControl, lookup transform success");
      }
      catch (tf2::TransformException &ex)
      {
        speed_limit = 0.0;
        ROS_INFO("TFSpeedControl, lookup transform fail");
      }

      std_msgs::Float32 msg;
      msg.data = speed_limit;
      limit_pub_.publish(msg);

      loopRate.sleep();
    }
  }
};  // class TFSpeedControlNodelet

PLUGINLIB_EXPORT_CLASS(Safety::TFSpeedControlNodelet, nodelet::Nodelet)
}  // namespace Safety
