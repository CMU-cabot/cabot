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
// Clutch Control Nodelet
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>


#include <boost/thread/thread.hpp>

namespace Safety
{
class ClearingTFNodelet : public nodelet::Nodelet
{
public:
  ClearingTFNodelet()
    : targetRate_(20)
  {
    ROS_INFO("NodeletClass Constructor");
  }

  ~ClearingTFNodelet()
  {
    ROS_INFO("NodeletClass Destructor");
  }


private:

  void onInit()
  {
    NODELET_INFO("Clearing TF Nodelet - %s", __FUNCTION__);
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("target_rate", targetRate_);

    boost::thread thread2(&ClearingTFNodelet::tfFakeLoop, this, targetRate_);
  }


  void tfFakeLoop(int publishRate)
  {
    ros::Rate loopRate(publishRate);
    static tf2_ros::TransformBroadcaster broadcaster;

    while (ros::ok())
    {
      double diff = 0.25 / 180.0 * M_PI;
      double yaw = std::rand() * diff / RAND_MAX - diff / 2;
      //double yaw = 0.10 / 180.0 * M_PI;

      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = ros::Time::now();
      // needs to be configualable
      transformStamped.header.frame_id = "lidar_link";
      transformStamped.child_frame_id = "hokuyo_link";
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;

      tf2::Quaternion q_;
      q_.setRPY(0, 0, yaw);

      transformStamped.transform.rotation.x = q_.x();
      transformStamped.transform.rotation.y = q_.y();
      transformStamped.transform.rotation.z = q_.z();
      transformStamped.transform.rotation.w = q_.w();

      broadcaster.sendTransform(transformStamped);

      loopRate.sleep();
    }
  }

  int targetRate_;
}; // class ClearingTFNodelet

PLUGINLIB_EXPORT_CLASS(Safety::ClearingTFNodelet, nodelet::Nodelet)
} // namespace Safety
