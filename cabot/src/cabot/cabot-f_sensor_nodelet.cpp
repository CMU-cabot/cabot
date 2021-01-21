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
// CaBot-F Sensor Nodelet
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>

namespace CaBot
{
  const double D2R = M_PI / 180;

  class CaBotFSensorNodelet : public nodelet::Nodelet
  {
  public:
    CaBotFSensorNodelet()
        : imuInput_("/imu"),
          imuOutput_("/imu_fixed")
    {
      ROS_INFO("NodeletClass Constructor");
    }

    ~CaBotFSensorNodelet()
    {
      ROS_INFO("NodeletClass Destructor");
    }

  private:
    void onInit()
    {
      NODELET_INFO("Cabot-F Sensor Nodelet - %s", __FUNCTION__);
      ros::NodeHandle &private_nh = getPrivateNodeHandle();

      private_nh.getParam("imu_input", imuInput_);
      imuSub = private_nh.subscribe(imuInput_, 10,
                                    &CaBotFSensorNodelet::imuCallback, this);

      private_nh.getParam("imu_output", imuOutput_);
      imuPub = private_nh.advertise<sensor_msgs::Imu>(imuOutput_, 10);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &input)
    {
      sensor_msgs::ImuPtr output(new sensor_msgs::Imu);

      //output = input;
      output->header.stamp = ros::Time::now();
      output->header.frame_id = input->header.frame_id;

      output->orientation = input->orientation;
      output->orientation_covariance = input->orientation_covariance;

      output->angular_velocity.x = input->angular_velocity.x * D2R;
      output->angular_velocity.y = input->angular_velocity.y * D2R;
      output->angular_velocity.z = input->angular_velocity.z * D2R;
      output->angular_velocity_covariance = input->angular_velocity_covariance;

      output->linear_acceleration = input->linear_acceleration;
      output->linear_acceleration_covariance = input->linear_acceleration_covariance;

      imuPub.publish(output);
    }

    std::string imuInput_;
    std::string imuOutput_;

    ros::Publisher imuPub;
    ros::Subscriber imuSub;

  }; // class CaBotFSensorNodelet

  PLUGINLIB_EXPORT_CLASS(CaBot::CaBotFSensorNodelet, nodelet::Nodelet)
} // namespace CaBot
