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
// CaBot-E Sensor Nodelet
// Author: Daisuke Sato <daisukes@cmu.edu>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>

namespace CaBot
{
  class CaBotESensorNodelet : public nodelet::Nodelet
  {
  public:
    CaBotESensorNodelet()
        : sensorInput_("/wrench"),
          eventOutput_("/event")
    {
      ROS_INFO("NodeletClass Constructor");
    }

    ~CaBotESensorNodelet()
    {
      ROS_INFO("NodeletClass Destructor");
    }

  private:
    void onInit()
    {
      NODELET_INFO("Cabot-E Sensor Nodelet - %s", __FUNCTION__);
      ros::NodeHandle &private_nh = getPrivateNodeHandle();

      private_nh.getParam("sensor_topic", sensorInput_);
      sensorSub = private_nh.subscribe(sensorInput_, 10,
                                       &CaBotESensorNodelet::sensorCallback, this);

      private_nh.getParam("event_topic", eventOutput_);
      eventPub = private_nh.advertise<std_msgs::String>(eventOutput_, 10);

      sensorPub = private_nh.advertise<geometry_msgs::WrenchStamped>("wrench_norm", 10);
    }

    void sensorCallback(const geometry_msgs::WrenchStamped::ConstPtr &input)
    {
      geometry_msgs::WrenchStampedPtr output(new geometry_msgs::WrenchStamped);

      static double f = 0, t = 0;
      static int c = 0;
      c++;
      double force = input->wrench.force.x;
      double torque = input->wrench.torque.x;
      f += force;
      t += torque;

      const double sigma = 7;

      // left handed handle
      // torque bigger -> right
      // torque smaller -> left
      const double torque_bias = 685.8;
      const double torque_std = 6.2 * sigma;
      const double torque_max = 1023;
      const double torque_min = 210;

      // force bigger -> pull
      // force smaller -> push
      const double force_bias = 599.7;
      const double force_std = 5.7 * sigma;
      const double force_max = 1023;
      const double force_min = 0;
      const double steps = 32;
      const double force_p_step = (force_max - (force_bias + force_std)) / steps;
      const double force_n_step = (force_bias - force_std - force_min) / steps;
      const double torque_p_step = (torque_max - (torque_bias + torque_std)) / steps;
      const double torque_n_step = (torque_bias - torque_std - torque_min) / steps;

      double findex = 0;
      double tindex = 0;
      if (force > force_bias + force_std)
      {
        findex = -((force - (force_bias + force_std)) / force_p_step + 1);
      }
      if (force < force_bias - force_std)
      {
        findex = -((force - (force_bias - force_std)) / force_n_step - 1);
      }
      if (torque > torque_bias + torque_std)
      {
        tindex = ((torque - (torque_bias + torque_std)) / torque_p_step) + 1;
      }
      if (torque < torque_bias - torque_std)
      {
        tindex = ((torque - (torque_bias - torque_std)) / torque_n_step) - 1;
      }

      if (findex != 0 || tindex != 0)
      {
        printf("%6.3f,%6.3f,%6f,%6f\n", findex, tindex, force, torque);
      }
      output->header = input->header;
      output->wrench.force.x = findex;
      output->wrench.torque.x = tindex;

      sensorPub.publish(output);
    }

    std::string sensorInput_;
    std::string eventOutput_;

    ros::Publisher eventPub;
    ros::Publisher sensorPub;
    ros::Subscriber sensorSub;

  }; // class CaBotESensorNodelet

  PLUGINLIB_EXPORT_CLASS(CaBot::CaBotESensorNodelet, nodelet::Nodelet)
} // namespace CaBot
