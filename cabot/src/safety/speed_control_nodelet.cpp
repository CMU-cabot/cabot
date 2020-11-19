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
	    mapSpeedLimit_(2.0) {
      ROS_INFO("SpeedControlNodeletClass Constructor");
  }

    ~SpeedControlNodelet() {
      ROS_INFO("SpeedControlNodeletClass Destructor");
  }


private:

    void onInit() {
    NODELET_INFO("Speed Control Adapter Nodelet - %s", __FUNCTION__);
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("cmd_vel_input", cmdVelInput_);
    cmdVelSub = private_nh.subscribe(cmdVelInput_, 10,
                                     &SpeedControlNodelet::cmdVelCallback, this);

    private_nh.getParam("cmd_vel_output", cmdVelOutput_);
    cmdVelPub = private_nh.advertise<geometry_msgs::Twist>(cmdVelOutput_, 10);


      if (private_nh.hasParam("speed_input")) {
	private_nh.getParam("speed_input", speedInput_);
	private_nh.getParam("speed_limit", speedLimit_);
	private_nh.getParam("speed_timeout", speedTimeOut_);
	private_nh.getParam("complete_stop", completeStop_);

	for(int index = 0; index < speedInput_.size(); index++) {
	  auto topic = speedInput_[index];
	  boost::function<void (const std_msgs::Float32::ConstPtr&)> callback =
	    [&, index] (const std_msgs::Float32::ConstPtr& input) {
	      //ROS_INFO("receive index=%d, limit=%.2f", index, input->data);
	    speedLimit_[index] = input->data;
	    callbackTime_[index] = ros::Time::now();
	  };
	  auto sub = private_nh.subscribe<std_msgs::Float32::ConstPtr>(topic, 10, callback);
	  speedSubs_.push_back(sub);
	  if (speedLimit_.size() <= index) {
	    speedLimit_.push_back(0);
	  }
	  if (speedTimeOut_.size() <= index) {
	    speedTimeOut_.push_back(0);
	  }
	  if (completeStop_.size() <= index) {
	    completeStop_.push_back(false);
	  }
	  if (callbackTime_.size() <= index) {
	    callbackTime_.push_back(ros::Time::now());
	  }
	  
	  ROS_INFO("Subscribe to %s (index=%d)", topic.c_str(), index);
	}
	timer_ = private_nh.createTimer(ros::Duration(0.1), &SpeedControlNodelet::timerCallback, this);
      } 
      else {
	// backward compatibility
    private_nh.getParam("user_speed_input", userSpeedInput_);
    userSpeedSub = private_nh.subscribe(userSpeedInput_, 10,
                                        &SpeedControlNodelet::userSpeedCallback, this);

    private_nh.getParam("map_speed_input", mapSpeedInput_);
    mapSpeedSub = private_nh.subscribe(mapSpeedInput_, 10,
                                       &SpeedControlNodelet::mapSpeedCallback, this);
  }
    }

    void timerCallback(const ros::TimerEvent&) {
    	for(size_t i = 0; i < callbackTime_.size(); i++) {
    		if (speedTimeOut_[i] <= 0.0)
    			continue;
    		if ((ros::Time::now() - callbackTime_[i]).toSec() > speedTimeOut_[i]) {
    			speedLimit_[i] = 0.0;
    		}
    	}
    }

    void userSpeedCallback(const std_msgs::Float32::ConstPtr& input) {
    userSpeedLimit_ = input->data;
  }

    void mapSpeedCallback(const std_msgs::Float32::ConstPtr& input) {
    mapSpeedLimit_ = input->data;
  }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& input) {
    double l = input->linear.x;
      double r = input->angular.z;

      if (speedLimit_.size() > 0){
	for(int index=0; index < speedLimit_.size(); index++){
	  double limit = speedLimit_[index];	  
	  if (limit < l) {
	    l = limit;
	  }
	  if (l < -limit) {
	    l = -limit;
	  }

	  // if limit equals zero and complete stop is true then angular is also zero
	  if (limit == 0 && completeStop_[index]) {
	    r = 0;
	  }
	}
      } else {
	// backward compatibility
    // limit the input speeed
	if (userSpeedLimit_ < l) {
      l = userSpeedLimit_;
    }
	if (l < -userSpeedLimit_) {
      l = -userSpeedLimit_;
    }

	if (mapSpeedLimit_ < l) {
      l = mapSpeedLimit_;
    }
	if (l < -mapSpeedLimit_) {
      l = -mapSpeedLimit_;
    }
      }

    geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
    cmd_vel->linear.x = l;
      
      if (input->linear.x != 0 && l != 0) {
	// to fit curve, adjust angular speed
	cmd_vel->angular.z = r / input->linear.x * l;
      } else {
	cmd_vel->angular.z = r;
      }
    cmdVelPub.publish(cmd_vel);
  }


  std::string cmdVelInput_;
  std::string cmdVelOutput_;
  std::string userSpeedInput_;
  std::string mapSpeedInput_;

    std::vector<std::string> speedInput_;
    std::vector<ros::Subscriber> speedSubs_;
    std::vector<double> speedLimit_;
    std::vector<ros::Time> callbackTime_;
    std::vector<float> speedTimeOut_;
    std::vector<bool> completeStop_;
    ros::Timer timer_;

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
