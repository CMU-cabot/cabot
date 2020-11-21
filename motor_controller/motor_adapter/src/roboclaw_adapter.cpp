/*******************************************************************************
 * Copyright (c) 2019  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

/*
 * RoboClaw Adapter Nodelet
 * 
 * Author: Daisuke Sato <daisukes@cmu.edu>
 */

#include "diff_drive.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <roboclaw_msgs/MotorStatus.h>
#include <roboclaw_msgs/MotorTarget.h>

#include <stdio.h>
#include <math.h>
#include <boost/thread/thread.hpp>

namespace MotorAdapter
{
    const double D2R = M_PI / 180;
    
    class RoboClawNodelet : public nodelet::Nodelet
    {
    public:
	RoboClawNodelet()
	    : diffDrive_(0),
	      cmdVelInput_("/cmd_vel"),
	      motorOutput_("/motor"),
	      encoderInput_("/encoder"),
	      odomOutput_("/odom"),
	      
	      lastCmdVel_(0),
	      targetSpdLinear_(0),
	      targetSpdTurn_(0),
	      currentSpdLinear_(0),
	      lastOdomTime_(0),
	      
	      targetRate_(20),
	      maxAcc_(0.5),
	      
	      bias_(0),
	      wheel_diameter_(0),
	      count_per_rotate_(0)
	{
	    ROS_INFO("NodeletClass Constructor");
	}
	
	~RoboClawNodelet()
	{
	    ROS_INFO("NodeletClass Destructor");
	}
	
	
    private:
	
	void onInit()
	{
	    NODELET_INFO("MotorAdapter RoboClawNodelet - %s", __FUNCTION__);
	    ros::NodeHandle& private_nh = getPrivateNodeHandle();

	    private_nh.getParam("encoder_topic", encoderInput_);
	    encoderSub = private_nh.subscribe(encoderInput_, 10,
					      &RoboClawNodelet::encoderCallback, this);
	    private_nh.getParam("odom_topic", odomOutput_);
	    odomPub = private_nh.advertise<nav_msgs::Odometry>(odomOutput_, 10);
	
	    private_nh.getParam("cmd_vel_topic", cmdVelInput_);
	    cmdVelSub = private_nh.subscribe(cmdVelInput_, 10,
					     &RoboClawNodelet::cmdVelCallback, this);

	    private_nh.getParam("max_acc", maxAcc_);
	    private_nh.getParam("target_rate", targetRate_);

	    private_nh.getParam("bias", bias_);
	    diffDrive_.set(bias_);
    
	    boost::thread thread(&RoboClawNodelet::cmdVelLoop, this, targetRate_);
	}

	void cmdVelLoop(int publishRate) {
	    ros::Rate loopRate(publishRate);

	    ros::NodeHandle& private_nh = getPrivateNodeHandle();
	    private_nh.getParam("motor_topic", motorOutput_);
	    motorPub = private_nh.advertise<roboclaw_msgs::MotorTarget>(motorOutput_, 10);

	    double minimumStep = maxAcc_ / publishRate;
	    
	    while (ros::ok()) {
		roboclaw_msgs::MotorTargetPtr target(new roboclaw_msgs::MotorTarget);

		double targetL = targetSpdLinear_;
		double targetT = targetSpdTurn_;

		// change linear speed by maximum acc rate
		double lDiff = targetL - currentSpdLinear_;
		if (fabs(lDiff) < minimumStep) {
		    currentSpdLinear_ = targetL;
		} else {
		    currentSpdLinear_ += minimumStep * lDiff / fabs(lDiff);
		}

		// adjust angular speed
		target->spdLeft = currentSpdLinear_ - targetT;
		target->spdRight = currentSpdLinear_ + targetT;

		motorPub.publish(target);

		loopRate.sleep();
	    }
	}

	void encoderCallback(const roboclaw_msgs::MotorStatus::ConstPtr& input)
	{
	    diffDrive_.update(input->distLeft,
			      input->distRight,
			      input->header.stamp.toSec());
	    Pose& pose = diffDrive_.pose();

	    //ROS_INFO("input %d, %d, pose %f, %f", input->distLeft_c, input->distRight_c, pose.x, pose.y);

	    nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

	    if (input->header.stamp.toSec() - lastOdomTime_ < 0.007) {
		return;
	    }
		   
	    lastOdomTime_ = input->header.stamp.toSec();
	    odom->header.stamp = input->header.stamp;
	    odom->header.frame_id = "odom";
	    odom->child_frame_id = "base_footprint";
	    
	    odom->pose.pose.position.x = pose.x;
	    odom->pose.pose.position.y = pose.y;
	    tf2::Quaternion q;
	    q.setRPY(0, 0, pose.a);
	    q.normalize();
	    odom->pose.pose.orientation.x = q[0];
	    odom->pose.pose.orientation.y = q[1];
	    odom->pose.pose.orientation.z = q[2];
	    odom->pose.pose.orientation.w = q[3];
	    odom->pose.covariance[0] = 0.1;
	    odom->pose.covariance[7] = 0.1;
	    odom->pose.covariance[14] = 0.1;
	    odom->pose.covariance[21] = 0.2;
	    odom->pose.covariance[28] = 0.2;
	    odom->pose.covariance[35] = 0.2;

	    LRdouble& vel = diffDrive_.velocity();
	    odom->twist.twist.linear.x = vel.l;
	    odom->twist.twist.angular.z = vel.r;
	    odom->twist.covariance[0] = 0.1;
	    odom->twist.covariance[7] = 0.1;
	    odom->twist.covariance[14] = 0.1;
	    odom->twist.covariance[21] = 0.2;
	    odom->twist.covariance[28] = 0.2;
	    odom->twist.covariance[35] = 0.2;
	    odomPub.publish(odom);
	}

	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& input)
	{
	    double now = ros::Time::now().toSec();
	    if (lastCmdVel_ > 0 && now - lastCmdVel_ < 0.2) {
		//return;
	    }
	    lastCmdVel_ = now;
	    double l = input->linear.x;
	    double w = input->angular.z;
	    
	    targetSpdLinear_ = l;
	    targetSpdTurn_ = bias_ / 2.0 * w;
	}

	MotorAdapter::DiffDrive diffDrive_;

	std::string cmdVelInput_;
	std::string motorOutput_;
	std::string encoderInput_;
	std::string odomOutput_;

	double lastCmdVel_;
	double targetSpdLinear_;
	double targetSpdTurn_;
	double currentSpdLinear_;
	double lastOdomTime_;

	int targetRate_;
	double maxAcc_;
	
	double bias_;
	double wheel_diameter_;
	double count_per_rotate_;
    
	ros::Publisher motorPub;
	ros::Publisher odomPub;

	ros::Subscriber encoderSub;
	ros::Subscriber cmdVelSub;
	
    }; // class RoboClawNodelet

    PLUGINLIB_EXPORT_CLASS(MotorAdapter::RoboClawNodelet, nodelet::Nodelet)
} // namespace MotorAdapter
