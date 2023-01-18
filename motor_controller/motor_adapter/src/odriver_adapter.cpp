/*******************************************************************************
 * Copyright (c) 2019, 2022  Carnegie Mellon University
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
 * Odriver motor controller adapter
 *
 * Author: Daisuke Sato <daisukes@cmu.edu>
 */

#include "odriver_adapter.hpp"

#include <stdio.h>
#include <math.h>
#include <memory>
//#include <boost/thread/thread.hpp>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace MotorAdapter
{
const double D2R = M_PI / 180;

ODriverNode::ODriverNode(rclcpp::NodeOptions options)
    : rclcpp::Node("odriver_node", options),
      diffDrive_(0),
      cmdVelInput_("/cmd_vel"),
      motorOutput_("/motor"),
      encoderInput_("/encoder"),
      odomOutput_("/odom"),
      pauseControlInput_("/pause_control"),

      lastCmdVelTime_(0, 0, get_clock()->get_clock_type()),
      targetSpdLinear_(0),
      targetSpdTurn_(0),
      currentSpdLinear_(0),
      lastOdomTime_(0, 0, get_clock()->get_clock_type()),

      targetRate_(20),
      maxAcc_(0.5),

      bias_(0),
      wheel_diameter_(0),
      count_per_rotate_(0),

      measuredSpdLinear_(0),
      measuredSpdTurn_(0),
      integral_linear_(0),
      integral_turn_(0),

      lastImuTime_(0, 0, get_clock()->get_clock_type()),
      lastImuAngularVelocity_(0),
      imuTimeTolerance_(50ms),
      pause_control_counter_(0)
{
  RCLCPP_INFO(get_logger(), "ODriverNode Constructor");

  bias_ = declare_parameter("bias", bias_);
  diffDrive_.set(bias_);

  encoderInput_ = declare_parameter("encoder_topic", encoderInput_);
  encoderSub = create_subscription<odriver_msgs::msg::MotorStatus>(
      encoderInput_, 10, std::bind(&ODriverNode::encoderCallback, this, _1));

  odomOutput_ = declare_parameter("odom_topic", odomOutput_);
  odomPub = create_publisher<nav_msgs::msg::Odometry>(odomOutput_, 10);

  cmdVelInput_ = declare_parameter("cmd_vel_topic", cmdVelInput_);
  cmdVelSub = create_subscription<geometry_msgs::msg::Twist>
              (cmdVelInput_, 10, std::bind(&ODriverNode::cmdVelCallback, this, _1));

  pauseControlInput_ = declare_parameter("pause_control_topic", pauseControlInput_);
  pauseControlSub = create_subscription<std_msgs::msg::Bool>(
      pauseControlInput_, 10, std::bind(&ODriverNode::pauseControlCallback, this, _1));

  imuSub = create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&ODriverNode::imuCallback, this, _1));

  maxAcc_ = declare_parameter("max_acc", maxAcc_);
  targetRate_ = declare_parameter("target_rate", targetRate_);
  motorOutput_ = declare_parameter("motor_topic", motorOutput_);

  // parameters for linear and angular velocity error feedback
  gain_vel_ = declare_parameter("gain_vel", 0.0);
  gain_omega_ = declare_parameter("gain_omega", 0.0);
  gain_vel_i_ = declare_parameter("gain_vel_i", 0.0);
  gain_omega_i_ = declare_parameter("gain_omega_i", 0.0);
  imuAngularVelocitySign_ = declare_parameter("imu_angular_velocity_sign", 1.0);
  imuAngularVelocityThreshold_ = declare_parameter("imu_angular_velocity_threshold", 0.01);
  feedbackSpdDeadzone_ = declare_parameter("feedback_speed_deadzone", 0.01);
  imuTimeTolerance_ = rclcpp::Duration(std::chrono::duration<double>(declare_parameter("imu_time_tolerance", 0.05)));

  thread = std::make_shared<std::thread>(&ODriverNode::cmdVelLoop, this, targetRate_);

  RCLCPP_INFO(get_logger(), "MotorAdapter ODriverNode - %s", __FUNCTION__);
}

ODriverNode::~ODriverNode()
{
  RCLCPP_INFO(get_logger(), "ODriverNode Destructor");
}


void ODriverNode::cmdVelLoop(int publishRate) {
  rclcpp::Rate loopRate(publishRate);

  motorPub = create_publisher<odriver_msgs::msg::MotorTarget>(motorOutput_, 10);

  double minimumStep = maxAcc_ / publishRate;

  while (rclcpp::ok()) {
    odriver_msgs::msg::MotorTarget target;

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
    target.spd_left = currentSpdLinear_ - targetT;
    target.spd_right = currentSpdLinear_ + targetT;

    // linear and velocity error feedback
    // apply feedback after receiving at least one motorStatus message to prevent integrator error accumulation
    if (lastOdomTime_ > rclcpp::Time(0, 0, get_clock()->get_clock_type())){
      rclcpp::Time now = get_clock()->now();
      double dt = 1.0/publishRate;
      double fixedMeasuredSpdTurn = measuredSpdTurn_;
      if (now - lastImuTime_ < imuTimeTolerance_){ // assumes imu is received continuously
        if (imuAngularVelocityThreshold_ <= fabs(lastImuAngularVelocity_)){
          fixedMeasuredSpdTurn = bias_ / 2.0 * lastImuAngularVelocity_; // radius * anguler_velocity converts rad/s to m/s dimension.
        }
      }

      // compute feedback wheel speed
      double errorSpdLinear = currentSpdLinear_ - measuredSpdLinear_;
      double errorSpdTurn = targetSpdTurn_ - fixedMeasuredSpdTurn;
      double feedbackSpdRight = gain_vel_*errorSpdLinear + gain_omega_*errorSpdTurn + gain_vel_i_*integral_linear_ + gain_omega_i_*integral_turn_ ;
      double feedbackSpdLeft = gain_vel_*errorSpdLinear - gain_omega_*errorSpdTurn + gain_vel_i_*integral_linear_ - gain_omega_i_*integral_turn_ ;
      integral_linear_ += errorSpdLinear*dt;
      integral_turn_ += errorSpdTurn*dt;

      // ignore small feedback speed to prevent slow rotation
      if ( feedbackSpdDeadzone_ < fabs(feedbackSpdRight)
           && feedbackSpdDeadzone_ < fabs(feedbackSpdLeft)){
        target.spd_right += feedbackSpdRight;
        target.spd_left += feedbackSpdLeft;
      }
    }

    if (pause_control_counter_ > 0){
      target.loop_ctrl = false;
      pause_control_counter_ -= 1;
    } else {
      target.loop_ctrl = true;
    }

    motorPub->publish(target);

    loopRate.sleep();
  }
}

void ODriverNode::encoderCallback(const odriver_msgs::msg::MotorStatus::SharedPtr input)
{
  double time = rclcpp::Time(input->header.stamp).nanoseconds() / 1000000000.0;
  diffDrive_.update(input->dist_left,
                    input->dist_right,
                    time);
  Pose& pose = diffDrive_.pose();

  // update measured velocity
  measuredSpdLinear_ = (input->spd_right + input->spd_left)/2.0;
  measuredSpdTurn_ = (input->spd_right - input->spd_left)/2.0;

  //ROS_INFO("input %d, %d, pose %f, %f", input->dist_left_c, input->dist_right_c, pose.x, pose.y);

  nav_msgs::msg::Odometry odom;

  if (rclcpp::Time(input->header.stamp) - lastOdomTime_ < rclcpp::Duration(7ms)) {
    return;
  }

  lastOdomTime_ = input->header.stamp;
  odom.header.stamp = input->header.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  double linear_covariance = 0.1;
  double angle_covariance = 0.2;

  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.a);
  q.normalize();
  odom.pose.pose.orientation.x = q[0];
  odom.pose.pose.orientation.y = q[1];
  odom.pose.pose.orientation.z = q[2];
  odom.pose.pose.orientation.w = q[3];
  odom.pose.covariance[0] = linear_covariance;
  odom.pose.covariance[7] = linear_covariance;
  odom.pose.covariance[14] = linear_covariance;
  odom.pose.covariance[21] = angle_covariance;
  odom.pose.covariance[28] = angle_covariance;
  odom.pose.covariance[35] = angle_covariance;

  LRdouble& vel = diffDrive_.velocity();
  odom.twist.twist.linear.x = vel.l;
  odom.twist.twist.angular.z = vel.r;
  odom.twist.covariance[0] = linear_covariance;
  odom.twist.covariance[7] = linear_covariance;
  odom.twist.covariance[14] = linear_covariance;
  odom.twist.covariance[21] = angle_covariance;
  odom.twist.covariance[28] = angle_covariance;
  odom.twist.covariance[35] = angle_covariance;
  odomPub->publish(odom);
}

void ODriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input)
{
  rclcpp::Time now = get_clock()->now();
  if (lastCmdVelTime_ > rclcpp::Time(0, 0, get_clock()->get_clock_type()) && now - lastCmdVelTime_ < rclcpp::Duration(200ms)) {
    //return;
  }
  lastCmdVelTime_ = now;
  double l = input->linear.x;
  double w = input->angular.z;

  targetSpdLinear_ = l;
  targetSpdTurn_ = bias_ / 2.0 * w;
}

void ODriverNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr input)
{
  lastImuTime_ = input->header.stamp;
  lastImuAngularVelocity_ = input->angular_velocity.z * imuAngularVelocitySign_;
}

void ODriverNode::pauseControlCallback(const std_msgs::msg::Bool::SharedPtr input)
{
  if (input->data) {
    pause_control_counter_ = targetRate_ * 5;
  } else {
    pause_control_counter_ = 0;
  }
}

} // namespace MotorAdapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MotorAdapter::ODriverNode);
