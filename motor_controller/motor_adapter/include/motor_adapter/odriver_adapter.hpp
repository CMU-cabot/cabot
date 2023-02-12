/*******************************************************************************
 * Copyright (c) 2022  Carnegie Mellon University
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

#ifndef MOTOR_ADAPTER__ODRIVER_ADAPTER_HPP_
#define MOTOR_ADAPTER__ODRIVER_ADAPTER_HPP_

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <motor_adapter/diff_drive.hpp>
#include <odriver_msgs/msg/motor_status.hpp>
#include <odriver_msgs/msg/motor_target.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace MotorAdapter
{

class ODriverNode : public rclcpp::Node
{
public:
  explicit ODriverNode(rclcpp::NodeOptions options);
  ~ODriverNode();

private:
  void cmdVelLoop(int publishRate);
  void encoderCallback(const odriver_msgs::msg::MotorStatus::SharedPtr input);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr input);
  void pauseControlCallback(const std_msgs::msg::Bool::SharedPtr input);

  MotorAdapter::DiffDrive diffDrive_;

  std::string cmdVelInput_;
  std::string motorOutput_;
  std::string encoderInput_;
  std::string odomOutput_;
  std::string pauseControlInput_;

  rclcpp::Time lastCmdVelTime_;
  double targetSpdLinear_;
  double targetSpdTurn_;
  double currentSpdLinear_;
  rclcpp::Time lastOdomTime_;

  int targetRate_;
  double maxAcc_;

  double bias_;
  double wheel_diameter_;
  double count_per_rotate_;

  double measuredSpdLinear_;
  double measuredSpdTurn_;
  double gain_vel_;
  double gain_omega_;
  double gain_vel_i_;
  double gain_omega_i_;
  double integral_linear_;
  double integral_turn_;

  rclcpp::Time lastImuTime_;
  double lastImuAngularVelocity_;
  double imuAngularVelocitySign_;
  double imuAngularVelocityThreshold_;
  double feedbackSpdDeadzone_;
  rclcpp::Duration imuTimeTolerance_;

  int pause_control_counter_;

  rclcpp::Publisher<odriver_msgs::msg::MotorTarget>::SharedPtr motorPub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;

  rclcpp::Subscription<odriver_msgs::msg::MotorStatus>::SharedPtr encoderSub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pauseControlSub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;

  std::shared_ptr<std::thread> thread;
};  // class ODriverNode

}  // namespace MotorAdapter

#endif  // MOTOR_ADAPTER__ODRIVER_ADAPTER_HPP_
