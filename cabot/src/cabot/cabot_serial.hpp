/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
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

#ifndef CABOT__CABOT_SERIAL_HPP_
#define CABOT__CABOT_SERIAL_HPP_

#include <unistd.h>
#include <termios.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <tuple>
#include <exception>

#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>
// #include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "arduino_serial.hpp"


class CaBotSerialNode;

class TopicCheckTask : public diagnostic_updater::HeaderlessTopicDiagnostic
{
public:
  TopicCheckTask(diagnostic_updater::Updater & updater, const std::string & name, double freq);
  void tick();

private:
  double min;
  double max;
};

class CheckConnectionTask : public diagnostic_updater::DiagnosticTask
{
public:
  CheckConnectionTask(rclcpp::Logger logger, const std::string & name);
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  rclcpp::Logger logger_;
};

typedef struct Vibration {
  uint8_t current;
  uint8_t target;
  int count;
} Vibration;

class CaBotSerialNode : public rclcpp::Node, public CaBotArduinoSerialDelegate
{
public:
  explicit CaBotSerialNode(const rclcpp::NodeOptions & options);
  ~CaBotSerialNode() = default;

  diagnostic_updater::Updater updater_;
  rclcpp::Logger client_logger_;

  // Override and delegate by CaBotArduinoSerialDelegate
  std::tuple<int, int> system_time() override;
  void stopped() override;
  void log(rclcpp::Logger::Level level, const std::string & text) override;
  void log_throttle(rclcpp::Logger::Level level, int interval_in_ms, const std::string & text) override;
  void get_param(const std::string & name, std::function<void(const std::vector<int> &)> callback) override;
  void publish(uint8_t cmd, const std::vector<uint8_t> & data) override;

  std::shared_ptr<CaBotArduinoSerial> client_;
  CaBotSerialNode();
  // port_name_ = this->declare_parameter("port", "/dev/ttyCABOT").get<std::string>();
  // const char* port_name_ = "/dev/ttyESP32";
  // const int baud_rate_ = 115200;

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr touch_speed_switched_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_touch_speed_active_mode_srv;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr touch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr button_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> btn_pubs;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<rclcpp::Time> imu_last_topic_time;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr calibration_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib1_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib2_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib3_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr vib4_sub_;
  rclcpp::TimerBase::SharedPtr vib_timer_ = nullptr;

  bool is_alive_;
  static const size_t NUMBER_OF_BUTTONS = 5;
  Vibration vibrations_[4];
  void vib_loop();
  void vib_callback(const uint8_t cmd, const std_msgs::msg::UInt8::SharedPtr msg);
  std::shared_ptr<sensor_msgs::msg::Imu> process_imu_data(const std::vector<uint8_t> & data);
  void process_button_data(const std_msgs::msg::Int8::SharedPtr msg);
  void touch_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void set_touch_speed_active_mode(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  bool touch_speed_active_mode_;
  double touch_speed_max_speed_;
  double touch_speed_max_speed_inactive_;
  int main();
  void run_once();
  void poling();
  double throttle_duration_sec;
  rclcpp::TimerBase::SharedPtr timer_;


  template<typename T>
  void callback(const T & msg);

  // Diagnostic Updater
  std::shared_ptr<TopicCheckTask> imu_check_task_;
  std::shared_ptr<TopicCheckTask> touch_check_task_;
  std::shared_ptr<TopicCheckTask> button_check_task_;
  std::shared_ptr<TopicCheckTask> pressure_check_task_;
  std::shared_ptr<TopicCheckTask> temp_check_task_;
  std::shared_ptr<CheckConnectionTask> check_connection_task_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr vib_sub_;
  // std::shared_ptr<serial::Serial> port_;
  rclcpp::TimerBase::SharedPtr system_timer_;
  double system_time_;
  rclcpp::TimerBase::SharedPtr log_throttle_timer_;
  int count_;
};
#endif  // CABOT__CABOT_SERIAL_HPP_
