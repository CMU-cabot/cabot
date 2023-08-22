#ifndef CABOTSERIAL_H_
#define CABOTSERIAL_H_

#include "arduino_serial.hpp"
#include <serial/serial.h>


#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <tuple>
#include <termios.h>
#include <unistd.h>
#include <exception>
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
//#include <diagnostic_msgs/msg/diagnostic_status.hpp>


class CaBotSerialNode;
class CheckConnectionTask;

class CaBotSerialNode : public rclcpp::Node, public diagnostic_updater::Updater, public CaBotArduinoSerialDelegate{
public:
  explicit CaBotSerialNode(const rclcpp::NodeOptions &options);
  ~CaBotSerialNode() = default;

  // Override and delegate by CaBotArduinoSerialDelegate
  std::tuple<int, int> system_time() override;
  void stopped() override;
  void log(int level, const std::string& text) override;
  void log_throttle(int level, int interval, const std::string& text) override;
  void get_param(const std::string& name, std::function<void(const std::vector<int>&)> callback) override;
  void publish(uint8_t cmd, const std::vector<uint8_t>& data) override;

  CaBotArduinoSerial cabot_arduino_serial;
  CaBotSerialNode();
  //port_name_ = this->declare_parameter("port", "/dev/ttyCABOT").get<std::string>();

private:
  class TopicCheckTask;
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
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  vib4_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr client_ = nullptr;
  std::shared_ptr<serial::Serial> port_;
  int topic_alive_ = 0;
  bool is_alive_;
  std::string port_name_;
  int baud_;
  rclcpp::Logger client_logger_;
  static const size_t NUMBER_OF_BUTTONS = 5;
  void vib_callback(const uint8_t cmd, const std_msgs::msg::UInt8::SharedPtr msg);
  std::shared_ptr<sensor_msgs::msg::Imu> process_imu_data(const std::vector<uint8_t>& data);
  void process_button_data(const std_msgs::msg::Int8::SharedPtr msg);
  void touch_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void set_touch_speed_active_mode(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);
  bool touch_speed_active_mode_;
  double touch_speed_max_speed_;
  double touch_speed_max_speed_inactive_;
  int main();
  void run_once();
  void polling();
  double throttle_duration_sec;
  std::string error_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

  template<typename T>
  void callback(const T& msg);

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;
  std::shared_ptr<CaBotSerialNode::TopicCheckTask> imu_check_task_;
  std::shared_ptr<CaBotSerialNode::TopicCheckTask> touch_check_task_;
  std::shared_ptr<CaBotSerialNode::TopicCheckTask> button_check_task_;
  std::shared_ptr<CaBotSerialNode::TopicCheckTask> pressure_check_task_;
  std::shared_ptr<CaBotSerialNode::TopicCheckTask> temp_check_task_;

  template<typename T>
  void callback(const uint8_t cmd, const T& msg){
    if(client_){
      std_msgs::msg::String string_msg;
      string_msg.data = std::to_string(static_cast<int>(msg));
      client_->publish(string_msg);
      //client_->publish(cmd, std::make_shared<std_msgs::msg::UInt8>(static_cast<int>(*msg)));
    }
  }

  class TopicCheckTask : public diagnostic_updater::HeaderlessTopicDiagnostic{
    public:
      TopicCheckTask(rclcpp::Node::SharedPtr node, diagnostic_updater::Updater &updater, const std::string &name, double freq, CaBotSerialNode* serial_node);
      void tick();
    private:
      rclcpp::Node::SharedPtr node_;
      CaBotSerialNode* serial_node_;
  };

  class CheckConnectionTask {
    public:
      CheckConnectionTask(rclcpp::Node::SharedPtr node, const std::string & name, CaBotSerialNode* serial_node);
      void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
    private:
      rclcpp::Node::SharedPtr node_;
      CaBotSerialNode* serial_node_;
  };

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr vib_sub_;
  //std::shared_ptr<serial::Serial> port_;
  rclcpp::TimerBase::SharedPtr system_timer_;
  double system_time_;
  rclcpp::TimerBase::SharedPtr log_throttle_timer_;
  int count_;
};
/*
std::tuple<int, int> system_time();
void stopped();
void log(int level, const std::string& text);
void log_throttle(int level, int interval, const std::string& text);
void get_param(const std::string& name, std::function<void(const std::vector<int>&)> callback);
void publish(uint8_t cmd, const std::vector<uint8_t>& data);
*/
#endif /* CABOTSERIAL_H_ */
