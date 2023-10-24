#ifndef CABOT__HANDLE_V2_HPP_
#define CABOT__HANDLE_V2_HPP_

#include <rclcpp/rclcpp.hpp>
#include "button.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <map>
#include <string>
#include <time.h>

class CaBotHandleV2Node;

class Handle : public rclcpp::Node
{
public:
  Handle(const std::vector<std::string>& buttonKeys);
  void executeStimulus(int index);
  std::shared_ptr<CaBotHandleV2Node> cabot_handle_v2_node_ = nullptr;
private:
  void buttonCallback(const std_msgs::msg::Bool::SharedPtr msg, int index);
  void buttonCheck(const std_msgs::msg::Bool::SharedPtr msg, int index);
  void eventCallback(const std_msgs::msg::String::SharedPtr msg);
  void vibrate(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub);
  void stop(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub);
  void vibrateAll(int time);
  void vibrateLeftTurn();
  void vibrateRightTurn();
  void vibrateLeftDeviation();
  void vibrateRightDeviation();
  void vibrateFront();
  void vibrateAboutLeftTurn();
  void vibrateAboutRightTurn();
  void vibrateBack();
  void vibrateButtonClick();
  void vibrateButtonHolddown();
  void vibratePattern(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibratorPub, int numberVibrations, int duration);
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator1_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator2_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator3_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibrator4_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_subs[9];
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Time last_up[9];
  rclcpp::Time last_dwn[9];
  int up_count[9];
  bool btn_dwn[9];
  int power_;
  int duration_;
  int duration_single_vibration_;
  int duration_about_turn_;
  int duration_button_click_;
  int duration_button_holddown_;
  unsigned int sleep_;
  bool up_down_;
  int num_vibrations_turn_;
  int num_vibrations_deviation_;
  int num_vibrations_about_turn_;
  int num_vibrations_confirmation_;
  int num_vibrations_button_click_;
  int num_vibrations_button_holddown_;
  std::vector<std::function<void()>> callbacks_;
  int button[10];
  static const std::string stimuli_names[10];
  static const std::string button_keys[10];
  static const rclcpp::Duration double_click_interval_;
  static const rclcpp::Duration ignore_interval_;
  static const rclcpp::Duration holddown_interval_;
};

#endif // HANDLE_V2_HPP_ 
