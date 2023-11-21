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
#include "cabot_handle_v2_node.hpp"

class CaBotHandleV2Node;

class Handle {
public:
  enum button_keys{
    UNKNOWN = 0, LEFT_TURN = 1, RIGHT_TURN = 2, LEFT_DEV = 3, RIGHT_DEV = 4, FRONT = 5, 
    LEFT_ABOUT_TURN = 6, RIGHT_ABOUT_TURN = 7, BUTTON_CLICK = 8, BUTTON_HOLDDOWN = 9 , STIMULI_COUNT = 10
  };
  Handle(std::shared_ptr<CaBotHandleV2Node> node, std::function<void(const std::map<std::string, std::string>&)> eventListener, const std::vector<std::string>& buttonKeys);
  void executeStimulus(int index);
  static const std::string stimuli_names[10];
private:
  void buttonCallback(const std_msgs::msg::Bool::SharedPtr msg, int index);
  void buttonCheck(const std_msgs::msg::Bool::SharedPtr msg, int index);
  void eventCallback(const std_msgs::msg::String::SharedPtr msg);
  void vibrate(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8>> pub);
  void stop(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8>> pub);
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
  void vibratePattern(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt8>> vibratorPub, int numberVibrations, int duration);
  std::shared_ptr<CaBotHandleV2Node> node_;
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
  std::map<std::string, std::string> event;
  std::function<void(const std::map<std::string, std::string>&)> eventListener_;
  std::vector<std::string> buttonKeys_;
  rclcpp::Logger logger_;
  std::vector<std::function<void()>> callbacks_;
  int button[10];
  static const rclcpp::Duration double_click_interval_;
  static const rclcpp::Duration ignore_interval_;
  static const rclcpp::Duration holddown_interval_;
  std::string get_name(int);
};

#endif // HANDLE_V2_HPP_ 
