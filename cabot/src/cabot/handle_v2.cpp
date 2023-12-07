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

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <utility>
#include <thread>
#include <chrono>
#include <algorithm>

#include "handle_v2.hpp"

using namespace std::chrono_literals;

const char* Handle::stimuli_names[] =
{"unknown", "left_turn", "right_turn", "left_dev", "right_dev", "front",
  "left_about_turn", "right_about_turn", "button_click", "button_holddown"};
const rclcpp::Duration Handle::double_click_interval_ = rclcpp::Duration(0, 250000000);
const rclcpp::Duration Handle::ignore_interval_ = rclcpp::Duration(0, 50000000);
const rclcpp::Duration Handle::holddown_interval_ = rclcpp::Duration(3, 0);

std::string Handle::get_name(int stimulus)
{
  return stimuli_names[stimulus];
}

Handle::Handle(
  std::shared_ptr<CaBotHandleV2Node> node,
  std::function<void(const std::map<std::string, std::string> &)> eventListener,
  const std::vector<std::string> & buttonKeys)
: node_(node), eventListener_(std::move(eventListener)), buttonKeys_(buttonKeys),
      logger_(rclcpp::get_logger("handle"))
{
  power_ = 255;
  vibrator1_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator1", 100);
  vibrator2_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator2", 100);
  vibrator3_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator3", 100);
  vibrator4_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator4", 100);
  for (int i = 0; i < 9; ++i) {
    last_up[i] = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_dwn[i] = rclcpp::Time(0, 0, RCL_ROS_TIME);
    up_count[i] = 0;
    btn_dwn[i] = false;
  }
  for (int i = 1; i <= static_cast<int>(ButtonType::BUTTON_CENTER); ++i) {
    std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback =
      [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
        buttonCallback(msg, static_cast<ButtonType>(i));
      };
    button_subs[i] = node_->create_subscription<std_msgs::msg::Bool>(
      "/cabot/pushed_" + std::to_string(button_keys(i)), rclcpp::SensorDataQoS(),
      [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
        buttonCallback(msg, static_cast<ButtonType>(i));
      });
  }
  event_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/cabot/event", rclcpp::SensorDataQoS(), [this](const std_msgs::msg::String::SharedPtr msg) {
      eventCallback(msg);
    });
  duration_ = 150;
  duration_single_vibration_ = 400;
  duration_about_turn_ = 400;
  duration_button_click_ = 50;
  duration_button_holddown_ = 100;
  sleep_ = 150;
  num_vibrations_turn_ = 4;
  num_vibrations_deviation_ = 2;
  num_vibrations_about_turn_ = 2;
  num_vibrations_confirmation_ = 1;
  num_vibrations_button_click_ = 1;
  num_vibrations_button_holddown_ = 1;
  callbacks_.resize(std::size(stimuli_names), nullptr);
  callbacks_[1] = [this]() {
      vibrateLeftTurn();
    };
  callbacks_[2] = [this]() {
      vibrateRightTurn();
    };
  callbacks_[3] = [this]() {
      vibrateLeftDeviation();
    };
  callbacks_[4] = [this]() {
      vibrateRightDeviation();
    };
  callbacks_[5] = [this]() {
      vibrateFront();
    };
  callbacks_[6] = [this]() {
      vibrateAboutLeftTurn();
    };
  callbacks_[7] = [this]() {
      vibrateAboutRightTurn();
    };
  callbacks_[8] = [this]() {
      vibrateButtonClick();
    };
  callbacks_[9] = [this]() {
      vibrateButtonHolddown();
    };
  vibration_timer_ = node_->create_wall_timer(0.01s, std::bind(&Handle::timer_callback, this));
}

void Handle::timer_callback()
{
  if (vibration_queue_.size() > 0) {
    Vibration & vibration = vibration_queue_.front();
    RCLCPP_DEBUG(rclcpp::get_logger("handle"), "vibration.i = %d", vibration.i);
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "handle"), "vibration.numberVibration = %d", vibration.numberVibrations);
    if (vibration.numberVibrations == 0) {
      vibration_queue_.erase(vibration_queue_.begin());
      RCLCPP_INFO(rclcpp::get_logger("handle"), "Done");
    } else if (vibration.i == 0 && vibration.numberVibrations > 0) {
      std_msgs::msg::UInt8 msg;
      msg.data = vibration.duration * 0.1;
      vibration.vibratorPub->publish(msg);
      RCLCPP_INFO(rclcpp::get_logger("handle"), "publish %d", vibration.duration);
      RCLCPP_INFO(rclcpp::get_logger("handle"), "sleep %d ms", vibration.duration);
      vibration.i++;
    } else if (vibration.i == vibration.duration * 0.1  && vibration.numberVibrations == 1) {
      vibration.numberVibrations = 0;
    } else if (vibration.i < (vibration.duration + vibration.sleep) * 0.1) {
      vibration.i++;
    } else {
      vibration.i = 0;
      vibration.numberVibrations--;
    }
  }
}

void Handle::buttonCallback(const std_msgs::msg::Bool::SharedPtr msg, int index)
{
  if (index >= 1 && index <= 5) {
    buttonCheck(msg, index);
  }
}

void Handle::buttonCheck(const std_msgs::msg::Bool::SharedPtr msg, int index)
{
  event.clear();
  rclcpp::Time now = node_->get_clock()->now();
  rclcpp::Time zerotime(0, 0, RCL_ROS_TIME);
  if (msg->data && !btn_dwn[index] &&
    !(last_up[index] != zerotime && now - last_up[index] < ignore_interval_))
  {
    event.insert(std::make_pair("button", std::to_string(button_keys(index))));
    event.insert(std::make_pair("up", "False"));
    btn_dwn[index] = true;
    last_dwn[index] = now;
  }
  if (!msg->data && btn_dwn[index]) {
    event.insert(std::make_pair("button", std::to_string(button_keys(index))));
    event.insert(std::make_pair("up", "True"));
    up_count[index]++;
    last_up[index] = now;
    btn_dwn[index] = false;
  }
  if (last_up[index] != zerotime &&
    !btn_dwn[index] &&
    now - last_up[index] > double_click_interval_)
  {
    if (last_dwn[index] != zerotime) {
      event.insert(std::make_pair("buttons", std::to_string(button_keys(index))));
      event.insert(std::make_pair("count", std::to_string(up_count[index])));
    }
    last_up[index] = zerotime;
    up_count[index] = 0;
  }
  if (msg->data && btn_dwn[index] &&
    last_dwn[index] != zerotime &&
    now - last_dwn[index] > holddown_interval_)
  {
    event.insert(std::make_pair("holddown", std::to_string(button_keys(index))));
    last_dwn[index] = zerotime;
  }
  if (!event.empty()) {
    eventListener_(event);
  }
}

void Handle::eventCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const size_t stimuli_names_size = std::size(Handle::stimuli_names);
  const std::string name = msg->data;
  const char** it = std::find(stimuli_names, stimuli_names + stimuli_names_size, name.c_str());
  if(it != stimuli_names + stimuli_names_size){
    int index = std::distance(stimuli_names, it);
    executeStimulus(index);
  }else{
    RCLCPP_DEBUG(logger_, "Stimulus '%s' not found.", name.c_str());
  }
}

void Handle::executeStimulus(int index)
{
  RCLCPP_INFO(logger_, "execute_stimulus, %d", index);
  const std::size_t size = sizeof(stimuli_names) / sizeof(stimuli_names[0]);
  if (index >= 0 && index < static_cast<int>(size) && callbacks_[index]) {
    callbacks_[index]();
    RCLCPP_INFO(logger_, "executed");
  }
}

void Handle::vibrate(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub)
{
  std::unique_ptr<std_msgs::msg::UInt8> msg = std::make_unique<std_msgs::msg::UInt8>();
  msg->data = power_;
  pub->publish(std::move(msg));
}

void Handle::stop(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub)
{
  std::unique_ptr<std_msgs::msg::UInt8> msg = std::make_unique<std_msgs::msg::UInt8>();
  msg->data = 0;
  pub->publish(std::move(msg));
}

void Handle::vibrateAll(int time)
{
  std_msgs::msg::UInt8 msg;
  msg.data = power_;
  vibrator1_pub_->publish(msg);
  vibrator2_pub_->publish(msg);
  vibrator3_pub_->publish(msg);
  vibrator4_pub_->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  msg.data = 0;
  vibrator1_pub_->publish(msg);
  vibrator2_pub_->publish(msg);
  vibrator3_pub_->publish(msg);
  vibrator4_pub_->publish(msg);
}

void Handle::vibrateLeftTurn()
{
  vibratePattern(vibrator3_pub_, num_vibrations_turn_, duration_);
}

void Handle::vibrateRightTurn()
{
  vibratePattern(vibrator4_pub_, num_vibrations_turn_, duration_);
}

void Handle::vibrateLeftDeviation()
{
  vibratePattern(vibrator3_pub_, num_vibrations_deviation_, duration_);
}

void Handle::vibrateRightDeviation()
{
  vibratePattern(vibrator4_pub_, num_vibrations_deviation_, duration_);
}

void Handle::vibrateFront()
{
  vibratePattern(vibrator1_pub_, num_vibrations_confirmation_, duration_single_vibration_);
}

void Handle::vibrateAboutLeftTurn()
{
  vibratePattern(vibrator3_pub_, num_vibrations_about_turn_, duration_about_turn_);
}

void Handle::vibrateAboutRightTurn()
{
  vibratePattern(vibrator4_pub_, num_vibrations_about_turn_, duration_about_turn_);
}

void Handle::vibrateBack()
{
  vibratePattern(vibrator2_pub_, num_vibrations_confirmation_, duration_single_vibration_);
}

void Handle::vibrateButtonClick()
{
  vibratePattern(vibrator1_pub_, num_vibrations_button_click_, duration_button_click_);
}

void Handle::vibrateButtonHolddown()
{
  vibratePattern(vibrator1_pub_, num_vibrations_button_holddown_, duration_button_holddown_);
}

void Handle::vibratePattern(
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibratorPub,
  int numberVibrations, int duration)
{
  int i = 0;
  RCLCPP_INFO(rclcpp::get_logger("handle"), "Start vibratePattern .");
  Vibration vibration;
  vibration.duration = duration;
  vibration.numberVibrations = numberVibrations;
  vibration.sleep = sleep_;
  vibration.vibratorPub = vibratorPub;
  vibration_queue_.push_back(vibration);
}
