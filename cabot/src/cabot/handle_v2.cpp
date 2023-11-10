#include "handle_v2.hpp"
#include <thread>
#include <chrono>
#include <algorithm>

const std::string Handle::stimuli_names[10] = {"unknown", "left_turn", "right_turn", "left_dev", "right_dev","front",
                                               "left_about_turn", "right_about_turn", "button_click", "button_holddown"};
const rclcpp::Duration Handle::double_click_interval_ = rclcpp::Duration(0, 250000000);
const rclcpp::Duration Handle::ignore_interval_ = rclcpp::Duration(0, 50000000);
const rclcpp::Duration Handle::holddown_interval_ = rclcpp::Duration(3, 0);

std::string Handle::get_name(int stimulus){
  return stimuli_names[stimulus];
}

Handle::Handle(std::shared_ptr<CaBotHandleV2Node> node, const std::function<void(const std::string&)>& eventListener, const std::vector<std::string>& buttonKeys)
  : Node("handle_node", "handle_name"), node_(node), eventListener_(eventListener), buttonKeys_(buttonKeys), logger_(get_logger()){
  power_ = 255;
  vibrator1_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator1", 100);
  vibrator2_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator2", 100);
  vibrator3_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator3", 100);
  vibrator4_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/cabot/vibrator4", 100);
  for(int i = 1; i <= static_cast<int>(ButtonType::BUTTON_CENTER); ++i){
    std::function<void(const std_msgs::msg::Bool::SharedPtr)> callback = [this, i](const std_msgs::msg::Bool::SharedPtr msg){
      buttonCallback(msg, static_cast<ButtonType>(i));
    };
    button_subs[i] = this->create_subscription<std_msgs::msg::Bool>(
    "/cabot/pushed_" + std::to_string(button_keys(i)), rclcpp::SensorDataQoS(), 
    [this, i](const std_msgs::msg::Bool::SharedPtr msg){
      buttonCallback(msg, static_cast<ButtonType>(i));
    });
  }
  event_sub_ = this->create_subscription<std_msgs::msg::String>(
  "/cabot/event", rclcpp::SensorDataQoS(), [this](const std_msgs::msg::String::SharedPtr msg){
    eventCallback(msg);
  });
  duration_ = 15;
  duration_single_vibration_ = 40;
  duration_about_turn_ = 40;
  duration_button_click_ = 5;
  duration_button_holddown_ = 10;
  sleep_ = 150;
  up_down_ = true;
  num_vibrations_turn_ = 4;
  num_vibrations_deviation_ = 2;
  num_vibrations_about_turn_ = 2;
  num_vibrations_confirmation_ = 1;
  num_vibrations_button_click_ = 1;
  num_vibrations_button_holddown_ = 1;
  eventListener_ = eventListener;
  callbacks_.resize(std::size(stimuli_names), nullptr);
  callbacks_[1] = [this](){
    vibrateLeftTurn();
  };
  callbacks_[2] = [this](){
    vibrateRightTurn();
  };
  callbacks_[3] = [this](){
    vibrateLeftDeviation();
  };
  callbacks_[4] = [this](){
    vibrateRightDeviation();
  };
  callbacks_[5] = [this](){
    vibrateFront();
  };
  callbacks_[6] = [this](){
    vibrateAboutLeftTurn();
  };
  callbacks_[7] = [this](){
    vibrateAboutRightTurn();
  };
  callbacks_[8] = [this](){
    vibrateButtonClick();
  };
  callbacks_[9] = [this](){
    vibrateButtonHolddown();
  };
}

void Handle::buttonCallback(const std_msgs::msg::Bool::SharedPtr msg, int index){
  if (index >= 1 && index <= 5) {
    buttonCheck(msg, static_cast<ButtonType>(index));
  }
}

void Handle::buttonCheck(const std_msgs::msg::Bool::SharedPtr msg, int index){
  std::map<std::string, std::string> event;
  rclcpp::Time now = this->get_clock()->now();
  if(msg->data && !btn_dwn[index] && !(last_up[index] != rclcpp::Time(0, 0) && now - last_up[index] < ignore_interval_)){
    event["button"] = button_keys(index);
    event["up"] = "False";
    btn_dwn[index] = true;
    last_dwn[index] = now;
  }
  if(!msg->data && btn_dwn[index]){
    event["button"] = button_keys(index);
    event["up"] = "True";
    up_count[index]++;
    last_up[index] = now;
    btn_dwn[index] = false;
  }
  if(last_up[index] != rclcpp::Time(0, 0) &&
    !btn_dwn[index] &&
    now - last_up[index] > double_click_interval_){
    if(last_dwn[index] != rclcpp::Time(0, 0)){
      event["buttons"] = button_keys(index);
      event["count"] = std::to_string(up_count[index]);
    }
    last_up[index] = rclcpp::Time(0, 0);
    up_count[index] = 0;
  }
  if(msg->data && btn_dwn[index] &&
    last_dwn[index] != rclcpp::Time(0, 0) &&
    now - last_dwn[index] > holddown_interval_){
    event["holddown"] = button_keys(index);
    last_dwn[index] = rclcpp::Time(0, 0);
  }
  if(!event.empty()){
      eventListener_(std::to_string(msg->data));
  }
}

void Handle::eventCallback(const std_msgs::msg::String::SharedPtr msg){
  const std::string name = msg->data;
  const std::string* it = std::find(stimuli_names, stimuli_names, name);
  if(it != stimuli_names){
    int index = it - stimuli_names;
    executeStimulus(index);
  }
}

void Handle::executeStimulus(int index){
  RCLCPP_INFO(logger_, "execute_stimulus, %d", index);
  if(index >= 0 && index < static_cast<int>(sizeof(stimuli_names) / sizeof(stimuli_names[0])) && callbacks_[index]){
    callbacks_[index]();
    RCLCPP_INFO(logger_, "executed");
  }
}

void Handle::vibrate(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub){
  std::unique_ptr<std_msgs::msg::UInt8> msg = std::make_unique<std_msgs::msg::UInt8>();
  msg->data = power_;
  pub->publish(std::move(msg));
}

void Handle::stop(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub){
  std::unique_ptr<std_msgs::msg::UInt8> msg = std::make_unique<std_msgs::msg::UInt8>();
  msg->data = 0;
  pub->publish(std::move(msg));
}

void Handle::vibrateAll(int time){
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

void Handle::vibrateLeftTurn(){
  if(up_down_){
    vibratePattern(vibrator3_pub_, num_vibrations_turn_, duration_);
  }else{
    vibratePattern(vibrator4_pub_, num_vibrations_turn_, duration_);
  }
}

void Handle::vibrateRightTurn(){
  if(up_down_){
    vibratePattern(vibrator4_pub_, num_vibrations_turn_, duration_);
  }else{
    vibratePattern(vibrator3_pub_, num_vibrations_turn_, duration_);
  }
}

void Handle::vibrateLeftDeviation(){
  if(up_down_){
    vibratePattern(vibrator3_pub_, num_vibrations_deviation_, duration_);
  }else{
    vibratePattern(vibrator4_pub_, num_vibrations_deviation_, duration_);
  }
}

void Handle::vibrateRightDeviation(){
  if(up_down_){
    vibratePattern(vibrator4_pub_, num_vibrations_deviation_, duration_);
  }else{
    vibratePattern(vibrator3_pub_, num_vibrations_deviation_, duration_);
  }
}

void Handle::vibrateFront(){
  vibratePattern(vibrator1_pub_, num_vibrations_confirmation_, duration_single_vibration_);
}

void Handle::vibrateAboutLeftTurn(){
  if(up_down_){
    vibratePattern(vibrator3_pub_, num_vibrations_about_turn_, duration_about_turn_);
  }else{
    vibratePattern(vibrator4_pub_, num_vibrations_about_turn_, duration_about_turn_);
  }
}

void Handle::vibrateAboutRightTurn(){
  if(up_down_){
    vibratePattern(vibrator4_pub_, num_vibrations_about_turn_, duration_about_turn_);
  }else{
    vibratePattern(vibrator3_pub_, num_vibrations_about_turn_, duration_about_turn_);
  }
}

void Handle::vibrateBack(){
  vibratePattern(vibrator2_pub_, num_vibrations_confirmation_, duration_single_vibration_);
}

void Handle::vibrateButtonClick(){
  vibratePattern(vibrator1_pub_, num_vibrations_button_click_, duration_button_click_);
}

void Handle::vibrateButtonHolddown(){
  vibratePattern(vibrator1_pub_, num_vibrations_button_holddown_, duration_button_holddown_);
}

void Handle::vibratePattern(rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr vibratorPub, int numberVibrations, int duration){
  int i = 0;
  std::string mode = "NEW";
  if(mode == "SAFETY"){
    while(true){
      for(int v = 0; v < duration; ++v){
        vibrate(vibratorPub);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      stop(vibratorPub);
      stop(vibratorPub);
      stop(vibratorPub);
      i++;
      if(i >= numberVibrations){
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_));
    }
    // Make sure it stops.
    for(int v = 0; v < 10; ++v){
      stop(vibratorPub);
    }
  }
  if(mode == "SIMPLE"){
    for(i = 0; i < numberVibrations; ++i){
      vibrate(vibratorPub);
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.01 * duration)));
      stop(vibratorPub);
      if(i < numberVibrations - 1){
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_));
      }
    }
    stop(vibratorPub);
  }
  if(mode == "NEW"){
    for(i = 0; i < numberVibrations; ++i){
      std_msgs::msg::UInt8 msg;
      msg.data = duration;
      vibratorPub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.01 * duration)));
      if(i < numberVibrations - 1){
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_));
      }
    }
  }
}
