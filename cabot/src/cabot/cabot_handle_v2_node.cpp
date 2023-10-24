#include "cabot_handle_v2_node.hpp"

CaBotHandleV2Node::CaBotHandleV2Node()
  : Node("cabot_handle_v2_node"){
  event_pub_ = this->create_publisher<std_msgs::msg::String>("/cabot/event", 10);
  rclcpp::Parameter button_keys_param = this->get_parameter("buttons");
  if(button_keys_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    RCLCPP_WARN(this->get_logger(), "Parameter 'buttons' not set. Using default value.");
    button_keys_ = {""};  // defalt set
  }else{
    button_keys_ = button_keys_param.as_string_array();
  }
  handle_ = std::make_shared<Handle>(button_keys_);
  std::string button_keys_str;
  for(const std::string& key : button_keys_){
    button_keys_str += key + " ";
  }
  RCLCPP_INFO(get_logger(), "buttons: %s", button_keys_str.c_str());
  bool no_vibration = this->declare_parameter("no_vibration", false);
  RCLCPP_INFO(this->get_logger(), "no_vibration = %d", no_vibration);
  if(!no_vibration){
    rclcpp::QoS qos(10);
    notification_sub_ = this->create_subscription<std_msgs::msg::Int8>("/cabot/notification",
    qos,[this](const std_msgs::msg::Int8::SharedPtr msg){
      this->notificationCallback(msg);
    });
  }
}

void CaBotHandleV2Node::notificationCallback(const std_msgs::msg::Int8::SharedPtr msg){
  std::string log_message = "Received notification: " + std::to_string(msg->data);
  RCLCPP_INFO(this->get_logger(), log_message.c_str());
  handle_->executeStimulus(msg->data);
}

void CaBotHandleV2Node::eventListener(const std::string& msg){
  RCLCPP_INFO(get_logger(), msg.c_str());
  std::shared_ptr<BaseEvent> event = nullptr;
  if(msg.find("button") != std::string::npos){
    event = std::make_shared<ButtonEvent>(msg);
    std::shared_ptr<ButtonEvent> buttonEvent = std::dynamic_pointer_cast<ButtonEvent>(event);
    if(buttonEvent && !buttonEvent->is_up()){
      handle_->executeStimulus(9);
    }
  }
  if(msg.find("buttons") != std::string::npos){
    event = std::make_shared<ClickEvent>(msg);
  }
  if(msg.find("holddown") != std::string::npos){
    event = std::make_shared<HoldDownEvent>(msg);
    std::shared_ptr<HoldDownEvent> holdDownEvent = std::dynamic_pointer_cast<HoldDownEvent>(event);
    if(holdDownEvent){
      handle_->executeStimulus(9);
    }
  }
  if(event){
    RCLCPP_INFO(get_logger(), event->toString().c_str());
    std_msgs::msg::String msg;
    msg.data = event->toString();
    event_pub_->publish(msg);
    // delete event;
  }
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaBotHandleV2Node>());
  rclcpp::shutdown();
  return 0;
}
