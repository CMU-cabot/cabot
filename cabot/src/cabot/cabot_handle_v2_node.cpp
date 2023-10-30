#include "cabot_handle_v2_node.hpp"

std::shared_ptr<Handle> handle_;

CaBotHandleV2Node::CaBotHandleV2Node(const rclcpp::NodeOptions & options)
  : Node("cabot_handle_v2_node"){
  event_pub_ = this->create_publisher<std_msgs::msg::String>("/cabot/event", 10);
  }

void CaBotHandleV2Node::notificationCallback(const std_msgs::msg::Int8::SharedPtr msg){
  std::string log_msg_ = "Received notification: " + std::to_string(msg->data);
  RCLCPP_INFO(this->get_logger(), log_msg_.c_str());
  handle_->executeStimulus(msg->data);
}

void CaBotHandleV2Node::eventListener(const std::string& msg){
  RCLCPP_INFO(get_logger(), msg.c_str());
  std::shared_ptr<BaseEvent> event = nullptr;
  if(msg.find("button") != std::string::npos){
    event = std::make_shared<ButtonEvent>(msg);
    std::shared_ptr<ButtonEvent> buttonEvent = std::dynamic_pointer_cast<ButtonEvent>(event);
    // button down confirmation
    if(buttonEvent && !buttonEvent->is_up()){
      handle_->executeStimulus(8);
    }
  }
  if(msg.find("buttons") != std::string::npos){
    event = std::make_shared<ClickEvent>(msg);
  }
  if(msg.find("holddown") != std::string::npos){
    event = std::make_shared<HoldDownEvent>(msg);
    std::shared_ptr<HoldDownEvent> holdDownEvent = std::dynamic_pointer_cast<HoldDownEvent>(event);
    // button hold down confirmation
    if(holdDownEvent){
      handle_->executeStimulus(9);
    }
  }
  if(event){
    RCLCPP_INFO(get_logger(), event->toString().c_str());
    std_msgs::msg::String msg;
    msg.data = event->toString();
    event_pub_->publish(msg);
  }
}

std::shared_ptr<CaBotHandleV2Node> node_;

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  node_ = std::make_shared<CaBotHandleV2Node>(rclcpp::NodeOptions());
  std::vector<std::string> button_keys_ = node_->declare_parameter("buttons", std::vector<std::string>{""});
  handle_ = std::make_shared<Handle>(node_, [&node_](const std::string& msg){
    node_->eventListener(msg);
  }, button_keys_);
  RCLCPP_INFO(node_->get_logger(), "buttons: %s", button_keys_);
  bool no_vibration = node_->declare_parameter("no_vibration", false);
  RCLCPP_INFO(node_->get_logger(), "no_vibration = %d", no_vibration);
  if(!no_vibration){ 
    rclcpp::CallbackGroup::SharedPtr callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr notification_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    "/cabot/notification", 10, /*callback_group,*/ [node_](const std_msgs::msg::Int8::SharedPtr msg){
      node_->notificationCallback(msg);
    });
  }
  auto executer = rclcpp::executors::MultiThreadedExecutor();
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}
