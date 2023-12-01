#include "cabot_handle_v2_node.hpp"

std::shared_ptr<CaBotHandleV2Node> node_;

CaBotHandleV2Node::CaBotHandleV2Node(const rclcpp::NodeOptions & options)
  : rclcpp::Node("cabot_handle_v2_node", options),handle_(nullptr), button_keys_({}), event_pub_(nullptr){}

void CaBotHandleV2Node::printStackTrace(){
  void *array[10];
  size_t size;
  size = backtrace(array, 10);
  char **symbols = backtrace_symbols(array, size);
  for(size_t i = 0; i < size; i++){
    RCLCPP_ERROR(rclcpp::get_logger("cabot_handle_v2_node"), "StackTrace[%zu]: %s", i, symbols[i]);
  }
  free(symbols);
}

void CaBotHandleV2Node::notificationCallback(const std_msgs::msg::Int8::SharedPtr msg){
  if(msg){
    std::string log_msg_ = "Received notification: " + std::to_string(msg->data);
    RCLCPP_INFO(this->get_logger(), log_msg_.c_str());
    node_->handle_->executeStimulus(msg->data);
  }else{
    RCLCPP_ERROR(this->get_logger(), "Received nullptr message in notificationCallback");
  }
  rclcpp::Clock::SharedPtr clock = node_->get_clock();
  RCLCPP_INFO(this->get_logger(), "Node clock type (in notificationCallback): %d", clock->get_clock_type());
}

void CaBotHandleV2Node::eventListener(const std::map<std::string, std::string>& msg){
  std::shared_ptr<BaseEvent> event = nullptr;
  std::string msg_str;
  for(std::map<std::string, std::string>::const_iterator it = msg.begin(); it != msg.end(); ++it){
    msg_str += "'" + it->first + "': " + it->second;
    if(std::next(it) != msg.end()){
      msg_str += ", ";
    }
  }
  msg_str = "{" + msg_str + "}";
  RCLCPP_INFO(get_logger(), msg_str.c_str());
 if(msg_str.find("buttons") != std::string::npos){
    int buttons = std::stoi(msg.at("buttons"));
    int count = std::stoi(msg.at("count"));
    event = std::make_shared<ClickEvent>(buttons, count);
  }else if(msg_str.find("button") != std::string::npos){
    int button = std::stoi(msg.at("button"));
    bool up = (msg.at("up") == "True") ? true : false;
    bool hold = (msg.find("hold") != msg.end()) ? true : false;
    event = std::make_shared<ButtonEvent>(button, up, hold);
    std::shared_ptr<ButtonEvent> buttonEvent = std::dynamic_pointer_cast<ButtonEvent>(event);
    // button down confirmation
    if(buttonEvent && !buttonEvent->is_up()){
      node_->handle_->executeStimulus(8);
    }
  }else if(msg_str.find("holddown") != std::string::npos){
    int hold = std::stoi(msg.at("holddown"));
    event = std::make_shared<HoldDownEvent>(hold);
    std::shared_ptr<HoldDownEvent> holdDownEvent = std::dynamic_pointer_cast<HoldDownEvent>(event);
    // button hold down confirmation
    if(holdDownEvent){
      node_->handle_->executeStimulus(9);
    }
  }
  if(event){
    RCLCPP_INFO(get_logger(), event->toString().c_str());
    std_msgs::msg::String msg;
    msg.data = event->toString();
    event_pub_->publish(msg);
  }
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  const std::vector<std::string> valgrind_prefix = { "valgrind", "--tool=callgrind", "--dump-instr=yes", "-v", "--instr-atstart=no"};
  rclcpp::NodeOptions options;
  options.append_parameter_override("executable", "cabot_handle_v2_node");
  options.append_parameter_override("prefix", valgrind_prefix);
  node_ = std::make_shared<CaBotHandleV2Node>(options);
  if(!node_){
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate memory for CaBotHandleV2Node .");
    return 1;
  }
  node_->event_pub_ = node_->create_publisher<std_msgs::msg::String>("/cabot/event", 10);
  std::vector<std::string> button_keys_ = node_->declare_parameter("buttons", std::vector<std::string>{""});
  std::string button_keys_str_ = std::accumulate(button_keys_.begin(), button_keys_.end(), std::string(),[](const std::string& result, const std::string& key){
    return result.empty() ? key : result + ", " + key;
  });
  node_->handle_ = std::make_shared<Handle>(node_, [node_](const std::map<std::string, std::string>& msg){
    node_->eventListener(msg);
  }, button_keys_);
  RCLCPP_INFO(node_->get_logger(), "buttons: %s", button_keys_str_.c_str());
  bool no_vibration = node_->declare_parameter("no_vibration", false);
  RCLCPP_INFO(node_->get_logger(), "no_vibration = %s", no_vibration ? "true" : "false");
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr notification_sub_;
  if(!no_vibration){
    rclcpp::SubscriptionOptions options;
    options.callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    std::shared_ptr<CaBotHandleV2Node> node_shared = node_;
    if(!node_shared){
      RCLCPP_ERROR(node_->get_logger(), "Invalid shared_ptr for CaBotHandleV2Node .");
      return 2;
    }
    notification_sub_ = node_shared->create_subscription<std_msgs::msg::Int8>(
    "/cabot/notification", 10, [node_shared](const std_msgs::msg::Int8::SharedPtr msg){
      node_shared->notificationCallback(msg);
    }, options);
  }
  rclcpp::Clock::SharedPtr clock = node_->get_clock();
  RCLCPP_INFO(node_->get_logger(), "Node clock type: %d", clock->get_clock_type());
  try{
    //rclcpp::spin_some(node_);
    //rclcpp::spin(node_);
    rclcpp::executors::MultiThreadedExecutor executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(node_);
    executor.spin();
  }catch(const std::exception& e){
    RCLCPP_ERROR(node_->get_logger(), "Exception during spinning: %s", e.what());
    node_->printStackTrace();
    return 3;
  }
  rclcpp::shutdown();
  return 0;
}
