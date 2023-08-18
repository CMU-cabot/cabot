#include "cabot_serial.hpp"

CaBotSerialNode::TopicCheckTask::TopicCheckTask(rclcpp::Node::SharedPtr node, diagnostic_updater::Updater &updater, const std::string &name, double freq)
  : diagnostic_updater::HeaderlessTopicDiagnostic(name, updater, diagnostic_updater::FrequencyStatusParam(&freq, &freq, 0.1, 2)), node_(node), topic_alive_(0) {}

void CaBotSerialNode::TopicCheckTask::tick(){
  topic_alive_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  diagnostic_updater::HeaderlessTopicDiagnostic::tick();
}

CaBotSerialNode::CheckConnectionTask::CheckConnectionTask(rclcpp::Node::SharedPtr node, const std::string &name, CaBotSerialNode* serial_node)
  : node_(node), client_(nullptr), serial_node_(serial_node), topic_alive_(0){} //, client_logger_(this->client_logger_){}

void CaBotSerialNode::CheckConnectionTask::run(diagnostic_updater::DiagnosticStatusWrapper & stat){
  if(client_ == nullptr){
    if(error_msg_.empty()){
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "connecting");
    }else{
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg_);
    }
  }else{
    if(topic_alive_ == 0 || (std::time(nullptr) - topic_alive_) > 1){
      RCLCPP_ERROR(node_->get_logger(), "connected but message coming");
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,"connected but no message coming");
      serial_node_->client_ = nullptr;
      serial_node_->port_ = nullptr;
      topic_alive_ = 0;
    }else{
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,"working");
    }
  }
}

CaBotSerialNode::CaBotSerialNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("cabot_serial_node", options), diagnostic_updater::Updater(this), cabot_arduino_serial("/dev/tty/ACM0", 115200),
    touch_speed_switched_pub_(nullptr), set_touch_speed_active_mode_srv(nullptr), touch_raw_pub_(nullptr), touch_pub_(nullptr),
    button_pub_(nullptr), btn_pubs(), imu_pub_(nullptr), imu_last_topic_time(nullptr), calibration_pub_(nullptr), pressure_pub_(nullptr),
    temperature_pub_(nullptr), wifi_pub_(nullptr), vib1_sub_(nullptr), vib2_sub_(nullptr), vib3_sub_(nullptr), vib4_sub_(nullptr),
    client_(nullptr), topic_alive_(0), client_logger_(rclcpp::get_logger("arduino_serial")),
    touch_speed_max_speed_(0.0), touch_speed_max_speed_inactive_(0.0), touch_speed_active_mode_(false), updater_(this),
    imu_check_task_(std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "IMU", 100)),
    touch_check_task_(std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Touch sensor", 50)),
    button_check_task_(std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Push Button", 50)),
    pressure_check_task_(std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Pressure", 2)),
    temp_check_task_(std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Temperature", 2)){
    cabot_arduino_serial.delegate_ = this;
    touch_raw_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch_raw", rclcpp::QoS(10));
    touch_pub_ = this->create_publisher<std_msgs::msg::Int16>("touch", rclcpp::QoS(10));
    button_pub_ = this->create_publisher<std_msgs::msg::Int8>("pushed", rclcpp::QoS(10));
    for(size_t i = 0; i < NUMBER_OF_BUTTONS; ++i){
      btn_pubs.push_back(this->create_publisher<std_msgs::msg::Bool>("pushed_"+ std::to_string(i + 1), rclcpp::QoS(10)));
    }
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",rclcpp::QoS(10));
    imu_last_topic_time = nullptr;
    calibration_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("calibration",rclcpp::QoS(10));
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure",rclcpp::QoS(10));
    temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temparature",rclcpp::QoS(10));
    wifi_pub_ = this->create_publisher<std_msgs::msg::String>("wifi",rclcpp::QoS(10));

    vib1_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator1",10,[this](const std_msgs::msg::UInt8::SharedPtr msg){
      this->vib_callback(0x20,msg);
    });
    vib2_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator2",10,[this](const std_msgs::msg::UInt8::SharedPtr msg){
      this->vib_callback(0x21,msg);
    });
    vib3_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator3",10,[this](const std_msgs::msg::UInt8::SharedPtr msg){
      this->vib_callback(0x22,msg);
    });
    vib4_sub_ = this->create_subscription<std_msgs::msg::UInt8>("vibrator4",10,[this](const std_msgs::msg::UInt8::SharedPtr msg){
      this->vib_callback(0x23,msg);
    });

    /* touch speed control
     * touch speed activw mode
     * True:  Touch - go,    Not Touch - no go
     * False: Touch - no go, Not Touch - go
     */

    std::shared_ptr<std_srvs::srv::SetBool::Request> req = std::make_shared<std_srvs::srv::SetBool::Request>();
    std::shared_ptr<std_srvs::srv::SetBool::Response> res = std::make_shared<std_srvs::srv::SetBool::Response>();
    set_touch_speed_active_mode (req, res);
    touch_speed_max_speed_ = this->declare_parameter("touch_speed_max_speed", 2.0);
    touch_speed_max_speed_inactive_ = this->declare_parameter("touch_speed_max_speed_inactive", 0.5);
    rclcpp::QoS transient_local_qos(1);
    transient_local_qos.transient_local();
    touch_speed_switched_pub_ = this->create_publisher<std_msgs::msg::Float32>("touch_speed_switched", transient_local_qos);
    set_touch_speed_active_mode_srv = this->create_service<std_srvs::srv::SetBool>("set_touch_speed_active_mode", std::bind(&CaBotSerialNode::set_touch_speed_active_mode,this, std::placeholders::_1, std::placeholders::_2));

    // Diagnostic Updater
    imu_check_task_ = std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "IMU",100);
    touch_check_task_ = std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Touch sensor",50);
    button_check_task_ = std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Push Button",50);
    pressure_check_task_ = std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Pressure",2);
    temp_check_task_ = std::make_shared<CaBotSerialNode::TopicCheckTask>(this->shared_from_this(), updater_, "Temperature",2);
    // updater_.add(diagnostic_updater::TopicCheckTask(this->shared_from_this(), updater_, "SerialConnection", 1.0)); //superfluous
    updater_.force_update();

    rclcpp::Logger client_logger = rclcpp::get_logger("arduino serial");
}

void CaBotSerialNode::vib_callback(uint8_t cmd, const std_msgs::msg::UInt8::SharedPtr msg){
  callback<uint8_t>(cmd, msg->data);
}

std::tuple<int, int> CaBotSerialNode::system_time(){
  rclcpp::Time now = this->get_clock()->now();
  int64_t nnow = now.nanoseconds();
  int sec = static_cast<int>(nnow / 1000000000);
  int nsec = static_cast<int>(nnow % 1000000000);
  // self.client_logger().info("current time={}".format(self.get_clock().now())
  return std::make_pair(sec, nsec);
}

void CaBotSerialNode::stopped(){
  cabot_arduino_serial.reset_serial();
  client_ = nullptr;
  port_.reset();
  topic_alive_ = 0;
  RCLCPP_ERROR(get_logger(), "stopped");
  }

void CaBotSerialNode::log(int level, const std::string& text){
  if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Info){
    RCLCPP_INFO(client_logger_, "%s", text.c_str());
  }else if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Warn){
    RCLCPP_WARN(client_logger_, "%s", text.c_str());
  }else if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Error){
    RCLCPP_ERROR(client_logger_, "%s", text.c_str());
  }else if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Debug){
    RCLCPP_DEBUG(client_logger_, "%s", text.c_str());
  }
}

void CaBotSerialNode::log_throttle(int level, int interval, const std::string& text){
  if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Info){
    RCLCPP_INFO_THROTTLE(client_logger_, *this->get_clock(), std::chrono::milliseconds(interval).count(), "%s", text.c_str());
  }else if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Warn){
    RCLCPP_WARN_THROTTLE(client_logger_, *this->get_clock(), std::chrono::milliseconds(interval).count(), "%s", text.c_str());
  }else if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Error){
    RCLCPP_ERROR_THROTTLE(client_logger_, *this->get_clock(), std::chrono::milliseconds(interval).count(), "%s", text.c_str());
  }else if(static_cast<rclcpp::Logger::Level>(level)  == rclcpp::Logger::Level::Debug){
    RCLCPP_DEBUG_THROTTLE(client_logger_, *this->get_clock(), std::chrono::milliseconds(interval).count(), "%s", text.c_str());
  }
}

void CaBotSerialNode::get_param(const std::string& name, std::function<void(const std::vector<int>&)> callback){
  rcl_interfaces::msg::ParameterDescriptor pd;
  std::vector<int> val;
  try{
    if(name == "run_imu_calibration"){
      pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    }else if(name == "calibration_params"){
      pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    }else if(name == "touch_params"){
      pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    }else{
      RCLCPP_INFO(get_logger(), "parameter %s is not defined", name.c_str());
      callback(val);
      return;
    }
    std::vector<long int> default_value;
    if(!this->has_parameter(name)){
      this->declare_parameter(name, rclcpp::ParameterValue(default_value), pd);
    }
    // traceback alternative but useless
    std::vector<long int> lval = this->get_parameter(name).get_value<std::vector<long int>>();
      val.assign(lval.begin(), lval.end());
    }catch(const rclcpp::exceptions::ParameterNotDeclaredException&){
       RCLCPP_ERROR(get_logger(), "parameter %s is not defined", name.c_str());
    }catch(const rclcpp::ParameterTypeException&){
       RCLCPP_ERROR(get_logger(), "invalid parameter type %s", name.c_str());
    }catch(const std::exception& ex){
       RCLCPP_ERROR(get_logger(), "exception while get parameter %s: %s", name.c_str(), ex.what());
    }catch(...){
       RCLCPP_ERROR(get_logger(), "unknown error occurred while get parameter %s", name.c_str());
    }
  RCLCPP_INFO(get_logger(), "get_param %s=%s", name.c_str(), val.size() > 0 ? std::to_string(val[0]) : "[]");
  callback(val);
}

std::shared_ptr<sensor_msgs::msg::Imu> CaBotSerialNode::process_imu_data(const std::vector<uint8_t>& data){
  // Discard possible corrupted data
  int count = 0;
  std::vector<float> data2;
  data2.reserve(12);
  for (int i = 0; i < 12; ++i) {
    float value;
    std::memcpy(&value, &data[i*4], sizeof(float));
    data2.push_back(value);
  }
  for (int i = 2; i < 12; ++i) {
    if (data2[i] == 0) {
      count++;
    }
  }
  if (count > 3) {
    return nullptr;
  }
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.orientation_covariance[0] = 0.1;
  imu_msg.orientation_covariance[4] = 0.1;
  imu_msg.orientation_covariance[8] = 0.1;

  // Convert float(32) to int(32)
  imu_msg.header.stamp.sec = static_cast<int>(data2[0]);
  imu_msg.header.stamp.nanosec = static_cast<int>(data2[1] * 1e9);
  rclcpp::Time imu_time = rclcpp::Time(imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec);

  // Check if the difference of time between the current time and the imu stamp is bigger than 1 sec
  rclcpp::Time now = this->get_clock()->now();
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), std::chrono::seconds(1).count(),"time diff = %d", std::abs((now - imu_time).nanoseconds()));
  if (std::abs((now - imu_time).nanoseconds()) > 1e9) {
    RCLCPP_ERROR(get_logger(), "IMU timestamp jumps more than 1 second, drop a message\n""imu time: %d > current time: %d", imu_time.nanoseconds(), now.nanoseconds());
    return nullptr;
  }
  if (imu_last_topic_time != nullptr) {
    if (*imu_last_topic_time > imu_time) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), std::chrono::seconds(1).count(),"IMU timestamp is not consistent, drop a message\n""last imu time: %d > current imu time: %d", imu_last_topic_time->nanoseconds(), imu_time.nanoseconds());
      return nullptr;
    }
  }
  imu_msg.header.frame_id = "imu_frame";
  imu_last_topic_time = std::make_shared<rclcpp::Time>(imu_time);
  imu_msg.orientation.x = data2[2];
  imu_msg.orientation.y = data2[3];
  imu_msg.orientation.z = data2[4];
  imu_msg.orientation.w = data2[5];
  imu_msg.angular_velocity.x = data2[6];
  imu_msg.angular_velocity.y = data2[7];
  imu_msg.angular_velocity.z = data2[8];
  imu_msg.linear_acceleration.x = data2[9];
  imu_msg.linear_acceleration.y = data2[10];
  imu_msg.linear_acceleration.z = data2[11];
  return std::make_shared<sensor_msgs::msg::Imu>(imu_msg);
}

void CaBotSerialNode::process_button_data(std_msgs::msg::Int8::SharedPtr msg){
  for(size_t i = 0; i < NUMBER_OF_BUTTONS; ++i){
    std_msgs::msg::Bool temp;
    temp.data = ((msg->data >> i) & 0x01) == 0x01;
    btn_pubs[i]->publish(temp);
  }
}

void CaBotSerialNode::touch_callback(std_msgs::msg::Int16::SharedPtr msg){
  std::shared_ptr<std_msgs::msg::Float32> touch_speed_msg = std::make_unique<std_msgs::msg::Float32>();
  touch_speed_msg->data = (touch_speed_active_mode_ && msg->data) ? touch_speed_max_speed_ : 0.0;
  if(!touch_speed_active_mode_){
    touch_speed_msg->data = msg->data ? 0.0 : touch_speed_max_speed_inactive_;
  }
  touch_speed_switched_pub_->publish(*touch_speed_msg);
}

void CaBotSerialNode::set_touch_speed_active_mode(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res){
  touch_speed_active_mode_ = req->data;
  if(touch_speed_active_mode_){
    res->message = "touch speed active mode = True";
  }else{
    res->message = "touch speed active mode = False";
  }
  res->success = true;
}

void CaBotSerialNode::publish(uint8_t cmd, const std::vector<uint8_t>& data){
 /* self.get_logger().info("%x: %d", cmd, int.from_bytes(data, "little"))
  * self.get_logger().info("%x: %s", cmd, str(data));
  * self.log_throttle(logging.INFO, 1, "got data %x"%(cmd))
  */
  if(cmd == 0x10){  // touch
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>((data[1] << 8) | data[0]);
    touch_pub_->publish(msg);
    touch_callback(std::make_shared<std_msgs::msg::Int16>(msg));
    touch_check_task_->tick();
  }
  if(cmd == 0x11){  // touch_raw
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>((data[1] << 8) | data[0]);
    touch_raw_pub_->publish(msg);
  }
  if(cmd == 0x12){  // buttons
    std_msgs::msg::Int8 msg;
    msg.data = static_cast<int8_t>(data[0]);
    button_pub_->publish(msg);
    process_button_data(std::make_shared<std_msgs::msg::Int8>(msg));
    button_check_task_->tick();
  }
  if(cmd == 0x13){  // imu
    std::shared_ptr<sensor_msgs::msg::Imu> msg = process_imu_data(data);
    if(msg){
      imu_pub_->publish(*msg);
      imu_check_task_->tick();
    }
  }
  if(cmd == 0x14){  // calibration
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = data;
    calibration_pub_->publish(msg);
  }
  if(cmd == 0x15){  // pressure
    sensor_msgs::msg::FluidPressure msg;
    float pressure;
    std::memcpy(&pressure, data.data(), sizeof(float));
    msg.fluid_pressure = pressure;
    msg.variance = 0.0;
    msg.header.stamp = this->now();
    msg.header.frame_id = "bmp_frame";
    pressure_pub_->publish(msg);
    pressure_check_task_->tick();
  }
  if(cmd == 0x16){  // temperature
    sensor_msgs::msg::Temperature msg;
    float temperature;
    std::memcpy(&temperature, data.data(), sizeof(float));
    msg.temperature = temperature;
    msg.variance = 0.0;
    msg.header.stamp = this->now();
    msg.header.frame_id = "bmp_frame";
    temperature_pub_->publish(msg);
    temp_check_task_->tick();
  }
  if(cmd == 0x20){  // wifi
    std_msgs::msg::String msg;
    msg.data = std::string(data.begin(), data.end());
    wifi_pub_->publish(msg);
  }
}

void CaBotSerialNode::run_once(){
  if(client_ == nullptr){
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "run_once", throttle_duration_sec = 1.0);
  try{
    //client_->run_once(); // recursive call
  }catch(const serial::SerialException& e){
    error_msg_ = e.what();
    RCLCPP_ERROR(this->get_logger(), error_msg_.c_str());
    client_ = nullptr;
    if (timer_){
      timer_->cancel();
    }
  }catch(const std::system_error& e){ // OSError
    error_msg_ = e.what();
    RCLCPP_ERROR(this->get_logger(), error_msg_.c_str());
    std::cerr << e.what() << std::endl;
    client_ = nullptr;
    if(timer_){
      timer_->cancel();
    }
  }catch(const std::ios_base::failure& e){ // IOError
    error_msg_ = e.what();
    RCLCPP_ERROR(this->get_logger(), error_msg_.c_str());
    RCLCPP_ERROR(this->get_logger(), "try to reconnect usb");
    client_ = nullptr;
    if(timer_){
      timer_->cancel();
    }
  /* 
  }catch(const termios::error& e){ // termios.error
    error_msg_ = e.what();
    RCLCPP_ERROR(this->get_logger(), error_msg_.c_str());
    RCLCPP_ERROR(this->get_logger(), "connection disconnected");
    client_ = nullptr;
    if(timer_){
      timer_->cancel();
    }
  }catch(const SystemExitException& e){ 
  // pass
  */ 
  }catch (...){
    RCLCPP_ERROR(this->get_logger(), "error occurred");
    rclcpp::shutdown();
    std::exit(EXIT_FAILURE);
  }
}

// polling to check if client (arduino) is disconnected and keep trying to reconnect
void CaBotSerialNode::polling(){
  RCLCPP_DEBUG(this->get_logger(), "polling");
  if(client_ && is_alive_){
    return;
  }
  client_.reset();
  port_.reset();
  RCLCPP_INFO(this->get_logger(), "Connecting to %s at %d baud", port_name_.c_str(), baud_);
  try{
    //port_ = std::make_shared<serial::Serial>(port_name_, baud_, serial::Timeout::simpleTimeout(5000),serial::Timeout::simpleTimeout(10000));
  }catch(const serial::SerialException& e){
    RCLCPP_ERROR(this->get_logger(), e.what());
    return;
  }
  //client_ = std::make_shared<CaBotArduinoSerial>(port_, baud_;
  updater_.setHardwareID(port_name_);
  topic_alive_ = 0;
  //client_->start();
  cabot_arduino_serial.start();
  RCLCPP_INFO(this->get_logger(), "Serial is ready");
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<CaBotSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotSerialNode);

