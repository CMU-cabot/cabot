#include "arduino_serial.hpp"
#include "cabot_serial.hpp"

CaBotArduinoSerialDelegate::CaBotArduinoSerialDelegate(CaBotSerialNode* delegate_node)
  : delegate_node_(delegate_node) {}

// Delegate definition for CaBotArduinoDriver class
std::tuple<int, int> CaBotArduinoSerialDelegate::system_time(){
    // return system time by tuple (sec, nsec)
    //CaBotArduinoSerial DelegateInstance("/dev/tty/ACM0", 115200);
    return delegate_node_->system_time();
  }
  void CaBotArduinoSerialDelegate::stopped(){
    // signal stopped
     delegate_node_->stopped();
  }
  void CaBotArduinoSerialDelegate::log(int level, const std::string & text){
    /*
     * implement log output for the system
     * @param level: logging level
     * @param text:logging text
     */
    delegate_node_->log(level, text);
  }
  void CaBotArduinoSerialDelegate::log_throttle(int level, int interval, const std::string & text){
    /*
     * implement log throttle output for the system
     * @param level: logging level
     * @param text:logging text
     */
    delegate_node_->log_throttle(level, interval, text);
  }
  void CaBotArduinoSerialDelegate::get_param(const std::string & name, std::function <void(const std::vector<int>&)> callback){
    /*
     * get parameter from the system
     * @param name: publish topic type
     * @param callback: call this callback with an int array
     */
    delegate_node_->get_param(name, callback);
  }
  void CaBotArduinoSerialDelegate::publish(uint8_t cmd, const std::vector<uint8_t>&data){
    /*
     * publish data according to the cmd
     * @param cmd: publish topic level
     * @param data: compact data of the message which needs to be converted
     *
     * defined topics and cmd
     * # touch       0x10
     * # touch_raw   0x11
     * # buttons     0x12
     * # imu         0x13
     * # calibration 0x14
     * # pressure    0x15
     * # temperature 0x16
     * # wifi        0x20
     */
    delegate_node_->publish(cmd, data);
  }

CaBotArduinoSerial::CaBotArduinoSerial(const std::string& port, int baud, std::chrono::milliseconds timeout)
  :Node("cabot_arduino_serial"), logger_(get_logger()), port_(port), baud_(baud), timeout_(timeout), delegate_(nullptr),// read_thread_(nullptr), write_thread_(nullptr),
   is_alive_(true), read_count_(0), time_synced_(false), no_input_count_(0) {}

void CaBotArduinoSerial::start(){
  reset_serial();
}

void CaBotArduinoSerial::reset_serial(){
  RCLCPP_INFO(get_logger(), "resetting serial port");
  //delegate_->log();
  port_.setDTR(false);
  sleep(0.2);
  port_.flushInput();
  port_.setDTR(true);
}

void CaBotArduinoSerial::stop(){
  is_alive_ = false;
  delegate_->stopped();
}

void CaBotArduinoSerial::run_once(){
  try{
    if(process_read_once()){
      no_input_count_ = 0;
    }else{
      no_input_count_ += 1;
      if(no_input_count_ > 10000){
        no_input_count_ = 0;
	stop();
      }
    }
  }catch(const std::exception& error){
    // sometimes read error can happen even if it is okay.
  }catch(const std::exception& e){
    RCLCPP_ERROR(get_logger(), "exception occurred during reading: %s", e.what());
    stop();
  }
  try{
    process_write_once();
  }catch(const std::exception& e){
    RCLCPP_ERROR(get_logger(), "exception occurred during reading: %s", e.what());
    stop();
  }
}

void CaBotArduinoSerial::send_command(uint8_t command, const std::vector<uint8_t>& arg){
  int count = arg.size();
  std::vector<uint8_t> data;
  data.push_back(0xAA);
  data.push_back(0xAA);
  data.push_back(command);
  data.push_back(count & 0xFF);
  /* If you want to extend the size more than 256
   * need to change cabot-arduino-serial as well
   * data.push_back((count >> 8) & 0XFF)
   */
  for(int i = 0; i< count; ++i){
    data.push_back(arg[i]);
  }
  data.push_back(checksum(arg));
  RCLCPP_DEBUG(get_logger(), "send" , data);
  write_queue_.push(data);
}

bool CaBotArduinoSerial::process_write_once(){
  if(write_queue_.empty()){
    return false;
  }
  std::vector<uint8_t> data = write_queue_.front();
  write_queue_.pop();
  int length = data.size();
  int total = 0; 
  while(total < length){
    int write_result = port_.write(&data[total], length - total);
    if(write_result < 0){
      RCLCPP_ERROR(get_logger(), "error writing data ");
      return false;
    }
    total += write_result;
    RCLCPP_DEBUG(get_logger(),  "%d bytes written", std::to_string(total));
  }
  // delegate_ -> log(logging::ERROR, "try to write invalid data type:" + type(data));
  RCLCPP_ERROR(get_logger(), "try to write invalid data type: %s" , typeid(data).name());
  return true;
}

bool CaBotArduinoSerial::try_read(int length, std::vector<uint8_t>& result){
  try{
    int bytes_remaining = length;
    std::string received;
    std::chrono::time_point<std::chrono::system_clock> read_start = std::chrono::system_clock::now();
    while(bytes_remaining != 0 && (std::chrono::system_clock::now() - read_start) < timeout_){
      std::lock_guard<std::mutex> lock(read_mutex_);
      received =port_.read(bytes_remaining);
      if(received.size() != 0){
        result.insert(result.end(), received.begin(), received.end());
	bytes_remaining -= received.size();
      }
    }
    if(bytes_remaining != 0){
      throw std::runtime_error(std::string("returned short excepted") + std::to_string(length) + "byte, received" + std::to_string(length - bytes_remaining) +"instead.");
    }
    read_count_ += length;
    return true;
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("Serial Port read failure:") +  e.what());
    return false;
  }
}

bool CaBotArduinoSerial::process_read_once(){
  /*
   * serial command format:
   * \xAA\xAA[cmd,1]{size,2][date,size][checksum]
   */
  if(port_.available() < 1){
    return false;
  }
  uint8_t cmd = 0 ;
  std::vector<uint8_t>received;
  try_read(1, received);
  if(received[0] != 0xAA){
    return false;
  }
  try_read(1, received);
  if(received[0] != 0xAA){
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "reading command");
  try_read(1, received);
  cmd = received[0];
  RCLCPP_DEBUG(get_logger(), "cmd = %d", cmd);
  int size = 0;
  try_read(2, received);
  for(int i = 0; i < 2; i++){
    size |= received[i] << (8 * i);
  }
  RCLCPP_DEBUG(get_logger(), "size =%d", size);
  std::vector<uint8_t> data;
  try{
    try_read(size, data);
  }catch(const std::exception& e){
    RCLCPP_DEBUG(get_logger(), "read error:%s", e.what());
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "data length =%d", data.size());
  uint8_t checksum1 = 0;
  try_read(1, received);
  checksum1  = received[0];
  uint8_t checksum2 = this->checksum(data);
  RCLCPP_DEBUG(get_logger(), "checksum %d %d", checksum1, checksum2);
  if(checksum1 != checksum2){
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "read data command =%d size =%d", cmd, size);
  if(cmd == 0x01){ // timesync
    send_time_sync(data);
  }else if(cmd == 0x02){ // logdebug
    RCLCPP_DEBUG(get_logger(), "%s", std::string(data.begin(), data.end()));
  }else if(cmd == 0x03){ // loginfo
    RCLCPP_INFO(get_logger(), "%s", std::string(data.begin(), data.end()));
  }else if(cmd == 0x04){ // logwarn
    RCLCPP_WARN(get_logger(), "%s", std::string(data.begin(), data.end()));
  }else if(cmd == 0x05){ // logerr
    RCLCPP_ERROR(get_logger(), "%s", std::string(data.begin(), data.end()));
  }else if(cmd == 0x08){
    send_param(data);
  }else if(cmd <= 0x10){
    delegate_->publish(cmd, data);
  }else{
    RCLCPP_ERROR(get_logger(), "unknown command %#04X", cmd);
    return false;
  }
  return true;
}

void CaBotArduinoSerial::send_time_sync(const std::vector<uint8_t>& data){
  // send current time
  uint32_t sec =0;
  uint32_t nsec =0;
  //delegate_->system_time(sec, nsec);
  std::vector<uint8_t> temp;
  for(int i = 0; i < 4; i++){
    temp.push_back(static_cast<uint8_t>((sec >> (8 * i))& 0xFF));
    temp.push_back(static_cast<uint8_t>((nsec >>  (8 * i))& 0xFF));
  }
  send_command(0x01, temp);
  time_synced_ = true;
  RCLCPP_DEBUG(get_logger(), "sync");
  //delegate
  uint32_t remote_sec = 0;
  uint32_t remote_nsec = 0;
  for(int i = 0; i < 4; i++){
    remote_sec |= static_cast<uint32_t>(data[i] << (8 * i));
    remote_nsec |= static_cast<uint32_t>(data[i+4] << (8 * i));
  }
  float diff_ms = (sec - remote_sec) * 1000.0 + (nsec - remote_nsec ) * 0.000001;
  RCLCPP_DEBUG(get_logger(), ",,,,,,,,,,%d.%f,%d,%f,diff,%f", remote_sec%1000, remote_nsec*0.000001, sec%1000, nsec*0.000001, diff_ms);
  //deligate_->log();
  }

uint8_t CaBotArduinoSerial::checksum(const std::vector<uint8_t>& data){
  uint8_t temp = 0;
  for(uint8_t d : data){
    temp += d;
  }
  return 0xFF - (0xFF & temp);
}
