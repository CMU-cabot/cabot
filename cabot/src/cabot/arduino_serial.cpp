#include "arduino_serial.hpp"
#include "cabot_serial.hpp"

Serial::Serial(std::string name, int baud, int read_timeout, int write_timeout){}

timespec timespec_from_ms(const uint32_t mills){
  timespec time;
  clock_gettime(CLOCK_REALTIME, &time);
  time.tv_sec += mills / 1000;
  time.tv_nsec += (mills % 1000) * 1000000;
  time.tv_sec += time.tv_nsec / 1000000000;
  time.tv_nsec %= 1000000000;
  return time;
}

void Serial::setDTR(bool flag){
  if(!is_open_){
    throw std::runtime_error("Serial::setDTR");
  }
  termios tio;
  if(tcgetattr(fd_, &tio) < 0){
    std::string error_msg = "setDTR failed: " + std::string(strerror(errno));
    throw std::runtime_error(error_msg);
  }
   if(flag){
     tio.c_cflag |= TIOCM_DTR;
   }else{
     tio.c_cflag &= ~TIOCM_DTR;
   }
   if(tcsetattr(fd_, TCSANOW, &tio) < 0){
    std::string error_msg = "setDTR failed: " + std::string(strerror(errno));
    throw std::runtime_error(error_msg);
   }
}

void Serial::flushInput(){
  if(!is_open_){
    throw std::runtime_error("Serial::flushInput");
  }           
  if(tcflush(fd_, TCIFLUSH) != 0){
    std::string error_msg = "tcflush failed: " + std::string(strerror(errno));
    throw std::runtime_error(error_msg);
  }
}

bool Serial::waitReadable(uint32_t timeout){
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);
  struct timespec timeout_ts = timespec_from_ms(timeout);
  int r = pselect(fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
  if(r < 0){
    if(errno == EINTR){
      return false;
    }
    std::string error_msg = "Error in waitReadable: " + std::string(strerror(errno));
    throw std::runtime_error(error_msg);
  }
  if(r == 0){
    return false;
  }
  if(!FD_ISSET(fd_, &readfds)){
    throw std::runtime_error("select reports ready to read, but our fd isn't in the list in the list, this shouldn't happen");
  }
  return true;
}

std::string Serial::read(int size){
  if(!is_open_){
    throw std::runtime_error("Serial::read");
  }
  std::string data;
  data.resize(size);
  size_t bytes_read = 0;
  uint32_t timeout_multiplier = 0;
  long total_timeout_ms = read_timeout * size;
  while(bytes_read < size){
    int64_t timeout_remaining_ms = total_timeout_ms - (bytes_read * timeout_multiplier);
    if(timeout_remaining_ms <= 0){
      break;
    }
    uint32_t timeout = static_cast<uint32_t>(timeout_remaining_ms);
    if(waitReadable(timeout)){
      ssize_t bytes_read_now = ::read(fd_, &data[bytes_read], size - bytes_read);
      if(bytes_read_now < 0){
        std::string error_msg = "Error reading data from Serial port: " + std::string(strerror(errno));
        throw std::runtime_error(error_msg);
      }else if (bytes_read_now  == 0){
        break; 
      }
      bytes_read += static_cast<size_t>(bytes_read_now);
    }
  }
  return data.substr(0, bytes_read);
}

int Serial::write(std::vector<uint8_t> data, int length){
  if(!is_open_){
    throw std::runtime_error("Serial::write");
  }
  size_t bytes_written = 0;
  while(bytes_written < length ){
    ssize_t bytes_written_now = ::write(fd_, &data[bytes_written], length - bytes_written);
    if(bytes_written_now < 0){
      std::string error_msg = "Error writing data from Serial port" + std::string(strerror(errno));
      throw std::runtime_error(error_msg);
    } 
    bytes_written += static_cast<size_t>(bytes_written_now);
  }
  return bytes_written;
}


int Serial::available(){
  if(!is_open_){
    return 0;
  }
  int count = 0;
  if(-1 == ioctl(fd_, TIOCINQ, &count)){
    std::string error_msg = "Error available: " + std::string(strerror(errno));
    throw std::runtime_error(error_msg);
  }else{
    return static_cast<size_t>(count);
  }
}

void Serial::reset(){
  if (!is_open_){
    throw std::runtime_error("Serial::reset");
  }
  try{
    int dtr_flag = TIOCM_DTR;
    if(ioctl(fd_, TIOCMBIC, &dtr_flag) == -1){ // DTR_claer
      std::string error_msg = "Error DTR_slear: " + std::string(strerror(errno));
      throw std::runtime_error(error_msg);
    }
    usleep(100000); // sleep 100ms
    if(ioctl(fd_, TIOCMBIS, &dtr_flag) == -1){ // DTR_start
      std::string error_msg = "Error DTR_start: " + std::string(strerror(errno));
      throw std::runtime_error(error_msg);
    }
  }catch(const std::exception &e){
    std::string error_msg = "Error reset: " + std::string(e.what());
    throw std::runtime_error(error_msg);
  }
}

CaBotArduinoSerialDelegate::CaBotArduinoSerialDelegate(CaBotSerialNode* delegate_node)
  : delegate_node_(delegate_node) {}

// Delegate definition for CaBotArduinoDriver class
std::tuple<int, int> CaBotArduinoSerialDelegate::system_time(){
    // return system time by tuple (sec, nsec)
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

CaBotArduinoSerial::CaBotArduinoSerial(std::shared_ptr<Serial> port, int baud, std::chrono::milliseconds timeout)
  :Node("cabot_arduino_serial"), logger_(get_logger()), port_(port), baud_(baud), timeout_(timeout), delegate_(nullptr),
   is_alive_(true), read_count_(0), time_synced_(false), no_input_count_(0) {}

void CaBotArduinoSerial::start(){
  reset_serial();
}

void CaBotArduinoSerial::reset_serial(){
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Info), "resetting serial port");
  port_->setDTR(false);
  sleep(0.2);
  port_->flushInput();
  port_->setDTR(true);
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
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Error), "exception occurred during reading: %s" + std::string(e.what()));
    stop();
  }
  try{
    process_write_once();
  }catch(const std::exception& e){
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Error), "exception occurred during reading: %s" + std::string(e.what()));
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
  std::string data_str;
  for(const auto& byte : data){
    data_str += std::to_string(byte) + " ";
  }
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "send " + data_str);
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
    int write_result = port_->write(data, length - total);
    if(write_result < 0){
      RCLCPP_ERROR(get_logger(), "error writing data ");
      return false;
    }
    total += write_result;
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "%d bytes written" + std::to_string(total));
  }
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), fmt::format("try to write invalid data type: %s", typeid(data).name())); 
  return true;
}

bool CaBotArduinoSerial::try_read(int length, std::vector<uint8_t>& result){
  try{
    int bytes_remaining = length;
    std::string received;
    std::chrono::time_point<std::chrono::system_clock> read_start = std::chrono::system_clock::now();
    while(bytes_remaining != 0 && (std::chrono::system_clock::now() - read_start) < timeout_){
      std::lock_guard<std::mutex> lock(read_mutex_);
      received =port_->read(bytes_remaining);
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
  if(port_->available() < 1){
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
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "reading command");
  try_read(1, received);
  cmd = received[0];
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "cmd = %d" + cmd);
  int size = 0;
  try_read(2, received);
  for(int i = 0; i < 2; i++){
    size |= received[i] << (8 * i);
  }
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "size =%d" + size);
  std::vector<uint8_t> data;
  try{
    try_read(size, data);
  }catch(const std::exception& e){
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "read error:%s" + std::string(e.what()));
    return false;
  }
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "data length =%d" + data.size());
  uint8_t checksum1 = 0;
  try_read(1, received);
  checksum1  = received[0];
  uint8_t checksum2 = this->checksum(data);
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "checksum %d %d" + checksum1 + checksum2);
  if(checksum1 != checksum2){
    return false;
  }
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "read data command =%d size =%d" + cmd + size);
  if(cmd == 0x01){ // timesync
    send_time_sync(data);
  }else if(cmd == 0x02){ // logdebug
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "%s" + std::string(data.begin(), data.end()));
  }else if(cmd == 0x03){ // loginfo
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Info), "%s" + std::string(data.begin(), data.end()));
  }else if(cmd == 0x04){ // logwarn
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Warn), "%s" + std::string(data.begin(), data.end()));
  }else if(cmd == 0x05){ // logerr
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Error), "%s" + std::string(data.begin(), data.end()));
  }else if(cmd == 0x08){
    send_param(data);
  }else if(cmd <= 0x10){
    delegate_->publish(cmd, data);
  }else{
    delegate_->log(static_cast<int>(rclcpp::Logger::Level::Error), "unknown command %#04X" + cmd);
    return false;
  }
  return true;
}

void CaBotArduinoSerial::send_time_sync(const std::vector<uint8_t>& data){
  // send current time
  uint32_t sec =0;
  uint32_t nsec =0;
  std::tuple<int, int> time_tuple = delegate_->system_time();
  sec = std::get<0>(time_tuple);
  nsec = std::get<1>(time_tuple);
  std::vector<uint8_t> temp;
  for(int i = 0; i < 4; i++){
    temp.push_back(static_cast<uint8_t>((sec >> (8 * i))& 0xFF));
    temp.push_back(static_cast<uint8_t>((nsec >>  (8 * i))& 0xFF));
  }
  send_command(0x01, temp);
  time_synced_ = true;
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), "sync");
  uint32_t remote_sec = 0;
  uint32_t remote_nsec = 0;
  for(int i = 0; i < 4; i++){
    remote_sec |= static_cast<uint32_t>(data[i] << (8 * i));
    remote_nsec |= static_cast<uint32_t>(data[i+4] << (8 * i));
  }
  float diff_ms = (sec - remote_sec) * 1000.0 + (nsec - remote_nsec ) * 0.000001;
  delegate_->log(static_cast<int>(rclcpp::Logger::Level::Debug), fmt::format(",,,,,,,,,,,{}.{}ms,{}.{},diff,{}ms", remote_sec % 1000, remote_nsec * 0.000001, sec % 1000, nsec * 0.000001, diff_ms));
}

uint8_t CaBotArduinoSerial::checksum(const std::vector<uint8_t>& data){
  uint8_t temp = 0;
  for(uint8_t d : data){
    temp += d;
  }
  return 0xFF - (0xFF & temp);
}
