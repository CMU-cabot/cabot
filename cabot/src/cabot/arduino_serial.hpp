#ifndef ARDUINOSERIAL_H_
#define ARDUINOSERIAL_H_


#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <queue>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <ctime>

#include <time.h>
#include <fcntl.h>
#include <stdexcept>

// https://stackoverflow.com/a/26221725
#include <memory>
#include <string>
#include <stdexcept>
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}
// end

class CaBotSerialNode;

class Serial {
public:
  Serial(std::string name, int baud, int read_timeout, int write_timeout);
  void openSerialPort(const std::string& port);
  void reconfigurePort();
  void setDTR(bool flag);
  void flushInput();
  bool waitReadable(uint32_t timeout);
  int read(uint8_t *buf, int size);
  int write(std::vector<uint8_t>, int length);
  int available();
  void reset();
private:
  int fd_;
  bool is_open_ = false;
  int read_timeout;
  int write_timeout;
};

class CaBotArduinoSerialDelegate {
public:
  virtual std::tuple<int, int> system_time() = 0;
  virtual void stopped() = 0;
  virtual void log(int level, const std::string & text) = 0;
  virtual void log_throttle(int level, int interval, const std::string & text) = 0;
  virtual void get_param(const std::string & name, std::function <void(const std::vector<int>&)> callback) = 0;
  virtual void publish(uint8_t cmd, const std::vector<uint8_t>&data) = 0; 
};

class CaBotArduinoSerial : public rclcpp::Node{
public:
  CaBotArduinoSerial(std::shared_ptr<Serial> port, int baud, std::chrono::milliseconds timeout =std::chrono::milliseconds(1000));
  
  void start();
  void send_command(uint8_t command, const std::vector<uint8_t>& arg);
  void stop();

  template<typename T>
  void send_param(const T& data);

  std::shared_ptr<CaBotArduinoSerialDelegate> delegate_;
  void reset_serial();
  bool is_alive_;
  void run_once();

private:
  rclcpp::Logger logger_;
  std::shared_ptr<Serial> port_;
  int baud_;
  std::chrono::milliseconds timeout_;
  std::thread read_thread_;
  std::mutex read_mutex_;
  std::thread write_thread_;
  std::queue<std::vector<uint8_t>> write_queue_;
  std::mutex write_mutex_;
  int read_count_;
  bool time_synced_;
  int no_input_count_;
  bool process_write_once();
  bool try_read(int length, std::vector<uint8_t>& result);
  bool process_read_once();
  void send_time_sync(const std::vector<uint8_t>& data);
  uint8_t checksum(const std::vector<uint8_t>& data);

  //friend class CaBotSerialNode;
};

template<typename T>
void CaBotArduinoSerial::send_param(const T& data){
  std::vector<uint8_t> temp;
  for(typename T::const_iterator D = data.begin(); D != data.end(); ++D){
    typename T::value_type d = *D;
    for(int i = 0; i < sizeof(typename T::value_type); i++){
      temp.push_back((d >> (8 * i))& 0xFF);
    }
  }
  send_command(0x08, temp);
  std::string data_str;
  for(typename T::const_iterator D = data.begin(); D != data.end(); ++D){
    typename T::value_type d = *D;
    data_str += std::to_string(d) +" ";
  }
  RCLCPP_INFO(get_logger(), "%s", data_str.c_str());
}


#endif /* ARDUINOSERIAL_H_ */
